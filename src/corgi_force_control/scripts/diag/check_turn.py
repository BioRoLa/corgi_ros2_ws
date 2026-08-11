"""Did the robot hold a circle, and how big was it?

Offline only -- reads a .npz written by check_ramp.py --dump. No simulator time.

Why this exists. check_ramp answers "is the gait alive" per segment and
check_yaw_phase answers "where inside the stride does yaw come from". Neither
can answer the question a commanded turn actually poses, which is geometric:
the robot was asked to yaw at a constant rate, so its path should be a circle of
radius v/psi_dot, and what we want is that radius, how well the path holds it,
and what fraction of the commanded rate was delivered.

Three numbers, and they are independent measurements of the same thing:

  R_fit    algebraic circle fit to the odom track. Purely positional -- it never
           looks at the heading, so it cannot inherit an orientation-convention
           mistake.
  R_rate   v / psi_dot from the measured speed and the measured yaw rate. Uses
           the orientation, not the path.
  R_cmd    v / turn_rate, the radius that was ASKED for.

R_fit == R_rate says the body is going where it is pointing, i.e. the turn is a
turn and not a crab. R_rate < R_cmd means the channel did not deliver the
commanded rate -- which is the envelope boundary, and the thing step 3 of the
handover is looking for.

The RMS radial residual of the fit is the cross-track error: how far off its own
best-fit circle the robot wandered. It is the number to quote for "held a
circle" as opposed to "curved".

CAUTION on short arcs. A Kasa fit to less than about a quarter turn is badly
conditioned -- it will happily return a plausible-looking radius with a large
uncertainty, and the algebraic form is biased small on top of that. The arc
actually covered is printed, and anything under 90 deg is flagged. Believe
R_rate over R_fit in that regime.

Usage:
    python3 check_turn.py <dump.npz> [--turn-rate 0.394] [--start 1.0]

--turn-rate is in rad/s. If omitted it is recovered from the controller log
(the "Turn: turn_rate=..." line gslip_pronk prints every run), which is safer
than retyping it and mislabelling a dump.
"""
import argparse
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_yaw_phase import contact_at, dedupe_time, yaw_from_quat  # noqa: E402
from ramp_segments import load as load_template  # noqa: E402

# Rim radius and track width. RIM_RADIUS_M is NOT the rolling radius -- see
# rolling_radius_m() below, and implementation log section 21.
RIM_RADIUS_M = 0.145

# CONTACT track, not the hip spacing. The wheel planes sit outboard of the hips
# by WHEEL_AXIAL_OFFSET = 0.091675 m on all four legs (traced through the proto
# transform tree in Stage 0: A/B rotate module-x to +y with offsets +/-0.0917,
# C/D rotate module-x to -y with offsets +/-0.0917, and the two flips cancel).
# So the feet are 0.4234 m apart, not the 0.240 m hip-to-hip that the
# differential-drive model originally used -- a factor of 1.76 in every yaw
# prediction.
TRACK_M = 0.4234

# OO_r(theta): hip to foot-rim centre, in metres, theta in DEGREES. Quartic fit
# to LegModel over theta 17..160 deg, max residual 1.3 mm. Embedded rather than
# imported because this script runs in the ROS workspace, where legwheel is not
# installed.
_OOR_POLY = [-4.392823600163269e-10, 1.1908398631167022e-07,
             -9.823205461288225e-06, 0.002055536221516562,
             -0.03398687310630567]


def rolling_radius_m(theta_deg):
    """-> the distance from hip to contact, which IS the rolling radius.

    The handover's differential-drive model used the RIM radius, 0.145 m. That
    is the correct rolling radius only at theta = 17 deg, where the leg-wheel is
    folded shut and all three rim centres sit exactly on the hip -- a true wheel
    with the hip as its axle. As theta opens, the rim centre walks away from the
    hip and the two diverge, reaching 2.02x at the theta = 100 deg running pose.
    Using 0.145 there halves every prediction; see section 21.

    Confirmed independently by Lee, Yu & Lin (2026 ICRA) eq 11, whose standing
    height is H = R + OO_r*cos(beta) -- the same quantity, from the group that
    designed the leg.

    Why the hip and not the rim centre: the foot rim's absolute orientation is
    exactly beta, so a beta rotation turns the rim by beta, the contact point is
    the instantaneous centre, and the hip -- which carries the body -- swings
    through |hip - contact| per radian.
    """
    return np.polyval(_OOR_POLY, np.asarray(theta_deg)) + RIM_RADIUS_M

# Module order is A=FL, B=FR, C=RR, D=RL, so A/D are the left pair and B/C the
# right -- the same partition gslip_pronk's steer_sign uses.
LEFT = (0, 3)
RIGHT = (1, 2)

DEFAULT_CTL_LOG = "/tmp/ramp_ctl.log"


def fit_circle(x, y):
    """Kasa algebraic circle fit. -> (cx, cy, R, signed radial residuals).

    Minimises the algebraic distance |p-c|^2 - R^2 rather than the geometric
    one, which makes it a single linear solve with no initial guess and no
    iteration to diverge. The price is a small-radius bias that grows as the arc
    shrinks; on a full circle it is negligible, which is why the covered arc is
    reported alongside every radius this prints.
    """
    A = np.column_stack((2.0 * x, 2.0 * y, np.ones_like(x)))
    b = x * x + y * y
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy, c = sol
    R = float(np.sqrt(max(c + cx * cx + cy * cy, 0.0)))
    return float(cx), float(cy), R, np.hypot(x - cx, y - cy) - R


def turn_rate_from_log(path):
    """-> commanded turn rate in rad/s from gslip_pronk's log line, else None."""
    try:
        with open(path) as f:
            text = f.read()
    except OSError:
        return None
    hits = re.findall(r"turn_rate=(-?\d+\.?\d*)", text)
    return float(hits[-1]) if hits else None


def swept_in_contact(t, beta, down):
    """-> signed beta swept while the foot was DOWN, summed over contact runs.

    Peak-to-peak beta is the wrong quantity and has produced a wrong answer
    before: the template sweeps -18.2 -> +18.2 deg in stance and swings straight
    back through the same range in flight, so p2p double-counts a swing that
    rolls nothing. Summing (end - start) over each contiguous contact run keeps
    only the part in contact, which is the part that rolls.
    """
    if len(t) < 2:
        return 0.0
    edges = np.flatnonzero(np.diff(down.astype(int)) != 0) + 1
    bounds = np.concatenate(([0], edges, [len(down)]))
    total = 0.0
    for a, b in zip(bounds[:-1], bounds[1:]):
        if b - a < 2 or not down[a]:
            continue
        total += beta[b - 1] - beta[a]
    return float(total)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump")
    ap.add_argument("--turn-rate", type=float, default=None,
                    help="commanded turn rate, rad/s. Default: read it out of "
                         "--ctl-log.")
    ap.add_argument("--ctl-log", default=DEFAULT_CTL_LOG)
    ap.add_argument("--start", type=float, default=3.5,
                    help="template time to start analysing at, seconds. The "
                         "default is not a stride or two of slack -- on the "
                         "constant-speed template the robot does not leave the "
                         "ground at all for the first ~3 s (measured: flight "
                         "0.2%%, 0.0%%, 6.2%% in the first three seconds, then "
                         "32-52%% thereafter). It starts from a standing settle "
                         "and has to accelerate into a fixed point solved for "
                         "2.035 m/s. Including that window halves the apparent "
                         "flight fraction and drags the speed down by 20%%.")
    ap.add_argument("--end", type=float, default=None)
    args = ap.parse_args()

    d = np.load(args.dump, allow_pickle=True)
    ot, ov = d["odom_t"], d["odom"]
    ct, cv = d["contact_t"], d["contact"]
    mt, mv = d["motor_t"], d["motor_deg"]
    if ov.shape[1] < 10:
        print("dump has no orientation columns -- needs a check_ramp.py that "
              "records odom quaternions (corgi_sim >= 77dcac1).")
        return

    cmd_rate = args.turn_rate
    src = "--turn-rate"
    if cmd_rate is None:
        cmd_rate = turn_rate_from_log(args.ctl_log)
        src = args.ctl_log
    if cmd_rate is None:
        print(f"no commanded turn rate given and none found in {args.ctl_log}; "
              f"reporting measured quantities only.")
        src = None

    # Duplicate odom timestamps make every dt-based derivative NaN. Handled in
    # check_yaw_phase; do not re-implement it here.
    ot, ov = dedupe_time(ot, ov)

    t_end = args.end if args.end is not None else ot.max()
    m = (ot >= args.start) & (ot <= t_end)
    if int(m.sum()) < 100:
        print(f"only {int(m.sum())} odom samples in "
              f"{args.start:.2f}..{t_end:.2f} s -- nothing to fit.")
        return
    t, seg = ot[m], ov[m]
    x, y = seg[:, 0], seg[:, 1]
    yaw = np.rad2deg(yaw_from_quat(seg[:, 6:10]))
    span = t[-1] - t[0]

    print(f"{os.path.basename(args.dump)}: template t "
          f"{t[0]:.2f}..{t[-1]:.2f} s ({span:.2f} s, {int(m.sum())} samples)")
    if src:
        print(f"commanded turn rate {cmd_rate:+.4f} rad/s "
              f"({np.rad2deg(cmd_rate):+.2f} deg/s), from {src}")
    print()

    # --- Validity gate ------------------------------------------------------
    # check_ramp cannot do this for a looping template. It slices by SEGMENT,
    # and ramp_segments finds one "whole template" segment 0.22 s long, so its
    # flight and theta columns describe the first stride of the run and nothing
    # else. Every run of a turning campaign would go ungated. So the gate lives
    # here, measured over the same window everything else is measured over.
    #
    # Roughly one run in five fails it, and failures cluster late in a session.
    # A failed gate means re-run, not a finding.
    cm = (ct >= t[0]) & (ct <= t[-1])
    gate_ok = True
    if int(cm.sum()) > 100:
        cseg = cv[cm]
        air = 100.0 * float((~cseg.any(axis=1)).mean())
        alld = 100.0 * float(cseg.all(axis=1).mean())
        mmg = (mt >= t[0]) & (mt <= t[-1])
        th_max = float(mv[mmg][:, :, 0].max()) if int(mmg.sum()) else float("nan")
        design_air = float("nan")
        try:
            _, _, _, _, in_stance = load_template(str(d["template"]))
            design_air = 100.0 * float(1.0 - in_stance.mean())
        except Exception:
            pass
        # The gate is "is this gait alive", NOT "did it reach design flight".
        # Those are different questions and conflating them would fail every
        # run of this template for a reason that is not a fault:
        #
        # measured, steady window, no steering -- flight 42%, design 56.4%. The
        # single-stride template is the v~1.20 FIXED POINT, solved for 2.035
        # m/s, and the robot starts from standstill and converges to ~0.72 m/s.
        # At a third of the design speed the touchdown state is nothing like
        # the one the template assumes, so sub-design flight is the expected
        # consequence of running this template from rest, not a bad run.
        #
        # What the gate has to catch is the failure the handover warns about --
        # 0% flight, 100% all-down, theta pinned below command, which is a dead
        # gait wearing a live gait's label. Those thresholds do that, and the
        # flight-vs-design gap is reported separately so it cannot be lost.
        # theta needs BOTH bounds: too low is a leg that never extended, too
        # high is the collapse signature of section 19 (theta overshooting to
        # 131.6 and 117.9 deg against a 100 deg command). A one-sided test
        # passes a robot thrashing at 136 deg, which it did.
        gate_ok = air > 25.0 and 97.0 < th_max < 110.0 and alld < 60.0
        verdict = "PASS" if gate_ok else "FAIL -- re-run, not a finding"
        print(f"  GATE  flight {air:5.1f}% (design {design_air:5.1f}%)"
              f"   all-down {alld:5.1f}%   theta max {th_max:6.2f} deg"
              f"   -> {verdict}")
        if gate_ok and np.isfinite(design_air) and air < 0.85 * design_air:
            print(f"        (flight is {100*air/design_air:.0f}% of design -- "
                  f"expected on this template, see the comment; compare runs "
                  f"against each other, not against design)")
        print()

    # --- Speed --------------------------------------------------------------
    # Path length, not end-to-end displacement. On a circle the two are wildly
    # different and it is the path the robot actually travelled that divides
    # into the yaw rate to give a radius.
    step = np.hypot(np.diff(x), np.diff(y))
    path_len = float(step.sum())
    v_path = path_len / span
    v_twist = float(np.mean(seg[:, 3]))
    print(f"  path length {path_len:8.3f} m over {span:.2f} s"
          f"   -> v {v_path:6.3f} m/s   (v twist {v_twist:6.3f} m/s)")

    # --- Yaw rate -----------------------------------------------------------
    # Least-squares slope over the window rather than (last - first)/span: on a
    # steady turn they agree, and when they do not the disagreement is the tell
    # that the rate was not steady. Both are printed.
    slope, _ = np.polyfit(t, yaw, 1)
    psi_dot_fit = float(np.deg2rad(slope))
    psi_dot_ends = float(np.deg2rad((yaw[-1] - yaw[0]) / span))
    print(f"  net yaw {yaw[-1]-yaw[0]:+8.2f} deg"
          f"   rate {np.rad2deg(psi_dot_fit):+7.2f} deg/s (fit)"
          f"   {np.rad2deg(psi_dot_ends):+7.2f} deg/s (endpoints)")

    # Per stride, counted from the TEMPLATE period rather than from contact
    # edges. The legs desynchronise by 32-45%, so `any foot down` rises two or
    # three times per stride and an edge-counted "stride" is really a contact
    # cycle -- which inflates the count and quietly deflates deg/stride. The
    # controller plays one row per 1 ms tick and wraps, so the period is exact
    # and needs no detection at all.
    stride_s = float("nan")
    try:
        tt_tpl, _, _, _, _ = load_template(str(d["template"]))
        stride_s = float(tt_tpl[-1])
    except Exception:
        pass
    if np.isfinite(stride_s) and stride_s > 0:
        n_str = span / stride_s
        print(f"  template stride {stride_s:.4f} s -> {n_str:5.1f} strides"
              f"   dyaw/stride {(yaw[-1]-yaw[0])/n_str:+7.3f} deg"
              f"   (fit {np.rad2deg(psi_dot_fit)*stride_s:+7.3f})")
        print("    deg/stride is the unit the authority table is built in. The")
        print("    fit figure is the one to quote: endpoints alone inherit")
        print("    whatever phase of the body's own yaw oscillation the window")
        print("    happens to start and end on.")
    if abs(psi_dot_fit) < 1e-4:
        print("\n  yaw rate is essentially zero -- this is a heading-hold run, "
              "not a turn.\n  Radius figures below are meaningless.")

    # --- The three radii ----------------------------------------------------
    cx, cy, r_fit, resid = fit_circle(x, y)
    rms = float(np.sqrt(np.mean(resid ** 2)))
    # How much of the circle was actually covered. A fit over a short arc is
    # poorly conditioned however clean the residual looks.
    phi = np.unwrap(np.arctan2(y - cy, x - cx))
    arc_deg = float(np.rad2deg(abs(phi[-1] - phi[0])))

    r_rate = abs(v_path / psi_dot_fit) if abs(psi_dot_fit) > 1e-6 else float("nan")
    r_cmd = (abs(v_path / cmd_rate)
             if cmd_rate not in (None, 0.0) else float("nan"))

    print()
    print(f"  circle fit   centre ({cx:+7.3f}, {cy:+7.3f})   "
          f"R_fit  {r_fit:7.3f} m")
    print(f"               cross-track RMS {rms*1000:7.1f} mm   "
          f"max |residual| {np.abs(resid).max()*1000:7.1f} mm")
    print(f"               arc covered {arc_deg:6.1f} deg"
          + ("   <-- SHORT ARC, R_fit is weakly determined; prefer R_rate"
             if arc_deg < 90.0 else ""))
    print(f"  from rates   R_rate {r_rate:7.3f} m   = v / psi_dot")
    if np.isfinite(r_cmd):
        print(f"  commanded    R_cmd  {r_cmd:7.3f} m   = v / turn_rate")
        deliv = abs(psi_dot_fit / cmd_rate) if cmd_rate else float("nan")
        print(f"               delivered {100.0*deliv:5.1f}% of the commanded "
              f"yaw rate"
              + ("   <-- SATURATED: past the envelope" if deliv < 0.85 else ""))
    if np.isfinite(r_rate) and r_fit > 0:
        ratio = r_fit / r_rate
        # Only worth flagging when a turn was actually commanded. With
        # turn_rate = 0 the "circle" is a parasitic drift with no centre to
        # speak of, the fit is unconstrained, and a ratio far from 1 says
        # nothing -- flagging it there would train the reader to ignore the
        # flag in the one case it matters.
        note = ""
        if cmd_rate:
            note = ("   (path and heading agree)" if 0.9 <= ratio <= 1.1
                    else "   <-- path and heading DISAGREE: the body is not "
                         "pointing along its path (crab, or a yaw convention "
                         "mistake)")
        elif not 0.9 <= ratio <= 1.1:
            note = "   (no turn commanded -- the fit has no real circle to find)"
        print(f"  R_fit / R_rate {ratio:6.3f}{note}")

    # --- Per stride ---------------------------------------------------------
    # A mean rate hides a turn that decays. Touchdown edges come from the
    # contact stream, resampled onto the odom clock the same way check_yaw_phase
    # does it.
    down = contact_at(t, ct, cv)
    any_down = down.any(axis=1)
    edges = np.flatnonzero(any_down[1:] & ~any_down[:-1]) + 1
    if len(edges) >= 3:
        print()
        print("  per CONTACT CYCLE (any-foot-down rising edge to the next).")
        print("  NOT one row per stride: with the legs 32-45% desynchronised a")
        print("  stride contains two or three of these. Read it for trend and")
        print("  for whether the turn is sustained, and take deg/stride from")
        print("  the template-period figure above.")
        print(f"    {'#':>3} {'t':>7} {'dyaw':>8} {'rate':>9} {'flight':>8} "
              f"{'v':>7} {'R_rate':>8}")
        for k in range(len(edges) - 1):
            i0, i1 = edges[k], edges[k + 1]
            dt_s = t[i1] - t[i0]
            if dt_s <= 0:
                continue
            dpsi = yaw[i1] - yaw[i0]
            vk = float(np.hypot(np.diff(x[i0:i1 + 1]),
                                np.diff(y[i0:i1 + 1])).sum() / dt_s)
            air = 100.0 * float((~any_down[i0:i1]).mean())
            rk = (abs(vk / np.deg2rad(dpsi / dt_s))
                  if abs(dpsi) > 1e-6 else float("nan"))
            print(f"    {k:3d} {t[i0]:7.3f} {dpsi:+8.2f} {dpsi/dt_s:+9.2f} "
                  f"{air:7.1f}% {vk:7.3f} {rk:8.3f}")
        print()
        print("    dyaw per stride is the number the authority table is built")
        print("    from. A column that decays means the turn is not sustained.")

    # --- Scrub, from the TEMPLATE sweep (the trustworthy version) -----------
    # Compare the distance actually travelled per stride against what the
    # template's COMMANDED stance sweep would roll at the corrected radius.
    #
    # This is the version to quote. The contact-gated measurement below uses
    # sim/leg_contact, which chatters -- 39-50 contact runs per leg over 29
    # strides, i.e. two bounces per stride -- and a signed sweep summed over
    # fragments silently cancels whenever a fragment lands in the swing-back.
    # That drags the measured sweep to a third of the commanded one and makes
    # the slip look enormous. The template sweep has no such problem: it is what
    # the controller actually asked the leg to do.
    if np.isfinite(stride_s) and stride_s > 0:
        try:
            _, th_tpl, b_tpl, _, st_tpl = load_template(str(d["template"]))
            sweep_tpl = float(b_tpl[st_tpl][-1] - b_tpl[st_tpl][0])
            th_bar = float(np.rad2deg(th_tpl[st_tpl].mean()))
            r_tpl = float(rolling_radius_m(th_bar))
            per_stride = path_len / (span / stride_s)
            pred = abs(sweep_tpl) * r_tpl
            print()
            print("  slip, from the template sweep (quote THIS one)")
            print(f"    commanded stance sweep {np.rad2deg(sweep_tpl):6.2f} deg"
                  f"   mean stance theta {th_bar:5.1f} deg"
                  f"   rolling radius {r_tpl:.4f} m")
            print(f"    predicted advance {pred*1000:7.1f} mm/stride"
                  f"   measured {per_stride*1000:7.1f} mm/stride"
                  f"   -> {100*per_stride/pred:5.1f}% of no-slip")
            print(f"    (with the old r = {RIM_RADIUS_M} m it would predict "
                  f"{abs(sweep_tpl)*RIM_RADIUS_M*1000:.1f} mm/stride, "
                  f"{100*per_stride/(abs(sweep_tpl)*RIM_RADIUS_M):.0f}% -- "
                  f"which is how a 2x radius error hides)")
        except Exception:
            pass

    # --- Scrub, as a proxy --------------------------------------------------
    # The physically interesting quantity, and the hard one. Stance is rolling
    # contact, so a foot down for a beta sweep of d_beta should roll its side of
    # the robot forward by r*d_beta. Whatever the body did NOT travel, it slid.
    #
    # Read the two halves of this differently. The ABSOLUTE ratio carries an
    # unvalidated calibration -- the leg is not a wheel bolted to the hip, the
    # hip translates relative to the contact point as well, so r*d_beta is a
    # lower bound on the advance and the ratio is expected to exceed 1 even with
    # no slip at all. The DIFFERENTIAL comparison below it does not have that
    # problem: the common-mode calibration divides out, so it is a real test of
    # whether the left/right sweep difference explains the measured yaw.
    mm = (mt >= args.start) & (mt <= t_end)
    if int(mm.sum()) > 100:
        mtt = mt[mm]
        mdown = contact_at(mtt, ct, cv)
        sweep = np.array([
            swept_in_contact(mtt, np.deg2rad(mv[mm][:, i, 1]), mdown[:, i])
            for i in range(4)])
        s_l = float(sweep[list(LEFT)].mean())
        s_r = float(sweep[list(RIGHT)].mean())
        # Rolling radius at the mean stance theta, not the rim radius. theta
        # moves 100 -> 84.4 -> 100 deg through stance, which is 0.264..0.293 m
        # of rolling radius, so the mean is used and the range is printed.
        th_stance = mv[mm][:, :, 0][mdown]
        r_roll = float(rolling_radius_m(float(np.mean(th_stance))))
        r_lo = float(rolling_radius_m(float(np.percentile(th_stance, 5))))
        r_hi = float(rolling_radius_m(float(np.percentile(th_stance, 95))))
        roll_len = r_roll * 0.5 * (s_l + s_r)
        runs = [int(np.count_nonzero(np.diff(mdown[:, i].astype(int)) > 0))
                for i in range(4)]
        print()
        print("  scrub (proxy)")
        print(f"    beta swept while in contact, summed over the window:")
        # Per leg as well as per side. The four legs desynchronise by 32-45%,
        # so they enter contact at different points of their own sweep and the
        # per-leg totals differ by an order of magnitude. Averaging that into a
        # left/right pair without showing it invites reading a desync artefact
        # as a steering differential.
        for i, nm in enumerate("ABCD"):
            print(f"      {nm}  {np.rad2deg(sweep[i]):8.1f} deg over "
                  f"{runs[i]:3d} contacts"
                  f"   ({np.rad2deg(sweep[i])/max(runs[i],1):6.2f} deg each)")
        print(f"      left  (A,D) {np.rad2deg(s_l):8.1f} deg"
              f"    right (B,C) {np.rad2deg(s_r):8.1f} deg"
              f"    diff {np.rad2deg(s_l-s_r):+8.1f} deg")
        print(f"    rolling radius {r_roll:.4f} m at mean stance theta "
              f"{float(np.mean(th_stance)):.1f} deg  (5-95%: "
              f"{r_lo:.4f}..{r_hi:.4f})")
        print(f"    rolling arc L*sweep {roll_len:7.3f} m"
              f"   vs path {path_len:7.3f} m"
              f"   PATH/ROLL {path_len/roll_len if roll_len else float('nan'):6.3f}")
        print("      This ratio IS now calibrated -- with the correct rolling")
        print("      radius, no-slip predicts 1.00. Above 1 means the robot")
        print("      travelled further than the feet rolled, i.e. the feet")
        print("      slipped forward or spent time not rolling at all.")
        if abs(s_l + s_r) > 1e-6:
            # Differential drive, made dimensionless so r cancels:
            #   psi_dot * track / v  ==  (sweep_L - sweep_R) / mean_sweep
            measured = psi_dot_fit * TRACK_M / v_path if v_path else float("nan")
            predicted = (s_l - s_r) / (0.5 * (s_l + s_r))
            print(f"    differential check (r cancels):")
            print(f"      psi_dot*track/v {measured:+8.4f}"
                  f"   vs (dL-dR)/mean {predicted:+8.4f}")
            print("      Same magnitude means the yaw is the sweep differential")
            print("      rolling the robot round. A measured value LARGER than")
            print("      predicted means something else is also turning it; the")
            print("      SIGN relation is the one to pin down first, and it is")
            print("      not derivable from the geometry -- take it from a +/-")
            print("      pair of open-loop runs, the way steer_offset's sense")
            print("      was taken.")


if __name__ == "__main__":
    main()
