#!/usr/bin/env python3
"""Signed path curvature at touchdown resolution -- the Menger estimator.

Offline only. No simulator time.

Why this exists. check_turn.py answers "did the robot hold a circle" with a
whole-window Kasa fit and a whole-window v/psi_dot. Both collapse the run to
one number, and both are blind to the question the Stage 3 mechanism
demonstration actually asks: does commanding an Ackermann camber pair bend the
path, by how much, in which direction, and is the bend sustained or a
transient? That needs curvature AS A SERIES, sampled where the gait is
comparable stride to stride -- at touchdown.

The estimator: interpolate the COM ground-truth position at each touchdown of
one fiducial leg, then over touchdown triples (P_{k-m}, P_k, P_{k+m}) compute
the signed Menger curvature

    kappa = 2 * cross(P2-P1, P3-P2) / (|P2-P1| |P3-P2| |P3-P1|)

which is 1/R of the circumcircle through the triple, signed + for a
left (CCW, world +z) turn. The median over the window is the headline; the
per-triple table and the early/mid/late split show whether the turn decays.

Why ONE fiducial leg and not any-foot-down: the legs desynchronise by 32-45%,
so `any foot down` rises two or three times per stride and a triple built on
those edges spans a fraction of a stride -- mostly the body's own oscillation,
not the path. One leg's rising edge fires once per stride by construction.

Why a fixed ARC LENGTH apart (--chord, default 1.0 m per half-chord) and not
consecutive touchdowns, nor a fixed stride count: measured on the banked
v070_db_stop_odo captures, the COM at touchdown carries 5-30 mm of the body's
own stride-frequency sway, and curvature noise from a sway of height h over a
chord L scales as ~8h/L^2. Consecutive touchdowns (L ~ 0.25 m) put that at
+-20 1/m, two orders above the real signal (R_rate ~ 4 m, kappa ~ 0.25), and
the median came out 10x off with the WRONG SIGN. A fixed stride count fixes
nothing at low speed: during spin-up the robot advances centimetres per
stride, 8-stride chords collapse to ~0.3 m, and the same failure returned
(kappa +4..+7 against a yaw-rate kappa of -0.7). Arc length is the quantity
the noise floor actually depends on, so the span is chosen in metres and the
touchdown grid just supplies the sample points. At 1 m half-chords the floor
is ~0.03 1/m. Each triple still centres on a touchdown; the table slides one
touchdown at a time.

Yaw cross-check, NOT a gate: kappa_yaw = dyaw/ds over the same span measures
the HEADING's curvature. On a clean rolling turn it matches Menger; when the
body drifts sideways (crab -- which these captures show during spin-up, and
which check_turn.py's R_fit/R_rate split exists to catch) the path curves
without the heading turning, and the two disagree because the physics does,
not because the tool is wrong. The printed comparison says which regime the
window is in; the position-only consistency check (Menger vs Kasa on the
steady tail) is the tool validation.

Inputs, primary (the repeat_gain_regime.sh capture pair):
    --odom-csv  odom_runN.csv   `ros2 topic echo /sim/base_odom --csv`
                                (stamp cols 0-1, position 4-6, quat x,y,z,w
                                7-10 -- field order fixed by the message spec)
    --torque-csv runN.csv       /tmp/corgi_torque_terms.csv copy, long format
                                t,leg,motor,in_contact,...,theta,beta,gamma
Secondary:
    --npz dump.npz              check_ramp.py --dump keys (odom_t, odom,
                                contact_t, contact). No gamma -> no pose gate.

CLOCKS. The torque CSV's t and the odom stamp are both sim time, but the odom
echo starts ~SETTLE_WALL after the trigger while the torque capture starts at
driver connect, so neither starts at zero and neither is trigger-anchored.
They are the SAME clock with different first samples; the overlap is computed
and everything is measured inside it. If the overlap is thin the run is
refused rather than silently fit (gate-analysers-against-invalid-input).

Pose-fidelity gate (--gamma-in/--gamma-out/--gamma-dir, degrees / +-1): the
ACHIEVED gamma over the window, as LEFT/RIGHT PAIR MEANS, is compared against
the commanded Ackermann pattern gamma_i = dir * lr_sign_i * (in if
dir*lr_sign_i < 0 else out), lr_sign = {+1,-1,-1,+1} for A,B,C,D (same
partition as gslip_pronk's roll_sign; gamma > 0 abducts outward). Pair means
because gamma_correction's yaw term is FRONT/REAR signed ({+,+,-,-}) and
cancels inside each left/right pair -- measured +-2.6 deg of it on the banked
baselines -- while the commanded lean does not. Worst-pair error > 1.5 deg
marks the run INVALID -- the radius of a pose the robot never held is not a
result. Per-leg achieved gamma is always printed; commanded lambda predicts
neither side better than ~3 deg at any kp, so claims quote ACHIEVED. NOTE the
roll-feedback term is left/right signed and does NOT cancel in pair means:
runs with k_roll active will show it as pair-mean deviation, which is a real
"pose not held", not an artefact.

Usage:
    python3 check_menger.py --odom-csv odom_run1.csv --torque-csv run1.csv
    python3 check_menger.py --odom-csv o.csv --torque-csv r.csv \
        --gamma-in 5.0 --gamma-out 4.5 --gamma-dir +1
    python3 check_menger.py --npz dump.npz
"""
import argparse
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_turn import fit_circle  # noqa: E402
from check_yaw_phase import dedupe_time, yaw_from_quat  # noqa: E402

LEGS = ("A", "B", "C", "D")
# Same left/right partition as gslip_pronk's roll_sign/steer_sign and
# camber_roll.py's PATTERNS["lr"]. gamma > 0 abducts OUTWARD on all four legs.
LR_SIGN = {"A": +1.0, "B": -1.0, "C": -1.0, "D": +1.0}

# Contact debounce, in consecutive 1 kHz samples. sim/leg_contact chatters
# (two bounces per stride were measured in check_turn's scrub section); three
# samples kills the chatter without eating a real 30 ms stance.
DEBOUNCE = 3

# v070 stride, for the touchdown-spacing sanity line. Not load-bearing.
V070_STRIDE_S = 0.265


def load_odom_csv(path):
    """-> (t, xy (n,2), quat (n,4) x,y,z,w) from a base_odom --csv capture."""
    t, xy, q = [], [], []
    with open(path) as f:
        for row in csv.reader(f):
            if len(row) < 11:
                continue
            try:
                sec, nsec = float(row[0]), float(row[1])
                x, y = float(row[4]), float(row[5])
                quat = [float(row[i]) for i in (7, 8, 9, 10)]
            except ValueError:
                continue
            t.append(sec + 1e-9 * nsec)
            xy.append((x, y))
            q.append(quat)
    if not t:
        raise SystemExit(f"{path}: no parseable odom rows -- wrong file?")
    return np.array(t), np.array(xy), np.array(q)


def load_torque_csv(path):
    """-> (t, in_contact (n,4) bool, gamma_deg (n,4), theta_deg (n,4)).

    The capture is long-format, one row per (t, leg, motor); contact and the
    joint state are per-LEG and repeat across that leg's motor rows, so the
    first row seen per (t, leg) wins and the rest are skipped.
    """
    per_leg = {leg: {"t": [], "c": [], "g": [], "th": []} for leg in LEGS}
    with open(path) as f:
        rdr = csv.reader(f)
        header = next(rdr)
        col = {name: i for i, name in enumerate(header)}
        need = ("t", "leg", "in_contact", "theta", "gamma")
        missing = [n for n in need if n not in col]
        if missing:
            raise SystemExit(f"{path}: missing columns {missing} -- not a "
                             f"corgi_torque_terms capture?")
        it, ileg = col["t"], col["leg"]
        ic, ith, ig = col["in_contact"], col["theta"], col["gamma"]
        last = {}
        for row in rdr:
            leg = row[ileg]
            if leg not in per_leg:
                continue
            tv = row[it]
            if last.get(leg) == tv:
                continue
            last[leg] = tv
            d = per_leg[leg]
            d["t"].append(float(tv))
            d["c"].append(row[ic] not in ("0", "0.0", ""))
            d["g"].append(float(row[ig]))
            d["th"].append(float(row[ith]))
    ts = [np.array(per_leg[leg]["t"]) for leg in LEGS]
    n = min(len(a) for a in ts)
    if n < 100:
        raise SystemExit(f"{path}: only {n} samples for the thinnest leg -- "
                         f"nothing to measure.")
    # The four legs are written on the same 1 kHz tick; trim to the common
    # length rather than resampling.
    t = ts[0][:n]
    contact = np.column_stack(
        [np.array(per_leg[leg]["c"][:n], dtype=bool) for leg in LEGS])
    gamma = np.rad2deg(np.column_stack(
        [np.array(per_leg[leg]["g"][:n]) for leg in LEGS]))
    theta = np.rad2deg(np.column_stack(
        [np.array(per_leg[leg]["th"][:n]) for leg in LEGS]))
    return t, contact, gamma, theta


def debounce(sig, n):
    """-> sig with runs shorter than n samples removed, both polarities."""
    out = sig.copy()
    edges = np.flatnonzero(np.diff(out.astype(int)) != 0) + 1
    bounds = np.concatenate(([0], edges, [len(out)]))
    for a, b in zip(bounds[:-1], bounds[1:]):
        if b - a < n:
            out[a:b] = not out[a] if b < len(out) else out[a - 1]
    return out


def touchdowns(t, down):
    """-> times of debounced rising edges."""
    d = debounce(down, DEBOUNCE)
    idx = np.flatnonzero(d[1:] & ~d[:-1]) + 1
    return t[idx]


def menger(p1, p2, p3):
    """-> signed curvature of the circumcircle through three 2D points.

    cross() is twice the signed triangle area, so 2*cross/(abc) is the
    standard 4A/(abc) with the CCW-positive sign kept.
    """
    a = p2 - p1
    b = p3 - p2
    c = p3 - p1
    la, lb, lc = np.hypot(*a), np.hypot(*b), np.hypot(*c)
    if min(la, lb, lc) < 1e-9:
        return 0.0, 0.0
    cross = a[0] * b[1] - a[1] * b[0]
    # Height of P2 over the chord P1-P3: the collinearity scale. A height at
    # the odom noise floor means the triple carries no curvature information
    # and 1/kappa would print a huge, meaningless radius.
    height = abs(cross) / lc
    return 2.0 * cross / (la * lb * lc), height


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--odom-csv")
    ap.add_argument("--torque-csv")
    ap.add_argument("--npz")
    ap.add_argument("--start", type=float, default=3.5,
                    help="seconds after the analysis clock's zero (the start "
                         "of the odom/contact overlap) to skip -- the robot "
                         "accelerates from a standing settle and barely flies "
                         "for ~3 s (check_turn.py --start rationale).")
    ap.add_argument("--end", type=float, default=None,
                    help="seconds after the overlap start to STOP at. The "
                         "first lam0_default run showed garbage curvature "
                         "both before ~start and late in a long run; a "
                         "bounded window keeps the medians on the steady "
                         "band.")
    ap.add_argument("--fiducial-leg", default="A", choices=LEGS)
    ap.add_argument("--chord", type=float, default=1.0,
                    help="triple half-chord in METRES of path: kappa at "
                         "touchdown k uses the touchdowns nearest to "
                         "+-chord of arc length either side. Below ~0.5 the "
                         "body's own sway swamps the signal -- see the module "
                         "docstring before lowering this.")
    ap.add_argument("--floor", type=float, default=2e-3,
                    help="collinearity floor in metres of mid-point height. "
                         "Triples flatter than this report kappa 0 +- floor.")
    ap.add_argument("--theta-lo", type=float, default=100.0,
                    help="gait-gate theta max bounds, DEGREES. Defaults fit "
                         "the v070 template under CORGI_THETA_STOP. The "
                         "~161 deg excursions are EARLY-run only, and the "
                         "steady-band theta max varies run to run (104-144 "
                         "measured), so the lower bound only rules out a "
                         "leg that never left the fold -- deadness is the "
                         "flight/all-down gates' job (a standing hold at "
                         "the 100 deg settle pose fails those two). "
                         "check_turn.py's 97..110 band is for the ramp "
                         "template and FAILS every healthy v070 window.")
    ap.add_argument("--theta-hi", type=float, default=165.0)
    ap.add_argument("--gamma-in", type=float, default=None,
                    help="commanded inner-pair camber, DEGREES (pose gate)")
    ap.add_argument("--gamma-out", type=float, default=None)
    ap.add_argument("--gamma-dir", type=float, default=None,
                    help="+1 or -1, the commanded turn side; 0/omitted for a "
                         "baseline run (gate checks gamma ~ 0 on all legs)")
    args = ap.parse_args()

    # --- Ingest -------------------------------------------------------------
    gamma = theta = None
    if args.npz:
        d = np.load(args.npz, allow_pickle=True)
        ot, ov = dedupe_time(d["odom_t"], d["odom"])
        xy, quat = ov[:, 0:2], ov[:, 6:10]
        ct, contact = d["contact_t"], d["contact"].astype(bool)
        label = os.path.basename(args.npz)
    elif args.odom_csv and args.torque_csv:
        ot, xy, quat = load_odom_csv(args.odom_csv)
        ot, xy, quat = dedupe_time(ot, xy, quat)
        ct, contact, gamma, theta = load_torque_csv(args.torque_csv)
        label = os.path.basename(args.odom_csv)
    else:
        ap.error("need --odom-csv AND --torque-csv, or --npz")

    # --- Clock overlap ------------------------------------------------------
    t0 = max(ot[0], ct[0])
    t1 = min(ot[-1], ct[-1])
    if args.end is not None:
        t1 = min(t1, t0 + args.end)
    if t1 - t0 < 5.0:
        raise SystemExit(
            f"odom [{ot[0]:.2f}..{ot[-1]:.2f}] and contact "
            f"[{ct[0]:.2f}..{ct[-1]:.2f}] overlap for only {t1-t0:.2f} s -- "
            f"streams from different runs, or a clock that is not shared. "
            f"Refusing to fit.")
    a0 = t0 + args.start
    om = (ot >= a0) & (ot <= t1)
    if int(om.sum()) < 100:
        raise SystemExit(f"only {int(om.sum())} odom samples in the analysis "
                         f"window -- nothing to measure.")
    t, seg_xy, seg_q = ot[om], xy[om], quat[om]
    span = t[-1] - t[0]
    print(f"{label}: overlap {t0:.2f}..{t1:.2f} s, analysing "
          f"{a0:.2f}..{t1:.2f} ({span:.2f} s, {int(om.sum())} odom samples)")

    # --- Gait-alive gate (air/all-down from check_turn.py; theta recalibrated
    # for the v070 template under the theta stop -- see --theta-lo help) -----
    cm = (ct >= a0) & (ct <= t1)
    gate_ok = True
    if int(cm.sum()) > 100:
        cseg = contact[cm]
        air = 100.0 * float((~cseg.any(axis=1)).mean())
        alld = 100.0 * float(cseg.all(axis=1).mean())
        th_max = float(theta[cm].max()) if theta is not None else float("nan")
        th_ok = ((args.theta_lo < th_max < args.theta_hi)
                 if np.isfinite(th_max) else True)
        gate_ok = air > 25.0 and th_ok and alld < 60.0
        verdict = "PASS" if gate_ok else "FAIL -- re-run, not a finding"
        print(f"  GATE  flight {air:5.1f}%   all-down {alld:5.1f}%   "
              f"theta max {th_max:6.2f} deg "
              f"(gate {args.theta_lo:.0f}..{args.theta_hi:.0f})   -> {verdict}")

    # --- Pose fidelity ------------------------------------------------------
    if gamma is not None:
        gm = gamma[cm] if int(cm.sum()) > 100 else gamma
        ach = gm.mean(axis=0)
        # Pair means: gamma_correction's yaw term is front/rear signed and
        # cancels within each left/right pair; the commanded lean does not.
        left = 0.5 * (ach[0] + ach[3])   # A + D
        right = 0.5 * (ach[1] + ach[2])  # B + C
        print(f"  achieved gamma (deg):  "
              + "  ".join(f"{leg} {ach[i]:+6.2f}" for i, leg in enumerate(LEGS))
              + f"   pair means L {left:+6.2f}  R {right:+6.2f}")
        if args.gamma_dir is not None:
            s = args.gamma_dir
            if s == 0.0:
                want_l = want_r = 0.0
            else:
                # Left pair carries lr_sign +1, right pair -1. Inner side is
                # where s*lr_sign < 0.
                mag_l = args.gamma_in if s < 0 else args.gamma_out
                mag_r = args.gamma_in if s > 0 else args.gamma_out
                want_l = s * (+1.0) * mag_l
                want_r = s * (-1.0) * mag_r
            worst = max(abs(left - want_l), abs(right - want_r))
            ok = worst <= 1.5
            print(f"  pose fidelity: pair means vs commanded "
                  f"(L {want_l:+.2f}, R {want_r:+.2f}) -> worst "
                  f"{worst:.2f} deg -> "
                  + ("OK" if ok else "INVALID -- the commanded pose was not "
                                     "held; do not quote this run's radius"))
            gate_ok = gate_ok and ok

    # --- Touchdown fiducial -------------------------------------------------
    fid = LEGS.index(args.fiducial_leg)
    td_all = touchdowns(ct, contact[:, fid])
    td = td_all[(td_all >= a0) & (td_all <= t1)]
    if len(td) < 5:
        raise SystemExit(f"only {len(td)} leg-{args.fiducial_leg} touchdowns "
                         f"in the window -- nothing to triangulate.")
    dt_td = np.diff(td)
    print(f"  {len(td)} leg-{args.fiducial_leg} touchdowns, median spacing "
          f"{np.median(dt_td)*1000:.0f} ms (v070 stride {V070_STRIDE_S*1000:.0f} "
          f"ms -- a large mismatch means the debounce or the clock is wrong)")

    # --- Menger series, paired with a yaw-only estimate ---------------------
    px = np.interp(td, t, seg_xy[:, 0])
    py = np.interp(td, t, seg_xy[:, 1])
    pts = np.column_stack((px, py))
    yaw = yaw_from_quat(seg_q)
    yaw_td = np.interp(td, t, yaw)
    # Cumulative path length on the odom clock, for arc-length spans.
    step = np.hypot(np.diff(seg_xy[:, 0]), np.diff(seg_xy[:, 1]))
    s_cum = np.concatenate(([0.0], np.cumsum(step)))
    s_td = np.interp(td, t, s_cum)

    if s_td[-1] - s_td[0] < 2.5 * args.chord:
        raise SystemExit(f"only {s_td[-1]-s_td[0]:.2f} m of path between the "
                         f"first and last touchdown -- too short for "
                         f"{args.chord:.2f} m half-chords.")
    rows = []
    for k in range(len(td)):
        # Nearest touchdowns to +-chord of arc length either side of k.
        i = int(np.argmin(np.abs(s_td - (s_td[k] - args.chord))))
        j = int(np.argmin(np.abs(s_td - (s_td[k] + args.chord))))
        if i >= k or j <= k:
            continue
        # A chord that could not reach at least 60% of the requested length
        # (window edge) would smuggle the short-chord noise back in.
        if (s_td[k] - s_td[i] < 0.6 * args.chord
                or s_td[j] - s_td[k] < 0.6 * args.chord):
            continue
        kap, height = menger(pts[i], pts[k], pts[j])
        ds = s_td[j] - s_td[i]
        kyaw = (yaw_td[j] - yaw_td[i]) / ds if ds > 1e-6 else 0.0
        rows.append((td[k], kap, kyaw, height))
    if len(rows) < 3:
        raise SystemExit("fewer than 3 valid triples -- lengthen the run or "
                         "lower --chord (and read the docstring first).")
    kaps = np.array([r[1] for r in rows])
    kyaws = np.array([r[2] for r in rows])
    heights = np.array([r[3] for r in rows])
    flat = heights < args.floor
    keep = ~flat if (~flat).any() else np.ones_like(flat)
    live, live_yaw = kaps[keep], kyaws[keep]

    med = float(np.median(live))
    med_yaw = float(np.median(live_yaw))
    q1, q3 = np.percentile(live, [25, 75])
    print()
    print(f"  Menger, half-chord {args.chord:.2f} m, {len(rows)} triples "
          f"({int(flat.sum())} below the {args.floor*1000:.1f} mm "
          f"collinearity floor, excluded):")
    print(f"    median kappa {med:+.5f} 1/m   IQR [{q1:+.5f}, {q3:+.5f}]")
    if med:
        signcons = float((np.sign(live) == np.sign(med)).mean())
        print(f"    -> R {1.0/abs(med):8.3f} m   sign "
              f"{'+CCW/left' if med > 0 else '-CW/right'}   "
              f"sign-consistency {100*signcons:.0f}%")
    else:
        print("    -> straight at this floor")
    # Sustainment: a transient turn shows up as a decaying |kappa|, which a
    # whole-window median hides.
    thirds = np.array_split(live, 3)
    if all(len(x) for x in thirds):
        e, mi, l = (float(np.median(x)) for x in thirds)
        print(f"    early/mid/late median kappa {e:+.5f} / {mi:+.5f} / "
              f"{l:+.5f}   (decay -> transient, not steady)")

    # --- Consistency checks -------------------------------------------------
    # Position-only: the Kasa fit over the same window is an independent
    # position-only radius; on a sustained arc the two must agree, and THAT is
    # the tool validation. Orientation: kappa_yaw matching says the body rolls
    # along its heading; a mismatch is crab (real physics), reported not gated.
    x, y = seg_xy[:, 0], seg_xy[:, 1]
    cx, cy, r_fit, resid = fit_circle(x, y)
    phi = np.unwrap(np.arctan2(y - cy, x - cx))
    arc_deg = float(np.rad2deg(abs(phi[-1] - phi[0])))
    slope, _ = np.polyfit(t, yaw, 1)
    v_path = float(step.sum()) / span
    r_rate = abs(v_path / slope) if abs(slope) > 1e-6 else float("inf")
    sign_agree = float((np.sign(live) == np.sign(live_yaw)).mean())
    print()
    print(f"  consistency:")
    print(f"    Kasa   R_fit  {r_fit:8.3f} m   arc {arc_deg:6.1f} deg"
          + ("   <-- SHORT ARC, weakly determined" if arc_deg < 90 else ""))
    if med:
        ratio = (1.0 / abs(med)) / r_fit if r_fit > 0 else float("nan")
        print(f"    Menger R / R_fit {ratio:6.3f}   (position-only vs "
              f"position-only: the tool check on a sustained arc)")
    print(f"    yaw    kappa_yaw median {med_yaw:+.5f} 1/m   sign agreement "
          f"{100*sign_agree:.0f}%   (mismatch = crab, not a tool fault)")
    print(f"    rates  R_rate {r_rate:8.3f} m   = v/psi_dot "
          f"({v_path:.3f} m/s / {np.rad2deg(slope):+.2f} deg/s), "
          f"sign {'+CCW' if slope > 0 else '-CW'}")
    if not gate_ok:
        print()
        print("  RUN INVALID (gate above). Numbers printed for diagnosis only.")

    print()
    print(f"    {'#':>3} {'t':>8} {'kappa':>9} {'k_yaw':>9} {'R':>8} "
          f"{'height_mm':>10}")
    for k, (tk, kap, kyaw, h) in enumerate(rows):
        rr = 1.0 / abs(kap) if abs(kap) > 1e-9 else float("inf")
        note = "  (flat)" if h < args.floor else ""
        print(f"    {k:3d} {tk:8.3f} {kap:+9.5f} {kyaw:+9.5f} {rr:8.3f} "
              f"{h*1000:10.2f}{note}")


if __name__ == "__main__":
    main()
