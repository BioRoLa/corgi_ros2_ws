#!/usr/bin/env python3
"""Did uniform camber steer the rolling leg-wheel, and by how much?

Offline only -- reads a .npz written by record_camber.py. No simulator time.

The question this answers is the one Stage 0 handed forward: the uniform-camber
command pattern gamma = lambda*{+1,-1,-1,+1} was DERIVED from the sign
conventions and never run, and every cambered stage rests on it.

Three radii, and they are independent measurements of the same thing:

  R_fit    algebraic circle fit to the odom track. Purely positional -- it
           never looks at the heading, so it cannot inherit an
           orientation-convention mistake.
  R_rate   v / psi_dot from the measured speed and the measured yaw rate. Uses
           the orientation, not the path.
  R_pred   rho*cos^2(lambda)/sin(lambda), the single-wheel cone-apex geometry.
           This is what the run is testing.

R_fit == R_rate says the body is going where it is pointing, i.e. it turned
rather than crabbed.

WHAT THE GATE IS, and it is not R_fit == R_pred. R_pred is a SINGLE wheel's
geometry. Four of them are bolted to one rigid body and turned at a matched
beta_dot on a 0.4234 m contact track, so at R = 0.809 m the inner and outer
pairs want radii 0.60 and 1.02 m and must fight each other; the achieved radius
will exceed the prediction by whatever fraction of rolling geometry this robot
delivers (32-56% in the sagittal work). A constant derating cancels in a ratio,
so the primary test is the SCALING between two camber angles:

    R(10 deg) / R(20 deg) = 2.17

Run this on both dumps and compare. The derating ratio R_fit/R_pred is worth
recording on its own: Open Issue 9 holds two camber derating figures that
disagree by 3x, and this measures it directly.

CAUTION on short arcs, inherited from check_turn.py: a Kasa fit to less than
about a quarter turn is badly conditioned and biased small. The arc covered is
printed and anything under 90 deg is flagged.

Usage:
    python3 check_camber_turn.py <dump.npz> --lam-deg 20 [--pattern lr]
"""
import argparse
import math

import numpy as np

# Stage 0's tread model. Flat at R_OUT for |w| <= W_FLAT, then a fillet of
# radius R_CORNER out to the 20 mm edge -- a bicycle tyre cut and fitted flat.
# Embedded rather than imported because legwheel is not installed in the ROS
# workspace; the same reason check_turn.py carries its own _OOR_POLY. The
# authority is LegWheel examples/gslip/crown_effect.py, and camber_roll.py
# carries the identical constants.
R_OUT, R_CORNER, W_FLAT = 0.145, 0.015, 0.005

# Wheel-mode rolling radius. 0.145 m is correct HERE and almost nowhere else:
# at theta = 17 deg every rim centre coincides with the hip, so the leg-wheel
# really is a wheel on its own axle. At the running pose the rolling radius is
# 0.293 m. See implementation log section 21 before "fixing" this.
ROLLING_RADIUS_M = 0.145

LEGS = ("A", "B", "C", "D")


def predicted_turn_radius(lam_deg):
    """Cone-apex turn radius of one cambered rolling wheel, metres.

    A cambered wheel turns because its contact circle is centred where the spin
    axis meets the ground. The crown enters only through rho, and both the real
    flat-and-filleted tread and a fully crowned one share R_OUT -- which is why
    the crown-radius gate was retired: R_turn differs by 0.1-0.5% between them.
    """
    a = math.radians(abs(lam_deg))
    if a < 1e-9:
        return float("inf")
    rho = (R_OUT - R_CORNER) + R_CORNER * math.cos(a)
    return rho * math.cos(a) ** 2 / math.sin(a)


def find_roll_start(d, thresh_deg=1.0):
    """-> when rolling began, in recorder time, from the COMMANDED beta.

    Do not assume camber_roll.py's nominal 7.5 s. The commander re-latches its
    schedule if the sim clock jumps backwards, and the recorder still anchors
    on the original trigger, so the two can disagree by seconds. Commanded beta
    is identically zero until the roll and then climbs monotonically, which
    makes it an unambiguous fiducial -- the same trick check_ramp.py uses on
    the stride template.

    If the schedule restarted there are several such crossings; take the LAST
    one, because that is the pass the robot actually completed.
    """
    if "cmd_deg" not in d:
        return 7.5
    kt, kv = d["cmd_t"], d["cmd_deg"]
    beta = np.abs(kv[:, :, 1].mean(axis=1))
    rising = beta > thresh_deg
    if not rising.any():
        return 7.5
    # Start of the final contiguous run of "beta is moving".
    idx = np.flatnonzero(rising)
    breaks = np.flatnonzero(np.diff(idx) > 1)
    first_of_last = idx[breaks[-1] + 1] if len(breaks) else idx[0]
    return float(kt[first_of_last])


def fit_circle(x, y):
    """Kasa algebraic circle fit. -> (cx, cy, R, signed radial residuals).

    Lifted from check_turn.py, which explains the trade: it minimises the
    algebraic distance rather than the geometric one, so it is a single linear
    solve with no initial guess and nothing to diverge, at the cost of a
    small-radius bias that grows as the arc shrinks.
    """
    A = np.column_stack((2.0 * x, 2.0 * y, np.ones_like(x)))
    b = x * x + y * y
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy, c = sol
    R = float(np.sqrt(max(c + cx * cx + cy * cy, 0.0)))
    return float(cx), float(cy), R, np.hypot(x - cx, y - cy) - R


def yaw_from_quat(q):
    """-> yaw in radians from an (N,4) array of (x, y, z, w)."""
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def unwrap_yaw(psi):
    return np.unwrap(psi)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("dump")
    ap.add_argument("--lam-deg", type=float, required=True,
                    help="the camber that was commanded, degrees")
    ap.add_argument("--pattern", default="lr",
                    help="lr / fr / none, for the report header only")
    ap.add_argument("--roll-start", type=float, default=None,
                    help="seconds after the trigger at which rolling begins. "
                         "By default it is RECOVERED from the commanded beta "
                         "rather than assumed, so a schedule that shifted "
                         "(e.g. camber_roll re-latching after a sim-clock "
                         "jump) still analyses the right window.")
    args = ap.parse_args()

    d = np.load(args.dump)
    ot, ov = d["odom_t"], d["odom"]
    mt, mv = d["motor_t"], d["motor_deg"]

    roll_start = args.roll_start
    if roll_start is None:
        roll_start = find_roll_start(d)
    m = ot >= roll_start
    if m.sum() < 20:
        print(f"Only {int(m.sum())} odom samples after t={roll_start:.1f}s. "
              f"The roll phase is missing -- did the run end early?")
        return

    x, y = ov[m, 0], ov[m, 1]
    vx, vy = ov[m, 3], ov[m, 4]
    t = ot[m]
    psi = unwrap_yaw(yaw_from_quat(ov[m, 6:10]))

    print(f"lambda {args.lam_deg:+.1f} deg   pattern {args.pattern}   "
          f"{len(t)} odom samples over {t[-1]-t[0]:.2f} s of rolling")
    print(f"  roll starts at t = {roll_start:.2f} s"
          + ("" if args.roll_start is not None
             else " (recovered from commanded beta)"))

    # Pitch as well as roll: a lifted pair sits the body nose-up, because the
    # cambered wheels support their hubs lower than upright ones do.
    qxp, qyp = ov[m, 6], ov[m, 7]
    qzp, qwp = ov[m, 8], ov[m, 9]
    pitch = np.degrees(np.arcsin(
        np.clip(2.0 * (qwp * qyp - qzp * qxp), -1.0, 1.0)))
    print(f"  body pitch      {pitch.mean():+7.2f} deg mean over the roll")

    # Which legs were actually carrying. A leg commanded off the ground that
    # still reports contact is riding light, not lifted -- and the difference
    # decides what the run measured.
    if "contact" in d:
        ct, cv = d["contact_t"], d["contact"]
        cm = ct >= roll_start
        if cm.sum():
            print("  contact %       " + "  ".join(
                f"{leg} {100*cv[cm][:, i].mean():5.1f}"
                for i, leg in enumerate(LEGS)))
    print()

    # --- pose fidelity first. Everything below is void if the robot was not in
    # the pose it was given, and the ABAD has a known 1.5-2 deg left/right
    # split (B, C above A, D).
    mm = mt >= roll_start
    print("  achieved pose during the roll (measured, mean over the roll):")
    for i, leg in enumerate(LEGS):
        th = mv[mm][:, i, 0].mean()
        ga = mv[mm][:, i, 2].mean()
        print(f"    {leg}: theta {th:6.2f} deg   gamma {ga:+7.2f} deg")
    if "cmd_deg" in d:
        kt, kv = d["cmd_t"], d["cmd_deg"]
        km = kt >= roll_start
        if km.sum():
            print("  commanded, and the error:")
            worst = 0.0
            for i, leg in enumerate(LEGS):
                want = kv[km][:, i, 2].mean()
                got = mv[mm][:, i, 2].mean()
                worst = max(worst, abs(got - want))
                print(f"    {leg}: gamma commanded {want:+7.2f} "
                      f"achieved {got:+7.2f}  err {got-want:+6.2f} deg")
            if worst > 1.5:
                print(f"    WARNING: worst camber error {worst:.2f} deg. The "
                      f"robot is not in the commanded pose, so the radius "
                      f"below is not the radius for this lambda.")

    # --- the path
    cx, cy, r_fit, resid = fit_circle(x, y)
    arc = abs(psi[-1] - psi[0])
    dist = float(np.sum(np.hypot(np.diff(x), np.diff(y))))
    speed = dist / (t[-1] - t[0])

    # psi_dot by least squares over the whole roll rather than an end
    # difference, which would be at the mercy of two samples.
    psi_dot = float(np.polyfit(t, psi, 1)[0])
    r_rate = abs(speed / psi_dot) if abs(psi_dot) > 1e-6 else float("inf")
    r_pred = predicted_turn_radius(args.lam_deg)

    # Body roll and ride height, which turned out to be what separates the
    # candidate patterns. A pattern that moves every contact to the same side
    # tips the body; a symmetric splay does not. And the ride height drop is
    # the Stage 0 contact model showing up directly: h = R cos(lambda) -
    # w_flat sin(lambda) + r_corner predicts 9.55 mm at 20 deg.
    qx, qy, qz, qw = ov[m, 6], ov[m, 7], ov[m, 8], ov[m, 9]
    roll = np.arctan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    pre = (ot > 4.0) & (ot < roll_start - 2.0)
    print()
    print(f"  body roll       {np.degrees(roll).mean():+7.2f} deg mean "
          f"over the roll")
    if pre.sum() > 10:
        print(f"  ride height     {1000*(ov[pre, 2].mean() - ov[m, 2].mean()):+7.2f} mm "
              f"dropped between the pre-lean hold and the roll")

    print()
    print(f"  travelled       {dist:7.3f} m at {speed:6.3f} m/s mean")
    print(f"  heading swept   {math.degrees(arc):7.1f} deg "
          f"({math.degrees(psi_dot):+.2f} deg/s)")
    print(f"  R_fit           {r_fit:7.3f} m   "
          f"(cross-track RMS {1000*float(np.sqrt((resid**2).mean())):.1f} mm)")
    print(f"  R_rate          {r_rate:7.3f} m")
    print(f"  R_pred          {r_pred:7.3f} m   single cambered wheel, "
          f"cone-apex geometry")
    if math.isfinite(r_pred) and r_fit > 0:
        print(f"  R_fit / R_pred  {r_fit/r_pred:7.2f}   "
              f"-> {100*r_pred/r_fit:.0f}% of the geometric turn is delivered")

    # A straight run is a real outcome here, not a degenerate fit, so say so
    # plainly rather than emitting short-arc and crab warnings that are both
    # true and both meaningless when there is no arc at all.
    if math.degrees(arc) < 5.0:
        print(f"  => NO MEASURABLE CURVATURE. {math.degrees(arc):.2f} deg of "
              f"heading over {dist:.2f} m. Whatever the fit says, this ran "
              f"straight; compare against the lambda = 0 baseline before "
              f"calling it a null.")
    else:
        if math.degrees(arc) < 90.0:
            print(f"  ! arc is {math.degrees(arc):.0f} deg, under a quarter "
                  f"turn. The Kasa fit is badly conditioned and biased small "
                  f"here -- believe R_rate over R_fit.")
        if abs(r_fit - r_rate) > 0.25 * max(r_fit, r_rate):
            print(f"  ! R_fit and R_rate disagree by "
                  f"{100*abs(r_fit-r_rate)/max(r_fit,r_rate):.0f}%. The body "
                  f"is not going where it points -- it is crabbing, not "
                  f"turning.")

    # --- did it roll, or did it scrub? Wheel mode is the one regime where
    # 0.145 m is the right rolling radius, so this is a clean check. If the
    # wheels are sliding, the geometric test is void whatever the fit says.
    if mm.sum() > 2:
        dbeta = np.deg2rad(mv[mm][:, :, 1].mean(axis=1))
        swept = float(dbeta[-1] - dbeta[0])
        no_slip = ROLLING_RADIUS_M * swept
        print()
        print(f"  beta swept      {math.degrees(swept):7.1f} deg "
              f"-> {no_slip:.3f} m of pure rolling")
        if abs(no_slip) > 1e-6:
            ratio = dist / no_slip
            print(f"  no-slip ratio   {ratio:7.2f}   "
                  f"(travelled / rolled; 1.00 is pure rolling)")
            if not 0.7 <= ratio <= 1.3:
                print(f"  ! the wheels are not rolling cleanly. A geometric "
                      f"turn-radius prediction assumes rolling contact, so "
                      f"treat the radius above as indicative only.")


if __name__ == "__main__":
    main()
