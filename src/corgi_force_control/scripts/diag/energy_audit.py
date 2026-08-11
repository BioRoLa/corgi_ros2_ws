"""Where does the energy go? Positive vs negative motor work, per stride.

Section 27 left the ~40% speed shortfall unexplained: no template reaches its
own design speed, and the template mismatch is not the reason.

A SLIP fixed point is CONSERVATIVE -- the spring returns everything it stores,
so a robot sitting exactly on one needs no net energy input at all. Real losses
therefore set where the robot actually settles: it slows until injected energy
equals what it bleeds. This measures that balance directly, the same way the
b_radial audit in section 10c did (105 W in, 99.6 W straight back out, netting
5.5 W and never lifting off).

Method. Motor power is tau . phi_dot at the two leg motors. The dumps carry
torque (torque_r, torque_l) and joint ANGLES but not joint velocities, so
phi_dot comes from differentiating the measured angles through the same
transform force_control.cpp uses:

    phi_l = theta + beta - 17 deg      phi_r = beta - theta + 17 deg

Differentiating a measured signal amplifies noise, so the angles are smoothed
over a window short against a stance (see SMOOTH_S) before differencing, and
the result is reported as a per-stride mean rather than a peak.

    python3 energy_audit.py <dir> <prefix> [<prefix> ...]
"""
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_yaw_phase import contact_at, smooth  # noqa: E402
from ramp_segments import load as load_template  # noqa: E402

START, END = 4.0, 9.5
SMOOTH_S = 0.005          # 5 ms: well inside a ~110 ms stance


def audit(path):
    d = np.load(path, allow_pickle=True)
    if "torque_nm" not in d.files or not len(d["torque_nm"]):
        return None
    mt, mv = d["motor_t"], d["motor_deg"]
    qt, q = d["torque_t"], d["torque_nm"]
    ct, cv = d["contact_t"], d["contact"]

    m = (mt >= START) & (mt <= END)
    t = mt[m]
    if len(t) < 500:
        return None
    th = np.deg2rad(mv[m][:, :, 0])
    be = np.deg2rad(mv[m][:, :, 1])

    # Joint angles, the driver's convention.
    off = np.deg2rad(17.0)
    phi_l = th + be - off
    phi_r = be - th + off

    dt = np.median(np.diff(t))
    def deriv(x):
        return np.gradient(
            np.column_stack([smooth(x[:, i], t, SMOOTH_S) for i in range(4)]),
            dt, axis=0)
    dphi_l, dphi_r = deriv(phi_l), deriv(phi_r)

    # Torque on the same clock. The two streams are published together, so an
    # index-align would usually work, but they are separate lists -- resample.
    qi = np.clip(np.searchsorted(qt, t) - 1, 0, len(qt) - 1)
    tau_r, tau_l = q[qi][:, :, 0], q[qi][:, :, 1]

    p_leg = tau_r * dphi_r + tau_l * dphi_l          # W, per leg
    p_tot = p_leg.sum(axis=1)

    down = contact_at(t, ct, cv)
    any_down = down.any(axis=1)

    pos = float(np.mean(np.clip(p_tot, 0, None)))
    neg = float(np.mean(np.clip(p_tot, None, 0)))
    stance_pos = float(np.mean(np.clip(p_tot[any_down], 0, None)))
    stance_neg = float(np.mean(np.clip(p_tot[any_down], None, 0)))
    flight_pos = float(np.mean(np.clip(p_tot[~any_down], 0, None)))

    tt, _, _, _, st_t = load_template(str(d["template"]))
    stride_s = float(tt[-1])
    return dict(pos=pos, neg=neg, net=pos + neg,
                stance_pos=stance_pos, stance_neg=stance_neg,
                flight_pos=flight_pos,
                e_stride=(pos + neg) * stride_s,
                recirc=-neg / pos if pos else float("nan"))


def main():
    root = sys.argv[1]
    print(f"window {START}-{END} s   motor power, both leg motors, all four legs")
    print()
    print(f"{'group':>7} {'n':>2} {'W in':>12} {'W out':>12} {'W net':>12} "
          f"{'recirc':>9} {'J/stride':>10} {'stance out':>12}")
    for pre in sys.argv[2:]:
        rows = [audit(p) for p in
                sorted(glob.glob(os.path.join(root, f"{pre}_*.npz")))]
        rows = [r for r in rows if r]
        if not rows:
            print(f"{pre:>7}  no runs with torque")
            continue

        def ms(k, f="{:6.1f}"):
            a = np.array([r[k] for r in rows], dtype=float)
            return f"{f.format(a.mean())}±{a.std():5.1f}"
        print(f"{pre:>7} {len(rows):2d} {ms('pos'):>12} {ms('neg'):>12} "
              f"{ms('net'):>12} {ms('recirc', '{:5.2f}'):>9} "
              f"{ms('e_stride', '{:5.2f}'):>10} {ms('stance_neg'):>12}")
    print()
    print("  W in   = mean POSITIVE motor power (energy injected)")
    print("  W out  = mean NEGATIVE motor power (energy taken back out)")
    print("  recirc = |out|/in. Near 1 means the motors are fighting")
    print("           themselves: almost everything put in comes straight back.")
    print("  A conservative SLIP fixed point needs net ~0 to SUSTAIN a speed,")
    print("  so a large recirculating flow with small net is the signature of")
    print("  a gait bleeding its energy into damping rather than into travel.")


if __name__ == "__main__":
    main()
