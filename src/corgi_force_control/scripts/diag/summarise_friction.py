"""Friction bracket: did more grip help, and did it help turning differently?

Registered prediction (sweep_friction.sh, written before the runs):
    straight-line no-slip % improves with mu;
    turning no-slip % improves much less;
    radius scatter narrows.

Usage:  python3 summarise_friction.py <dir>
"""
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_turn import rolling_radius_m  # noqa: E402
from check_yaw_phase import dedupe_time, yaw_from_quat  # noqa: E402
from ramp_segments import load as load_template  # noqa: E402

START, END = 4.0, 8.0


def one(path):
    d = np.load(path, allow_pickle=True)
    ot, ov = dedupe_time(d["odom_t"], d["odom"])
    ct, cv, mt, mv = d["contact_t"], d["contact"], d["motor_t"], d["motor_deg"]
    m = (ot >= START) & (ot <= END)
    if int(m.sum()) < 50:
        return None
    t, seg = ot[m], ov[m]
    x, y = seg[:, 0], seg[:, 1]
    span = t[-1] - t[0]
    path_len = float(np.hypot(np.diff(x), np.diff(y)).sum())
    v = path_len / span
    yaw = np.rad2deg(yaw_from_quat(seg[:, 6:10]))
    slope, _ = np.polyfit(t, yaw, 1)
    psi = np.deg2rad(slope)

    tt, th_t, b_t, _, st_t = load_template(str(d["template"]))
    stride_s = float(tt[-1])
    sweep = abs(float(b_t[st_t][-1] - b_t[st_t][0]))
    r_roll = float(rolling_radius_m(float(np.rad2deg(th_t[st_t].mean()))))
    noslip = 100 * (path_len / (span / stride_s)) / (sweep * r_roll)

    cm = (ct >= t[0]) & (ct <= t[-1])
    air = 100.0 * float((~cv[cm].any(axis=1)).mean())
    mmg = (mt >= t[0]) & (mt <= t[-1])
    th = float(mv[mmg][:, :, 0].max())

    sat = float("nan")
    if "torque_nm" in d.files and len(d["torque_nm"]):
        qt, q = d["torque_t"], np.abs(d["torque_nm"])
        qm = (qt >= START) & (qt <= END)
        sat = 100.0 * float((q[qm][:, :, :2] >= 34.9).mean())

    return dict(v=v, noslip=noslip, air=air, th=th, sat=sat,
                r=abs(v / psi) if abs(psi) > 1e-6 else float("nan"),
                psi=np.rad2deg(psi))


def main():
    root = sys.argv[1]
    print(f"{'group':>16} {'n':>3} {'v':>14} {'of no-slip':>16} "
          f"{'flight':>13} {'theta max':>11} {'sat%':>11}")
    for mu in ("0.6", "1.6"):
        for mode in ("straight", "turn"):
            rows = [one(p) for p in
                    sorted(glob.glob(os.path.join(root, f"mu{mu}_{mode}_*.npz")))]
            rows = [r for r in rows if r]
            if not rows:
                continue

            def ms(k):
                a = np.array([r[k] for r in rows], dtype=float)
                a = a[np.isfinite(a)]
                return (f"{a.mean():6.2f}±{a.std():5.2f}" if len(a)
                        else f"{'--':>12}")
            print(f"  mu={mu:>4} {mode:<8} {len(rows):3d} {ms('v'):>14} "
                  f"{ms('noslip'):>16} {ms('air'):>13} {ms('th'):>11} "
                  f"{ms('sat'):>11}")
        # radius scatter, turning only
        rows = [one(p) for p in
                sorted(glob.glob(os.path.join(root, f"mu{mu}_turn_*.npz")))]
        rows = [r for r in rows if r and np.isfinite(r["r"])]
        if len(rows) >= 2:
            rr = np.array([r["r"] for r in rows])
            print(f"           turn radii: "
                  + ", ".join(f"{x:.2f}" for x in rr)
                  + f"   spread {rr.max()-rr.min():.2f} m")
    print()
    print("  sat% = fraction of leg-motor samples pegged at the 35 N.m clamp")


if __name__ == "__main__":
    main()
