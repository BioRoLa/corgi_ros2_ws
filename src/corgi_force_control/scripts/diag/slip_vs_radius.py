"""Does slip explain why the turn radius does not repeat?

Four runs at the identical command (turn_rate = +0.288 rad/s) produced radii
from 1.35 to 2.69 m and delivered yaw rates from 32% to 123%. That scatter is
what blocks the Stage 4 matched-radius comparison.

Slip is the obvious suspect: it is measurable now that the rolling radius is
corrected, it varies run to run, and turning is already known to quadruple it
(8% -> 32%). If it tracks the radius, the scatter has a mechanism and a lever.
If it does not, the scatter is something else and this is one command's worth of
evidence saying so.

    python3 slip_vs_radius.py <tag>=<dump.npz> ...
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_turn import rolling_radius_m  # noqa: E402
from check_yaw_phase import contact_at, dedupe_time, yaw_from_quat  # noqa: E402
from ramp_segments import load as load_template  # noqa: E402

START, END = 4.0, 8.0
CMD_RATE = 0.288


def one(path):
    d = np.load(path, allow_pickle=True)
    ot, ov = dedupe_time(d["odom_t"], d["odom"])
    ct, cv, mt, mv = d["contact_t"], d["contact"], d["motor_t"], d["motor_deg"]
    m = (ot >= START) & (ot <= END)
    t, seg = ot[m], ov[m]
    x, y = seg[:, 0], seg[:, 1]
    yaw = np.rad2deg(yaw_from_quat(seg[:, 6:10]))
    span = t[-1] - t[0]
    path_len = float(np.hypot(np.diff(x), np.diff(y)).sum())
    v = path_len / span
    slope, _ = np.polyfit(t, yaw, 1)
    psi_dot = np.deg2rad(slope)

    tt, th_t, b_t, _, st_t = load_template(str(d["template"]))
    stride_s = float(tt[-1])
    sweep = abs(float(b_t[st_t][-1] - b_t[st_t][0]))
    r_roll = float(rolling_radius_m(float(np.rad2deg(th_t[st_t].mean()))))
    pred = sweep * r_roll
    meas = path_len / (span / stride_s)

    cm = (ct >= t[0]) & (ct <= t[-1])
    air = 100.0 * float((~cv[cm].any(axis=1)).mean())
    # Contact fragmentation: rising edges per leg per stride. A clean stance is
    # one edge per leg per stride; more than that is the foot bouncing, and a
    # bouncing foot cannot maintain rolling contact.
    dn = contact_at(ct[cm], ct, cv)
    edges = sum(int(np.count_nonzero(np.diff(dn[:, i].astype(int)) > 0))
                for i in range(4)) / 4.0
    frag = edges / (span / stride_s)

    mmg = (mt >= t[0]) & (mt <= t[-1])
    th_max = float(mv[mmg][:, :, 0].max())

    return dict(v=v, r_rate=abs(v / psi_dot) if psi_dot else np.nan,
                deliv=100 * abs(psi_dot / CMD_RATE),
                noslip=100 * meas / pred, air=air, frag=frag, th=th_max)


def main():
    rows = []
    for arg in sys.argv[1:]:
        tag, path = arg.split("=", 1)
        r = one(path)
        r["tag"] = tag
        rows.append(r)

    print(f"{'run':>9} {'v':>7} {'R_rate':>8} {'delivered':>10} "
          f"{'of no-slip':>11} {'flight':>8} {'contacts/leg':>13} {'theta':>8}")
    for r in rows:
        print(f"{r['tag']:>9} {r['v']:7.3f} {r['r_rate']:8.3f} "
              f"{r['deliv']:9.1f}% {r['noslip']:10.1f}% {r['air']:7.1f}% "
              f"{r['frag']:13.2f} {r['th']:8.2f}")

    if len(rows) >= 3:
        ns = np.array([r["noslip"] for r in rows])
        rr = np.array([r["r_rate"] for r in rows])
        dv = np.array([r["deliv"] for r in rows])
        fr = np.array([r["frag"] for r in rows])
        print()
        print(f"  n = {len(rows)}")
        print(f"  corr(no-slip %, R_rate)      {np.corrcoef(ns, rr)[0,1]:+.3f}")
        print(f"  corr(no-slip %, delivered %) {np.corrcoef(ns, dv)[0,1]:+.3f}")
        print(f"  corr(contacts/leg, no-slip)  {np.corrcoef(fr, ns)[0,1]:+.3f}")
        print()
        print("  With n = 4 a correlation is a hint, not a result. What would")
        print("  make it one: the SIGN being consistent and the effect being")
        print("  large enough to matter -- a run that slips more should turn")
        print("  WIDER (bigger R) and deliver LESS of the commanded rate.")


if __name__ == "__main__":
    main()
