"""Head-to-head comparison of runs grouped by filename prefix.

Written for the template-speed experiment (v120 / v070 / v045), but takes any
set of prefixes, so it works for any "same conditions, one thing changed"
campaign.

Reports the quantities that decide whether a template suits the robot: achieved
forward speed, flight fraction against the template's OWN design duty, theta
overshoot, torque saturation, and slip against the template's own commanded
sweep. Every one of those is per-template, so the comparison is fair even
though the three templates have different periods and different beta sweeps.

    python3 compare_templates.py <dir> v120 v070 v045
"""
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_turn import rolling_radius_m  # noqa: E402
from check_yaw_phase import dedupe_time  # noqa: E402
from ramp_segments import load as load_template  # noqa: E402

START, END = 4.0, 9.5


def one(path):
    d = np.load(path, allow_pickle=True)
    ot, ov = dedupe_time(d["odom_t"], d["odom"])
    ct, cv, mt, mv = d["contact_t"], d["contact"], d["motor_t"], d["motor_deg"]
    m = (ot >= START) & (ot <= END)
    if int(m.sum()) < 50:
        return None
    t, seg = ot[m], ov[m]
    span = t[-1] - t[0]
    path_len = float(np.hypot(np.diff(seg[:, 0]), np.diff(seg[:, 1])).sum())
    v = path_len / span

    # Each template has its own period, duty and commanded sweep -- compare
    # each run against ITS OWN design, never against v~1.20's.
    tt, th_t, b_t, _, st_t = load_template(str(d["template"]))
    stride_s = float(tt[-1])
    design_air = 100.0 * float(1.0 - st_t.mean())
    sweep = abs(float(b_t[st_t][-1] - b_t[st_t][0]))
    r_roll = float(rolling_radius_m(float(np.rad2deg(th_t[st_t].mean()))))
    noslip = 100.0 * (path_len / (span / stride_s)) / (sweep * r_roll)

    cm = (ct >= t[0]) & (ct <= t[-1])
    air = 100.0 * float((~cv[cm].any(axis=1)).mean())
    mmg = (mt >= t[0]) & (mt <= t[-1])
    th_max = float(mv[mmg][:, :, 0].max())

    sat = strides_at_clamp = float("nan")
    if "torque_nm" in d.files and len(d["torque_nm"]):
        qt, q = d["torque_t"], np.abs(d["torque_nm"])
        qm = (qt >= START) & (qt <= END)
        leg = q[qm][:, :, :2]
        sat = 100.0 * float((leg >= 34.9).mean())
        tq, ql = qt[qm], leg.reshape(int(qm.sum()), -1)
        peaks = [ql[(tq >= a) & (tq < a + stride_s)].max()
                 for a in np.arange(tq[0], tq[-1] - stride_s, stride_s)]
        strides_at_clamp = 100.0 * float(np.mean(np.array(peaks) >= 34.9))

    return dict(v=v, air=air, design=design_air, th=th_max, sat=sat,
                clamped=strides_at_clamp, noslip=noslip,
                stride=stride_s, sweep=np.rad2deg(sweep))


def main():
    root = sys.argv[1]
    print(f"window {START}-{END} s   (each run vs ITS OWN template's design)")
    print()
    print(f"{'group':>7} {'n':>2} {'stride':>7} {'sweep':>7} {'v m/s':>13} "
          f"{'flight%':>13} {'design':>7} {'theta max':>13} "
          f"{'sat%':>11} {'strides@clamp':>14} {'of no-slip':>13}")
    for pre in sys.argv[2:]:
        rows = [one(p) for p in
                sorted(glob.glob(os.path.join(root, f"{pre}_*.npz")))]
        rows = [r for r in rows if r]
        if not rows:
            print(f"{pre:>7}  no runs")
            continue

        def ms(k, f="{:6.2f}"):
            a = np.array([r[k] for r in rows], dtype=float)
            a = a[np.isfinite(a)]
            if not len(a):
                return f"{'--':>12}"
            return f"{f.format(a.mean())}±{a.std():5.2f}"
        print(f"{pre:>7} {len(rows):2d} {rows[0]['stride']:7.4f} "
              f"{rows[0]['sweep']:6.1f}d {ms('v'):>13} {ms('air'):>13} "
              f"{rows[0]['design']:6.1f}% {ms('th'):>13} "
              f"{ms('sat'):>11} {ms('clamped'):>14} {ms('noslip'):>13}")
    print()
    print("  design      = 1 - duty of that template; the flight it asks for")
    print("  sat%        = leg-motor samples pegged at the 35 N.m clamp")
    print("  strides@clamp = % of strides whose PEAK touches the clamp")


if __name__ == "__main__":
    main()
