"""Body speed, path straightness and yaw rate from a campaign odom CSV.

Nothing else read `odom_runN.csv` for speed -- check_turn.py wants a
check_ramp npz, and check_menger.py only wants the xy path. This closes
that gap for the k_flight campaign (log S103), over the SAME 20 s tail
`tau_demand_window.py` uses, so speed and torque describe one window.
Both clocks are sim time and the odom capture starts ~7 s in, so the
tail sits inside odom coverage -- verified on banked captures.

`load_odom_csv` is IMPORTED from check_menger rather than re-implemented:
one parser for one file format, the discipline aggregate_menger.py
follows. That parser is also the reason quaternion columns are not
guessed here.

Reported:
    v_path        path length / span   -- how fast the body actually moves
    v_chord       |end - start| / span -- how fast it gets ANYWHERE
    straightness  chord / path_len     -- 1.0 is a straight line; S64's
                  honest cell measured ~0.99
    yaw_rate      least-squares slope over the window, with the endpoint
                  difference printed beside it. check_turn.py prints both
                  for a recorded reason: on a meandering path the two
                  disagree, and the disagreement IS the signal.

Note S64's speed caveat: "speed has two meanings" -- v~ indexes touchdown
speed magnitude, these are mean forward. Do not compare across them.

Usage:
    speed_from_odom.py odom_run1.csv [odom_run2.csv ...]
    speed_from_odom.py --dir ~/corgi_runs/kflight/k2500/lam0_default
    speed_from_odom.py --selftest
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_menger import load_odom_csv  # noqa: E402

TAIL_S = 20.0
# Path-length resampling interval, seconds. Set from the S64 calibration
# below (--calibrate); 0 disables and recovers raw per-sample summation.
DECIMATE_S = 1.0


class Unfit(Exception):
    pass


def yaw_from_quat(q):
    """-> yaw (rad) from (x, y, z, w) columns, same form as check_menger."""
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def stats(path, tail_s=TAIL_S, decimate_s=DECIMATE_S):
    t, xy, quat = load_odom_csv(path)
    if len(t) < 200:
        raise Unfit(f"only {len(t)} odom samples -- capture likely died "
                    f"(the daemon-wedge failure mode); re-run, do not score")
    m = t >= (t.max() - tail_s)
    if int(m.sum()) < 100:
        raise Unfit(f"only {int(m.sum())} samples in the last {tail_s:.0f}s")
    t, xy, quat = t[m], xy[m], quat[m]
    span = float(t[-1] - t[0])
    if span < 1.0:
        raise Unfit(f"window spans only {span:.2f}s")

    # Path length is decimated to `decimate_s` before summing. Raw
    # per-sample summation on a HOPPING robot counts intra-stride surge
    # and sway as travel and inflates path length ~25%, which reads as a
    # meander that is not there: on S64's honest cell raw summation gives
    # straightness 0.78-0.80 where S64 reports ~0.99. Resampling below
    # stride frequency (stride ~0.25 s) measures where the body actually
    # went. v_path uses the same decimated length so speed and
    # straightness stay consistent with each other.
    if decimate_s > 0.0:
        edges = np.arange(t[0], t[-1] + decimate_s, decimate_s)
        idx = np.unique(np.searchsorted(t, edges).clip(0, len(t) - 1))
        pxy = xy[idx]
    else:
        pxy = xy
    step = np.hypot(np.diff(pxy[:, 0]), np.diff(pxy[:, 1]))
    path_len = float(step.sum())
    chord = float(np.hypot(xy[-1, 0] - xy[0, 0], xy[-1, 1] - xy[0, 1]))
    yaw = np.unwrap(yaw_from_quat(quat))
    slope = float(np.polyfit(t, yaw, 1)[0])
    endpoint = float((yaw[-1] - yaw[0]) / span)
    # SIGNED forward speed. v_chord is |end - start| and cannot tell
    # forward from backward -- which matters, because a gait can roll its
    # feet backward and still show a healthy-looking chord speed
    # (2026-08-20: the +65 ms cell rolls backward in every stance bin and
    # the unsigned metric said nothing).
    #
    # INTEGRATED along the path, not projected on the initial heading:
    # this plant yaws up to -6 deg/s, i.e. >100 deg inside a 20 s window,
    # so an initial-heading projection reports a turning robot as going
    # sideways or backwards. Summing each step against the heading HELD AT
    # THAT STEP measures progress in the body frame, which is what
    # "forward" means for a machine that curves.
    hx, hy = np.cos(yaw[:-1]), np.sin(yaw[:-1])
    dx, dy = np.diff(xy[:, 0]), np.diff(xy[:, 1])
    v_fwd = float(np.sum(dx * hx + dy * hy)) / span
    # kept for comparison: the naive version, valid only when yaw is small
    v_fwd_naive = float((xy[-1] - xy[0]) @
                        np.array([np.cos(yaw[0]), np.sin(yaw[0])])) / span
    return {
        "v_path": path_len / span,
        "v_chord": chord / span,
        "v_fwd": v_fwd,
        "v_fwd_naive": v_fwd_naive,
        "straightness": chord / path_len if path_len > 1e-6 else float("nan"),
        "yaw_fit_deg_s": slope * 180.0 / np.pi,
        "yaw_end_deg_s": endpoint * 180.0 / np.pi,
        "span": span,
        "n": int(len(t)),
    }


def report(path, s):
    print(f"{'/'.join(path.split('/')[-2:]):30} "
          f"v_fwd {s['v_fwd']:+6.3f} (naive {s['v_fwd_naive']:+6.3f})  "
          f"v_path {s['v_path']:5.3f}  "
          f"straight {s['straightness']:5.3f}  "
          f"yaw {s['yaw_fit_deg_s']:+6.2f} deg/s "
          f"(endpoints {s['yaw_end_deg_s']:+6.2f})  "
          f"[{s['span']:.1f}s, n {s['n']}]")


def calibrate():
    """Straightness vs decimation on S64's cell, which the log calls a
    straight line at ~0.99. Pick the interval from this, then FREEZE it --
    do not tune it per campaign."""
    base = os.path.expanduser(
        "~/corgi_runs/shift_duty_sweep/v070_cor_soft_odo")
    if not os.path.isdir(base):
        print(f"CALIBRATE SKIP: {base} not present")
        return 0
    print(f"{'decimate_s':>11}  {'straightness (run1/2/3)':>28}  v_path r1")
    for d in (0.0, 0.10, 0.25, 0.50, 1.00, 2.00, 4.00):
        st, vp = [], None
        for n in (1, 2, 3):
            p = os.path.join(base, f"odom_run{n}.csv")
            if not os.path.exists(p) or os.path.getsize(p) < 10000:
                continue
            try:
                s = stats(p, decimate_s=d)
            except Unfit:
                continue
            st.append(s["straightness"])
            if vp is None:
                vp = s["v_path"]
        if st:
            print(f"{d:11.2f}  " + "  ".join(f"{v:.3f}" for v in st)
                  + f"        {vp:.3f}")
    print("\nS64 reports chord ~ path (0.99) and v_chord 0.153/0.148/0.156.")
    return 0


def selftest():
    """Reproduce log S64's soft-gain cell: chord ~0.15, straightness ~0.99."""
    base = os.path.expanduser(
        "~/corgi_runs/shift_duty_sweep/v070_cor_soft_odo")
    if not os.path.isdir(base):
        print(f"SELFTEST SKIP: {base} not present")
        return 0
    ok, seen = True, 0
    for n in (1, 2, 3):
        p = os.path.join(base, f"odom_run{n}.csv")
        if not os.path.exists(p) or os.path.getsize(p) < 10000:
            print(f"SELFTEST SKIP run{n}: absent or empty")
            continue
        try:
            s = stats(p)
        except Unfit as e:
            print(f"SELFTEST FAIL run{n}: {e}")
            ok = False
            continue
        seen += 1
        # S64: chord 0.153/0.148/0.156 m/s, "chord ~ path (0.99)".
        # Straightness bound is set from --calibrate: the statistic
        # plateaus at 0.978 for every decimation >= one stride (0.25 s),
        # so this asserts the plateau, not a tuned threshold.
        if not (0.12 <= s["v_chord"] <= 0.19
                and 0.95 <= s["straightness"] <= 1.001):
            print(f"SELFTEST FAIL run{n}: v_chord {s['v_chord']:.3f} "
                  f"(want 0.12-0.19), straightness {s['straightness']:.3f} "
                  f"(want 0.95-1.00)")
            ok = False
        else:
            print(f"selftest run{n}: v_chord {s['v_chord']:.3f}, "
                  f"straightness {s['straightness']:.3f}, "
                  f"yaw {s['yaw_fit_deg_s']:+.2f} deg/s  OK")
    if seen == 0:
        print("SELFTEST SKIP: no usable odom captures")
        return 0
    print("SELFTEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="*")
    ap.add_argument("--dir")
    ap.add_argument("--tail", type=float, default=TAIL_S)
    ap.add_argument("--decimate", type=float, default=DECIMATE_S,
                    help="path-length resampling interval, s (0 = raw)")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--calibrate", action="store_true",
                    help="straightness vs decimation on S64's cell")
    args = ap.parse_args()
    if args.calibrate:
        sys.exit(calibrate())
    if args.selftest:
        sys.exit(selftest())

    paths = list(args.runs)
    if args.dir:
        paths += sorted(glob.glob(os.path.join(
            os.path.expanduser(args.dir), "odom_run[0-9].csv")))
    if not paths:
        ap.error("give odom CSVs, --dir, or --selftest")

    vals = []
    for p in paths:
        try:
            s = stats(p, args.tail)
        except (Unfit, SystemExit) as e:
            print(f"{p}: REFUSED -- {e}")
            continue
        report(p, s)
        vals.append(s)
    if len(vals) > 1:
        vc = np.array([v["v_chord"] for v in vals])
        vf = np.array([v["v_fwd"] for v in vals])
        st = np.array([v["straightness"] for v in vals])
        print(f"{'CELL (n=' + str(len(vc)) + ')':30} "
              f"v_fwd median {np.median(vf):+6.3f} "
              f"[{vf.min():+.3f}, {vf.max():+.3f}]   "
              f"v_chord median {np.median(vc):5.3f}   "
              f"straightness median {np.median(st):5.3f}")


if __name__ == "__main__":
    main()
