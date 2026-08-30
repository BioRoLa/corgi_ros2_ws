"""Where inside the stride does the yaw actually come from?

Offline only -- reads a .npz written by check_ramp.py --dump. No simulator time.

Why this exists. Every yaw measurement so far has been an average: per segment,
or at best per stride. That is enough to establish THAT the robot yaws at a
constant 2.3-3.6 deg/stride independent of speed and leg sweep, and it is what
killed the differential-rolling hypothesis. It cannot distinguish the mechanism
classes that remain, because they all produce the same per-stride average:

  (a) an impulse at touchdown          -- rate steps up at each contact edge
  (b) a torque distributed over stance -- rate ramps while feet are down
  (c) a single initial kick, then no yaw damping at all
                                       -- rate is FLAT from the first stride,
                                          and stance neither adds nor removes it

(c) has never been tested and is the one the log lists as untried ("an
initial-condition bias from the support-box removal that then persists"). It
also explains, for free, the fact that broke every other hypothesis: a body
already spinning does not care about speed or sweep, so a constant rate across
a 6x speed range and from beta = 0 to 18 deg is exactly what it predicts.

The discriminator is the yaw RATE, not the yaw. During flight the robot is
ballistic, so the rate is whatever contact last left it with; stance is the only
place it can change. So:

  rate flat across the whole run, stance == flight  ->  (c)
  rate rises in steps at contact edges              ->  (a)
  rate ramps through stance, flat in flight         ->  (b)

Reads the HOP segment by default: beta is commanded exactly zero there, the
robot bounces in place, and the log establishes it exhibits the full effect.
Eight strides, no forward motion, no template complexity.

Usage:
    python3 check_yaw_phase.py <dump.npz> [--template <csv>] [--segment hop]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ramp_segments import segment_template  # noqa: E402

DEFAULT_TEMPLATE = ("/home/alexc/corgi_ws/corgi_ros2_ws/src/corgi_force_control"
                    "/config/gslip_speed_ramp_template.csv")


def yaw_from_quat(q):
    """-> yaw in radians, unwrapped. q is (n,4) as (x, y, z, w)."""
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.unwrap(np.arctan2(2.0 * (w * z + x * y),
                                1.0 - 2.0 * (y * y + z * z)))


def dedupe_time(t, *arrays):
    """Drop samples sharing a timestamp with their predecessor.

    At CORGI_CONTACT_INTERVAL=1 the odom stream carries two samples in one clock
    tick, which makes any dt-based differentiation divide by zero and return NaN
    everywhere. That silently produced "0 usable strides" twice before it was
    noticed, so it is handled here rather than left to the caller.
    """
    keep = np.concatenate(([True], np.diff(t) > 0))
    return (t[keep],) + tuple(a[keep] for a in arrays)


def contact_at(sample_t, contact_t, contact):
    """Nearest-previous contact state resampled onto sample_t. -> (n,4) bool."""
    idx = np.searchsorted(contact_t, sample_t, side="right") - 1
    idx = np.clip(idx, 0, len(contact_t) - 1)
    return contact[idx]


def smooth(x, t, window_s):
    """Centred moving average over a fixed time window.

    Yaw is differentiated to get a rate, and the raw odom yaw carries the body's
    own oscillation at stride frequency. Without smoothing the per-sample rate is
    dominated by that and the trend is invisible. The window is a fraction of a
    stride, not a whole one, so it cannot smear a touchdown step into a ramp --
    which is the exact distinction this script exists to make.
    """
    if window_s <= 0:
        return x
    dt = np.median(np.diff(t))
    n = max(1, int(round(window_s / dt)))
    if n % 2 == 0:
        n += 1
    if n <= 1 or n >= len(x):
        return x
    kern = np.ones(n) / n
    pad = n // 2
    xp = np.concatenate((np.full(pad, x[0]), x, np.full(pad, x[-1])))
    return np.convolve(xp, kern, mode="valid")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump")
    ap.add_argument("--template", default=DEFAULT_TEMPLATE)
    ap.add_argument("--segment", default="hop",
                    help="substring of the segment name, or 'all'")
    ap.add_argument("--smooth", type=float, default=0.02,
                    help="yaw smoothing window in seconds, before "
                         "differentiating. Keep well under a stride (~0.24 s).")
    args = ap.parse_args()

    d = np.load(args.dump, allow_pickle=True)
    ot, ov = d["odom_t"], d["odom"]
    ct, cv = d["contact_t"], d["contact"]
    if ov.shape[1] < 10:
        print("dump has no orientation columns -- needs a check_ramp.py that "
              "records odom quaternions (corgi_sim >= 77dcac1).")
        return

    ot, ov = dedupe_time(ot, ov)
    yaw = np.rad2deg(yaw_from_quat(ov[:, 6:10]))

    segs = segment_template(args.template)
    if args.segment == "all":
        window = (min(s["t_start"] for s in segs),
                  max(s["t_end"] for s in segs))
        label = "whole first pass"
    else:
        sel = [s for s in segs if args.segment.lower() in s["name"].lower()]
        if not sel:
            print(f"no segment matching {args.segment!r}; have: "
                  + ", ".join(s["name"] for s in segs))
            return
        window = (sel[0]["t_start"], sel[-1]["t_end"])
        label = ", ".join(s["name"] for s in sel)

    # --- The settle window, before the gait starts ------------------------
    # gslip_pronk removes the support box on the trigger and then holds
    # template_.front() for settle_ticks before playing a single row. Template
    # time is anchored at the first stride, so everything at t < 0 is the robot
    # standing still on its own legs with the box gone. If it is already yawing
    # there, no part of the gait can be responsible.
    pre = ot < 0
    if int(pre.sum()) >= 20:
        pt, py = ot[pre], yaw[pre]
        pdown = contact_at(pt, ct, cv).all(axis=1)
        span = pt[-1] - pt[0]
        print(f"  BEFORE the first stride ({span:.2f} s of settle, "
              f"{int(pre.sum())} samples, all four feet down "
              f"{100.0*pdown.mean():.0f}% of it):")
        print(f"    yaw {py[-1]-py[0]:+7.2f} deg    "
              f"rate {(py[-1]-py[0])/span:+7.2f} deg/s")
        print("    The legs hold a fixed pose here and the box is already gone,")
        print("    so any yaw in this window is the plant, not the gait.")
        print()

    m = (ot >= window[0]) & (ot <= window[1])
    if int(m.sum()) < 50:
        print(f"only {int(m.sum())} odom samples in {label} -- nothing to say.")
        return
    t, y = ot[m], yaw[m]
    y = y - y[0]
    down = contact_at(t, ct, cv)
    any_down = down.any(axis=1)

    rate = np.gradient(smooth(y, t, args.smooth), t)

    print(f"{os.path.basename(args.dump)}: {label}  "
          f"t {window[0]:.2f}..{window[1]:.2f} s, {int(m.sum())} odom samples")
    print(f"  contact sample rate {len(ct)/max(ct.max()-ct.min(),1e-9):7.1f} Hz"
          f"   odom {len(ot)/max(ot.max()-ot.min(),1e-9):7.1f} Hz")
    print(f"  net yaw over the window: {y[-1]-y[0]:+8.2f} deg")
    print()

    # --- Where the yaw is accumulated: stance vs flight -------------------
    dy = np.diff(y)
    dt = np.diff(t)
    in_stance = any_down[:-1]
    for name, mask in (("stance (>=1 foot down)", in_stance),
                       ("flight (all feet up)", ~in_stance)):
        tt = dt[mask].sum()
        yy = dy[mask].sum()
        frac = 100.0 * tt / dt.sum()
        r = yy / tt if tt > 0 else float("nan")
        print(f"  {name:<24} {frac:5.1f}% of time   "
              f"dyaw {yy:+7.2f} deg   rate {r:+7.2f} deg/s")
    print()
    print("  If the yaw were applied by the ground, stance would carry nearly")
    print("  all of it and the flight rate would be a leftover constant. If the")
    print("  two rates are the SAME, the body is simply spinning and contact is")
    print("  neither adding nor removing yaw.")
    print()

    # --- Who CHANGES the rate: stance or flight? --------------------------
    # Equal mean rates in stance and flight is necessary for "nothing is
    # torquing it" but not sufficient: a weak stance torque shifts the rate a
    # little each stride and would still leave the two means close. The
    # decisive quantity is the CHANGE in rate across each interval. Flight is
    # ballistic, so its change is the measurement's own noise floor -- which is
    # what makes it the right control for the stance number.
    def rate_change(mask_value):
        """-> (n, mean delta-rate, mean duration) over runs of one contact state."""
        idx = np.flatnonzero(np.diff(any_down.astype(int)) != 0) + 1
        bounds = np.concatenate(([0], idx, [len(any_down)]))
        deltas, spans = [], []
        for a, b in zip(bounds[:-1], bounds[1:]):
            if b - a < 3 or bool(any_down[a]) != mask_value:
                continue
            deltas.append(rate[b - 1] - rate[a])
            spans.append(t[b - 1] - t[a])
        if not deltas:
            return 0, float("nan"), float("nan")
        return len(deltas), float(np.mean(deltas)), float(np.mean(spans))

    print("  change in yaw RATE across each interval (deg/s per interval):")
    for name, val in (("stance", True), ("flight", False)):
        n_i, mean_d, mean_s = rate_change(val)
        print(f"    {name:<8} n={n_i:3d}  mean {mean_d:+7.3f} deg/s  "
              f"over {mean_s:.3f} s  -> {mean_d/mean_s if mean_s else float('nan'):+8.2f} deg/s^2")
    print("    Flight is ballistic, so its number is the noise floor. A stance")
    print("    figure that is not clearly larger means the ground is not")
    print("    torquing the body in yaw.")
    print()

    # --- Is the rate flat, or does it build stride by stride? -------------
    edges = np.flatnonzero(any_down[1:] & ~any_down[:-1]) + 1
    if len(edges) >= 3:
        print("  per stride (touchdown to touchdown):")
        print(f"    {'#':>3} {'t':>7} {'dyaw':>8} {'rate':>9} "
              f"{'rate@TD':>9} {'stance dyaw':>12} {'flight dyaw':>12}")
        for k in range(len(edges) - 1):
            i0, i1 = edges[k], edges[k + 1]
            seg_dy, seg_dt = dy[i0:i1], dt[i0:i1]
            st = in_stance[i0:i1]
            span = seg_dt.sum()
            if span <= 0:
                continue
            print(f"    {k:3d} {t[i0]:7.3f} {y[i1]-y[i0]:+8.2f} "
                  f"{(y[i1]-y[i0])/span:+9.2f} {rate[i0]:+9.2f} "
                  f"{seg_dy[st].sum():+12.2f} {seg_dy[~st].sum():+12.2f}")
        print()
        print("    rate@TD is the instantaneous yaw rate at each touchdown. A")
        print("    column that climbs means every stride adds angular momentum;")
        print("    a flat column means the spin was there before the first one.")
        print()

        # --- Phase profile, averaged over strides -------------------------
        nb = 10
        acc = np.zeros(nb)
        occ = np.zeros(nb)
        stance_frac = np.zeros(nb)
        for k in range(len(edges) - 1):
            i0, i1 = edges[k], edges[k + 1]
            span = t[i1] - t[i0]
            if span <= 0:
                continue
            ph = (t[i0:i1] - t[i0]) / span
            b = np.clip((ph * nb).astype(int), 0, nb - 1)
            np.add.at(acc, b, dy[i0:i1])
            np.add.at(occ, b, dt[i0:i1])
            np.add.at(stance_frac, b, dt[i0:i1] * in_stance[i0:i1])
        good = occ > 0
        print("  yaw accumulated by phase of stride, summed over all strides")
        print("  (phase 0 = touchdown):")
        print(f"    {'phase':>10} {'dyaw/stride':>12} {'rate':>9} {'in stance':>10}")
        for b in range(nb):
            if not good[b]:
                continue
            print(f"    {b/nb:4.1f}-{(b+1)/nb:<4.1f} "
                  f"{acc[b]/(len(edges)-1):+12.3f} "
                  f"{acc[b]/occ[b]:+9.2f} "
                  f"{100*stance_frac[b]/occ[b]:9.0f}%")
        print()
        print("    A touchdown impulse concentrates in the first bin or two. A")
        print("    stance torque spreads across the bins that are in contact.")
        print("    A pre-existing spin is flat across every bin, in contact or")
        print("    not.")
    else:
        print(f"  only {len(edges)} touchdown edges found -- no per-stride view.")


if __name__ == "__main__":
    main()
