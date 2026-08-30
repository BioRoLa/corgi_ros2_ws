#!/usr/bin/env python3
"""Does any excursion metric separate the COLLAPSED yaw runs from the healthy?
Log S195.

WHY. S192 falsified saturated fraction as the predictor of locking: `clamp`
pins 0.788 of the band and collapses 1-in-5; `gentle` pins 0.746 and collapses
0-in-5. Four points apart, opposite outcomes. What differed instead was the
PEAK excursion -- |F/R| max 2.375 vs 1.899 deg against a 1.003 deg clamp -- so
the candidate hypothesis is that locking is driven by how hard and how fast the
loop swings, not by how long it sits pinned.

⚠⚠ WHAT THIS CAN AND CANNOT DO. There are exactly TWO collapsed runs on record
(yaw_hold_n5/clamp run3, yaw_gentle/clamp run2). Two is not a sample. This
CANNOT establish a discriminator and does not try to.

What it CAN do is KILL candidates: a metric that fails to put both collapsed
runs at the extreme of the distribution is dead, because a real predictor must
at minimum order them correctly. Surviving that screen is necessary and nowhere
near sufficient -- it earns a candidate the right to be tested at n where a
test means something, and nothing more.

This is the S178/S187 failure mode stated in advance: a mechanism inferred from
n = 1 (or 2) and then built into a bar is how five bars failed together.

OUTCOME (S195): every candidate is DEAD, including the one that looked alive.
slew_p99, signed yaw_rate and straightness all separate the two collapsed runs
cleanly over the whole band -- and all three are disqualified by --onset for
the same structural reason: the odom capture starts ~7 s into the run, and BOTH
collapsed runs already read v_fwd +0.081 / -0.001 in the FIRST 4 s window. They
are collapsed before the recording begins. Anything measured over that window
separates them by construction; none of it predicts anything.

The two positive things this leaves:
  * the failure is a PIROUETTE, not a stall -- chord 0.60-0.84 m over 24 s with
    yaw +51/+52 deg, against -37/-45 deg for healthy runs. A sign flip against
    the plant's own drift, not a magnitude.
  * to get a mechanism, the capture must start at t = 0. The lock is
    established in the first ~7 s, which no banked run covers.

Usage:
    yaw_excursion.py --selftest
    yaw_excursion.py --dir <cell> --label <name> [--dir ...]
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_menger import LEGS, load_odom_csv, load_torque_csv  # noqa: E402
from speed_from_odom import yaw_from_quat                     # noqa: E402
from yaw_saturation import FR_SIGN, saturated_fraction        # noqa: E402
import speed_from_odom as sfo                                 # noqa: E402

START, END = 12.0, 30.0
COLLAPSE_VFWD = 0.10       # a run below this is the absorbing failure (S191)


def metrics(tq, od, limit_deg):
    """-> dict of excursion metrics over the S88-matched band."""
    t, _c, gamma, _th = load_torque_csv(tq)
    if od and os.path.exists(od):
        ot, xy, _q = load_odom_csv(od)
        t0, t1 = max(ot[0], t[0]), min(ot[-1], t[-1])
    else:
        return None
    a0, a1 = t0 + START, min(t1, t0 + END)
    m = (t >= a0) & (t <= a1)
    if m.sum() < 500:
        return None
    tb = t[m]
    fr = sum(FR_SIGN[l] * gamma[m][:, i] for i, l in enumerate(LEGS)) / 4.0

    dt = float(np.median(np.diff(tb)))
    span = float(tb[-1] - tb[0])
    # sign crossings of the F/R command, per second
    sgn = np.sign(fr)
    sgn[sgn == 0] = 1
    crossings = int(np.sum(np.abs(np.diff(sgn)) > 1))
    # how hard it slews
    dfr = np.abs(np.diff(fr)) / dt

    # Speed, to label the run. Use speed_from_odom's OWN v_fwd: it is
    # heading-projected and decimates the path to 1 s before summing, because
    # raw per-sample summation inflates path length ~25% on a hopping robot
    # (that tool's docstring). A first version of this script summed raw
    # displacement and labelled BOTH collapsed runs healthy -- v_path 0.25
    # where speed_from_odom reads v_fwd +0.013. A robot spinning in place has
    # plenty of path and no progress.
    # Catch ONLY the unfit-capture case. A bare `except Exception` here hid a
    # KeyError (yaw_rate vs yaw_fit_deg_s) behind an empty results table --
    # every run silently returned None and the screen reported "no collapsed
    # runs". A swallowed exception that produces a clean-looking null is worse
    # than a crash.
    try:
        st = sfo.stats(od)
    except sfo.Unfit:
        return None
    v_fwd = float(st["v_fwd"])
    yaw_rate = float(st["yaw_fit_deg_s"])     # already deg/s
    straight = float(st["straightness"])

    return {
        "fr_max": float(np.max(np.abs(fr))),
        "fr_p99": float(np.percentile(np.abs(fr), 99)),
        "fr_med": float(np.median(np.abs(fr))),
        "over_clamp": float(np.max(np.abs(fr))) / limit_deg if limit_deg else np.nan,
        "cross_hz": crossings / span if span > 0 else 0.0,
        "slew_p99": float(np.percentile(dfr, 99)),
        "sat": saturated_fraction(fr, limit_deg),
        "v_fwd": v_fwd,
        "yaw_rate": yaw_rate,
        "yaw_abs": abs(yaw_rate),
        "straight": straight,
    }


# ---------------------------------------------------------------- onset ----
# Is slew_p99 a LEADING INDICATOR of the collapse, or the collapse's own
# signature? It is measured over [12, 30] s -- the same window the collapse
# happens in. A robot that has already locked up is thrashing, and thrashing
# slews. So the whole-band separation (51.7/55.3 vs 30.4-36.9) is consistent
# with slew CAUSING the lock AND with slew being what a lock LOOKS like, and
# those have opposite consequences: a leading indicator can gate a config, a
# post-hoc signature cannot.
#
# The discriminator is time. Split the band into 6 s sub-windows, locate the
# first window where forward progress dies (onset), and ask whether slew was
# already extreme BEFORE it. Same v_fwd convention as speed_from_odom.stats --
# heading integrated step by step, because this plant yaws >100 deg per window.
# Scanned from the START of odom coverage, NOT from START=12. The first
# attempt windowed [12, 30] -- the S88 settling band -- and found both
# collapsed runs already at v_fwd +0.047 / -0.019 in the FIRST window. The
# onset is earlier than the band, so a band-restricted scan cannot see it and
# every "separated" verdict it printed was scored after the fact.
#
# Coverage is also shorter than END=30 implies: the odom capture starts ~7 s
# in and spans ~24 s, so the scored band is really [t0+12, t0+24], ~12 s. That
# is uniform across all 25 runs (11.83-12.19 s), so the whole-band comparison
# is sound -- but it leaves only two 6 s windows, hence 4 s here.
WIN_S = 4.0


def _v_fwd_window(ot, xy, quat, lo, hi):
    m = (ot >= lo) & (ot <= hi)
    if int(m.sum()) < 50:
        return np.nan
    t, p, q = ot[m], xy[m], quat[m]
    span = float(t[-1] - t[0])
    if span < 1.0:
        return np.nan
    yaw = np.unwrap(yaw_from_quat(q))
    dx, dy = np.diff(p[:, 0]), np.diff(p[:, 1])
    return float(np.sum(dx * np.cos(yaw[:-1]) + dy * np.sin(yaw[:-1]))) / span


def windows(tq, od):
    """-> [(offset_s, v_fwd, slew_p99)] over the whole torque/odom overlap.

    Offsets are k*WIN_S EXACTLY, not lo - t0: subtracting two large sim-time
    floats leaves noise that scattered a shared grid into near-duplicate keys
    (a header reading t+11, t+12, t+12).
    """
    t, _c, gamma, _th = load_torque_csv(tq)
    ot, xy, quat = load_odom_csv(od)
    t0, t1 = max(ot[0], t[0]), min(ot[-1], t[-1])
    out, k = [], 0
    while t0 + (k + 1) * WIN_S <= t1 + 1e-9:
        lo, hi = t0 + k * WIN_S, t0 + (k + 1) * WIN_S
        m = (t >= lo) & (t <= hi)
        if m.sum() >= 200:
            tb = t[m]
            fr = sum(FR_SIGN[l] * gamma[m][:, i] for i, l in enumerate(LEGS)) / 4.0
            dt = float(np.median(np.diff(tb)))
            slew = float(np.percentile(np.abs(np.diff(fr)) / dt, 99))
            out.append((k * WIN_S, _v_fwd_window(ot, xy, quat, lo, hi), slew))
        k += 1
    return out


def selftest():
    ok = True

    def chk(name, good):
        nonlocal ok
        ok = ok and good
        print(f"  {'ok ' if good else 'FAIL'} {name}")

    # a pure L/R pattern must project to zero on F/R (the yaw partition)
    lr = {"A": +10.0, "B": -10.0, "C": -10.0, "D": +10.0}
    chk("pure L/R -> F/R is zero",
        abs(sum(FR_SIGN[l] * lr[l] for l in LEGS) / 4.0) < 1e-12)
    # crossing count on a known square wave
    fr = np.tile(np.concatenate([np.ones(50), -np.ones(50)]), 10)
    sgn = np.sign(fr)
    n = int(np.sum(np.abs(np.diff(sgn)) > 1))
    chk(f"20 half-cycles -> 19 crossings (got {n})", n == 19)
    chk("saturation of a pinned signal is 1.0",
        saturated_fraction(np.full(100, 1.003), 1.003) == 1.0)

    # _v_fwd_window on a CIRCLE. Heading tangent, radius R, rate w: the true
    # forward speed is R*w everywhere, while a projection on the INITIAL
    # heading averages to nearly nothing over a half turn. This plant yaws
    # >100 deg inside a window, so the integrated convention is not a
    # refinement -- it is the difference between "turning" and "going
    # sideways". Same reason speed_from_odom keeps v_fwd_naive only "for
    # comparison".
    R, w = 2.0, 0.30
    tt = np.linspace(0.0, 10.0, 4000)
    xy = np.column_stack([R * np.sin(w * tt), R * (1.0 - np.cos(w * tt))])
    yaw = w * tt
    q = np.column_stack([np.zeros_like(tt), np.zeros_like(tt),
                         np.sin(yaw / 2.0), np.cos(yaw / 2.0)])
    got = _v_fwd_window(tt, xy, q, 0.0, 10.0)
    chk("circle: v_fwd = R*w = %.3f (got %.4f)" % (R * w, got),
        abs(got - R * w) < 1e-3)
    naive = float((xy[-1] - xy[0]) @
                  np.array([np.cos(yaw[0]), np.sin(yaw[0])])) / 10.0
    chk("...and the naive projection does NOT (%.3f)" % naive,
        abs(naive - R * w) > 0.1)
    return ok


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", action="append", default=[])
    ap.add_argument("--label", action="append", default=[])
    ap.add_argument("--limit-deg", action="append", default=[])
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--onset", action="store_true",
                    help="per-6s-window v_fwd and slew: leading indicator or signature?")
    a = ap.parse_args()
    if a.selftest:
        print("yaw_excursion.py selftest\n")
        good = selftest()
        print(f"\n  SELFTEST {'PASS' if good else 'FAIL'}")
        return 0 if good else 1
    if not a.dir:
        ap.error("need --dir, or --selftest")

    print("excursion metrics vs the absorbing failure. TWO collapsed runs exist,")
    print("so this can only KILL candidates that mis-order them, never confirm.\n")
    print(f"  {'cell/run':22} {'v_fwd':>7} {'state':>9} {'|F/R|max':>9} "
          f"{'/clamp':>7} {'cross Hz':>9} {'slew p99':>9} {'sat':>6}")
    rows = []
    for i, d in enumerate(a.dir):
        d = os.path.expanduser(d)
        name = a.label[i] if i < len(a.label) else os.path.basename(d.rstrip("/"))
        lim = float(a.limit_deg[i]) if i < len(a.limit_deg) else 1.003
        for tq in sorted(glob.glob(os.path.join(d, "run[0-9].csv"))):
            od = os.path.join(d, "odom_" + os.path.basename(tq))
            try:
                m = metrics(tq, od, lim)
            except (SystemExit, Exception):        # noqa: BLE001
                m = None
            if m is None:
                continue
            state = "COLLAPSED" if m["v_fwd"] < COLLAPSE_VFWD else "healthy"
            tag = f"{name}/{os.path.basename(tq)[:-4]}"
            rows.append((tag, state, m))
            print(f"  {tag:22} {m['v_fwd']:7.3f} {state:>9} {m['fr_max']:9.3f} "
                  f"{m['over_clamp']:7.2f} {m['cross_hz']:9.3f} "
                  f"{m['slew_p99']:9.2f} {m['sat']:6.3f} "
                  f"{m['yaw_rate']:8.2f} {m['straight']:8.3f}")

    bad = [r for r in rows if r[1] == "COLLAPSED"]
    good = [r for r in rows if r[1] == "healthy"]
    if not bad:
        print("\nno collapsed runs in this set -- nothing to separate.")
        return 0
    print(f"\n-- the screen: do BOTH collapsed runs sit at an extreme? "
          f"({len(bad)} collapsed, {len(good)} healthy) --")
    for key, hi_is_bad in (("fr_max", True), ("over_clamp", True),
                           ("cross_hz", True), ("slew_p99", True),
                           ("sat", True), ("fr_med", True),
                           ("yaw_abs", True), ("yaw_rate", True),
                           ("straight", False)):
        gv = [r[2][key] for r in good]
        bv = [r[2][key] for r in bad]
        if hi_is_bad:
            worst_good = max(gv)
            sep = all(v > worst_good for v in bv)
            direction = "above every healthy run"
        else:
            worst_good = min(gv)
            sep = all(v < worst_good for v in bv)
            direction = "below every healthy run"
        rank = [sorted(gv + bv, reverse=hi_is_bad).index(v) + 1 for v in bv]
        print(f"  {key:11} collapsed {['%.3f' % v for v in bv]}  "
              f"healthy range [{min(gv):.3f}, {max(gv):.3f}]  "
              f"ranks {rank}/{len(rows)}  -> "
              f"{'SURVIVES (' + direction + ')' if sep else 'KILLED -- does not order them'}")
    print("\n⚠ Surviving this screen is NECESSARY, not sufficient. n(collapsed)"
          f" = {len(bad)}.")

    if a.onset:
        print()
        print("=== is a surviving metric LEADING, or the collapse's own "
              "signature? ===")
        print("   6 s sub-windows. If slew is only extreme AFTER forward "
              "progress dies, it")
        print("   is a symptom, and a symptom cannot gate a config.")
        print()
        wins = {}
        for i, d in enumerate(a.dir):
            d = os.path.expanduser(d)
            name = a.label[i] if i < len(a.label) else os.path.basename(d.rstrip("/"))
            for tq in sorted(glob.glob(os.path.join(d, "run[0-9].csv"))):
                od = os.path.join(d, "odom_" + os.path.basename(tq))
                tag = name + "/" + os.path.basename(tq)[:-4]
                if not os.path.exists(od) or not any(r[0] == tag for r in rows):
                    continue
                try:
                    wins[tag] = windows(tq, od)
                except Exception:                          # noqa: BLE001
                    continue
        hdr = sorted({w[0] for v in wins.values() for w in v})
        if hdr:
            print("  %-22s %9s " % ("cell/run", "state") +
                  " ".join("%16s" % ("t+%d" % h) for h in hdr))
            print("  %-22s %9s " % ("", "") +
                  " ".join("%16s" % "v_fwd    slew" for _ in hdr))
            for tag, state, _m in rows:
                if tag not in wins:
                    continue
                cells = []
                for h in hdr:
                    hit = [w for w in wins[tag] if abs(w[0] - h) < 1e-6]
                    cells.append("%7.3f %8.2f" % (hit[0][1], hit[0][2])
                                 if hit else "%16s" % "--")
                print("  %-22s %9s " % (tag, state) + " ".join(cells))
            print()
            print("  -- the leading-indicator test, window by window --")
            for h in hdr:
                gv = [w[2] for t_, s_, _ in rows if s_ == "healthy"
                      for w in wins.get(t_, []) if abs(w[0] - h) < 1e-6]
                bv = [(w[1], w[2]) for t_, s_, _ in rows if s_ == "COLLAPSED"
                      for w in wins.get(t_, []) if abs(w[0] - h) < 1e-6]
                if not gv or not bv:
                    continue
                sep = all(v[1] > max(gv) for v in bv)
                print("    t+%-3d collapsed slew %s (their v_fwd %s)  "
                      "healthy max %.1f  -> %s"
                      % (h, [round(v[1], 1) for v in bv],
                         ["%+.3f" % v[0] for v in bv], max(gv),
                         "separated" if sep else "NOT separated"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
