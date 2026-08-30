#!/usr/bin/env python3
"""How much of the time is the heading loop PINNED against its clamp? Log S191.

S187 diagnosed the `clamp` cell's 1-in-5 collapse: the clamp bounds S163's
saturation pathology from 5 deg to 1 deg but does NOT remove it. A run that
started slowly drove the loop to saturate the WRONG WAY and never recovered --
S163's own sentence still describes it: "a saturated steering command is not a
controller, it is a constant commanded lean".

So the property that predicts locking is not speed or straightness -- it is
WHETHER THE LOOP REACHES ITS CLAMP AT ALL. A loop that never saturates cannot
lock. This measures that directly.

    saturated fraction = P(|F/R camber projection| >= (1 - TOL) * gamma_yaw_limit)

over the analysis band, where F/R is the {+1,+1,-1,-1} projection -- the
partition gamma_correction's YAW term uses (gslip_pronk.cpp:2202,
yaw_sign = {+1,+1,-1,-1}), so it isolates the heading loop from the roll term,
which rides the {+1,-1,-1,+1} L/R partition instead.

Usage:
    yaw_saturation.py --selftest
    yaw_saturation.py --dir <cell> --limit-deg 1.003 [--dir <cell> ...]
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_menger import LEGS, load_odom_csv, load_torque_csv  # noqa: E402

FR_SIGN = {"A": +1.0, "B": +1.0, "C": -1.0, "D": -1.0}
TOL = 0.05          # within 5% of the clamp counts as pinned
START, END = 12.0, 30.0


def fr_series(torque_csv, odom_csv=None):
    """-> (t, F/R projection in deg) over the S88-matched band."""
    t, _contact, gamma, _theta = load_torque_csv(torque_csv)
    if odom_csv and os.path.exists(odom_csv):
        ot, _, _ = load_odom_csv(odom_csv)
        t0 = max(ot[0], t[0])
        t1 = min(ot[-1], t[-1])
    else:
        t0, t1 = t[0], t[-1]
    a0, a1 = t0 + START, min(t1, t0 + END)
    m = (t >= a0) & (t <= a1)
    fr = sum(FR_SIGN[l] * gamma[m][:, i] for i, l in enumerate(LEGS)) / 4.0
    return t[m], fr


def saturated_fraction(fr, limit_deg):
    """Fraction of samples pinned within TOL of the clamp, either sign."""
    if limit_deg <= 0:
        return float("nan")      # no clamp: saturation is undefined, not zero
    return float(np.mean(np.abs(fr) >= (1.0 - TOL) * limit_deg))


def selftest():
    ok = True

    def chk(name, got, want, tol=1e-9):
        nonlocal ok
        good = abs(got - want) <= tol
        ok = ok and good
        print(f"  {'ok ' if good else 'FAIL'} {name:52} {got:7.4f} "
              f"(want {want:.4f})")

    lim = 1.003
    print("1. planted signals against a 1.003 deg clamp")
    chk("all pinned at the clamp", saturated_fraction(np.full(1000, lim), lim), 1.0)
    chk("all pinned NEGATIVE (sign-blind)",
        saturated_fraction(np.full(1000, -lim), lim), 1.0)
    chk("all at half the clamp", saturated_fraction(np.full(1000, lim / 2), lim), 0.0)
    chk("half pinned, half idle",
        saturated_fraction(np.concatenate([np.full(500, lim),
                                           np.zeros(500)]), lim), 0.5)
    print("\n2. the 5% tolerance band")
    chk("just inside  (0.96 x clamp) counts",
        saturated_fraction(np.full(10, 0.96 * lim), lim), 1.0)
    chk("just outside (0.94 x clamp) does not",
        saturated_fraction(np.full(10, 0.94 * lim), lim), 0.0)

    print("\n3. an unclamped cell is NaN, not zero")
    v = saturated_fraction(np.zeros(10), 0.0)
    good = np.isnan(v)
    ok = ok and good
    print(f"  {'ok ' if good else 'FAIL'} limit 0 -> NaN "
          f"(reporting 0.0 would read as 'never saturates')")

    print("\n4. the F/R projection isolates the YAW partition")
    # a pure L/R pattern (the roll term's partition) must project to ZERO on F/R
    lr = {"A": +10.0, "B": -10.0, "C": -10.0, "D": +10.0}
    fr = sum(FR_SIGN[l] * lr[l] for l in LEGS) / 4.0
    chk("pure L/R pattern -> F/R", fr, 0.0)
    yaw = {"A": +5.0, "B": +5.0, "C": -5.0, "D": -5.0}
    fr = sum(FR_SIGN[l] * yaw[l] for l in LEGS) / 4.0
    chk("pure F/R pattern -> F/R", fr, 5.0)
    return ok


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", action="append", default=[])
    ap.add_argument("--label", action="append", default=[])
    ap.add_argument("--limit-deg", action="append", default=[],
                    help="gamma_yaw_limit in DEGREES for the matching --dir")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()

    if a.selftest:
        print("yaw_saturation.py selftest\n")
        good = selftest()
        print(f"\n  SELFTEST {'PASS' if good else 'FAIL'}")
        return 0 if good else 1
    if not a.dir:
        ap.error("need --dir, or --selftest")

    print("heading-loop saturation -- the fraction of the band with the F/R")
    print(f"camber projection pinned within {100*TOL:.0f}% of its clamp.")
    print("A loop that never reaches its clamp cannot lock (S187).\n")
    print(f"  {'cell':14} {'limit':>7} {'sat frac':>9} {'|F/R| med':>10} "
          f"{'|F/R| max':>10} {'n':>6}")
    for i, d in enumerate(a.dir):
        d = os.path.expanduser(d)
        name = a.label[i] if i < len(a.label) else os.path.basename(d.rstrip("/"))
        lim = float(a.limit_deg[i]) if i < len(a.limit_deg) else 0.0
        fracs, meds, maxs, n = [], [], [], 0
        for tq in sorted(glob.glob(os.path.join(d, "run[0-9].csv"))):
            od = os.path.join(d, "odom_" + os.path.basename(tq))
            try:
                _t, fr = fr_series(tq, od)
            except (SystemExit, Exception):          # noqa: BLE001
                continue
            fracs.append(saturated_fraction(fr, lim))
            meds.append(float(np.median(np.abs(fr))))
            maxs.append(float(np.max(np.abs(fr))))
            n += 1
        if not n:
            print(f"  {name:14} no readable runs")
            continue
        sf = float(np.nanmedian(fracs)) if not all(np.isnan(fracs)) else float("nan")
        print(f"  {name:14} {lim:7.3f} {sf:9.3f} {np.median(meds):10.3f} "
              f"{np.max(maxs):10.3f} {n:6d}")
        print(f"  {'':14} per-run sat: "
              + " ".join("nan" if np.isnan(f) else f"{f:.3f}" for f in fracs))
    return 0


if __name__ == "__main__":
    sys.exit(main())
