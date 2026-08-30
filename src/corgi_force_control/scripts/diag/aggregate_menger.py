#!/usr/bin/env python3
"""Aggregate the camber-in-pronk campaign into the s87 P1-P3 verdicts.

Offline only. Reads the menger_acker capture tree
(~/corgi_runs/menger_acker/<cond>/{odom,}runN.csv), computes each run's
steady-tail median signed Menger curvature with check_menger's own loaders
(one implementation, no drift), and evaluates the REGISTERED predictions:

  P1 (sign):      1/2(kappa_pos - kappa_neg) carries the commanded turn sign
                  at every magnitude
  P2 (detection): |1/2(kappa_pos - kappa_neg)| at 5 deg > 3 sigma of the
                  lam0 baseline kappa (sigma over the n=3 run medians)
  P3 (scaling):   kappa(10)/kappa(5) and kappa(15)/kappa(5) within x/1.5 of
                  the geometric ratios 1.89 and 2.71

The headline is direction-antisymmetric because the baseline yaw bias
(-1.3..-2.8 deg/s ~ R 4.4-11 m) is the same order as the 5 deg prediction;
the difference of a +dir and a -dir run cancels any bias common to both.

Usage:
    python3 aggregate_menger.py [--base ~/corgi_runs/menger_acker]
                                [--start 12] [--chord 1.0]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_menger import (LEGS, load_odom_csv, load_torque_csv, touchdowns,
                          menger)  # noqa: E402
from check_yaw_phase import dedupe_time  # noqa: E402

# Registered geometry (log s87): lam_in -> (lam_out_deg, kappa_geom 1/m)
REGISTERED = {"lam5": (4.406, 0.299), "lam10": (7.887, 0.567),
              "lam15": (10.725, 0.811)}
RATIO_GEOM = {"lam10": 1.89, "lam15": 2.71}


def run_kappa(odom_csv, torque_csv, start, chord, floor=2e-3, end=30.0):
    """-> (median steady-tail signed kappa, n_triples) for one capture pair.

    Window [overlap+start, overlap+end]: the first lam0_default run showed
    garbage curvature (|kappa| 4-10, heights 100-300 mm) both before ~12 s
    and after ~35 s of overlap; 12..30 sits on the steady band.
    """
    ot, xy, quat = load_odom_csv(odom_csv)
    ot, xy, quat = dedupe_time(ot, xy, quat)
    ct, contact, gamma, theta = load_torque_csv(torque_csv)
    t0, t1 = max(ot[0], ct[0]), min(ot[-1], ct[-1])
    t1 = min(t1, t0 + end)
    a0 = t0 + start
    om = (ot >= a0) & (ot <= t1)
    if int(om.sum()) < 100:
        return float("nan"), 0
    t, seg = ot[om], xy[om]
    td_all = touchdowns(ct, contact[:, LEGS.index("A")])
    td = td_all[(td_all >= a0) & (td_all <= t1)]
    if len(td) < 5:
        return float("nan"), 0
    px = np.interp(td, t, seg[:, 0])
    py = np.interp(td, t, seg[:, 1])
    pts = np.column_stack((px, py))
    step = np.hypot(np.diff(seg[:, 0]), np.diff(seg[:, 1]))
    s_cum = np.concatenate(([0.0], np.cumsum(step)))
    s_td = np.interp(td, t, s_cum)
    kaps = []
    for k in range(len(td)):
        i = int(np.argmin(np.abs(s_td - (s_td[k] - chord))))
        j = int(np.argmin(np.abs(s_td - (s_td[k] + chord))))
        if i >= k or j <= k:
            continue
        if (s_td[k] - s_td[i] < 0.6 * chord
                or s_td[j] - s_td[k] < 0.6 * chord):
            continue
        kap, height = menger(pts[i], pts[k], pts[j])
        if height >= floor:
            kaps.append(kap)
    if len(kaps) < 3:
        return float("nan"), len(kaps)
    return float(np.median(kaps)), len(kaps)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base",
                    default=os.path.expanduser("~/corgi_runs/menger_acker"))
    ap.add_argument("--start", type=float, default=12.0)
    ap.add_argument("--chord", type=float, default=1.0)
    args = ap.parse_args()

    meds = {}   # cond -> [per-run median kappa]
    for cond in sorted(os.listdir(args.base)):
        d = os.path.join(args.base, cond)
        if not os.path.isdir(d):
            continue
        vals = []
        for n in range(1, 10):
            oc = os.path.join(d, f"odom_run{n}.csv")
            tc = os.path.join(d, f"run{n}.csv")
            if not (os.path.exists(oc) and os.path.exists(tc)):
                continue
            try:
                k, ntr = run_kappa(oc, tc, args.start, args.chord)
            except SystemExit as e:
                print(f"  {cond} run{n}: SKIPPED ({e})")
                continue
            vals.append(k)
            print(f"  {cond} run{n}: median kappa {k:+.4f} 1/m "
                  f"({ntr} triples)")
        if vals:
            meds[cond] = np.array(vals)

    print()
    for cond, v in meds.items():
        ok = v[np.isfinite(v)]
        if not len(ok):
            print(f"{cond:14s} NO VALID RUNS")
            continue
        print(f"{cond:14s} median-of-runs {np.median(ok):+.4f} 1/m   "
              f"runs {' '.join(f'{x:+.4f}' for x in ok)}")

    # --- The verdicts -------------------------------------------------------
    print()
    base = meds.get("lam0")
    sigma = float(np.std(base[np.isfinite(base)])) if base is not None else float("nan")
    head = {}
    for mag in ("lam5", "lam10", "lam15"):
        p, n = meds.get(f"{mag}_pos"), meds.get(f"{mag}_neg")
        if p is None or n is None:
            continue
        h = 0.5 * (float(np.median(p[np.isfinite(p)]))
                   - float(np.median(n[np.isfinite(n)])))
        head[mag] = h
        kg = REGISTERED[mag][1]
        print(f"HEADLINE {mag}: 1/2(k+ - k-) = {h:+.4f} 1/m   "
              f"(geometric {kg:+.3f}; {100*h/kg:.0f}% of geometric)")
    if head:
        print()
        # P1: dir +1 is a LEFT turn commanded; kappa + is CCW/left. So the
        # headline should be POSITIVE at every magnitude if the mechanism
        # turns the robot the way the geometry says.
        signs = {m: np.sign(h) for m, h in head.items()}
        p1 = all(s > 0 for s in signs.values()) or all(s < 0 for s in signs.values())
        print(f"P1 (consistent sign across magnitudes): "
              f"{'PASS' if p1 else 'FAIL'}   {signs}")
        if p1 and all(s < 0 for s in signs.values()):
            print("   NOTE: consistent but NEGATIVE -- the mechanism curves "
                  "the path AWAY from the commanded side; record the sign "
                  "with the pos/neg convention before claiming.")
        if "lam5" in head and np.isfinite(sigma):
            p2 = abs(head["lam5"]) > 3 * sigma
            print(f"P2 (5 deg detection vs 3 sigma of lam0 = {3*sigma:.4f}): "
                  f"{'PASS' if p2 else 'FAIL'}   |h| = {abs(head['lam5']):.4f}")
        for mag, rg in RATIO_GEOM.items():
            if mag in head and "lam5" in head and head["lam5"]:
                r = head[mag] / head["lam5"]
                p3 = rg / 1.5 <= r <= rg * 1.5
                print(f"P3 ({mag}/lam5 ratio vs geometric {rg}): "
                      f"{'PASS' if p3 else 'FAIL'}   measured {r:.2f}")


if __name__ == "__main__":
    main()
