#!/usr/bin/env python3
"""Repeatability of two curvature actuators AT MATCHED CURVATURE. Log S196.

WHY. S194 measured `k_steer` and noticed, unregistered, that relative turn
repeatability improved from 76% to 49% of the median -- then flagged its own
confound: "a larger signal against fixed noise always looks relatively
cleaner". S194's comparison was k106 (kappa 0.44) against k0 (kappa 0.26), so
the two cells differ in the size of the thing being measured. The honest test
is repeatability at MATCHED kappa, and S194 said so.

That test needs no simulator time. It is already banked:

    camber_lambda/lam15   camber 15 deg, k_steer 0      median kappa -0.4453
    inner_outer/k106      camber 10 deg, k_steer 0.106  median kappa -0.4401

Matched to 1.2%. Both self-certify to the config of record (k_flight 7150,
b_flight 115.8, 265 template rows / 0.2642 s, k_yaw 0, turn_rate 0); their ctl
logs differ in exactly two lines, the camber magnitude and k_steer.

WHAT THIS IS NOT. turn_rate is 0 in both, so neither cell TRACKS a commanded
radius -- both are OPEN-LOOP curvature actuators asked to hold a heading. Open
Issue #1 ("the turn radius does not repeat") is about commanded turns via
turn_rate and is NOT settled here. S129 already found the camber repeatability
advantage does not transfer to the differential-drive steering channel
(steer_offset + k_steer_yaw); k_steer is a third, different actuator.

n = 5 per cell. A variance ratio from n = 5 is weak evidence and the exhaustive
permutation test below is reported precisely so the weakness is visible rather
than hidden behind a ratio. Two campaigns run 14 h apart, NOT interleaved --
so a plant drift between them is an alternative reading that this data cannot
exclude.

Usage:
    matched_kappa.py --selftest
    matched_kappa.py
"""
import argparse
import itertools
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aggregate_menger import run_kappa                          # noqa: E402

START, CHORD = 12.0, 1.0

CELLS = [
    ("camber lam15  (k_steer 0)",
     os.path.expanduser("~/corgi_runs/camber_lambda/lam15"), -0.4453),
    ("k_steer 0.106 (camber 10)",
     os.path.expanduser("~/corgi_runs/inner_outer/k106"), -0.4401),
]


def cell_kappa(d):
    """-> per-run median signed kappa, via aggregate_menger's own function."""
    out = []
    for n in range(1, 10):
        oc, tc = os.path.join(d, "odom_run%d.csv" % n), os.path.join(d, "run%d.csv" % n)
        if not (os.path.exists(oc) and os.path.exists(tc)):
            continue
        try:
            k, _ntr = run_kappa(oc, tc, START, CHORD)
        except SystemExit:
            continue
        out.append(k)
    return np.array(out)


def sd_ratio_permutation(a, b, n_max=200000):
    """Exhaustive two-sided permutation p for the ratio of sample sds.

    Residuals are centred on EACH GROUP'S OWN mean before permuting: the
    question is scale, not location, and the two cells are deliberately
    matched in location. With n = 5 + 5 there are C(10,5) = 252 partitions, so
    this is exact, not sampled -- which matters, because an F test at (4, 4)
    df needs a ratio of 9.6 to reach p = 0.05 and would report almost anything
    as null.
    """
    ra, rb = a - a.mean(), b - b.mean()
    pool = np.concatenate([ra, rb])
    na = len(ra)
    obs = np.std(rb, ddof=1) / np.std(ra, ddof=1)
    stat = []
    for idx in itertools.combinations(range(len(pool)), na):
        m = np.zeros(len(pool), bool)
        m[list(idx)] = True
        s1, s2 = np.std(pool[m], ddof=1), np.std(pool[~m], ddof=1)
        if s1 > 0:
            stat.append(s2 / s1)
    stat = np.array(stat)
    # two-sided on the log ratio, which is symmetric under swapping groups
    p = float(np.mean(np.abs(np.log(stat)) >= abs(np.log(obs))))
    return obs, p, len(stat)


def _perm_p(ra, rb, rng, n_perm=1500):
    """Sampled version of sd_ratio_permutation, for the power simulation."""
    pool = np.concatenate([ra - ra.mean(), rb - rb.mean()])
    na = len(ra)
    obs = abs(np.log(np.std(rb, ddof=1) / np.std(ra, ddof=1)))
    hit = 0
    for _ in range(n_perm):
        q = rng.permutation(pool)
        s1, s2 = np.std(q[:na], ddof=1), np.std(q[na:], ddof=1)
        if s1 > 0 and abs(np.log(s2 / s1)) >= obs:
            hit += 1
    return hit / float(n_perm)


def power_curve(ratio, ns, trials=300, alpha=0.05, seed=0):
    """How many runs per cell would it take to resolve this sd ratio?

    Simulates normal residuals with the OBSERVED sd ratio and asks how often
    the permutation test rejects. Costs seconds; the campaign it might avoid
    costs hours of exclusive simulator. Run it BEFORE booking the plant.
    """
    rng = np.random.RandomState(seed)
    out = []
    for n in ns:
        rej = 0
        for _ in range(trials):
            ra = rng.normal(0.0, 1.0, n)
            rb = rng.normal(0.0, ratio, n)
            if _perm_p(ra, rb, rng) <= alpha:
                rej += 1
        out.append((n, rej / float(trials)))
    return out


def selftest():
    ok = True

    def chk(name, good):
        nonlocal ok
        ok = ok and good
        print("  %s %s" % ("ok " if good else "FAIL", name))

    # 1. identical scales -> permutation p must be large
    rng = np.random.RandomState(0)
    a = rng.normal(0, 1, 5)
    _obs, p, n = sd_ratio_permutation(a, a * 1.0)
    chk("identical residuals -> p = 1.0 (got %.3f, %d partitions)" % (p, n),
        p > 0.99 and n == 252)
    # 2. a 10x scale difference must be detected even at n = 5
    _obs, p2, _n = sd_ratio_permutation(a, a * 10.0)
    chk("10x scale difference -> p <= 0.05 (got %.4f)" % p2, p2 <= 0.05)
    # 3. sd convention is the sample sd, matching numpy ddof=1
    v = np.array([-0.5316, -0.3965, -0.4453, -0.4598, -0.3997])
    chk("sample sd of lam15 = 0.0550 (got %.4f)" % np.std(v, ddof=1),
        abs(np.std(v, ddof=1) - 0.0550) < 5e-4)
    # 4. reproduce aggregate_menger's PUBLISHED per-cell medians from the files
    for label, d, published in CELLS:
        if not os.path.isdir(d):
            chk("%s -- capture tree missing, cannot verify" % label, False)
            continue
        got = float(np.median(cell_kappa(d)))
        chk("%s median kappa = %+.4f (aggregate_menger: %+.4f)"
            % (label, got, published), abs(got - published) < 1e-4)
    return ok


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--power", action="store_true",
                    help="what n would resolve the observed ratio?")
    a = ap.parse_args()
    if a.selftest:
        print("matched_kappa.py selftest\n")
        good = selftest()
        print("\n  SELFTEST %s" % ("PASS" if good else "FAIL"))
        return 0 if good else 1

    vals = {}
    print("per-run median signed Menger curvature (1/m), band from t0+%.0f s\n"
          % START)
    for label, d, _pub in CELLS:
        v = cell_kappa(d)
        vals[label] = v
        print("  %-26s n=%d  %s" % (label, len(v),
                                    " ".join("%+.4f" % x for x in v)))
    print()
    print("  %-26s %9s %9s %9s %9s" %
          ("cell", "median", "mean", "sd", "sd/|mean|"))
    for label, v in vals.items():
        print("  %-26s %+9.4f %+9.4f %9.4f %8.1f%%" %
              (label, np.median(v), v.mean(), np.std(v, ddof=1),
               100.0 * np.std(v, ddof=1) / abs(v.mean())))

    (la, va), (lb, vb) = list(vals.items())
    match = abs(np.median(va) - np.median(vb)) / abs(
        0.5 * (np.median(va) + np.median(vb)))
    print("\n  curvature match: %.1f%% apart in median -- the premise of the "
          "comparison" % (100.0 * match))

    obs, p, npart = sd_ratio_permutation(va, vb)
    print("\n-- is one actuator more repeatable at the SAME curvature? --")
    print("  sd ratio (%s / %s) = %.2fx" % (lb.split()[0], la.split()[0], obs))
    print("  exhaustive permutation over %d partitions: p = %.3f" % (npart, p))
    try:
        from scipy import stats
        w, pl = stats.levene(va, vb, center="mean")
        print("  Levene (scipy, for comparison): W = %.3f, p = %.3f" % (w, pl))
    except ImportError:
        pass
    print("\n  %s" % ("SEPARATED at p <= 0.05" if p <= 0.05 else
                      "NOT SEPARATED -- n = 5 per cell cannot resolve this "
                      "ratio"))
    if a.power:
        print()
        print("-- what n per cell would resolve a %.2fx sd ratio? --" % obs)
        print("   (normal residuals, same permutation test, alpha 0.05)")
        for n, pw in power_curve(obs, [5, 8, 10, 15, 20, 30, 40]):
            bar = "#" * int(round(pw * 40))
            print("     n = %-3d power %.2f  %s" % (n, pw, bar))
        print()
        print("   Read this BEFORE booking the plant, not after.")
    else:
        print()
        print("  Run with --power to see what n would resolve it.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
