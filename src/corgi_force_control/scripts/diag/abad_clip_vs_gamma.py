#!/usr/bin/env python3
"""The two zero-sim tests S205 S3 named, for the candidate that a silent
per-leg ABAD torque ceiling is one cause behind three puzzles (the 29-42%
authority deficit, the CW/CCW asymmetry, the L/R achieved-gamma split).

TEST 1 -- does the CLIP predict the SHORTFALL, run by run and leg by leg?
    For every (run, leg): stance clip fraction (abad_torque.stats) against
    the achieved-gamma shortfall |lambda_cmd| - |gamma_achieved| (median of
    the MEASURED gamma column in stance -- the CSV's gamma is leg.get_states(),
    corgi_driver.py ~1257). If the ceiling is the mechanism, the legs and runs
    that clip more should fall further short. Reported within each lambda
    separately (the controller's own tracking error also scales with lambda)
    and as the leg ordering: is the most-clipped leg the most-short leg?

TEST 2 -- does reversing dir MOVE the clip from A to B?
    The L/R pattern {A:+, B:-, C:-, D:+} x dir loads one front leg and unloads
    the other. Prediction, stated before looking: dir = +1 clips A (S205),
    dir = -1 clips B. Two sources: menger_acker_final lam10_pos vs lam10_neg
    (same era, so a clean within-campaign sign test even though it is the
    k12000 plant and an Ackermann in/out pair), and matched_turn/cam rep 1
    (13 deg, dir -1, config of record).

Neither test is registered; both are cheap and both can KILL the candidate.
Surviving them earns a registration, not a conclusion.

Usage:
    abad_clip_vs_gamma.py --selftest
    abad_clip_vs_gamma.py --test1
    abad_clip_vs_gamma.py --test2
"""
import argparse
import glob
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from abad_torque import stats as abad_stats, START          # noqa: E402
from check_menger import load_torque_csv, LEGS              # noqa: E402

LR = {"A": +1.0, "B": -1.0, "C": -1.0, "D": +1.0}
RUNS = os.path.expanduser("~/corgi_runs")

TEST1_CELLS = [  # all dir +1, config of record (k7150)
    ("camber_lambda/lam10", 10.0), ("inner_outer/k0", 10.0),
    ("inner_outer/k035", 10.0), ("inner_outer/k106", 10.0),
    ("camber_lambda/lam15", 15.0),
]
TEST2_SETS = [
    ("menger_acker_final/lam10_pos", +1), ("menger_acker_final/lam10_neg", -1),
    ("matched_turn/cam", -1),
]


def cmd_lambda(ctl_log):
    """in/out/dir from the ACKER banner; returns (in_deg, out_deg, dir)."""
    try:
        txt = open(ctl_log).read()
    except OSError:
        return None
    m = re.search(r"ACKER CAMBER set: in=([0-9.]+) deg out=([0-9.]+) deg dir=([+-]?\d)", txt)
    return (float(m.group(1)), float(m.group(2)), int(m.group(3))) if m else None


def achieved_gamma(tq):
    """median |gamma| per leg in stance over the band, degrees."""
    t, contact, gamma, _th = load_torque_csv(tq)
    m = t >= t[0] + START
    out = {}
    for i, leg in enumerate(LEGS):
        st = m & contact[:, i]
        out[leg] = float(np.median(np.abs(gamma[st, i]))) if st.sum() > 50 else float("nan")
    return out


def spearman(x, y):
    x, y = np.asarray(x, float), np.asarray(y, float)
    ok = ~(np.isnan(x) | np.isnan(y))
    x, y = x[ok], y[ok]
    if len(x) < 4:
        return float("nan"), len(x)
    rx = np.argsort(np.argsort(x)); ry = np.argsort(np.argsort(y))
    return float(np.corrcoef(rx, ry)[0, 1]), len(x)


def test1():
    print("TEST 1 -- clip fraction vs achieved-gamma shortfall, per (run, leg)\n")
    rows = []
    for cell, lam in TEST1_CELLS:
        d = os.path.join(RUNS, cell)
        for tq in sorted(glob.glob(os.path.join(d, "run[0-9].csv"))):
            n = os.path.basename(tq)[3:-4]
            s = abad_stats(tq)
            g = achieved_gamma(tq)
            if not s:
                continue
            for leg in LEGS:
                if leg not in s["per_leg"] or np.isnan(g[leg]):
                    continue
                rows.append({"cell": cell, "run": n, "leg": leg, "lam": lam,
                             "clip": s["per_leg"][leg]["clip_pct"],
                             "gamma": g[leg], "short": lam - g[leg]})
    print("  %-24s %3s %3s %5s %7s %9s %8s" % ("cell", "run", "leg", "lam", "clip%", "|gamma|", "short"))
    for r in rows:
        print("  %-24s %3s %3s %5.0f %7.2f %9.3f %8.3f" %
              (r["cell"], r["run"], r["leg"], r["lam"], r["clip"], r["gamma"], r["short"]))
    print()
    for lam in sorted({r["lam"] for r in rows}):
        sub = [r for r in rows if r["lam"] == lam]
        rho, n = spearman([r["clip"] for r in sub], [r["short"] for r in sub])
        print("  lambda %2.0f, all legs pooled: Spearman(clip, shortfall) = %+.3f  (n=%d points)" % (lam, rho, n))
        for leg in LEGS:
            sl = [r for r in sub if r["leg"] == leg]
            rho_l, n_l = spearman([r["clip"] for r in sl], [r["short"] for r in sl])
            print("     leg %s across runs: rho %+.3f (n=%d)   median clip %.2f%%, median shortfall %+.3f deg"
                  % (leg, rho_l, n_l, np.median([r["clip"] for r in sl]), np.median([r["short"] for r in sl])))
    print("\n  Leg ORDERING at each lambda (most clipped first) vs (most short first):")
    for lam in sorted({r["lam"] for r in rows}):
        sub = [r for r in rows if r["lam"] == lam]
        mc = {leg: np.median([r["clip"] for r in sub if r["leg"] == leg]) for leg in LEGS}
        ms = {leg: np.median([r["short"] for r in sub if r["leg"] == leg]) for leg in LEGS}
        oc = "".join(sorted(LEGS, key=lambda l: -mc[l]))
        osh = "".join(sorted(LEGS, key=lambda l: -ms[l]))
        print("     lambda %2.0f: clip order %s   shortfall order %s   %s"
              % (lam, oc, osh, "SAME leading leg" if oc[0] == osh[0] else "different leading leg"))
    return rows


def test2():
    print("TEST 2 -- does reversing dir move the clip from A to B?")
    print("  prediction (before looking): dir +1 -> A clips most; dir -1 -> B clips most\n")
    print("  %-30s %3s %4s %6s %6s %6s %6s   %s" % ("set", "run", "dir", "A", "B", "C", "D", "most clipped"))
    verdicts = []
    for cell, want_dir in TEST2_SETS:
        d = os.path.join(RUNS, cell)
        for tq in sorted(glob.glob(os.path.join(d, "run[0-9].csv"))):
            n = os.path.basename(tq)[3:-4]
            cfg = cmd_lambda(os.path.join(d, "ctl_run%s.log" % n))
            s = abad_stats(tq)
            if not s or not cfg:
                continue
            clip = {leg: s["per_leg"].get(leg, {}).get("clip_pct", float("nan")) for leg in LEGS}
            top = max(LEGS, key=lambda l: clip[l] if not np.isnan(clip[l]) else -1)
            pred = "A" if cfg[2] > 0 else "B"
            hit = top == pred
            verdicts.append((cfg[2], hit, max(clip.values())))
            print("  %-30s %3s %+4d %6.2f %6.2f %6.2f %6.2f   %s  %s" %
                  (cell, n, cfg[2], clip["A"], clip["B"], clip["C"], clip["D"], top,
                   "(as predicted)" if hit else "(NOT as predicted)"))
    pos = [v for v in verdicts if v[0] > 0]; neg = [v for v in verdicts if v[0] < 0]
    print("\n  dir +1: %d/%d runs clip A most.   dir -1: %d/%d runs clip B most."
          % (sum(h for _, h, _ in pos), len(pos), sum(h for _, h, _ in neg), len(neg)))
    low = [v for v in verdicts if v[2] < 0.5]
    if low:
        print("  caution: %d run(s) have max clip < 0.5%% -- 'most clipped' is noise there." % len(low))
    return verdicts


def selftest():
    ok = True

    def chk(name, good):
        nonlocal ok
        ok = ok and good
        print("  %s %s" % ("ok " if good else "FAIL", name))

    rho, n = spearman([1, 2, 3, 4, 5], [2, 4, 6, 8, 10])
    chk("spearman of a monotone pair is +1 (got %+.2f)" % rho, abs(rho - 1) < 1e-9)
    rho, n = spearman([1, 2, 3, 4, 5], [5, 4, 3, 2, 1])
    chk("...and -1 reversed (got %+.2f)" % rho, abs(rho + 1) < 1e-9)
    rho, n = spearman([1, 2, 3, float("nan")], [1, 2, 3, 4])
    chk("nan pairs dropped; n<4 -> nan (n=%d)" % n, n == 3 and np.isnan(rho))
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, "ctl_run1.log")
        open(p, "w").write("x ACKER CAMBER set: in=13.00 deg out=13.00 deg dir=-1 (ramp)\n")
        chk("ACKER banner parsed: %s" % (cmd_lambda(p),), cmd_lambda(p) == (13.0, 13.0, -1))
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--test1", action="store_true")
    ap.add_argument("--test2", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        print("abad_clip_vs_gamma.py selftest\n")
        g = selftest()
        print("\n  SELFTEST %s" % ("PASS" if g else "FAIL"))
        return 0 if g else 1
    if a.test1:
        test1()
        print()
    if a.test2:
        test2()
    if not (a.test1 or a.test2):
        ap.error("need --test1 and/or --test2, or --selftest")
    return 0


if __name__ == "__main__":
    sys.exit(main())
