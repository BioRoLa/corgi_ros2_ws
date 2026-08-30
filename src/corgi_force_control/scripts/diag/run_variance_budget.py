#!/usr/bin/env python3
"""Where does the run-to-run scatter come from, and should we buy n or T?

WHY THIS EXISTS. Every campaign here runs n = 3 and every campaign here says
"n = 3 is a screen". Nobody has ever measured WHY the scatter is so large, so
"run more" and "run longer" have never been priced against each other. They are
not interchangeable:

    Var(cell estimate) = sigma_between^2 / n  +  sigma_within^2(T) / n

A LONGER run shrinks only the second term -- it is sampling noise, and it falls
like 1/T. MORE runs shrink both. But a run is not free before it produces data:
~45 s of settle plus ~60 s of launch and teardown, ~105 s of wall before the
first useful sample, against ~209 s for a 24 s capture at RTF 0.115. So a run is
~1/3 overhead, and if the scatter is dominated by WITHIN-run sampling noise,
lengthening the capture buys the same precision for about two thirds of the
wall clock. If it is dominated by BETWEEN-run variation -- initial conditions,
the block removal, whatever the plant does differently each time -- then
lengthening buys nothing at all and only n helps.

This measures which, by SPLIT-HALF: each run's scored window is cut in two, the
metric computed on each half, and a one-way random-effects decomposition run
with runs as groups and halves as replicates.

    MS_w = sum (x_ij - xbar_i)^2 / (n * (k-1))          k = 2 halves
    MS_b = k * sum (xbar_i - xbar)^2 / (n - 1)
    sigma_w^2 = MS_w                       (at HALF length)
    sigma_b^2 = max(0, (MS_b - MS_w) / k)

sigma_w^2 is then halved to express it at full window length, since sampling
variance falls like 1/T.

⚠ WHAT THIS CANNOT SEE. A split-half within one run shares that run's initial
conditions, its block removal and its particular limit cycle. So sigma_b^2 here
is "everything that differs between runs", which is exactly the quantity that
more runs buys down -- but it also absorbs any slow drift WITHIN a run, which
would inflate sigma_w^2 instead. Read the halves' means beside the variance: a
consistent first-half/second-half offset means the gait is still settling and
neither n nor T is the right answer, a longer SETTLE is.

⚠ n = 3 runs gives 2 degrees of freedom on sigma_b^2. These estimates are
themselves noisy and are a guide to setting NPER, not a result to quote.

Metrics decomposed are the two that gate P-N-1 and P-N-2, both from the torque
capture where the window is ours to choose:
    swept beta   beta_LO - beta_TD per debounced stance episode, pooled
    min vLeg     S155's laggard metric, min over legs of the per-leg median

Usage:
    run_variance_budget.py --dir DIR [--dir DIR ...] [--tail 20]
    run_variance_budget.py --selftest
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_menger import debounce, DEBOUNCE                    # noqa: E402
from touchdown_phase import load, Unfit, TAIL_S                # noqa: E402
import leg_demand                                              # noqa: E402

# Cost model, measured on this box 2026-08-22 (clock_ff campaign):
# 24.11 s / 24.11 s / 24.00 s of sim in 213 / 199 / 209 s of wall.
RTF = 0.115
FIXED_WALL_S = 105.0          # 45 s settle + ~60 s launch/teardown per run


def window_metrics(path, t_lo, t_hi, leg_length=leg_demand.LEG_LENGTH_M):
    """-> (swept_beta_median, min_vLeg) over stance episodes inside [t_lo, t_hi].

    Same edges as touchdown_phase and leg_demand: shared debounce, the same
    3-sample minimum episode. Windowing is by absolute sim time so the two
    halves are disjoint by construction.
    """
    legs = load(path)
    sweeps, per_leg = [], {}
    for leg, (t, c, b, _bc) in legs.items():
        m = (t >= t_lo) & (t <= t_hi)
        t, c, b = t[m], c[m], b[m]
        if len(t) < 200:
            continue
        d = debounce(c, DEBOUNCE)
        chg = np.diff(d.astype(int))
        rise = np.flatnonzero(chg > 0) + 1
        fall = np.flatnonzero(chg < 0) + 1
        v = []
        for r0 in rise:
            nxt = fall[fall > r0]
            if not len(nxt):
                continue
            f0 = nxt[0]
            if f0 - r0 < 3:
                continue
            dt = t[f0] - t[r0]
            if dt <= 0:
                continue
            sweeps.append(b[f0] - b[r0])
            v.append(leg_length * (b[f0] - b[r0]) / dt)
        if len(v) >= leg_demand.MIN_EPISODES:
            per_leg[leg] = float(np.median(v))
    if len(sweeps) < 20 or len(per_leg) < leg_demand.MIN_LEGS:
        raise Unfit("only %d episodes / %d legs in [%.1f, %.1f]"
                    % (len(sweeps), len(per_leg), t_lo, t_hi))
    return float(np.median(sweeps)), float(min(per_leg.values()))


def decompose(values_by_run):
    """One-way random effects on {run: (half1, half2)} -> variance components."""
    runs = [v for v in values_by_run.values() if len(v) == 2]
    n, k = len(runs), 2
    if n < 2:
        return None
    arr = np.array(runs, dtype=float)
    run_means = arr.mean(axis=1)
    grand = run_means.mean()
    ms_w = float(((arr - run_means[:, None]) ** 2).sum() / (n * (k - 1)))
    ms_b = float(k * ((run_means - grand) ** 2).sum() / (n - 1))
    sig_w_half = ms_w
    sig_b = max(0.0, (ms_b - ms_w) / k)
    sig_w_full = sig_w_half / 2.0        # sampling variance falls like 1/T
    return {"grand": grand, "sig_b": sig_b, "sig_w_full": sig_w_full,
            "n": n, "run_means": run_means.tolist(),
            "half_gap": float((arr[:, 1] - arr[:, 0]).mean())}


def budget_table(sig_b, sig_w_full, t_ref):
    """Cheapest (n, T) for a target precision, on the measured cost model."""
    print("      cheapest (n, T) for a given SE of the cell estimate:")
    print("      %-10s %-6s %-8s %-10s %s"
          % ("target SE", "n", "T (s)", "wall (min)", "note"))
    for target in (0.6, 0.4, 0.3):
        best = None
        for n in range(2, 11):
            for T in (12, 18, 24, 36, 48, 72):
                var = (sig_b + sig_w_full * t_ref / T) / n
                se = np.sqrt(var)
                if se > target * np.sqrt(sig_b + sig_w_full):
                    continue
                wall = n * (FIXED_WALL_S + T / RTF)
                if best is None or wall < best[0]:
                    best = (wall, n, T, se)
        if best is None:
            print("      %-10s unreachable within n<=10, T<=72 s"
                  % ("%.0f%% of 1 run" % (target * 100)))
            continue
        wall, n, T, se = best
        note = "longer runs win" if T > t_ref else (
            "more runs win" if T < t_ref else "current window")
        print("      %-10s %-6d %-8d %-10.1f %s"
              % ("%.0f%% of 1 run" % (target * 100), n, T, wall / 60.0, note))


def report(dirs, tail_s):
    for d in dirs:
        d = os.path.expanduser(d)
        runs = sorted(glob.glob(os.path.join(d, "run[0-9].csv")))
        print()
        print("=" * 72)
        print("CELL %s   (%d runs)" % (os.path.basename(os.path.normpath(d)),
                                       len(runs)))
        print("=" * 72)
        sweep_by_run, vleg_by_run = {}, {}
        for p in runs:
            name = os.path.basename(p)
            try:
                legs = load(p)
            except Unfit as e:
                print("  %s: skipped -- %s" % (name, e))
                continue
            t_all = np.concatenate([v[0] for v in legs.values()])
            t_end = t_all.max()
            t_start = t_end - tail_s
            t_mid = 0.5 * (t_start + t_end)
            halves = []
            try:
                halves.append(window_metrics(p, t_start, t_mid))
                halves.append(window_metrics(p, t_mid, t_end))
            except Unfit as e:
                print("  %s: split-half UNFIT -- %s" % (name, e))
                continue
            sweep_by_run[name] = [h[0] for h in halves]
            vleg_by_run[name] = [h[1] for h in halves]
            print("  %-12s swept beta %+.4f / %+.4f    min vLeg %+.4f / %+.4f"
                  % (name, halves[0][0], halves[1][0],
                     halves[0][1], halves[1][1]))

        for label, data in (("swept beta", sweep_by_run),
                            ("min vLeg", vleg_by_run)):
            r = decompose(data)
            print()
            if r is None:
                print("  %s: need >= 2 usable runs" % label)
                continue
            sd_b = np.sqrt(r["sig_b"])
            sd_w = np.sqrt(r["sig_w_full"])
            tot = r["sig_b"] + r["sig_w_full"]
            print("  %s: mean %+.4f, n = %d runs" % (label, r["grand"], r["n"]))
            print("      sd BETWEEN runs      %.4f   (%4.1f%% of variance)"
                  % (sd_b, 100.0 * r["sig_b"] / tot if tot else 0.0))
            print("      sd WITHIN a run      %.4f   (%4.1f%%)  at the full %.0f s window"
                  % (sd_w, 100.0 * r["sig_w_full"] / tot if tot else 0.0, tail_s))
            print("      half-to-half drift   %+.4f   (2nd half minus 1st; a "
                  "consistent offset means it is still settling)" % r["half_gap"])
            if r["sig_b"] > 3.0 * r["sig_w_full"]:
                print("      -> BETWEEN dominates. Longer runs buy almost nothing;")
                print("         only more runs, or a quieter plant, reduce this.")
            elif r["sig_w_full"] > 3.0 * r["sig_b"]:
                print("      -> WITHIN dominates. Lengthening the capture is the")
                print("         cheap lever: no settle or launch overhead to pay.")
            else:
                print("      -> comparable. Trade them off on wall clock:")
            budget_table(r["sig_b"], r["sig_w_full"], tail_s)


def selftest():
    """Known answers: the arithmetic, and agreement with leg_demand."""
    ok = True
    # 1. Variance decomposition on synthetic data with a KNOWN answer.
    #    Runs offset by exactly +-0.10 with zero within-run scatter must give
    #    sig_w = 0 and a positive sig_b.
    r = decompose({"a": [1.10, 1.10], "b": [0.90, 0.90], "c": [1.00, 1.00]})
    print("  1. pure BETWEEN: sd_b %.4f (want 0.1000), sd_w %.4f (want 0)"
          % (np.sqrt(r["sig_b"]), np.sqrt(r["sig_w_full"])))
    if abs(np.sqrt(r["sig_b"]) - 0.1) > 1e-9 or r["sig_w_full"] > 1e-12:
        ok = False
        print("     FAIL")
    # 2. The mirror case: identical run means, all scatter inside the runs.
    r = decompose({"a": [1.10, 0.90], "b": [1.10, 0.90], "c": [1.10, 0.90]})
    # Halves at +-0.10 give MS_w = 0.02, so sd at HALF length is 0.1414 and
    # at full length 0.1414/sqrt(2) = 0.1000. The first version of this test
    # expected 0.0707 -- I had divided by sqrt(2) twice. The TEST was wrong,
    # not the estimator, and it is recorded here because a self-test that
    # fails for its own arithmetic teaches you to ignore self-tests.
    print("  2. pure WITHIN:  sd_b %.4f (want 0), sd_w %.4f (want 0.1000)"
          % (np.sqrt(r["sig_b"]), np.sqrt(r["sig_w_full"])))
    if np.sqrt(r["sig_b"]) > 1e-9 or abs(np.sqrt(r["sig_w_full"]) - 0.1000) > 1e-9:
        ok = False
        print("     FAIL")
    # 3. AGREEMENT WITH THE SHIPPED ANALYSER. window_metrics over the whole
    #    tail must reproduce leg_demand's min vLeg for the same run, or the
    #    windowing has changed the measurement rather than just sliced it.
    base = os.path.expanduser("~/corgi_runs/laggard/base")
    p = os.path.join(base, "run1.csv")
    if not os.path.isfile(p):
        print("  3. banked laggard/base absent -- SELFTEST PASS (partial)")
        return ok
    legs = load(p)
    t_end = np.concatenate([v[0] for v in legs.values()]).max()
    _sw, mine = window_metrics(p, t_end - TAIL_S, t_end)
    theirs = leg_demand.run_stats(p)["min_vleg"]
    good = abs(mine - theirs) < 1e-9
    ok = ok and good
    print("  3. agrees with leg_demand on run1: %+.6f vs %+.6f  %s"
          % (mine, theirs, "ok" if good else "MISMATCH"))
    return ok


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", action="append", default=[])
    ap.add_argument("--tail", type=float, default=TAIL_S)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        print("run_variance_budget.py selftest")
        ok = selftest()
        print("\n  SELFTEST %s" % ("PASS" if ok else "FAIL"))
        return 0 if ok else 1
    if not a.dir:
        ap.error("need at least one --dir")
    print("run-to-run variance budget: is the scatter WITHIN runs or BETWEEN them?")
    print("cost model: %.0f s fixed per run, RTF %.3f" % (FIXED_WALL_S, RTF))
    report(a.dir, a.tail)
    return 0


if __name__ == "__main__":
    sys.exit(main())
