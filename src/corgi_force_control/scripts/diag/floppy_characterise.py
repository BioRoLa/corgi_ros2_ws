#!/usr/bin/env python3
"""Does floppy flight PREDICT anything? Open Issue #17's characterisation.

Registered in log S137 BEFORE any number here was computed. Read-only:
scores ~/corgi_runs as it already stands, no Webots, no new captures.

S118 measured the defect (42-51% of every flight phase on soft stance
gains, in every configuration). It did not measure whether that varies
usefully, or whether it costs anything. This does both.

    P-F1  floppy_given_air is near-constant: pooled IQR <= 15 points
    P-F2  it does NOT predict flight tracking error: |rho| < 0.4
    P-F3  it correlates POSITIVELY with tau p99.5 (rho >= +0.3)
          -- scored ONLY if P-F1 is falsified (see below)

WHAT IS REFUSED, AND WHY
------------------------
Flight fraction is NOT a legal outcome here.  floppy_given_air is
floppy_frac / air_frac, so regressing it on air_frac regresses a ratio
against its own denominator and manufactures correlation from nothing.
floppy_frac vs air_frac is worse: floppy is a SUBSET of air by
construction.  Both are refused rather than reported; see S137.

P-F3's dependency on P-F1 is not decoration.  If floppy really is
near-constant across cells there is no between-cell variance for tau to
co-vary with, and any rho computed on that residual noise is noise.

CELLS, NOT RUNS
---------------
Cells are different plants (camber / turn / scheduler / label-shift /
k_flight campaigns).  Pooling per-RUN rows across them is
pseudo-replication and is exactly the Simpson's-paradox exposure that
broke the flight-fraction statistic in S131.  Correlations are therefore
computed on per-cell MEDIANS; per-run values are still emitted so the
figure can show dots and the within-cell spread stays visible.

GATING (--selftest), three independent surfaces
-----------------------------------------------
  1. statistics    planted-rho recovery incl. a NULL designed to fool it
  2. data path     reproduce S118's five published cells via the shipped
                   floppy_flight.stats
  3. new quantity  my flight-error code shares its loader and its
                   decomposition with beta_tracking.py, so the STANCE
                   branch must reproduce beta_tracking's own stance RMS;
                   only the contact mask differs

A tool that has not reproduced a published number does not get used.
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from audit_degradation import Unfit                       # noqa: E402
import beta_tracking                                       # noqa: E402
import floppy_flight                                       # noqa: E402
import tau_demand_window                                   # noqa: E402

RUNS = os.path.expanduser("~/corgi_runs")
MIN_RUNS_PER_CELL = 3
BOOT = 2000
SEED = 20260821


# ----------------------------------------------------------------- stats
def _rank(x):
    """Average ranks, ties shared (what Spearman requires)."""
    x = np.asarray(x, float)
    order = np.argsort(x, kind="mergesort")
    r = np.empty(len(x), float)
    r[order] = np.arange(len(x), dtype=float)
    # average tied ranks
    for v in np.unique(x):
        m = x == v
        if m.sum() > 1:
            r[m] = r[m].mean()
    return r


def spearman(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    m = np.isfinite(a) & np.isfinite(b)
    if m.sum() < 4:
        return float("nan"), int(m.sum())
    ra, rb = _rank(a[m]), _rank(b[m])
    ra, rb = ra - ra.mean(), rb - rb.mean()
    d = np.sqrt((ra ** 2).sum() * (rb ** 2).sum())
    if d == 0:
        return float("nan"), int(m.sum())
    return float((ra * rb).sum() / d), int(m.sum())


def spearman_ci(a, b, boot=BOOT, seed=SEED):
    """Bootstrap over CELLS (the independent unit), percentile CI."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    m = np.isfinite(a) & np.isfinite(b)
    a, b = a[m], b[m]
    rho, n = spearman(a, b)
    if not np.isfinite(rho) or n < 4:
        return rho, (float("nan"), float("nan")), n
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(boot):
        i = rng.integers(0, n, n)
        if len(np.unique(a[i])) < 3 or len(np.unique(b[i])) < 3:
            continue
        r, _k = spearman(a[i], b[i])
        if np.isfinite(r):
            out.append(r)
    if len(out) < boot // 4:
        return rho, (float("nan"), float("nan")), n
    return rho, (float(np.percentile(out, 2.5)),
                 float(np.percentile(out, 97.5))), n


def verdict(rho, ci, thresh, want_above):
    """Report a DIRECTION only when the CI excludes zero (S125 discipline)."""
    if not np.isfinite(rho) or not np.isfinite(ci[0]):
        return "no estimate"
    if ci[0] <= 0.0 <= ci[1]:
        return "no detectable relationship (CI spans 0)"
    strong = rho >= thresh if want_above else abs(rho) >= thresh
    return "ABOVE threshold" if strong else "below threshold"


# ------------------------------------------------------- new quantity
def tracking_rms(path):
    """|d_beta| RMS in FLIGHT and in STANCE, radians.

    Shares beta_tracking's loader and its exact decomposition
    d_beta = (err_L + err_R)/2; only the contact mask differs.  The
    stance branch is redundant on purpose -- it is what --selftest
    checks against beta_tracking.py's own published stance number.
    """
    a = beta_tracking.load(path)
    if not len(a):
        raise Unfit("no paired L/R samples")
    nz = np.flatnonzero(a[:, 5] > 1e-9)
    if not nz.size:
        raise Unfit("no gait window (t_ff never non-zero)")
    a = a[a[:, 0] >= a[nz[0], 0]]
    eL, eR = a[:, 2], a[:, 3]
    good = (np.abs(eL) < beta_tracking.UNWRAP_GUARD) & \
           (np.abs(eR) < beta_tracking.UNWRAP_GUARD)
    dbeta = 0.5 * (eL + eR)
    stance = (a[:, 4] > 0.5) & good
    flight = (a[:, 4] <= 0.5) & good
    if stance.sum() < 200:
        raise Unfit(f"only {int(stance.sum())} stance samples")
    if flight.sum() < 200:
        raise Unfit(f"only {int(flight.sum())} flight samples")
    rms = lambda m: float(np.sqrt(np.mean(dbeta[m] ** 2)))
    return {"flight_dbeta": rms(flight), "stance_dbeta": rms(stance),
            "n_flight": int(flight.sum()), "n_stance": int(stance.sum())}


# ------------------------------------------------------------- corpus
def cells(root=RUNS):
    out = {}
    for p in glob.glob(os.path.join(root, "**", "run[0-9].csv"),
                       recursive=True):
        out.setdefault(os.path.dirname(p), []).append(p)
    return {d: sorted(v) for d, v in sorted(out.items())
            if len(v) >= MIN_RUNS_PER_CELL}


def score_run(path):
    row = {"path": path, "refusals": []}
    try:
        s = floppy_flight.stats(path)
        row["floppy_given_air"] = 100.0 * s["floppy_given_air"]
        row["air_frac"] = 100.0 * s["air_frac"]
        row["honesty"] = s["honesty"]
    except (Unfit, SystemExit, ValueError, KeyError) as e:
        row["refusals"].append(f"floppy: {e}")
    try:
        row.update(tracking_rms(path))
    except (Unfit, ValueError, KeyError) as e:
        row["refusals"].append(f"tracking: {e}")
    try:
        row["tau_p995"] = tau_demand_window.stats(path)["pooled_p995"]
    except (Unfit, ValueError, KeyError) as e:
        row["refusals"].append(f"tau: {e}")
    return row


# ----------------------------------------------------------- selftest
def _synth_pairs(rho_target, n, seed):
    rng = np.random.default_rng(seed)
    x = rng.normal(size=n)
    z = rng.normal(size=n)
    y = rho_target * x + np.sqrt(max(0.0, 1 - rho_target ** 2)) * z
    return x, y


def selftest():
    ok = True

    print("--- 1. statistics: planted-rho recovery ---")
    for target in (0.0, 0.3, 0.6, -0.5):
        x, y = _synth_pairs(target, 60, SEED + int(target * 100))
        rho, ci, n = spearman_ci(x, y, boot=800)
        hit = ci[0] <= target <= ci[1]
        print(f"  planted {target:+.2f} -> rho {rho:+.3f} "
              f"CI [{ci[0]:+.3f},{ci[1]:+.3f}] n={n}  "
              f"{'OK' if hit else 'FAIL (CI misses truth)'}")
        ok &= hit
    # the case designed to fool it: pure noise must NOT read as a signal
    x, y = _synth_pairs(0.0, 60, SEED + 7)
    rho, ci, _n = spearman_ci(x, y, boot=800)
    fooled = not (ci[0] <= 0.0 <= ci[1])
    print(f"  NULL trap: rho {rho:+.3f} CI [{ci[0]:+.3f},{ci[1]:+.3f}] "
          f"-> {'FAIL (called a signal)' if fooled else 'OK (spans 0)'}")
    ok &= not fooled
    # and the ratio trap the module refuses: prove it WOULD have fired
    rng = np.random.default_rng(SEED + 11)
    denom = rng.uniform(15, 30, 60)
    numer = rng.uniform(6, 14, 60)          # independent of denom
    ratio = numer / denom
    rho_trap, _ci, _n = spearman_ci(ratio, denom, boot=400)
    print(f"  ratio-vs-denominator trap: rho {rho_trap:+.3f} on INDEPENDENT "
          f"parts -> spurious, which is why it is refused")
    ok &= abs(rho_trap) > 0.3

    print("\n--- 2. data path: reproduce S118's published cells ---")
    want = {  # S118 table, '% of FLIGHT on soft gains'
        "kflight/k7150/lam0_default": 42.0,
        "kflight/duty047_k7150/lam0_default": 48.9,
        "kflight/k12000/lam0_default": 43.1,
        "kflight/b27_k7150/lam0_default": 50.7,
    }
    seen = 0
    for rel, target in want.items():
        d = os.path.join(RUNS, rel)
        paths = sorted(glob.glob(os.path.join(d, "run[0-9].csv")))
        if not paths:
            print(f"  SKIP {rel}: absent")
            continue
        vals = []
        for p in paths:
            try:
                vals.append(100.0 * floppy_flight.stats(p)
                            ["floppy_given_air"])
            except (Unfit, SystemExit):
                pass
        if not vals:
            print(f"  SKIP {rel}: all runs refused")
            continue
        got = float(np.median(vals))
        good = abs(got - target) <= 0.6
        seen += 1
        print(f"  {rel:42} got {got:5.1f}%  want {target:5.1f}%  "
              f"{'OK' if good else 'FAIL'}")
        ok &= good
    if not seen:
        print("  FAIL: reproduced nothing -- refusing to trust the loader")
        ok = False

    print("\n--- 3. new quantity: stance branch vs beta_tracking.py ---")
    d = os.path.join(RUNS, "kflight/k7150/lam0_default")
    checked = 0
    for p in sorted(glob.glob(os.path.join(d, "run[0-9].csv"))):
        a = beta_tracking.load(p)
        nz = np.flatnonzero(a[:, 5] > 1e-9)
        a2 = a[a[:, 0] >= a[nz[0], 0]]
        eL, eR = a2[:, 2], a2[:, 3]
        good = (np.abs(eL) < beta_tracking.UNWRAP_GUARD) & \
               (np.abs(eR) < beta_tracking.UNWRAP_GUARD)
        ref = float(np.sqrt(np.mean((0.5 * (eL + eR))
                                    [(a2[:, 4] > 0.5) & good] ** 2)))
        mine = tracking_rms(p)["stance_dbeta"]
        agree = abs(mine - ref) < 1e-12
        checked += 1
        print(f"  {os.path.basename(p)}: stance RMS mine {np.degrees(mine):.4f}"
              f" deg vs beta_tracking {np.degrees(ref):.4f} deg  "
              f"{'OK' if agree else 'FAIL'}")
        ok &= agree
    if not checked:
        print("  SKIP: cell absent")

    print("\nSELFTEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# --------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--csv", help="write per-run rows here (for the figure)")
    ap.add_argument("--root", default=RUNS)
    args = ap.parse_args()
    if args.selftest:
        sys.exit(selftest())

    found = cells(args.root)
    print(f"{len(found)} cells with >= {MIN_RUNS_PER_CELL} runs "
          f"under {args.root}\n")

    rows, cellmed, refused = [], [], {}
    for d, paths in found.items():
        rel = os.path.relpath(d, args.root)
        per = [score_run(p) for p in paths]
        for r in per:
            r["cell"] = rel
            for msg in r["refusals"]:
                refused.setdefault(msg.split(":")[0], 0)
                refused[msg.split(":")[0]] += 1
        rows.extend(per)
        med = {"cell": rel, "n": len(per)}
        for k in ("floppy_given_air", "air_frac", "honesty",
                  "flight_dbeta", "stance_dbeta", "tau_p995"):
            v = [r[k] for r in per if k in r and np.isfinite(r[k])]
            med[k] = float(np.median(v)) if v else float("nan")
            med[k + "_n"] = len(v)
        cellmed.append(med)

    print("per-metric run-level refusals:", refused or "none")

    fga = np.array([c["floppy_given_air"] for c in cellmed], float)
    ok = np.isfinite(fga)
    print(f"\ncells with a usable floppy figure: {int(ok.sum())} / "
          f"{len(cellmed)}")

    # ---------- P-F1
    q1, q2, q3 = np.percentile(fga[ok], [25, 50, 75])
    iqr = q3 - q1
    print("\n=== P-F1  floppy_given_air is near-constant (IQR <= 15 pts) ===")
    print(f"  median {q2:.1f}%   IQR {q1:.1f}-{q3:.1f} = {iqr:.1f} points"
          f"   full range {fga[ok].min():.1f}-{fga[ok].max():.1f}%")
    pf1 = iqr <= 15.0
    print(f"  P-F1 {'HOLDS' if pf1 else 'FALSIFIED'}")

    # ---------- P-F2
    print("\n=== P-F2  floppy does NOT predict flight tracking error "
          "(|rho| < 0.4) ===")
    fl = np.array([c["flight_dbeta"] for c in cellmed], float)
    rho2, ci2, n2 = spearman_ci(fga, fl)
    print(f"  rho {rho2:+.3f}  CI [{ci2[0]:+.3f},{ci2[1]:+.3f}]  cells {n2}"
          f"  -> {verdict(rho2, ci2, 0.4, False)}")
    pf2 = not (np.isfinite(rho2) and abs(rho2) >= 0.4
               and not (ci2[0] <= 0 <= ci2[1]))
    print(f"  P-F2 {'HOLDS' if pf2 else 'FALSIFIED -- promotes #17 toward RED'}")

    # ---------- P-F3, gated
    print("\n=== P-F3  floppy correlates POSITIVELY with tau p99.5 "
          "(rho >= +0.3) ===")
    if pf1:
        print("  NOT SCORED. P-F1 held, so there is no between-cell variance"
              "\n  for tau to co-vary with. Registered in S137 as a"
              " dependency,\n  and honoured here.")
    else:
        ta = np.array([c["tau_p995"] for c in cellmed], float)
        rho3, ci3, n3 = spearman_ci(fga, ta)
        print(f"  rho {rho3:+.3f}  CI [{ci3[0]:+.3f},{ci3[1]:+.3f}]  "
              f"cells {n3}  -> {verdict(rho3, ci3, 0.3, True)}")

    # context, not a prediction
    print("\n--- context (no prediction registered on these) ---")
    for k, lab in (("stance_dbeta", "stance tracking err"),
                   ("honesty", "gain honesty"),
                   ("tau_p995", "tau p99.5")):
        v = np.array([c[k] for c in cellmed], float)
        r, ci, n = spearman_ci(fga, v)
        print(f"  floppy vs {lab:20} rho {r:+.3f} "
              f"CI [{ci[0]:+.3f},{ci[1]:+.3f}] n={n}")

    print("\n--- REFUSED BY CONSTRUCTION (see S137) ---")
    print("  floppy_given_air vs air_frac : ratio against its own denominator")
    print("  floppy_frac       vs air_frac : floppy is a subset of air")

    if args.csv:
        import csv as _csv
        keys = ["cell", "path", "floppy_given_air", "air_frac", "honesty",
                "flight_dbeta", "stance_dbeta", "tau_p995"]
        with open(args.csv, "w", newline="") as fh:
            w = _csv.DictWriter(fh, fieldnames=keys, extrasaction="ignore")
            w.writeheader()
            for r in rows:
                w.writerow(r)
        print(f"\nper-run rows -> {args.csv}  ({len(rows)} runs)")


if __name__ == "__main__":
    main()
