#!/usr/bin/env python3
"""Score the matched-curvature campaign against the bars S202 registered.
Owns P-M-0..P-M-6. Nothing here is computed twice: every per-run quantity
comes from cross_track.run_row (itself a thin wrapper over check_turn,
aggregate_menger, speed_from_odom, tau_demand_window) and the permutation test
is matched_kappa.sd_ratio_permutation.

COHERENT = arc >= 90 deg AND band v_fwd >= 0.10 m/s (S191's collapse
threshold). Incoherent runs are NOT dropped -- they are the point of P-M-2 and
they stay in P-M-3's all-runs sd, which is the repeatability a user of the
robot actually experiences. They are excluded only from the bars that measure
the quality of a turn that happened (P-M-3b, P-M-4, P-M-5, P-M-6).

A run with no odom, or one quarantined as uncertified by the sweep, is absent
from every bar and its absence is printed.

Usage:
    score_matched_turn.py --selftest
    score_matched_turn.py --base ~/corgi_runs/matched_turn [--calibrate]
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cross_track import run_row                              # noqa: E402
from matched_kappa import sd_ratio_permutation               # noqa: E402

COLLAPSE_VFWD, MIN_ARC = 0.10, 90.0
MATCH_TOL = 0.20          # P-M-1: +-20% on median |kappa|
NPER = 8


def coherent(r):
    return (r["arc"] >= MIN_ARC and not math.isnan(r["v_fwd"])
            and r["v_fwd"] >= COLLAPSE_VFWD and not math.isnan(r["kappa"]))


def load(base):
    cells = {}
    for name in ("cam", "diff"):
        d = os.path.join(base, name)
        rows = [r for r in (run_row(d, n) for n in range(1, NPER + 2)) if r]
        cells[name] = rows
    return cells


def verdict(name, ok, detail):
    print("  %-6s %-4s  %s" % (name, "PASS" if ok else "FAIL", detail))
    return ok


def score(cells, verbose=True):
    cam, dif = cells["cam"], cells["diff"]
    ccam, cdif = [r for r in cam if coherent(r)], [r for r in dif if coherent(r)]
    out = {}
    if verbose:
        print("runs found: cam %d (coherent %d), diff %d (coherent %d)\n"
              % (len(cam), len(ccam), len(dif), len(cdif)))
        for name, rows in (("cam", cam), ("diff", dif)):
            for r in rows:
                print("  %-4s run%d  kappa %+.3f  arc %4.0f  v %+.3f  rms %5.1f mm"
                      "  tau %6.2f  %s" % (name, r["run"], r["kappa"], r["arc"],
                                           r["v_fwd"], r["rms_mm"], r["tau"],
                                           "coherent" if coherent(r) else "INCOHERENT"))
        print()

    if len(ccam) < 3 or len(cdif) < 2:
        print("  UNSCORED: fewer than 3 coherent cam or 2 coherent diff runs.")
        return {"scored": False}

    kc = np.array([r["kappa"] for r in cam])
    kd = np.array([r["kappa"] for r in dif])
    mc = float(np.median([abs(r["kappa"]) for r in ccam]))
    md = float(np.median([abs(r["kappa"]) for r in cdif]))
    sc = float(np.sign(np.median([r["kappa"] for r in ccam])))
    sd = float(np.sign(np.median([r["kappa"] for r in cdif])))

    print("-- P-M-1 MATCHING (gates everything) --")
    mism = abs(mc - md) / md
    out["P-M-1"] = verdict("P-M-1", mism <= MATCH_TOL,
                           "median |kappa| coherent: cam %.3f vs diff %.3f -> %.0f%% apart"
                           % (mc, md, 100 * mism))
    if not out["P-M-1"]:
        print("  -> comparison UNSCORED. The arms are not at the same curvature.")
        return {"scored": False, **out}

    print("-- P-M-0 SIGN --")
    out["P-M-0"] = verdict("P-M-0", sc == sd, "cam %+d, diff %+d" % (sc, sd))

    print("-- P-M-2 COHERENT-TURN FRACTION (descriptive, NOT powered) --")
    try:
        from scipy import stats
        p = stats.fisher_exact([[len(ccam), len(cam) - len(ccam)],
                                [len(cdif), len(dif) - len(cdif)]])[1]
        ptxt = "Fisher p=%.3f" % p
    except ImportError:
        ptxt = "(scipy absent)"
    out["P-M-2"] = verdict("P-M-2", len(ccam) >= 7 and len(cdif) <= 5,
                           "cam %d/%d, diff %d/%d  %s" %
                           (len(ccam), len(cam), len(cdif), len(dif), ptxt))

    print("-- P-M-3 REPEATABILITY, all runs, failures included (powered) --")
    s_c, s_d = float(np.std(kc, ddof=1)), float(np.std(kd, ddof=1))
    ratio = s_d / s_c if s_c > 0 else float("inf")
    obs, pp, npart = sd_ratio_permutation(kc, kd)
    out["P-M-3"] = verdict("P-M-3", ratio >= 3.0,
                           "sd(kappa) diff %.3f / cam %.3f = %.2fx  (permutation p=%.3f, %d partitions)"
                           % (s_d, s_c, ratio, pp, npart))

    print("-- P-M-3b REGISTERED NULL: coherent runs only, ratio in [0.5, 2] --")
    kcc = np.array([r["kappa"] for r in ccam])
    kdc = np.array([r["kappa"] for r in cdif])
    rc = (float(np.std(kdc, ddof=1)) / float(np.std(kcc, ddof=1))
          if len(kdc) > 1 and np.std(kcc, ddof=1) > 0 else float("nan"))
    out["P-M-3b"] = verdict("P-M-3b", 0.5 <= rc <= 2.0,
                            "sd coherent diff/cam = %.2fx" % rc)

    def med(rows, k):
        return float(np.median([r[k] for r in rows]))

    print("-- P-M-4 CROSS-TRACK, coherent: cam <= 1.25 x diff --")
    a, b = med(ccam, "rms_mm"), med(cdif, "rms_mm")
    out["P-M-4"] = verdict("P-M-4", a <= 1.25 * b,
                           "median RMS cam %.1f vs diff %.1f mm (%.2fx)" % (a, b, a / b))
    print("-- P-M-5 PEAK TORQUE, coherent: cam <= 0.90 x diff --")
    a, b = med(ccam, "tau"), med(cdif, "tau")
    out["P-M-5"] = verdict("P-M-5", a <= 0.90 * b,
                           "median tau p99.5 cam %.2f vs diff %.2f (%.2fx)" % (a, b, a / b))
    print("-- P-M-6 SPEED, coherent: cam >= diff --")
    a, b = med(ccam, "v_fwd"), med(cdif, "v_fwd")
    out["P-M-6"] = verdict("P-M-6", a >= b,
                           "median v_fwd cam %+.3f vs diff %+.3f" % (a, b))
    out["scored"] = True
    return out


def calibrate(cells):
    """Rep-1 matching check for the sweep's pre-registered rescale rule."""
    c = [r for r in cells["cam"] if r["run"] == 1]
    d = [r for r in cells["diff"] if r["run"] == 1]
    if not c or not d or not coherent(c[0]) or not coherent(d[0]):
        print("CALIBRATION: INCONCLUSIVE (rep-1 run missing or incoherent)")
        return
    kc, kd = abs(c[0]["kappa"]), abs(d[0]["kappa"])
    mism = abs(kc - kd) / kd
    print("rep 1: cam |kappa| %.3f  diff |kappa| %.3f  -> %.0f%% apart" % (kc, kd, 100 * mism))
    if mism <= MATCH_TOL:
        print("CALIBRATION: OK")
    else:
        lam = float(os.environ.get("CAM_LAM_DEG", "13"))
        print("CALIBRATION: RESCALE -- new lambda = %.1f" % (lam * kd / kc))


def selftest():
    ok = True

    def chk(name, good):
        nonlocal ok
        ok = ok and good
        print("  %s %s" % ("ok " if good else "FAIL", name))

    rng = np.random.RandomState(1)

    def mk(kappas, arcs, vs, rms, tau):
        return [{"run": i + 1, "kappa": k, "arc": a, "v_fwd": v, "rms_mm": r,
                 "tau": t, "R_fit": 1 / abs(k) if k else 9, "R_menger": 1 / abs(k) if k else 9}
                for i, (k, a, v, r, t) in enumerate(zip(kappas, arcs, vs, rms, tau))]

    # 1. a textbook S199/S129-shaped campaign must PASS every bar
    cam = mk(0.33 + rng.normal(0, 0.02, 8), [120] * 8, [0.35] * 8, [60] * 8, [46] * 8)
    # S129's shape: 2-3 failures in 5 -> 3 in 8 here (a pirouette, a straight
    # backwards run, and a backwards curl), 5 coherent.
    dif = mk([0.31, 0.34, 0.29, 0.32, 0.30, -0.01, 0.02, 0.33],
             [110, 105, 100, 115, 108, 150, 40, 112],
             [0.30, 0.29, 0.31, 0.28, 0.30, -0.003, -0.19, -0.14],
             [50] * 8, [54] * 8)
    import io, contextlib
    with contextlib.redirect_stdout(io.StringIO()):
        r = score({"cam": cam, "diff": dif}, verbose=False)
    chk("textbook campaign scores and passes P-M-1,0,2,3,4,5,6",
        r["scored"] and all(r[k] for k in ("P-M-1", "P-M-0", "P-M-2", "P-M-3",
                                           "P-M-4", "P-M-5", "P-M-6")))
    # 2. mismatched curvature must gate out (P-M-1 fails, nothing else scored)
    cam2 = mk(0.50 + rng.normal(0, 0.02, 8), [120] * 8, [0.35] * 8, [60] * 8, [46] * 8)
    with contextlib.redirect_stdout(io.StringIO()):
        r2 = score({"cam": cam2, "diff": dif}, verbose=False)
    chk("mismatched kappa -> P-M-1 FAIL and unscored", not r2["scored"] and not r2["P-M-1"])
    # 3. opposite-sign turn must fail P-M-0
    cam3 = mk(-(0.33 + rng.normal(0, 0.02, 8)), [120] * 8, [0.35] * 8, [60] * 8, [46] * 8)
    with contextlib.redirect_stdout(io.StringIO()):
        r3 = score({"cam": cam3, "diff": dif}, verbose=False)
    chk("opposite sign -> P-M-0 FAIL", r3["scored"] and not r3["P-M-0"])
    # 4. coherence rule: an arc of 80 deg or v_fwd 0.05 is INCOHERENT
    chk("arc 80 deg is incoherent", not coherent(mk([0.3], [80], [0.3], [50], [50])[0]))
    chk("v_fwd 0.05 is incoherent", not coherent(mk([0.3], [120], [0.05], [50], [50])[0]))
    # 5. the registered null (P-M-3b) must FAIL when coherent diff is 3x worse
    dif5 = mk([0.31, 0.40, 0.22, 0.38, 0.25, 0.34, 0.28, 0.36],
              [110] * 8, [0.30] * 8, [50] * 8, [54] * 8)
    with contextlib.redirect_stdout(io.StringIO()):
        r5 = score({"cam": cam, "diff": dif5}, verbose=False)
    chk("3x worse coherent precision -> P-M-3b FAIL (null falsified)",
        r5["scored"] and not r5["P-M-3b"])
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", default=os.path.expanduser("~/corgi_runs/matched_turn"))
    ap.add_argument("--calibrate", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        print("score_matched_turn.py selftest\n")
        good = selftest()
        print("\n  SELFTEST %s" % ("PASS" if good else "FAIL"))
        return 0 if good else 1
    cells = load(os.path.expanduser(a.base))
    if a.calibrate:
        calibrate(cells)
        return 0
    print("matched-curvature campaign -- S202 bars, scored as registered\n")
    r = score(cells)
    print()
    if r.get("scored"):
        print("  Scored. Every FAIL above STANDS (S126). Record them as they are.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
