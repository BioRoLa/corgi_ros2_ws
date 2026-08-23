#!/usr/bin/env python3
"""Score the stance-peak dip campaign against the bars S210 registers.
Owns P-D-1..P-D-6. Combines, does not compute: abad_torque.stats (clip
fraction, tau p99.5, per leg), aggregate_menger.run_kappa, speed_from_odom.

THE QUESTION (S209). The ABAD ceiling is a hardware load fact, not a control
mechanism -- so the dip is THERMAL AND STRUCTURAL RELIEF WITH kappa HELD. Every
bar is a mean shift on a per-run median (the affordable kind, S196) with a
kappa guard. Cells: nodip (dip 0) / dip15 (0.15) / dip30 (0.30), lambda 15,
dir +1 (the loaded leg is A, S205), n = 5 each, interleaved.

    P-D-1  validity: S152 screen + certification (the sweep)     gates
    P-D-2  PRIMARY  leg-A stance clip fraction, median: dip30 <= 2%   (nodip 7-10%, S205)
    P-D-3           leg-A tau p99.5, median: dip30 <= 44.25 N.m      (nodip 55-59)
    P-D-4  GUARD    median |kappa| within +-15% of nodip, BOTH dip cells
    P-D-5  GUARD    median v_fwd within +-10% of nodip; collapses not more than nodip
    P-D-6  hardware leg-A stance time above 37 N.m (usable at speed): dip30 <= 0.5 x nodip

dip15 is reported on every bar and scored only on P-D-4/5 (it is the "does a
small dip already hold kappa" cell); dip30 is the scored treatment.

Usage:
    score_acker_dip.py --selftest
    score_acker_dip.py --base ~/corgi_runs/acker_dip
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from abad_torque import stats as abad_stats, START            # noqa: E402
from aggregate_menger import run_kappa                          # noqa: E402
import speed_from_odom as sfo                                   # noqa: E402

USABLE = 37.0
CLAMP = 44.25
LEG = "A"


def time_above(tq, leg, thr):
    """fraction of leg's stance samples with |tau_demand| > thr (band)."""
    rows = []
    with open(tq, newline="") as fh:
        for r in csv.DictReader(fh):
            if r.get("motor") == "ABAD" and r.get("leg") == leg:
                try:
                    rows.append((float(r["t"]), float(r["in_contact"]), abs(float(r["tau_demand"]))))
                except (ValueError, KeyError):
                    pass
    a = np.array(rows)
    if len(a) < 200:
        return float("nan")
    a = a[a[:, 0] >= a[0, 0] + START]
    st = a[a[:, 1] > 0.5]
    return float(np.mean(st[:, 2] > thr)) if len(st) else float("nan")


def cell_rows(d):
    out = []
    for tq in sorted(glob.glob(os.path.join(d, "run[0-9].csv"))):
        n = int(os.path.basename(tq)[3:-4])
        od = os.path.join(d, "odom_run%d.csv" % n)
        s = abad_stats(tq)
        if not s or LEG not in s["per_leg"]:
            continue
        try:
            kap, _ = run_kappa(od, tq, START, 1.0)
        except SystemExit:
            kap = float("nan")
        try:
            v = float(sfo.stats(od)["v_fwd"])
        except Exception:                                          # noqa: BLE001
            v = float("nan")
        out.append({"run": n, "clip": s["per_leg"][LEG]["clip_pct"],
                    "tau": s["per_leg"][LEG]["d_p995"], "kappa": kap, "v": v,
                    "above37": 100.0 * time_above(tq, LEG, USABLE),
                    "collapsed": (not np.isnan(v)) and v < 0.10})
    return out


def med(rows, k):
    return float(np.median([r[k] for r in rows]))


def score(cells, verbose=True):
    nd, d15, d30 = cells.get("nodip", []), cells.get("dip15", []), cells.get("dip30", [])
    out = {}
    if verbose:
        for name, rows in (("nodip", nd), ("dip15", d15), ("dip30", d30)):
            print("  == %s (n=%d) ==" % (name, len(rows)))
            for r in rows:
                print("    run%d  clipA %5.2f%%  tauA %6.2f  >37: %5.1f%%  |kappa| %.3f  v %+.3f%s"
                      % (r["run"], r["clip"], r["tau"], r["above37"], abs(r["kappa"]), r["v"],
                         "  COLLAPSED" if r["collapsed"] else ""))
        print()
    if len(nd) < 3 or len(d30) < 3:
        print("  UNSCORED: fewer than 3 runs in nodip or dip30."); return {"scored": False}

    def v(name, ok, txt):
        print("  %-6s %-4s %s" % (name, "PASS" if ok else "FAIL", txt)); out[name] = ok

    v("P-D-2", med(d30, "clip") <= 2.0,
      "leg-A clip median: nodip %.2f%% -> dip15 %s -> dip30 %.2f%%  (bar <= 2%%)"
      % (med(nd, "clip"), ("%.2f%%" % med(d15, "clip")) if d15 else "--", med(d30, "clip")))
    v("P-D-3", med(d30, "tau") <= CLAMP,
      "leg-A tau p99.5 median: nodip %.2f -> dip15 %s -> dip30 %.2f  (bar <= %.2f)"
      % (med(nd, "tau"), ("%.2f" % med(d15, "tau")) if d15 else "--", med(d30, "tau"), CLAMP))
    k0 = abs(med(nd, "kappa"))
    ok4 = all(abs(abs(med(c, "kappa")) - k0) / k0 <= 0.15 for c in (d15, d30) if c)
    v("P-D-4", ok4, "|kappa| median: nodip %.3f -> dip15 %s -> dip30 %.3f  (guard +-15%%)"
      % (k0, ("%.3f" % abs(med(d15, "kappa"))) if d15 else "--", abs(med(d30, "kappa"))))
    v0 = med(nd, "v")
    ok5 = all(abs(med(c, "v") - v0) / abs(v0) <= 0.10 for c in (d15, d30) if c) and \
        all(sum(r["collapsed"] for r in c) <= sum(r["collapsed"] for r in nd) for c in (d15, d30) if c)
    v("P-D-5", ok5, "v_fwd median: nodip %+.3f -> dip15 %s -> dip30 %+.3f; collapses %d/%s/%d  (guard +-10%%, not more collapses)"
      % (v0, ("%+.3f" % med(d15, "v")) if d15 else "--", med(d30, "v"),
         sum(r["collapsed"] for r in nd), str(sum(r["collapsed"] for r in d15)) if d15 else "-",
         sum(r["collapsed"] for r in d30)))
    a0 = med(nd, "above37")
    v("P-D-6", med(d30, "above37") <= 0.5 * a0,
      "leg-A stance time > 37 N.m: nodip %.1f%% -> dip30 %.1f%%  (bar <= half)" % (a0, med(d30, "above37")))
    out["scored"] = True
    return out


def selftest():
    ok = True

    def chk(name, good):
        nonlocal ok
        ok = ok and good
        print("  %s %s" % ("ok " if good else "FAIL", name))

    import io, contextlib

    def mk(clips, taus, ks, vs, ab):
        return [{"run": i + 1, "clip": c, "tau": t, "kappa": k, "v": v, "above37": a,
                 "collapsed": v < 0.10} for i, (c, t, k, v, a) in enumerate(zip(clips, taus, ks, vs, ab))]
    nd = mk([8, 9, 7, 10, 8], [57, 58, 55, 59, 56], [.44, .46, .43, .45, .44], [.35] * 5, [30, 32, 28, 31, 30])
    d15 = mk([4, 3, 5, 4, 3], [48, 47, 49, 46, 48], [.43, .44, .42, .45, .43], [.34] * 5, [18, 17, 19, 18, 17])
    d30 = mk([1, 2, 1, 0, 1], [41, 43, 40, 42, 41], [.41, .42, .40, .43, .41], [.34] * 5, [10, 12, 9, 11, 10])
    with contextlib.redirect_stdout(io.StringIO()):
        r = score({"nodip": nd, "dip15": d15, "dip30": d30}, verbose=False)
    chk("a textbook relief passes P-D-2..6", r["scored"] and all(r[k] for k in ("P-D-2", "P-D-3", "P-D-4", "P-D-5", "P-D-6")))
    lost = mk([1] * 5, [41] * 5, [.30] * 5, [.34] * 5, [10] * 5)   # kappa down 32%
    with contextlib.redirect_stdout(io.StringIO()):
        r2 = score({"nodip": nd, "dip15": d15, "dip30": lost}, verbose=False)
    chk("a dip that loses the turn fails P-D-4 only", r2["scored"] and not r2["P-D-4"] and r2["P-D-2"] and r2["P-D-6"])
    coll = mk([1] * 5, [41] * 5, [.42] * 5, [.34, .34, 0.0, .34, .34], [10] * 5)
    with contextlib.redirect_stdout(io.StringIO()):
        r3 = score({"nodip": nd, "dip15": d15, "dip30": coll}, verbose=False)
    chk("a dip that adds a collapse fails P-D-5", r3["scored"] and not r3["P-D-5"])
    with contextlib.redirect_stdout(io.StringIO()):
        r4 = score({"nodip": nd[:2], "dip30": d30}, verbose=False)
    chk("fewer than 3 control runs -> unscored", not r4["scored"])
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", default=os.path.expanduser("~/corgi_runs/acker_dip"))
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        print("score_acker_dip.py selftest\n")
        g = selftest()
        print("\n  SELFTEST %s" % ("PASS" if g else "FAIL"))
        return 0 if g else 1
    base = os.path.expanduser(a.base)
    cells = {c: cell_rows(os.path.join(base, c)) for c in ("nodip", "dip15", "dip30")}
    print("stance-peak dip campaign -- S210 bars, scored as registered. Leg %s (loaded, dir +1).\n" % LEG)
    r = score(cells)
    if r.get("scored"):
        print("\n  Scored. Every FAIL above STANDS (S126).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
