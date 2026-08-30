#!/usr/bin/env python3
"""The lambda -> kappa map, per direction, drift-subtracted (log S212).

Merges the new cells (p20/m10/m15/m20) with the BANKED config-of-record
points and the banked drift, and prints the map every downstream calibration
should read from. Then scores the three claims S211 made about its shape:

    P-L-1  SHAPE     per-degree authority at 20 >= at 15 on the +1 side
                     (superlinear continues). Falsified if it falls back to
                     the 10-deg value or below.
    P-L-2  SYMMETRY  drift-subtracted |authority| ratio +1/-1 within
                     [0.8, 1.25] at each of 10 / 15 / 20.
    P-L-3  CLAMP     20 deg is not rate/clamp-limited: achieved median |gamma|
                     on the commanded legs >= 18 deg.

authority = |median kappa - drift|, where drift is the median kappa of the
uncambered, unturned cells (S211: -0.067, n = 15) and the sign convention is
that +1 turns with kappa < 0 (with the drift) and -1 with kappa > 0 (against).
Pirouettes (|kappa| > 1 or band v_fwd < 0.10) are excluded from the map and
COUNTED in the printout.

Usage:
    score_lambda_map.py --selftest
    score_lambda_map.py --base ~/corgi_runs/lambda_map
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aggregate_menger import run_kappa                          # noqa: E402
from check_menger import load_torque_csv, LEGS                  # noqa: E402
import speed_from_odom as sfo                                   # noqa: E402

R = os.path.expanduser("~/corgi_runs")
START = 12.0
BANKED = {  # (lam, dir) -> list of cell dirs, config of record only (S207)
    (10.0, +1): ["camber_lambda/lam10", "inner_outer/k0"],
    (15.0, +1): ["camber_lambda/lam15"],
    (13.0, -1): ["matched_turn/cam"],
    (17.7, -1): ["matched_turn_cal2/cam"],
}
DRIFT_CELLS = ["camber_lambda/lam0", "yaw_hold_n5/nofb", "yaw_gentle/nofb"]
NEW = {"p20": (20.0, +1), "m10": (10.0, -1), "m15": (15.0, -1), "m20": (20.0, -1)}


def cell_runs(d):
    """-> list of (kappa, v_fwd, gamma_med_deg) per run, pirouettes flagged by kappa/v."""
    out = []
    for tq in sorted(glob.glob(os.path.join(d, "run[0-9].csv"))):
        n = int(os.path.basename(tq)[3:-4]); od = os.path.join(d, "odom_run%d.csv" % n)
        if not os.path.exists(od):
            continue
        try:
            k, _ = run_kappa(od, tq, START, 1.0)
        except SystemExit:
            continue
        try:
            v = float(sfo.stats(od)["v_fwd"])
        except Exception:                                        # noqa: BLE001
            v = float("nan")
        try:
            t, c, g, _ = load_torque_csv(tq)
            m = t >= t[0] + START
            gm = float(np.median([np.median(np.abs(g[m & c[:, i], i])) for i in range(4) if (m & c[:, i]).sum() > 50]))
        except Exception:                                        # noqa: BLE001
            gm = float("nan")
        out.append((k, v, gm))
    return out


def coherent(r):
    return abs(r[0]) < 1.0 and not np.isnan(r[1]) and r[1] >= 0.10


def build_map(base):
    pts = {}
    for name, key in NEW.items():
        pts.setdefault(key, []).extend(cell_runs(os.path.join(base, name)))
    for key, dirs in BANKED.items():
        for d in dirs:
            pts.setdefault(key, []).extend(cell_runs(os.path.join(R, d)))
    drift = [r[0] for d in DRIFT_CELLS for r in cell_runs(os.path.join(R, d))]
    return pts, float(np.median(drift)) if drift else float("nan"), len(drift)


def summarise(pts, drift):
    rows = []
    for (lam, dr), runs in sorted(pts.items()):
        ok = [r for r in runs if coherent(r)]
        if not ok:
            rows.append({"lam": lam, "dir": dr, "n": 0, "excl": len(runs)}); continue
        med = float(np.median([r[0] for r in ok]))
        auth = abs(med - drift)
        rows.append({"lam": lam, "dir": dr, "n": len(ok), "excl": len(runs) - len(ok),
                     "kappa": med, "auth": auth, "per_deg": auth / lam,
                     "gamma": float(np.nanmedian([r[2] for r in ok]))})
    return rows


def score(rows, verbose=True):
    def get(lam, dr):
        return next((r for r in rows if r["lam"] == lam and r["dir"] == dr and r["n"]), None)
    out = {}
    if verbose:
        print("  %5s %4s %3s %5s %8s %9s %9s %8s" % ("lam", "dir", "n", "excl", "kappa", "auth", "per_deg", "|gamma|"))
        for r in rows:
            if r["n"]:
                print("  %5.1f %+4d %3d %5d %+8.3f %9.3f %9.4f %8.2f" %
                      (r["lam"], r["dir"], r["n"], r["excl"], r["kappa"], r["auth"], r["per_deg"], r["gamma"]))
            else:
                print("  %5.1f %+4d %3d %5d   (no coherent runs)" % (r["lam"], r["dir"], 0, r["excl"]))
        print()

    def v(name, ok, txt):
        print("  %-6s %-4s %s" % (name, "PASS" if ok else "FAIL", txt)); out[name] = ok

    p10, p15, p20 = get(10.0, 1), get(15.0, 1), get(20.0, 1)
    if p15 and p20:
        v("P-L-1", p20["per_deg"] >= p15["per_deg"],
          "+1 per-degree authority: 10 %.4f -> 15 %.4f -> 20 %.4f" %
          (p10["per_deg"] if p10 else float("nan"), p15["per_deg"], p20["per_deg"]))
    else:
        print("  P-L-1 UNSCORED (missing +1 at 15 or 20)")
    rat = []
    for lam in (10.0, 15.0, 20.0):
        a, b = get(lam, 1), get(lam, -1)
        if a and b and b["auth"] > 0:
            rat.append((lam, a["auth"] / b["auth"]))
    if rat:
        v("P-L-2", all(0.8 <= x <= 1.25 for _, x in rat),
          "drift-subtracted +1/-1 authority: " + "  ".join("%g deg %.2fx" % (l, x) for l, x in rat))
    else:
        print("  P-L-2 UNSCORED")
    g20 = [r for r in rows if r["lam"] == 20.0 and r["n"]]
    if g20:
        v("P-L-3", all(r["gamma"] >= 18.0 for r in g20),
          "achieved |gamma| at 20 deg: " + "  ".join("dir %+d %.2f" % (r["dir"], r["gamma"]) for r in g20))
    else:
        print("  P-L-3 UNSCORED")
    out["scored"] = bool(rat)
    return out


def selftest():
    ok = True

    def chk(name, good):
        nonlocal ok
        ok = ok and good
        print("  %s %s" % ("ok " if good else "FAIL", name))
    import io, contextlib
    drift = -0.067
    # a symmetric, superlinear map: authority a(lam) = 0.0012*lam^2
    pts = {}
    for lam in (10.0, 15.0, 20.0):
        a = 0.0012 * lam * lam
        pts[(lam, 1)] = [(-a + drift + e, 0.35, lam - 0.5) for e in (0.01, -0.01, 0.0)]
        pts[(lam, -1)] = [(a + drift + e, 0.35, lam - 0.5) for e in (0.01, -0.01)]
    rows = summarise(pts, drift)
    with contextlib.redirect_stdout(io.StringIO()):
        r = score(rows, verbose=False)
    chk("symmetric superlinear map passes P-L-1..3", r["P-L-1"] and r["P-L-2"] and r["P-L-3"])
    r20 = next(x for x in rows if x["lam"] == 20 and x["dir"] == 1)
    chk("drift subtracted: +1@20 authority 0.480 (got %.3f)" % r20["auth"], abs(r20["auth"] - 0.48) < 0.012)
    # an asymmetric map fails P-L-2 only
    pts2 = dict(pts); pts2[(15.0, -1)] = [(0.5 * 0.0012 * 225 + drift, 0.35, 14.5)] * 2
    with contextlib.redirect_stdout(io.StringIO()):
        r2 = score(summarise(pts2, drift), verbose=False)
    chk("2x asymmetry at 15 fails P-L-2 only", (not r2["P-L-2"]) and r2["P-L-1"] and r2["P-L-3"])
    # a clamp-limited 20 fails P-L-3; saturating per-degree fails P-L-1
    pts3 = dict(pts); pts3[(20.0, 1)] = [(-0.0012 * 225 + drift, 0.35, 16.0)] * 3   # same authority as 15, gamma 16
    with contextlib.redirect_stdout(io.StringIO()):
        r3 = score(summarise(pts3, drift), verbose=False)
    chk("saturated 20 deg fails P-L-1 and P-L-3", (not r3["P-L-1"]) and (not r3["P-L-3"]))
    # pirouettes excluded and counted
    pts4 = dict(pts); pts4[(10.0, 1)] = pts[(10.0, 1)] + [(-3.8, 0.007, 9.0)]
    rows4 = summarise(pts4, drift); r10 = next(x for x in rows4 if x["lam"] == 10 and x["dir"] == 1)
    chk("pirouette excluded from the map and counted (n=%d excl=%d)" % (r10["n"], r10["excl"]), r10["n"] == 3 and r10["excl"] == 1)
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", default=os.path.expanduser("~/corgi_runs/lambda_map"))
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        print("score_lambda_map.py selftest\n")
        g = selftest()
        print("\n  SELFTEST %s" % ("PASS" if g else "FAIL"))
        return 0 if g else 1
    pts, drift, nd = build_map(os.path.expanduser(a.base))
    print("lambda -> kappa map, config of record, drift-subtracted. drift = %+.4f (n=%d)\n" % (drift, nd))
    r = score(summarise(pts, drift))
    if r.get("scored"):
        print("\n  Scored. Every FAIL above STANDS (S126). The table IS the deliverable -- cite it, not a fit.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
