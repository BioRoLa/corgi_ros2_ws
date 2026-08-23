#!/usr/bin/env python3
"""Does the ABAD torque clip cost anything at the STANCE PEAK? The test S206
named, for the narrowed candidate: "a per-leg torque ceiling at stance peak
limits peak lateral force on the loaded side, and which side flips with dir."

WHY THIS AND NOT MEDIAN GAMMA. S206 killed the median-angle route: achieved
gamma is 95-97% of commanded whether or not the leg clips. But the clip is a
7-10% slice of stance concentrated where demand is highest, and a held median
says nothing about the moment of peak load. The CSV carries pos_error per
ABAD row, so the angle error AT THE LOAD PEAK is directly readable.

PER RUN, PER LEG. Each contact interval is one stance event. Within it the
load peak is the sample of max |tau_demand|. Recorded at that sample (and the
max within +-20 ms of it):
    pe_peak      |pos_error| at the load peak, deg      (median over events)
    pe_free      |pos_error| median over the event's non-clipped samples,
                 i.e. the baseline tracking error away from the ceiling
    pe_ratio     pe_peak / pe_free -- how much worse the peak is than the rest
    clip_at_peak fraction of events whose peak sample is clipped
                 (|tau_demand| > |tau_applied| + 0.01)

PREDICTION, written before the data (S126). On S202's cam cell (dir -1, so
the LOADED leg is B, S206): if the ceiling costs lateral force, then
  P-E-1  pe_peak on B exceeds pe_peak on A (the unloaded front leg) by >= 2x,
         in >= 6 of 8 runs;
  P-E-2  across the 8 cam runs, pe_peak on B correlates NEGATIVELY with |kappa|
         (Spearman <= -0.5): the runs that lose more angle at the peak turn
         less. The unloaded leg A shows no such correlation (|rho| < 0.3).
  P-E-3  on the diff cell (no camber), pe_peak is similar on A and B (ratio
         within [0.5, 2]) -- the asymmetry is the camber pattern's, not the
         plant's.
P-E-2 is the one that matters. n = 8 on a correlation is weak; a clean
|rho| >= 0.5 earns a registration, it does not settle anything.

Usage:
    abad_peak_error.py --selftest
    abad_peak_error.py --base ~/corgi_runs/matched_turn_cal2
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aggregate_menger import run_kappa                         # noqa: E402

START = 12.0
WIN = 0.020


def load_abad(path):
    out = {}
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            if row.get("motor") != "ABAD":
                continue
            try:
                out.setdefault(row["leg"], []).append(
                    (float(row["t"]), float(row["in_contact"]), float(row["tau_demand"]),
                     float(row["tau_applied"]), float(row["pos_error"])))
            except (ValueError, KeyError):
                continue
    return {k: np.array(v) for k, v in out.items() if v}


def stance_events(t, contact):
    """-> list of (i0, i1) index ranges where contact is on."""
    on = contact > 0.5
    d = np.diff(on.astype(int))
    starts = list(np.where(d == 1)[0] + 1)
    ends = list(np.where(d == -1)[0] + 1)
    if on[0]:
        starts.insert(0, 0)
    if on[-1]:
        ends.append(len(on))
    return [(a, b) for a, b in zip(starts, ends) if b - a >= 5]


def leg_metrics(a):
    """a: rows (t, contact, demand, applied, pos_error) for one leg."""
    t0 = a[0, 0]
    a = a[a[:, 0] >= t0 + START]
    if len(a) < 200:
        return None
    t, c, d, ap, pe = a.T
    d, ap, pe = np.abs(d), np.abs(ap), np.degrees(np.abs(pe))
    clipped = d > ap + 0.01
    peaks, frees, clip_pk, peak_win = [], [], [], []
    for i0, i1 in stance_events(t, c):
        seg = slice(i0, i1)
        k = i0 + int(np.argmax(d[seg]))
        peaks.append(pe[k])
        w = (t >= t[k] - WIN) & (t <= t[k] + WIN)
        peak_win.append(float(pe[w].max()) if w.any() else pe[k])
        free = pe[seg][~clipped[seg]]
        frees.append(float(np.median(free)) if len(free) else np.nan)
        clip_pk.append(bool(clipped[k]))
    if len(peaks) < 8:
        return None
    pk, fr = float(np.median(peaks)), float(np.nanmedian(frees))
    return {"pe_peak": pk, "pe_peak_win": float(np.median(peak_win)), "pe_free": fr,
            "pe_ratio": pk / fr if fr > 0 else float("nan"),
            "clip_at_peak": float(np.mean(clip_pk)), "n_events": len(peaks)}


def run_metrics(tq):
    legs = load_abad(tq)
    return {leg: leg_metrics(a) for leg, a in sorted(legs.items())}


def spearman(x, y):
    x, y = np.asarray(x, float), np.asarray(y, float)
    ok = ~(np.isnan(x) | np.isnan(y))
    x, y = x[ok], y[ok]
    if len(x) < 4:
        return float("nan")
    rx, ry = np.argsort(np.argsort(x)), np.argsort(np.argsort(y))
    return float(np.corrcoef(rx, ry)[0, 1])


def analyse(base, loaded="B", unloaded="A"):
    print("ABAD position error AT THE STANCE-LOAD PEAK, per leg, deg. Band [t0+%.0f, end]."
          % START)
    print("loaded leg (dir -1, S206): %s   unloaded front leg: %s\n" % (loaded, unloaded))
    table = {}
    for cell in ("cam", "diff"):
        d = os.path.join(base, cell)
        rows = []
        for tq in sorted(glob.glob(os.path.join(d, "run[0-9].csv"))):
            n = os.path.basename(tq)[3:-4]
            m = run_metrics(tq)
            od = os.path.join(d, "odom_run%s.csv" % n)
            try:
                kap, _ = run_kappa(od, tq, START, 1.0)
            except SystemExit:
                kap = float("nan")
            rows.append((n, m, kap))
        table[cell] = rows
        print("  == %s ==" % cell)
        print("  %3s %7s | %s" % ("run", "|kappa|", "  ".join(
            "%s: pe_peak pe_free ratio clip@pk" % leg for leg in "ABCD")))
        for n, m, kap in rows:
            cells = []
            for leg in "ABCD":
                v = m.get(leg)
                cells.append("%s: %6.3f %6.3f %5.2f %5.0f%%" %
                             (leg, v["pe_peak"], v["pe_free"], v["pe_ratio"], 100 * v["clip_at_peak"])
                             if v else "%s:   --" % leg)
            print("  %3s %7.3f | %s" % (n, abs(kap) if not np.isnan(kap) else float("nan"), "  ".join(cells)))
        print()

    cam = [(n, m, k) for n, m, k in table.get("cam", []) if m.get(loaded) and m.get(unloaded)]
    dif = [(n, m, k) for n, m, k in table.get("diff", []) if m.get(loaded) and m.get(unloaded)]
    if len(cam) < 4:
        print("  too few cam runs to score."); return
    print("-- P-E-1: loaded-leg peak error >= 2x the unloaded leg, in >= 6 of 8 cam runs --")
    wins = sum(m[loaded]["pe_peak"] >= 2.0 * m[unloaded]["pe_peak"] for _, m, _ in cam)
    print("  %d of %d runs   (median %s %.3f deg vs %s %.3f deg)   -> %s" %
          (wins, len(cam), loaded, np.median([m[loaded]["pe_peak"] for _, m, _ in cam]),
           unloaded, np.median([m[unloaded]["pe_peak"] for _, m, _ in cam]),
           "PASS" if wins >= 6 else "FAIL"))
    print("-- P-E-2: loaded-leg peak error vs |kappa| across cam runs: Spearman <= -0.5 --")
    ks = [abs(k) for _, _, k in cam]
    rl = spearman([m[loaded]["pe_peak"] for _, m, _ in cam], ks)
    ru = spearman([m[unloaded]["pe_peak"] for _, m, _ in cam], ks)
    print("  loaded %s: rho %+.3f   unloaded %s: rho %+.3f  (n=%d)   -> %s" %
          (loaded, rl, unloaded, ru, len(cam),
           "PASS" if (rl <= -0.5 and abs(ru) < 0.3) else "FAIL"))
    if dif:
        print("-- P-E-3: on diff (no camber), A and B peak error within 2x of each other --")
        r = np.median([m[loaded]["pe_peak"] for _, m, _ in dif]) / \
            max(np.median([m[unloaded]["pe_peak"] for _, m, _ in dif]), 1e-9)
        print("  median ratio %s/%s = %.2f   -> %s" % (loaded, unloaded, r,
              "PASS" if 0.5 <= r <= 2.0 else "FAIL"))
    print("\n  n = 8 on a correlation is weak. A clean P-E-2 earns a registration, not a verdict.")


def selftest():
    ok = True

    def chk(name, good):
        nonlocal ok
        ok = ok and good
        print("  %s %s" % ("ok " if good else "FAIL", name))

    # synthetic leg: 120 stance events of 160 ms (31 s, so ~74 survive the settle) at 1 kHz, triangular demand
    # peaking at 50 in the middle; pos_error 0.5 deg baseline with a 3 deg
    # bump exactly at the peak; clamp at 44.25 so the peak samples clip.
    dt, rows, t = 0.001, [], 0.0
    for ev in range(120):
        for k in range(160):
            tri = 50.0 * (1 - abs(k - 80) / 80.0)
            pe = np.radians(0.5 + (3.0 if abs(k - 80) <= 3 else 0.0))
            rows.append((t, 1.0, tri, min(tri, 44.25), pe)); t += dt
        for k in range(100):
            rows.append((t, 0.0, 5.0, 5.0, np.radians(0.2))); t += dt
    a = np.array(rows)
    m = leg_metrics(a)
    chk("peak error read as 3.5 deg (got %.2f)" % m["pe_peak"], abs(m["pe_peak"] - 3.5) < 0.05)
    chk("free (unclipped) error read as 0.5 deg (got %.2f)" % m["pe_free"], abs(m["pe_free"] - 0.5) < 0.05)
    chk("ratio ~7 (got %.1f)" % m["pe_ratio"], 6.5 < m["pe_ratio"] < 7.5)
    chk("every event's peak sample is clipped (got %.0f%%)" % (100 * m["clip_at_peak"]),
        m["clip_at_peak"] == 1.0)
    chk("only events after the 12 s settle counted (got %d of 120; expect ~74)" % m["n_events"],
        65 < m["n_events"] < 80)
    # control leg: same demand, flat 0.5 deg error -> ratio 1
    rows2 = [(r[0], r[1], r[2], r[3], np.radians(0.5) if r[1] > 0.5 else r[4]) for r in rows]
    m2 = leg_metrics(np.array(rows2))
    chk("flat-error control has ratio 1.0 (got %.2f)" % m2["pe_ratio"], abs(m2["pe_ratio"] - 1.0) < 0.05)
    chk("spearman sanity", abs(spearman([1, 2, 3, 4, 5], [5, 4, 3, 2, 1]) + 1) < 1e-9)
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", default=os.path.expanduser("~/corgi_runs/matched_turn_cal2"))
    ap.add_argument("--loaded", default="B")
    ap.add_argument("--unloaded", default="A")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        print("abad_peak_error.py selftest\n")
        g = selftest()
        print("\n  SELFTEST %s" % ("PASS" if g else "FAIL"))
        return 0 if g else 1
    analyse(os.path.expanduser(a.base), a.loaded, a.unloaded)
    return 0


if __name__ == "__main__":
    sys.exit(main())
