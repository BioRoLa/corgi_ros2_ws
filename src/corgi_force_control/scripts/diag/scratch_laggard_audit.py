#!/usr/bin/env python3
"""Audit the min vLeg laggard claim: per-run, per-leg, for off/on/both."""
import os
import sys
import glob

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import leg_demand as ld
from touchdown_phase import Unfit, TAIL_S

BASE = os.path.expanduser("~/corgi_runs/clock_ff")
CELLS = ["off", "on", "both"]

print("=" * 78)
print("PER-RUN, PER-LEG v_demand medians (m/s), tail %.0f s, L=%.4f"
      % (TAIL_S, ld.LEG_LENGTH_M))
print("=" * 78)

cellinfo = {}
for cell in CELLS:
    d = os.path.join(BASE, cell)
    runs = sorted(glob.glob(os.path.join(d, "run[0-9].csv")))
    print("\n[%s]" % cell)
    print("  %-8s %8s %8s %8s %8s   %-6s %9s %8s"
          % ("run", "A", "B", "C", "D", "lagleg", "min", "spread"))
    per_run = []
    for p in runs:
        try:
            s = ld.run_stats(p)
        except Unfit as e:
            print("  %-8s UNFIT %s" % (os.path.basename(p), e))
            continue
        bl = s["by_leg"]
        lag = min(bl, key=lambda k: bl[k])
        print("  %-8s %8.3f %8.3f %8.3f %8.3f   %-6s %9.3f %8.3f"
              % (os.path.basename(p),
                 bl.get("A", float("nan")), bl.get("B", float("nan")),
                 bl.get("C", float("nan")), bl.get("D", float("nan")),
                 lag, s["min_vleg"], s["spread"]))
        per_run.append((os.path.basename(p), bl, s))
    cellinfo[cell] = per_run
    # cell aggregation exactly as leg_demand.cell_stats does
    legs = sorted({l for _n, bl, _s in per_run for l in bl})
    by_leg = {l: float(np.median([bl[l] for _n, bl, _s in per_run if l in bl]))
              for l in legs}
    minv = float(np.median([s["min_vleg"] for _n, _bl, s in per_run]))
    spread = float(np.median([s["spread"] for _n, _bl, s in per_run]))
    print("  %-8s %8.3f %8.3f %8.3f %8.3f   %-6s %9.3f %8.3f   <-- CELL"
          % ("median", by_leg.get("A", float("nan")), by_leg.get("B", float("nan")),
             by_leg.get("C", float("nan")), by_leg.get("D", float("nan")),
             min(by_leg, key=lambda k: by_leg[k]), minv, spread))
    print("  min over the four CELL leg medians = %.3f ; median of per-run "
          "minima = %.3f" % (min(by_leg.values()), minv))
    cellinfo[cell + "_agg"] = (by_leg, minv, spread)

print()
print("=" * 78)
print("THE CHALLENGE'S CEILING, checked per run")
print("=" * 78)
off = cellinfo["off"]
on = cellinfo["on"]
for name, runs in (("off", off), ("on", on)):
    for n, bl, s in runs:
        srt = sorted(bl.items(), key=lambda kv: kv[1])
        print("  %-5s %-9s sorted: %s" % (name, n,
              "  ".join("%s=%.3f" % (k, v) for k, v in srt)))

print()
print("Episode counts per leg per run (does a 'flat' leg have enough data?)")
for cell in CELLS:
    for n, bl, s in cellinfo[cell]:
        print("  %-5s %-9s %s" % (cell, n, s["n_by_leg"]))
