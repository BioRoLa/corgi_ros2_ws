"""Scratch: WHERE in the cycle does the p99.5 torque saving live?

Clock phase from the logged kp (validated: duty 0.408 == template 108/265).
Per-motor |tau_demand|, the same quantity tau_demand_window.py scores.

Read-only scratch. 2026-08-22 adversarial review.
"""
import csv
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_menger import debounce, DEBOUNCE  # noqa: E402

TAIL_S = 20.0


def load(path):
    """-> per (leg,motor) arrays plus the leg's contact and kp."""
    rows = []
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            if r["motor"] not in ("L_Motor", "R_Motor"):
                continue
            try:
                rows.append((float(r["t"]), r["leg"], r["motor"],
                             int(r["in_contact"]), float(r["tau_demand"]),
                             float(r["t_ff"]), float(r["t_stiff"]),
                             float(r["kp"])))
            except (ValueError, KeyError):
                continue
    return rows


def main():
    base = os.path.expanduser("~/corgi_runs/clock_ff")
    cells = ["off", "on", "both"]
    print("per-motor |tau_demand|, last 20 s, split by clock phase (kp) and"
          " real contact\n")
    print("%-6s %10s %10s %10s %10s %10s %8s" % (
        "cell", "p99.5_all", "cs&air", "cs&con", "cf&con", "cf&air",
        "frac>p99.5"))
    store = {}
    for c in cells:
        cat = {k: [] for k in ("all", "cs_air", "cs_con", "cf_con", "cf_air")}
        loc = {k: 0 for k in ("cs_air", "cs_con", "cf_con", "cf_air")}
        for p in sorted(glob.glob(os.path.join(base, c, "run[0-9].csv"))):
            rows = load(p)
            tmax = max(r[0] for r in rows)
            by = {}
            for t, leg, mot, ic, td, ff, st, kp in rows:
                if t < tmax - TAIL_S:
                    continue
                by.setdefault((leg, mot), []).append((t, ic, td, kp))
            for (leg, mot), v in by.items():
                v.sort()
                a = np.array(v)
                ic = a[:, 1].astype(bool)
                td = np.abs(a[:, 2])
                kp = a[:, 3]
                if kp.max() - kp.min() < 1.0:
                    continue
                thr = 0.5 * (kp.min() + kp.max())
                hi = kp > thr
                cs = hi if hi.mean() < 0.5 else ~hi
                con = debounce(ic, DEBOUNCE)
                cat["all"].append(td)
                cat["cs_air"].append(td[cs & ~con])
                cat["cs_con"].append(td[cs & con])
                cat["cf_con"].append(td[~cs & con])
                cat["cf_air"].append(td[~cs & ~con])
                q = np.percentile(td, 99.5)
                top = td >= q
                loc["cs_air"] += int((top & cs & ~con).sum())
                loc["cs_con"] += int((top & cs & con).sum())
                loc["cf_con"] += int((top & ~cs & con).sum())
                loc["cf_air"] += int((top & ~cs & ~con).sum())
        f = lambda k: float(np.percentile(np.concatenate(cat[k]), 99.5))
        tot = sum(loc.values()) or 1
        store[c] = (loc, tot)
        print("%-6s %10.2f %10.2f %10.2f %10.2f %10.2f %8s" % (
            c, f("all"), f("cs_air"), f("cs_con"), f("cf_con"), f("cf_air"),
            ""))
    print("\nwhere the top-0.5%% samples actually sit (%% of them):")
    print("%-6s %10s %10s %10s %10s" % (
        "cell", "cs&air", "cs&con", "cf&con", "cf&air"))
    for c in cells:
        loc, tot = store[c]
        print("%-6s %9.1f%% %9.1f%% %9.1f%% %9.1f%%" % (
            c, 100.0 * loc["cs_air"] / tot, 100.0 * loc["cs_con"] / tot,
            100.0 * loc["cf_con"] / tot, 100.0 * loc["cf_air"] / tot))
    print("\nlegend: cs = clock says STANCE, cf = clock says FLIGHT;"
          " con/air = foot really down / up")


if __name__ == "__main__":
    main()
