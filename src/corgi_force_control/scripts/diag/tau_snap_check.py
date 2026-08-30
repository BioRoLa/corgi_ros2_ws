#!/usr/bin/env python3
"""P93-4 (S4): tau_demand p99.5 against the 1.9x tolerance (~29.3 N.m),
overall AND restricted to +-0.25 s windows around scheduler snap events.
Snap times come from the sched CSV; both files share the sim clock.

Usage: tau_snap_check.py runN.csv [sched_runN.csv]
"""
import csv
import sys

import numpy as np

TOL = 29.3


def main():
    tq = sys.argv[1]
    sched = sys.argv[2] if len(sys.argv) > 2 else None
    t, tau = [], []
    # LEG MOTORS ONLY: the 29.3 tolerance is 1.9x the 15.43 N.m model
    # value, which is a LEG-motor figure. Pooling ABAD rows (kp_h ~1000)
    # fails even healthy baselines and measures the wrong thing.
    with open(tq, newline="") as fh:
        for r in csv.DictReader(fh):
            if r["motor"] not in ("R_Motor", "L_Motor"):
                continue
            t.append(float(r["t"]))
            tau.append(abs(float(r["tau_demand"])))
    t = np.array(t)
    tau = np.array(tau)
    if len(t) < 1000:
        print(f"{tq}: REFUSED, only {len(t)} rows")
        return
    p = float(np.percentile(tau, 99.5))
    line = f"{tq.split('/')[-2]}/{tq.split('/')[-1]}: tau p99.5 {p:6.2f}"
    verdict = "PASS" if p <= TOL else "FAIL"
    if sched:
        snap_t = []
        with open(sched, newline="") as fh:
            for r in csv.DictReader(fh):
                if r.get("event", "").startswith("SNAP"):
                    try:
                        snap_t.append(float(r["t_node"]))
                    except ValueError:
                        pass
        if snap_t:
            m = np.zeros(len(t), dtype=bool)
            for st in snap_t:
                m |= np.abs(t - st) <= 0.25
            if m.sum() > 200:
                ps = float(np.percentile(tau[m], 99.5))
                line += f"   around-snaps p99.5 {ps:6.2f} ({len(snap_t)} snaps)"
                if ps > TOL:
                    verdict = "FAIL(snap-windows)"
            else:
                line += f"   ({len(snap_t)} snaps, too few samples in windows)"
        else:
            line += "   (no snap events)"
    print(f"{line}   vs {TOL} -> {verdict}")


if __name__ == "__main__":
    main()
