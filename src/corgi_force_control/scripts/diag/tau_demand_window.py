#!/usr/bin/env python3
"""Peak leg-motor torque demand, on the demand-reduction thread's OWN
statistic (log S49, demand thread):

    p99.5 of pre-clamp |tau_demand|, over rows with motor in
    {R_Motor, L_Motor}, restricted to the uniform LAST 20 s of the run,
    pooled across all four legs and both leg motors.

Three parts of that definition each exist for a recorded reason:

  PRE-CLAMP    tau_demand is logged before the per-joint clamp
               (corgi_driver), which is what makes this measurable at
               the stock ceiling. Never raise the ceiling to see peaks:
               S28 measured at 200 N.m and S38 then found the robot
               FLIPS there -- the 29.5 clamp is an accidental stabiliser.
  LEG MOTORS   the tolerance lines are leg-motor figures. Pooling the
               ABAD (kp_h ~ 1000) fails even healthy baselines, which is
               how the scheduler thread first mis-scored S4 (log S94).
  LAST 20 s    a uniform tail, so entry transients cannot move the
               number. It is NOT the same window as audit_degradation's
               `flight_last` (last third of gait-band windows) -- report
               both and never silently mix them.

This is a SIBLING of tau_snap_check.py, not a replacement: that file
belongs to the event-scheduler thread (its snap-window logic is that
thread's question) and is left byte-identical.

Validate before trusting: --selftest reproduces log S64's soft-gain cell
(17.7 / 17.6 / 17.7 pooled, 15.9-18.3 per leg, 0.0% clip) from banked
captures. A tool that has not reproduced a published number does not get
to score a campaign.

Usage:
    tau_demand_window.py run1.csv [run2.csv ...]
    tau_demand_window.py --dir ~/corgi_runs/kflight/k2500/lam0_default
    tau_demand_window.py --selftest
    tau_demand_window.py --dir ... --tol 25.0      # 6:1 stock hardware
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np

MODEL_NM = 15.43        # G-SLIP quasi-static peak, v~0.70 pronk (S28/S63)
TAIL_S = 20.0           # S49's uniform window
LEG_MOTORS = ("R_Motor", "L_Motor")
LEGS = "ABCD"
# The three ABSOLUTE hardware lines. The 1.87x ratio behind 29.3 was
# formed on the TROT template minimum (37/19.81, S38) and then applied to
# the pronk's 15.43 -- the ratio does not transfer, the limits do.
LINES = {"6:1 stock usable": 25.0, "registered gate": 29.3, "9:1 swap": 37.0}


class Unfit(Exception):
    """Input cannot support the question -- refuse rather than guess."""


def load(path):
    """-> (t, tau_demand, tau_applied, leg) over LEG MOTOR rows only."""
    t, td, ta, leg = [], [], [], []
    with open(path, newline="") as fh:
        rdr = csv.DictReader(fh)
        need = ("t", "leg", "motor", "tau_demand", "tau_applied")
        missing = [c for c in need if c not in (rdr.fieldnames or [])]
        if missing:
            raise Unfit(f"missing columns {missing} -- not a torque capture")
        for r in rdr:
            if r["motor"] not in LEG_MOTORS:
                continue
            try:
                t.append(float(r["t"]))
                td.append(abs(float(r["tau_demand"])))
                ta.append(abs(float(r["tau_applied"])))
                leg.append(r["leg"])
            except ValueError:
                continue           # torn tail line from a kill -9
    if len(t) < 1000:
        raise Unfit(f"only {len(t)} leg-motor rows")
    return (np.array(t), np.array(td), np.array(ta), np.array(leg))


def stats(path, tail_s=TAIL_S):
    t, td, ta, leg = load(path)
    m = t >= (t.max() - tail_s)
    if int(m.sum()) < 1000:
        raise Unfit(f"only {int(m.sum())} rows in the last {tail_s:.0f}s "
                    f"(run too short for the S49 window)")
    t, td, ta, leg = t[m], td[m], ta[m], leg[m]
    per_leg = []
    for L in LEGS:
        sel = leg == L
        per_leg.append(float(np.percentile(td[sel], 99.5))
                       if sel.sum() > 100 else float("nan"))
    pooled = float(np.percentile(td, 99.5))
    return {
        "pooled_p995": pooled,
        "per_leg": per_leg,
        "pooled_max": float(td.max()),
        "erosion": pooled / MODEL_NM,
        "clip_pct": 100.0 * float((ta < td - 1e-6).mean()),
        "n_rows": int(len(t)),
        "band": (float(t.min()), float(t.max())),
    }


def report(path, s, tol):
    name = "/".join(path.split("/")[-2:])
    pl = " ".join(f"{v:5.1f}" for v in s["per_leg"])
    print(f"{name:28} p99.5 {s['pooled_p995']:7.2f}  "
          f"per-leg [{pl}]  max {s['pooled_max']:8.1f}  "
          f"erosion {s['erosion']:5.2f}x  clip {s['clip_pct']:5.2f}%  "
          f"{'PASS' if s['pooled_p995'] <= tol else 'over'} @{tol}")


def selftest():
    """Reproduce log S64's soft-gain cell from banked captures."""
    base = os.path.expanduser(
        "~/corgi_runs/shift_duty_sweep/v070_cor_soft_odo")
    want = [17.7, 17.6, 17.7]
    ok = True
    if not os.path.isdir(base):
        print(f"SELFTEST SKIP: {base} not present")
        return 0
    for n, w in zip((1, 2, 3), want):
        p = os.path.join(base, f"run{n}.csv")
        if not os.path.exists(p):
            print(f"SELFTEST SKIP run{n}: absent")
            continue
        try:
            s = stats(p)
        except Unfit as e:
            print(f"SELFTEST FAIL run{n}: {e}")
            ok = False
            continue
        got = s["pooled_p995"]
        pl_ok = all(15.0 <= v <= 19.0 for v in s["per_leg"]
                    if v == v)
        if abs(got - w) > 0.15 or s["clip_pct"] > 0.05 or not pl_ok:
            print(f"SELFTEST FAIL run{n}: pooled {got:.2f} (want {w}), "
                  f"per-leg {np.round(s['per_leg'], 1)} (want 15.9-18.3), "
                  f"clip {s['clip_pct']:.2f}% (want 0.00)")
            ok = False
        else:
            print(f"selftest run{n}: pooled {got:.2f} == S64's {w}, "
                  f"per-leg {np.round(s['per_leg'], 1)}, "
                  f"clip {s['clip_pct']:.2f}%  OK")
    print("SELFTEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="*")
    ap.add_argument("--dir", help="score every runN.csv in a cell directory")
    ap.add_argument("--tol", type=float, default=29.3)
    ap.add_argument("--tail", type=float, default=TAIL_S)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        sys.exit(selftest())

    paths = list(args.runs)
    if args.dir:
        paths += sorted(glob.glob(
            os.path.join(os.path.expanduser(args.dir), "run[0-9].csv")))
    if not paths:
        ap.error("give run CSVs, --dir, or --selftest")

    vals = []
    for p in paths:
        try:
            s = stats(p, args.tail)
        except Unfit as e:
            print(f"{p}: REFUSED -- {e}")
            continue
        report(p, s, args.tol)
        vals.append(s)
    if len(vals) > 1:
        a = np.array([v["pooled_p995"] for v in vals])
        f = np.array([v["clip_pct"] for v in vals])
        print(f"{'CELL (n=' + str(len(a)) + ')':28} "
              f"p99.5 median {np.median(a):7.2f}  min {a.min():7.2f}  "
              f"max {a.max():7.2f}   clip median {np.median(f):5.2f}%")
        # The decision rule keys on the MAX, not the median: with n=5 the
        # min-max range covers the true median with p = 93.75%.
        for label, line in sorted(LINES.items(), key=lambda kv: kv[1]):
            print(f"    vs {label:18} {line:5.1f} N.m : "
                  f"median {'under' if np.median(a) <= line else 'OVER'}, "
                  f"max {'under' if a.max() <= line else 'OVER'}")


if __name__ == "__main__":
    main()
