#!/usr/bin/env python3
"""S257's registered instrument: ABAD effort per meter of path.

    effort = mean over the band of  sum_legs |tau_applied, ABAD|   [N.m]
    v_bar  = odom (x, y) path length in the band / band duration   [m/s]
    effort per meter = effort / v_bar                              [N.m.s/m]

Numbers come from tau_applied (post-clamp) -- the clamp is part of the plant;
clip%% is abad_torque.py's job and is reported there, not here. The mean over
the band is computed as mean over ABAD rows x number of legs present, which
equals the time-mean of the leg-sum when every tick carries one ABAD row per
leg (asserted). gamma median (deg, |gamma| over all rows in band) is reported
beside it, DESCRIPTIVE ONLY -- S250: gamma is not a controlled variable under
the loop and must not gate anything.

Refuses unfit input (S-rule: gate analysers against invalid input): no ABAD
rows, band not covered by the capture, legs missing, or odom shorter than the
band.

Usage:
    abad_effort_per_m.py --selftest
    abad_effort_per_m.py --cell <dir> [--cell ...] --start 12 --end 48
"""
import argparse
import csv
import glob
import math
import os
import sys

import numpy as np

LEGS = ("A", "B", "C", "D")


def load_run(path, start, end):
    """-> (abad_tau_by_leg, gamma_all) for rows with start <= t <= end."""
    tau = {}
    gamma = []
    tmax = 0.0
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            try:
                t = float(row["t"])
            except (ValueError, KeyError):
                continue
            tmax = max(tmax, t)
            if not (start <= t <= end):
                continue
            gamma.append(float(row["gamma"]))
            if row.get("motor") == "ABAD":
                tau.setdefault(row["leg"], []).append(float(row["tau_applied"]))
    if tmax < end:
        raise SystemExit(f"REFUSING: capture ends at {tmax:.1f}s < band end {end}s ({path})")
    missing = [l for l in LEGS if l not in tau]
    if missing:
        raise SystemExit(f"REFUSING: no ABAD rows for leg(s) {missing} in band ({path})")
    counts = {l: len(v) for l, v in tau.items()}
    if max(counts.values()) - min(counts.values()) > 2:
        raise SystemExit(f"REFUSING: unbalanced ABAD row counts per leg {counts} ({path})")
    return tau, np.array(gamma)


def load_odom_speed(path, start, end):
    """-> mean speed over the band from (x, y) path length. Odom rows:
    sec,nsec,frame,child,x,y,z,... ; t = sec + nsec*1e-9."""
    pts = []
    with open(path, newline="") as fh:
        for row in csv.reader(fh):
            try:
                t = float(row[0]) + float(row[1]) * 1e-9
                x, y = float(row[4]), float(row[5])
            except (ValueError, IndexError):
                continue
            if start <= t <= end:
                pts.append((t, x, y))
    if len(pts) < 10:
        raise SystemExit(f"REFUSING: {len(pts)} odom rows in band ({path})")
    if pts[-1][0] - pts[0][0] < 0.8 * (end - start):
        raise SystemExit(f"REFUSING: odom covers {pts[-1][0]-pts[0][0]:.1f}s of a {end-start}s band ({path})")
    arr = np.array(pts)
    seg = np.hypot(np.diff(arr[:, 1]), np.diff(arr[:, 2]))
    return seg.sum() / (arr[-1, 0] - arr[0, 0])


def score_run(run_csv, odom_csv, start, end):
    tau, gamma = load_run(run_csv, start, end)
    effort = float(np.mean([np.mean(np.abs(v)) for v in tau.values()])) * len(LEGS)
    vbar = load_odom_speed(odom_csv, start, end)
    if vbar <= 0.02:
        raise SystemExit(f"REFUSING: v_bar {vbar:.3f} m/s -- not locomoting ({odom_csv})")
    return {
        "effort_Nm": effort,
        "v_bar": vbar,
        "effort_per_m": effort / vbar,
        "gamma_med_deg": float(np.median(np.abs(gamma))) * 180.0 / math.pi,
    }


def selftest():
    """Synthetic known answers, written to /tmp."""
    import tempfile
    d = tempfile.mkdtemp(prefix="abad_effort_st_")
    run = os.path.join(d, "run1.csv")
    odom = os.path.join(d, "odom_run1.csv")
    # 4 legs x ABAD |tau| = 2.5 each -> effort = 10 N.m exactly; gamma = 0.1 rad.
    with open(run, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "leg", "motor", "in_contact", "t_stiff", "t_damp", "t_ff",
                    "tau_demand", "tau_applied", "pos_error", "kp", "kd",
                    "theta", "beta", "gamma"])
        t = 0.0
        while t <= 50.0:
            for leg in LEGS:
                w.writerow([f"{t:.3f}", leg, "ABAD", 1, 0, 0, 0, 3.0,
                            2.5 if leg in ("A", "B") else -2.5,
                            0, 120, 1.75, 0.3, 0.0, 0.1])
                w.writerow([f"{t:.3f}", leg, "R_Motor", 1, 0, 0, 0, 1.0, 1.0,
                            0, 120, 1.75, 0.3, 0.0, 0.1])
            t += 0.01
    # straight line at 0.25 m/s -> effort_per_m = 10 / 0.25 = 40 exactly.
    with open(odom, "w", newline="") as fh:
        w = csv.writer(fh)
        t = 0.0
        while t <= 50.0:
            sec = int(t)
            nsec = int(round((t - sec) * 1e9))
            w.writerow([sec, nsec, "odom", "base_link", 0.25 * t, 0.0, 0.03, 0, 0, 0, 1])
            t += 0.01
    r = score_run(run, odom, 12.0, 48.0)
    assert abs(r["effort_Nm"] - 10.0) < 1e-6, r
    assert abs(r["v_bar"] - 0.25) < 1e-3, r
    assert abs(r["effort_per_m"] - 40.0) < 0.2, r
    assert abs(r["gamma_med_deg"] - math.degrees(0.1)) < 1e-6, r
    # refusal: band beyond capture must refuse
    try:
        score_run(run, odom, 12.0, 60.0)
    except SystemExit:
        pass
    else:
        raise AssertionError("did not refuse a band beyond the capture")
    print("selftest OK (effort 10 N.m, v 0.25 m/s, 40 N.m.s/m, gamma 5.73 deg; refusal fires)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--cell", action="append", default=[])
    ap.add_argument("--start", type=float, default=12.0)
    ap.add_argument("--end", type=float, default=48.0)
    a = ap.parse_args()
    if a.selftest:
        selftest()
        return
    for cell in a.cell:
        runs = sorted(glob.glob(os.path.join(cell, "run[0-9]*.csv")))
        runs = [r for r in runs if "uncertified" not in r]
        print(f"cell {cell}  band [{a.start:g}, {a.end:g}] s  n={len(runs)}")
        print(" run  effort_Nm   v_bar  effort_per_m  gamma_med_deg")
        vals = []
        for r in runs:
            n = os.path.basename(r)[3:-4]
            o = os.path.join(cell, f"odom_run{n}.csv")
            if not os.path.exists(o):
                print(f"  {n}: REFUSING (no odom)"); continue
            s = score_run(r, o, a.start, a.end)
            vals.append(s)
            print(f"  {n}   {s['effort_Nm']:8.3f}  {s['v_bar']:6.3f}  "
                  f"{s['effort_per_m']:10.3f}  {s['gamma_med_deg']:8.2f}")
        if vals:
            med = {k: float(np.median([v[k] for v in vals])) for k in vals[0]}
            print(f" median over {len(vals)}: effort {med['effort_Nm']:.3f}  "
                  f"v {med['v_bar']:.3f}  effort/m {med['effort_per_m']:.3f}  "
                  f"gamma {med['gamma_med_deg']:.2f} deg")
        print()


if __name__ == "__main__":
    main()
