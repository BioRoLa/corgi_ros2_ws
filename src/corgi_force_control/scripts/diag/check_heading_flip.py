#!/usr/bin/env python3
"""S256's per-run validity instrument: did the run actually start heading-flipped?

The unconfounding campaign is a WORLD-POSE edit that nothing fingerprints, so
every run must prove its own flip from odom: initial yaw ~ pi (|wrap(yaw0)| >
90 deg) AND net x displacement NEGATIVE (the un-flipped robot pronks toward
+x). A run without both signatures is INVALID for S256.

Odom rows: sec,nsec,frame,child,x,y,z,qx,qy,qz,qw,...  yaw = 2*atan2(qz, qw)
(planar assumption -- roll/pitch are degrees, not tens of degrees, in every
banked pronk).

Usage:
    check_heading_flip.py --selftest
    check_heading_flip.py --cell <dir> [--cell ...] [--expect flipped|normal]
"""
import argparse
import csv
import glob
import math
import os
import sys


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def read_odom(path):
    rows = []
    with open(path, newline="") as fh:
        for row in csv.reader(fh):
            try:
                t = float(row[0]) + float(row[1]) * 1e-9
                x, y = float(row[4]), float(row[5])
                qz, qw = float(row[9]), float(row[10])
            except (ValueError, IndexError):
                continue
            rows.append((t, x, y, 2.0 * math.atan2(qz, qw)))
    if len(rows) < 50:
        raise SystemExit(f"REFUSING: {len(rows)} usable odom rows ({path})")
    return rows


def check(path, expect):
    rows = read_odom(path)
    # median yaw over the first second on the clock of the first row
    t0 = rows[0][0]
    early = sorted(wrap(r[3]) for r in rows if r[0] <= t0 + 1.0)
    yaw0 = early[len(early) // 2]
    dx = rows[-1][1] - rows[0][1]
    flipped_yaw = abs(yaw0) > math.pi / 2
    flipped_dx = dx < 0
    verdict = "FLIPPED" if (flipped_yaw and flipped_dx) else \
              "NORMAL" if (not flipped_yaw and dx > 0) else "AMBIGUOUS"
    ok = (expect is None) or (verdict.lower() == expect)
    print(f"  {os.path.basename(path)}: yaw0 {math.degrees(yaw0):+7.1f} deg  "
          f"dx {dx:+7.2f} m  -> {verdict}" + ("" if ok else "  !! EXPECTED " + expect.upper()))
    return ok


def selftest():
    import tempfile
    d = tempfile.mkdtemp(prefix="flip_st_")
    # flipped: yaw pi (qz=1, qw=0), x decreasing; normal: yaw 0, x increasing.
    cases = {"flip.csv": (1.0, 0.0, -1.0), "norm.csv": (0.0, 1.0, +1.0)}
    for name, (qz, qw, sgn) in cases.items():
        with open(os.path.join(d, name), "w", newline="") as fh:
            w = csv.writer(fh)
            for i in range(200):
                t = i * 0.1
                w.writerow([int(t), int((t - int(t)) * 1e9), "odom", "base_link",
                            sgn * 0.25 * t, 0.0, 0.03, 0, 0, qz, qw])
    assert check(os.path.join(d, "flip.csv"), "flipped")
    assert check(os.path.join(d, "norm.csv"), "normal")
    assert not check(os.path.join(d, "flip.csv"), "normal")
    print("selftest OK")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--cell", action="append", default=[])
    ap.add_argument("--expect", choices=["flipped", "normal"])
    a = ap.parse_args()
    if a.selftest:
        selftest()
        return
    bad = 0
    for cell in a.cell:
        print(f"cell {cell}")
        for o in sorted(glob.glob(os.path.join(cell, "odom_run[0-9]*.csv"))):
            if "uncertified" in o:
                continue
            if not check(o, a.expect):
                bad += 1
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
