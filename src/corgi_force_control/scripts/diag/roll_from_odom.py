#!/usr/bin/env python3
"""GT roll statistics from a full-Odometry --csv capture (log S270's metric).

Per file: median / mean / p16 / p84 of body roll (deg) over the steady band
(last STEADY_S seconds), roll computed from the odom quaternion with the SAME
atan2 as gslip_pronk's update_attitude (REP-103). Multiple files: also prints
the across-run median of |rollMED| (P-DC-1's statistic).

Column layout (no header): 0 sec, 1 nsec, 2 frame, 3 child, 4-6 pos,
7-10 quat xyzw, 11-46 pose cov, 47-52 twist. Refuses files that do not
parse at least MIN_ROWS rows in the steady band (Unfit guard, S152 style).

--selftest: synthesizes a constant-roll quaternion series (5 deg about x)
plus noise-free zero-roll rows and asserts the outputs to 1e-6.
"""
import csv
import math
import statistics
import sys

STEADY_S = 8.0
MIN_ROWS = 200


def roll_deg(qx, qy, qz, qw):
    return math.degrees(math.atan2(2.0 * (qw * qx + qy * qz),
                                   1.0 - 2.0 * (qx * qx + qy * qy)))


def read_rolls(path):
    ts, rolls = [], []
    with open(path, newline="") as f:
        for row in csv.reader(f):
            if len(row) < 53:
                continue
            try:
                t = float(row[0]) + float(row[1]) * 1e-9
                qx, qy, qz, qw = (float(v) for v in row[7:11])
            except ValueError:
                continue
            n = qx * qx + qy * qy + qz * qz + qw * qw
            if not (0.9 < n < 1.1):   # not a unit quaternion -> wrong columns
                continue
            ts.append(t)
            rolls.append(roll_deg(qx, qy, qz, qw))
    if not ts:
        return None
    t1 = ts[-1]
    steady = [r for t, r in zip(ts, rolls) if t >= t1 - STEADY_S]
    return steady


def selftest():
    # roll = 5 deg about x: q = (sin(2.5deg), 0, 0, cos(2.5deg))
    half = math.radians(2.5)
    q = (math.sin(half), 0.0, 0.0, math.cos(half))
    got = roll_deg(*q)
    assert abs(got - 5.0) < 1e-6, f"selftest: 5 deg case got {got}"
    assert abs(roll_deg(0, 0, 0, 1)) < 1e-12, "selftest: zero case"
    # negative roll symmetry
    qn = (-math.sin(half), 0.0, 0.0, math.cos(half))
    assert abs(roll_deg(*qn) + 5.0) < 1e-6, "selftest: -5 deg case"
    print("selftest PASS (5 / 0 / -5 deg quaternions reproduce to 1e-6)")


def main(argv):
    if "--selftest" in argv:
        selftest()
        return 0
    files = [a for a in argv if not a.startswith("--")]
    if not files:
        print("usage: roll_from_odom.py [--selftest] odom_run*.csv")
        return 2
    meds = []
    for path in files:
        steady = read_rolls(path)
        if steady is None or len(steady) < MIN_ROWS:
            n = 0 if steady is None else len(steady)
            print(f"{path}: UNFIT ({n} steady rows < {MIN_ROWS}) -- refused")
            continue
        med = statistics.median(steady)
        srt = sorted(steady)
        p16 = srt[int(0.16 * (len(srt) - 1))]
        p84 = srt[int(0.84 * (len(srt) - 1))]
        meds.append(med)
        print(f"{path}: rollMED {med:+.3f} mean {statistics.fmean(steady):+.3f} "
              f"p16 {p16:+.3f} p84 {p84:+.3f} deg  (n {len(steady)})")
    if len(meds) > 1:
        print(f"across-run median rollMED {statistics.median(meds):+.3f} deg; "
              f"median |rollMED| {statistics.median(abs(m) for m in meds):.3f} deg "
              f"over {len(meds)} runs")
    return 0 if meds else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
