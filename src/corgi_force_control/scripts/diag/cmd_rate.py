#!/usr/bin/env python3
"""Did the motor command stream actually run at 1 kHz? Per-run gate.

WHY THIS EXISTS (log S313). On 2026-08-31, two of three hardware air runs
published motor commands whose VALUES changed at ~15 Hz while every
timestamp -- imp_cmd, motor_cmd, motor_state, power -- still said 1 kHz.
force_control was publishing on schedule but computing from STALE
impedance commands: its single-threaded spin_some loop was starved of
CPU. At 1 kHz the v070 template moves beta ~0.0012 rad per tick; at 15 Hz
each update is a ~50x jump the impedance law snaps the leg toward. The
robot visibly "spazzes", and the capture is worthless -- but NOTHING in
the banners, the timestamps or the state stream says so. Only the rate at
which the commanded VALUES change reveals it.

So: run this on every hardware capture before trusting it.

    python3 cmd_rate.py <capture.csv> [more.csv ...]
    python3 cmd_rate.py --selftest

VERDICT: PASS  >= 900 Hz   (healthy run 3 measured 986 Hz)
         FAIL  <  900 Hz   (broken runs 1 and 2 measured 14.6 / 15.5 Hz)

Measured INSIDE the gait window only (first to last command change), so
the pre-trigger hold -- which legitimately holds one command -- cannot
dilute the number.
"""
import csv
import statistics
import sys

BAR_HZ = 900.0
COL = "cmd_beta_a"


def rate(path):
    """(hz, n_changes, span_rows, median_gap, max_gap) or None if unfit."""
    idx = []
    prev = None
    rows = 0
    with open(path, newline="") as f:
        rd = csv.DictReader(f)
        if rd.fieldnames is None or COL not in rd.fieldnames:
            return None
        for i, row in enumerate(rd):
            try:
                v = float(row[COL])
            except (TypeError, ValueError):
                continue
            rows += 1
            if prev is not None and v != prev:
                idx.append(i)
            prev = v
    if len(idx) < 3:
        return None
    span = idx[-1] - idx[0]
    gaps = [b - a for a, b in zip(idx, idx[1:])]
    # rows are the recorder's own 1 kHz cadence
    hz = 1000.0 * len(idx) / max(span, 1)
    return hz, len(idx), span, statistics.median(gaps), max(gaps)


def selftest():
    """Known answers: a synthetic 1 kHz stream and a synthetic 15 Hz one."""
    import os
    import tempfile
    ok = True
    for hz_target, expect_pass in ((1000, True), (15, False)):
        fd, p = tempfile.mkstemp(suffix=".csv")
        os.close(fd)
        hold = max(1, int(round(1000.0 / hz_target)))
        with open(p, "w", newline="") as f:
            f.write(COL + "\n")
            for i in range(20000):
                f.write("%.6f\n" % (0.001 * (i // hold)))
        r = rate(p)
        os.unlink(p)
        if r is None:
            print("selftest %d Hz: UNFIT -- FAIL" % hz_target)
            ok = False
            continue
        got = r[0]
        passed = got >= BAR_HZ
        good = (passed == expect_pass)
        print("selftest %4d Hz -> measured %7.1f Hz  %s  %s"
              % (hz_target, got, "PASS" if passed else "FAIL",
                 "ok" if good else "*** SELFTEST FAILED ***"))
        ok = ok and good
    return ok


def main(argv):
    if "--selftest" in argv:
        return 0 if selftest() else 1
    if len(argv) < 2:
        print(__doc__)
        return 2
    worst = 0
    for path in argv[1:]:
        r = rate(path)
        name = path.split("/")[-1]
        if r is None:
            print("%-32s UNFIT (no %s column, or no command changes)"
                  % (name, COL))
            worst = max(worst, 2)
            continue
        hz, n, span, med, mx = r
        verdict = "PASS" if hz >= BAR_HZ else "*** FAIL -- DISCARD ***"
        print("%-32s %7.1f Hz  (%d changes over %d rows, gap median %.0f "
              "max %.0f)  %s" % (name, hz, n, span, med, mx, verdict))
        if hz < BAR_HZ:
            worst = max(worst, 1)
    return worst


if __name__ == "__main__":
    sys.exit(main(sys.argv))
