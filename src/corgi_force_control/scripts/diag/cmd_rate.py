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
    python3 cmd_rate.py --watch          # live, during a run

TWO BARS, both must pass:

  MEAN   >= 900 Hz over the gait window   (registered S313; healthy run 3
         measured 986.7 Hz, broken runs 1 and 2 measured 14.6 / 15.5 Hz)

  BURST  worst freeze <= 25 ms            (added S316; healthy run 3's worst
         freeze was 11 ms, the broken runs' were 231 and 485 ms)

WHY THE SECOND BAR EXISTS. The mean is an average over the whole window, so
a run that is perfectly healthy except for a burst of starvation still
passes it. Solve 1000*[(1-f) + 0.015f] >= 900 and the mean bar tolerates
f = 10.2% of the gait window fully starved -- in a ten-second run, a whole
second of visible spazzing, scored PASS. That is exactly what "the pronk
is jerky SOMETIMES, but the run passed" looks like. The burst bar catches
it because a healthy 1 kHz stream never freezes for more than ~11 ms.

Measured INSIDE the gait window only (first to last command change), so
the pre-trigger hold -- which legitimately holds one command -- cannot
dilute the number.
"""
import csv
import statistics
import sys

BAR_HZ = 900.0          # registered S313
BAR_GAP_ROWS = 25       # added S316; rows are ms at the recorder's 1 kHz
STALL_ROWS = 5          # a gap longer than this counts as "frozen" time
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
    frozen = sum(g - 1 for g in gaps if g > STALL_ROWS)
    return (hz, len(idx), span, statistics.median(gaps), max(gaps),
            100.0 * frozen / max(span, 1))


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
        got, mx = r[0], r[4]
        passed = got >= BAR_HZ and mx <= BAR_GAP_ROWS
        good = (passed == expect_pass)
        print("selftest %4d Hz -> measured %7.1f Hz  gap max %3d  %s  %s"
              % (hz_target, got, mx, "PASS" if passed else "FAIL",
                 "ok" if good else "*** SELFTEST FAILED ***"))
        ok = ok and good

    # The case the mean bar CANNOT see: healthy except for one 5% burst.
    # Known answer -- mean must pass, burst must fail.
    fd, p = tempfile.mkstemp(suffix=".csv")
    os.close(fd)
    N, burst = 20000, 1000
    with open(p, "w", newline="") as f:
        f.write(COL + "\n")
        v = 0.0
        for i in range(N):
            if not (N // 2 <= i < N // 2 + burst):
                v = 0.001 * i
            f.write("%.6f\n" % v)
    r = rate(p)
    os.unlink(p)
    if r is None:
        print("selftest burst: UNFIT -- FAIL")
        return False
    hz, _, _, _, mx, frozen = r
    mean_ok = hz >= BAR_HZ
    burst_ok = mx <= BAR_GAP_ROWS
    good = mean_ok and not burst_ok
    print("selftest burst (%d ms freeze in %d) -> mean %7.1f Hz %s, "
          "gap max %d %s, frozen %.1f%%  %s"
          % (burst, N, hz, "PASS" if mean_ok else "FAIL",
             mx, "PASS" if burst_ok else "FAIL", frozen,
             "ok -- mean is blind, burst catches it"
             if good else "*** SELFTEST FAILED ***"))
    return ok and good


def watch(topic="motor/command"):
    """Live value-change rate, so starvation can be caught in the act.

    Prints the MESSAGE rate and the VALUE-CHANGE rate side by side, which is
    the whole of the S313 diagnosis in one line: messages at 1 kHz with
    values changing at 15 Hz is the failure. A capture tells you afterwards;
    this tells you while you can still see the robot.
    """
    import time
    import rclpy
    from rclpy.node import Node
    from corgi_msgs.msg import MotorCmdStamped

    class W(Node):
        def __init__(self):
            super().__init__("cmd_rate_watch")
            self.msgs = 0
            self.changes = []
            self.prev = None
            self.last_change = time.monotonic()
            self.worst_gap = 0.0
            self.create_subscription(MotorCmdStamped, topic, self.cb, 50)
            self.create_timer(1.0, self.tick)
            print("watching %s (module_a.beta)   ctrl-C to stop" % topic)
            print("%8s %12s %12s %10s   %s"
                  % ("", "msgs/s", "changes/s", "worst gap", "verdict"))

        def cb(self, m):
            now = time.monotonic()
            self.msgs += 1
            v = m.module_a.beta
            if self.prev is not None and v != self.prev:
                gap = (now - self.last_change) * 1000.0
                self.worst_gap = max(self.worst_gap, gap)
                self.last_change = now
                self.changes.append(now)
            self.prev = v

        def tick(self):
            now = time.monotonic()
            self.changes = [t for t in self.changes if now - t <= 1.0]
            chz = len(self.changes)
            stuck = (now - self.last_change) * 1000.0
            gap = max(self.worst_gap, stuck)
            if self.msgs == 0:
                verdict = "no commands -- is the controller running?"
            elif chz >= BAR_HZ and gap <= BAR_GAP_ROWS:
                verdict = "ok"
            elif chz < BAR_HZ and self.msgs >= BAR_HZ:
                verdict = "*** STARVED -- publishing but not computing ***"
            else:
                verdict = "*** DEGRADED ***"
            print("%8s %12d %12d %8.0f ms   %s"
                  % (time.strftime("%H:%M:%S"), self.msgs, chz, gap, verdict))
            self.msgs = 0
            self.worst_gap = 0.0

    rclpy.init()
    n = W()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0


def main(argv):
    if "--selftest" in argv:
        return 0 if selftest() else 1
    if "--watch" in argv:
        return watch()
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
        hz, n, span, med, mx, frozen = r
        bad = []
        if hz < BAR_HZ:
            bad.append("mean")
        if mx > BAR_GAP_ROWS:
            bad.append("burst")
        verdict = ("PASS" if not bad
                   else "*** FAIL (%s) -- DISCARD ***" % "+".join(bad))
        print("%-32s %7.1f Hz  (%d changes over %d rows, gap median %.0f "
              "max %.0f ms, frozen %.1f%%)  %s"
              % (name, hz, n, span, med, mx, frozen, verdict))
        if bad:
            worst = max(worst, 1)
    return worst


if __name__ == "__main__":
    sys.exit(main(sys.argv))
