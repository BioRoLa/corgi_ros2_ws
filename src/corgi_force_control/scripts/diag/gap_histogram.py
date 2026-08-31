#!/usr/bin/env python3
"""Every command freeze, timestamped -- the S318.4 item-1 discriminator.

WHY A HISTOGRAM AND NOT A RATE. S313's "14.6 Hz" is a mean over a stream
that was 98.4% frozen: a lossy projection that constrains nothing about
SHAPE. Two very different symptoms share it --

    a dense ~67 ms comb, tens of gaps      -> a BUZZ ("spazzing out")
    a handful of long gaps                 -> a few visible LURCHES

-- and which one it is decides the diagnosis. S318.4's prediction is
specific: THE NUMBER OF GAPS OVER 25 ms EQUALS THE NUMBER OF JERKS ALEX
COUNTS BY EYE, AND THEIR TIMESTAMPS LAND ON THEM.

Reads a rosbag2 (the standup is pre-trigger, and corgi_data_recorder only
writes inside `if (trigger)` -- see S318.3; it would hand you an empty
file). Also reads a recorder CSV, for triggered runs.

    ros2 bag record -o standup_x /impedance/command /motor/command /motor/state
    python3 gap_histogram.py standup_x
    python3 gap_histogram.py capture.csv
    python3 gap_histogram.py --selftest

Reports, per field: every gap over the bar with its time from start, the
count, the total frozen fraction, and the largest. Also compares theta
against gamma, because S318.4 item 2 uses which axis moved to separate
starvation (theta) from the CAN wrap (gamma only, at fixed times).
"""

import sys

BAR_MS = 25.0          # S317's burst bar: healthy worst freeze was 11 ms


def gaps_from_series(times_s, values, bar_ms=BAR_MS):
    """(gaps, span_s, frozen_frac) from parallel time/value sequences.

    A "gap" is the interval between consecutive CHANGES of value. Held
    values are the signal here -- the publisher keeps publishing.
    """
    change_t = []
    prev = None
    for t, v in zip(times_s, values):
        if prev is None or v != prev:
            change_t.append(t)
        prev = v
    if len(change_t) < 3:
        return [], 0.0, 0.0
    span = change_t[-1] - change_t[0]
    gaps = []
    for a, b in zip(change_t, change_t[1:]):
        ms = (b - a) * 1000.0
        if ms > bar_ms:
            gaps.append((a - change_t[0], ms))
    frozen = sum(ms for _, ms in gaps) / 1000.0
    return gaps, span, (frozen / span if span > 0 else 0.0)


def looks_periodic(values, min_reversals=8):
    """Does this series turn around repeatedly?

    The standup ramp is monotone: every hold in it is a freeze. A gait is
    periodic: theta plateaus once per stride BY DESIGN, and counting those
    as freezes reports a healthy robot as 61.5% frozen -- which is exactly
    what this tool did to air run 3 before the guard existed.
    """
    direction, reversals, prev = 0, 0, None
    for v in values:
        if prev is not None and v != prev:
            d = 1 if v > prev else -1
            if direction and d != direction:
                reversals += 1
            direction = d
        prev = v
    return reversals >= min_reversals, reversals


def report(label, times_s, values, bar_ms=BAR_MS):
    gaps, span, frozen = gaps_from_series(times_s, values, bar_ms)
    if span <= 0:
        print("  %-16s no usable changes" % label)
        return 0

    periodic, reversals = looks_periodic(values)
    if periodic:
        print("  %-16s PERIODIC (%d reversals) -- NOT SCORED"
              % (label, reversals))
        print("        This looks like a gait, not the standup ramp. Holds "
              "are expected once")
        print("        per stride and counting them as freezes is "
              "meaningless: on the HEALTHY")
        print("        air run 3 that produced '223 gaps, 61.5% frozen'. "
              "This tool is for")
        print("        the PRE-TRIGGER STANDUP. For a gait, use cmd_rate.py "
              "on cmd_beta_a.")
        return 0
    print("  %-16s span %6.2f s   gaps>%.0fms: %-4d   frozen %5.1f%%   "
          "worst %6.1f ms"
          % (label, span, bar_ms, len(gaps), 100.0 * frozen,
             max((ms for _, ms in gaps), default=0.0)))
    for t, ms in gaps:
        print("        at t=%6.3f s   held %6.1f ms" % (t, ms))
    return len(gaps)


def from_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    want = {
        '/motor/command': [('module_a.theta', 'cmd theta A'),
                           ('module_a.beta', 'cmd beta A'),
                           ('module_a.gamma', 'cmd gamma A'),
                           ('module_a.kp_h', 'cmd kp_h A'),
                           ('module_a.torque_r', 'cmd trq_r A')],
        '/motor/state':   [('module_a.theta', 'state theta A'),
                           ('module_a.gamma', 'state gamma A')],
        '/impedance/command': [('module_a.theta', 'imp theta A')],
    }
    series = {}
    t0 = None
    while reader.has_next():
        topic, data, tns = reader.read_next()
        if topic not in want:
            continue
        if t0 is None:
            t0 = tns
        msg = deserialize_message(data, get_message(types[topic]))
        for dotted, label in want[topic]:
            obj = msg
            try:
                for part in dotted.split('.'):
                    obj = getattr(obj, part)
            except AttributeError:
                continue
            series.setdefault(label, ([], []))
            series[label][0].append((tns - t0) / 1e9)
            series[label][1].append(obj)
    if not series:
        print("no usable topics in %s -- recorded %s"
              % (path, ', '.join(sorted(types)) or 'nothing'))
        return 2

    print("bag: %s" % path)
    counts = {}
    for label, (ts, vs) in series.items():
        counts[label] = report(label, ts, vs)
    verdict(counts)
    return 0


def from_csv(path):
    import csv
    cols = {'cmd_theta_a': 'cmd theta A', 'cmd_beta_a': 'cmd beta A',
            'cmd_gamma_a': 'cmd gamma A', 'cmd_trq_r_a': 'cmd trq_r A',
            'state_theta_a': 'state theta A', 'state_gamma_a': 'state gamma A'}
    series = {v: ([], []) for v in cols.values()}
    with open(path, newline='') as f:
        for i, row in enumerate(csv.DictReader(f)):
            for col, label in cols.items():
                if col not in row:
                    continue
                try:
                    series[label][0].append(i / 1000.0)   # recorder is 1 kHz
                    series[label][1].append(float(row[col]))
                except (TypeError, ValueError):
                    series[label][0].pop()
    print("csv: %s" % path)
    counts = {}
    for label, (ts, vs) in series.items():
        if vs:
            counts[label] = report(label, ts, vs)
    verdict(counts)
    return 0


def verdict(counts):
    th = counts.get('cmd theta A', 0)
    ga = counts.get('cmd gamma A', 0)
    print()
    if th == 0 and ga == 0:
        print("  VERDICT: no freeze over %.0f ms on any axis -- the command "
              "stream was healthy." % BAR_MS)
        print("           If it still jerked, it is NOT command starvation.")
        return
    print("  VERDICT: %d theta freezes, %d gamma freezes over %.0f ms."
          % (th, ga, BAR_MS))
    print("           S318.4: theta -> starvation; gamma ONLY at fixed times")
    print("           (~0.59 / 1.43 / 1.84 s) -> the CAN gain wrap.")
    print("           Compare the count above against the jerks counted by eye.")


def selftest():
    """Known answers on the gap finder itself, before trusting it on a run."""
    ok = True

    # 1 kHz, always changing: no gaps.
    t = [i / 1000.0 for i in range(5000)]
    v = [i * 0.001 for i in range(5000)]
    g, span, fr = gaps_from_series(t, v)
    good = (len(g) == 0 and abs(span - 4.999) < 0.01)
    print("selftest clean 1 kHz      -> %d gaps, span %.2f s   %s"
          % (len(g), span, "ok" if good else "*** FAILED ***"))
    ok &= good

    # three deliberate freezes: 40 ms at 1.0 s, 200 ms at 2.0 s, 80 ms at 3.0 s
    v2, held = [], 0.0
    for i in range(5000):
        s = i / 1000.0
        if 1.0 <= s < 1.04 or 2.0 <= s < 2.2 or 3.0 <= s < 3.08:
            pass                       # hold
        else:
            held = i * 0.001
        v2.append(held)
    g2, _, fr2 = gaps_from_series(t, v2)
    # A gap spans the hold PLUS the tick that ends it, so a 40 ms hold
    # measures 41 ms. The tool is right; the first expectation was not.
    want = [41, 81, 201]
    got = sorted(round(ms) for _, ms in g2)
    good = (len(g2) == 3 and got == want)
    print("selftest 3 freezes        -> %d gaps %s ms, frozen %.1f%%   %s"
          % (len(g2), got, 100 * fr2, "ok" if good else "*** FAILED ***"))
    ok &= good

    # a 15 Hz comb must read as MANY gaps, not a few -- the buzz case
    v3 = [(i // 67) * 0.001 for i in range(5000)]
    g3, _, _ = gaps_from_series(t, v3)
    good = len(g3) > 50
    print("selftest 15 Hz comb       -> %d gaps (want >50, a buzz)   %s"
          % (len(g3), "ok" if good else "*** FAILED ***"))
    ok &= good

    # under the bar must not be reported
    v4 = [(i // 10) * 0.001 for i in range(5000)]
    g4, _, _ = gaps_from_series(t, v4)
    good = len(g4) == 0
    print("selftest 10 ms holds      -> %d gaps (want 0, under the bar)   %s"
          % (len(g4), "ok" if good else "*** FAILED ***"))
    ok &= good
    return ok


def main(argv):
    if '--selftest' in argv:
        return 0 if selftest() else 1
    if len(argv) < 2:
        print(__doc__)
        return 2
    import os
    p = argv[1]
    if os.path.isdir(p):
        return from_bag(p)
    if p.endswith('.csv'):
        return from_csv(p)
    return from_bag(p)


if __name__ == '__main__':
    sys.exit(main(sys.argv))
