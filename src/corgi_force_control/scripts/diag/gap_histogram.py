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
        return [], (0.0, 0.0), 0.0, None
    span = change_t[-1] - change_t[0]

    gaps = []
    for a, b in zip(change_t, change_t[1:]):
        ms = (b - a) * 1000.0
        if ms > bar_ms:
            # Times are BAG-relative, so gaps on different series -- and the
            # jerks on the video -- can be lined up with each other.
            gaps.append((a, ms))

    # A hold starting at the very first change is initialisation, not a
    # freeze: force_control holds a constant pose until the first impedance
    # command arrives. Counting it made one run read 97.1% frozen off a
    # single 1912 ms leading hold.
    # Within 50 ms of the first change, not exactly at it: a single
    # initialisation step (an uninitialised 0 -> the clamped pose) often
    # lands just before the hold and used to defeat this.
    leading = None
    if gaps and gaps[0][0] <= change_t[0] + 0.050:
        leading = gaps.pop(0)
        span = change_t[-1] - (leading[0] + leading[1] / 1000.0)

    frozen = sum(ms for _, ms in gaps) / 1000.0
    return gaps, (change_t[0], change_t[-1]), \
        (frozen / span if span > 0 else 0.0), leading


def step_report(label, times_s, values, factor=10.0, deg=True):
    """Per-tick step distribution, and every outlier above factor x median.

    A jerk is a JUMP. Holds are what everything else here measures, and a
    stream can hold nothing while still stepping violently.
    """
    import statistics
    steps = []
    for (t0, v0), (t1, v1) in zip(zip(times_s, values), zip(times_s[1:], values[1:])):
        d = abs(v1 - v0)
        if d > 0:
            steps.append((t1, d))
    if len(steps) < 20:
        print("  %-16s too few moves to characterise" % label)
        return 0

    mags = sorted(d for _, d in steps)
    med = statistics.median(mags)
    p99 = mags[int(0.99 * (len(mags) - 1))]
    biggest = mags[-1]
    k = 180.0 / 3.141592653589793 if deg else 1.0

    print("  %-16s steps: median %8.4f  p99 %8.4f  max %8.4f %s  (%d moves)"
          % (label, med * k, p99 * k, biggest * k,
             "deg" if deg else "", len(steps)))
    if med <= 0:
        return 0

    bar = factor * med
    out = [(t, d) for t, d in steps if d > bar]
    print("        %d step(s) over %.0fx the median (%.4f %s):"
          % (len(out), factor, bar * k, "deg" if deg else ""))
    for t, d in out[:25]:
        print("          at t=%8.3f s   step %8.4f %s   (%.0fx median)"
              % (t, d * k, "deg" if deg else "", d / med))
    if len(out) > 25:
        print("          ... and %d more" % (len(out) - 25))
    return len(out)


def monotonicity(values):
    """|net travel| / |total travel|, in [0, 1].

    Near 1: a ramp -- almost all movement is in one direction, and any hold
    in it is a genuine freeze. Near 0: a gait -- it comes back to where it
    started, so holds are the shape of the signal, not a fault.

    Counting sign changes instead was the first attempt, and it refused a
    real standup for "50 reversals": measured theta wobbles by a
    micro-radian on the way up and every wobble looked like a turn.
    """
    total = 0.0
    prev = None
    for v in values:
        if prev is not None:
            total += abs(v - prev)
        prev = v
    if total <= 0:
        return 1.0
    return abs(values[-1] - values[0]) / total


def report(label, times_s, values, bar_ms=BAR_MS, score_window=None):
    """Print one series. score_window restricts what COUNTS as a freeze.

    The window and the series' own extent are deliberately separate names:
    the first version called both `window`, the argument was overwritten by
    the return value on the first line, and every gap was silently kept.
    """
    gaps, extent, _frozen_all, leading = gaps_from_series(times_s, values,
                                                          bar_ms)
    t_first, t_last = extent
    if t_last <= t_first:
        print("  %-16s no usable changes" % label)
        return 0

    mono = monotonicity(values)
    if mono < 0.5:
        print("  %-16s NOT MONOTONE (%.2f) -- not scored" % (label, mono))
        print("        Only %.0f%% of this signal's travel is net, so it "
              "returns to where it" % (100 * mono))
        print("        started -- a gait, or a toggling term. Holds are then "
              "the SHAPE of the")
        print("        signal, not freezes: scoring them reported the healthy "
              "air run 3 as")
        print("        '223 gaps, 61.5% frozen'. This test is for the "
              "monotone standup ramp;")
        print("        for a gait use cmd_rate.py on cmd_beta_a.")
        return 0

    outside = []
    if score_window is not None:
        w0, w1 = score_window
        outside = [(t, ms) for t, ms in gaps if not (w0 <= t <= w1)]
        gaps = [(t, ms) for t, ms in gaps if w0 <= t <= w1]
        scored_span = min(t_last, w1) - max(t_first, w0)
    else:
        scored_span = t_last - t_first

    frozen = (sum(ms for _, ms in gaps) / 1000.0 / scored_span
              if scored_span > 0 else 0.0)

    # The MESSAGE span, beside the CHANGE span. A series that is
    # published throughout but holds one value is a different fault
    # from one that is not published at all, and the change window
    # alone cannot tell them apart.
    print("  %-16s msgs %6.2f-%6.2f s (%d)"
          % (label, times_s[0], times_s[-1], len(times_s)))
    print("  %-16s active %6.2f-%6.2f s   gaps>%.0fms: %-4d   frozen "
          "%5.1f%%   worst %6.1f ms"
          % (label, t_first, t_last, bar_ms, len(gaps), 100.0 * frozen,
             max((ms for _, ms in gaps), default=0.0)))
    if leading is not None:
        print("        (leading hold %.1f ms at the series start -- "
              "initialisation," % leading[1])
        print("         not a freeze; excluded from the count and the "
              "frozen fraction)")
    for t, ms in gaps:
        print("        at t=%6.3f s   held %6.1f ms" % (t, ms))
    if outside:
        print("        %d gap(s) OUTSIDE the commanded window, NOT counted "
              "-- the producer" % len(outside))
        print("        was not publishing then, so a held command is not a "
              "freeze:")
        for t, ms in outside:
            print("          at t=%6.3f s   held %6.1f ms" % (t, ms))
    return len(gaps)


def preflight_bag(path):
    """Say what is wrong with a bag before rosbag2 raises about it.

    Four states worth telling apart, because each needs a different action:
    still recording, recorded but never closed, a directory with nothing in
    it, and not a bag at all.
    """
    import glob
    import os
    import subprocess

    if not os.path.isdir(path):
        if os.path.isfile(path) and path.endswith('.db3'):
            return path, None
        return path, "not a directory and not a .db3 file: %s" % path

    db3 = sorted(glob.glob(os.path.join(path, '*.db3')))
    meta = os.path.join(path, 'metadata.yaml')

    if not db3:
        return path, ("no .db3 inside %s -- nothing was recorded there" % path)

    # A recorder still holding the file is the common case, and the sqlite
    # "disk I/O error" on a read-only open is its signature.
    try:
        out = subprocess.run(['pgrep', '-af', 'bag record'],
                             capture_output=True, text=True, timeout=5).stdout
    except Exception:
        out = ''
    if out.strip() and (os.path.basename(path) in out or path in out):
        return path, (
            "a `ros2 bag record` is STILL RUNNING on this bag:\n    %s\n"
            "Stop it first (Ctrl-C in its terminal, or pkill -f 'bag "
            "record'), then re-run.\nThe writer holds the sqlite file, and "
            "metadata.yaml is not written until it stops."
            % out.strip().splitlines()[0])

    if not os.path.exists(meta):
        return path, (
            "metadata.yaml is missing from %s, so the recording was never "
            "closed cleanly.\nEither a recorder is still running, or it was "
            "killed. Rebuild the index with:\n    ros2 bag reindex %s\n"
            "then re-run. (Pointing this script at the .db3 directly also "
            "works:\n    %s )" % (path, path, db3[0]))

    return path, None


def steps_from_bag(path, factor=10.0):
    """--steps entry point: characterise JUMPS in every commanded series."""
    path, problem = preflight_bag(path)
    if problem:
        print("cannot read the bag.\n")
        print("  " + problem.replace("\n", "\n  "))
        return 2
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        print("the rosbag2 python API is not importable (%s)." % exc)
        print("  source /opt/ros/humble/setup.bash && source install/setup.bash")
        return 2

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    want = {
        '/motor/command': [('module_a.theta', 'cmd theta A'),
                           ('module_a.beta',  'cmd beta A'),
                           ('module_b.theta', 'cmd theta B'),
                           ('module_c.theta', 'cmd theta C'),
                           ('module_d.theta', 'cmd theta D')],
        '/impedance/command': [('module_a.theta', 'imp theta A'),
                               ('module_a.beta',  'imp beta A')],
        '/motor/state':   [('module_a.theta', 'state theta A')],
    }
    series, t0 = {}, None
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
        print("no usable topics in %s" % path)
        return 2

    print("bag: %s   -- STEP analysis (jumps, not holds)\n" % path)
    print("  A jerk is a single tick moving far more than its neighbours.")
    print("  Everything else in this tool measures holds, which is why five")
    print("  hypotheses were tested without ever measuring the symptom.\n")
    total = 0
    for label in ('imp theta A', 'imp beta A', 'cmd theta A', 'cmd beta A',
                  'cmd theta B', 'cmd theta C', 'cmd theta D',
                  'state theta A'):
        if label in series:
            total += step_report(label, series[label][0], series[label][1],
                                 factor)
    print()
    if total == 0:
        print("  No outlier steps. The commanded stream is smooth -- if the")
        print("  robot is jerking, the discontinuity is DOWNSTREAM of")
        print("  /motor/command (CAN packing, firmware, or mechanical).")
    else:
        print("  Outlier steps found. Compare their timestamps against the")
        print("  jerks. If they appear on imp_* too, the producer is making")
        print("  them; if only on cmd_*, force_control is.")
    return 0


def from_bag(path):
    # Preflight BEFORE importing rosbag2: a bag that is still recording
    # and a shell with no ROS sourced are different problems, and a
    # ModuleNotFoundError should not stand in for both.
    path, problem = preflight_bag(path)
    if problem:
        print("cannot read the bag.\n")
        print("  " + problem.replace("\n", "\n  "))
        return 2

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        print('the rosbag2 python API is not importable (%s).' % exc)
        print('  Source your ROS environment first:')
        print('      source /opt/ros/humble/setup.bash && '
              'source install/setup.bash')
        return 2

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
    except Exception as exc:
        print("cannot read the bag: %s" % exc)
        print("  If a recorder is still running, stop it and re-run.")
        print("  If it was killed mid-write:  ros2 bag reindex %s" % path)
        return 2
    topics = reader.get_all_topics_and_types()
    types = {t.name: t.type for t in topics}

    # Inventory first. A bag that is missing the topics should say so before
    # a silent pass over every message, not after.
    print("bag: %s" % path)
    print("  topics recorded:")
    for t in sorted(types):
        print("    %-24s %s" % (t, types[t]))
    wanted_present = [t for t in ('/motor/command', '/motor/state',
                                  '/impedance/command') if t in types]
    if not wanted_present:
        print()
        print("  NONE of /motor/command, /motor/state, /impedance/command is")
        print("  in this bag, so there is nothing to score. Record with:")
        print("    ros2 bag record -o standup_$(date +%H%M%S) \\")
        print("        /impedance/command /motor/command /motor/state")
        print("  and start it BEFORE launching the controller.")
        return 2
    print("  reading %s ..." % ", ".join(wanted_present))
    sys.stdout.flush()

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
            import datetime
            print("  bag starts at %s  (epoch %.3f)"
                  % (datetime.datetime.fromtimestamp(tns / 1e9)
                     .strftime('%H:%M:%S.%f')[:-3], tns / 1e9))
            print("  all times below are seconds from that moment\n")
            sys.stdout.flush()
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
        print()
        print("  the topics are present but no messages were read from them.")
        print("  Most likely the bag was started and stopped without the")
        print("  controller ever publishing -- i.e. no standup happened while")
        print("  it was recording.")
        return 2

    # Per series, so a stream that was never produced can be told from
    # one the recording lost: a 1.95 s ramp at 1 kHz is ~1950 messages.
    print('  messages read:')
    for _lab in sorted(series):
        print('    %-16s %6d' % (_lab, len(series[_lab][0])))
    print()

    # The producer's active window. Anything the consumer does outside it
    # is a controller settling with no input, not a freeze under load.
    prod = series.get('imp theta A')
    window = None
    if prod:
        pg, pw, _pf, _pl = gaps_from_series(prod[0], prod[1])
        window = pw
        print("  COMMANDED WINDOW (from /impedance/command): "
              "%.2f - %.2f s\n" % pw)

    counts = {}
    for label, (ts, vs) in series.items():
        w = None if label.startswith('imp') else window
        counts[label] = report(label, ts, vs, score_window=w)

    handover(series, window)
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


def handover(series, window):
    """When did the consumer first respond to the producer?

    The number that matters if the consumer is not tracking: producer first
    change -> consumer first change. On standup_024548 that gap was 1.74 s
    of a 1.95 s ramp, so the leg had nothing new commanded for almost the
    whole extension and then received it all at once.
    """
    prod, cons = series.get('imp theta A'), series.get('cmd theta A')
    if not (prod and cons and window):
        return
    cg, cw, _cf, _cl = gaps_from_series(cons[0], cons[1])
    print("  HANDOVER")
    print("    producer first change   t = %6.3f s" % window[0])
    print("    consumer first change   t = %6.3f s" % cw[0])
    lag = cw[0] - window[0]
    ramp = window[1] - window[0]
    print("    consumer lag            %+6.3f s   (%.0f%% of the %.2f s "
          "commanded window)" % (lag, 100.0 * lag / ramp if ramp > 0 else 0,
                                 ramp))
    if lag > 0.25 * ramp:
        print("    -> the consumer ignored the command for most of the ramp.")
        print("       That is a HANDOVER problem, not CPU starvation: no")
        print("       amount of scheduling priority moves this number.")
        print("       Suspect the QoS match (S318.2: /impedance/command is")
        print("       VOLATILE, so everything published before the reader")
        print("       matches is dropped).")
    elif lag < -0.05:
        print("    -> the consumer was moving BEFORE the producer commanded")
        print("       anything: that is force_control's own position_control()")
        print("       path running on a zero-initialised imp_cmd_ (S318.2).")


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
    g, (w0, w1), fr, _lead = gaps_from_series(t, v)
    span = w1 - w0
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
    g2, _w, fr2, _l2 = gaps_from_series(t, v2)
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
    g3, _w3, _f3, _l3 = gaps_from_series(t, v3)
    good = len(g3) > 50
    print("selftest 15 Hz comb       -> %d gaps (want >50, a buzz)   %s"
          % (len(g3), "ok" if good else "*** FAILED ***"))
    ok &= good

    # under the bar must not be reported
    v4 = [(i // 10) * 0.001 for i in range(5000)]
    g4, _w4, _f4, _l4 = gaps_from_series(t, v4)
    good = len(g4) == 0
    print("selftest 10 ms holds      -> %d gaps (want 0, under the bar)   %s"
          % (len(g4), "ok" if good else "*** FAILED ***"))
    ok &= good
    return ok


def main(argv):
    if '--selftest' in argv:
        return 0 if selftest() else 1
    if '--steps' in argv:
        argv = [a for a in argv if a != '--steps']
        factor = 10.0
        if '--factor' in argv:
            i = argv.index('--factor')
            factor = float(argv[i + 1])
            del argv[i:i + 2]
        if len(argv) < 2:
            print(__doc__)
            return 2
        return steps_from_bag(argv[1], factor)
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
