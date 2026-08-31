#!/usr/bin/env python3
"""Live per-channel power readout -- and the way to settle which board is
which side.

The panel shows one current per board. That is a sum, and a sum cannot tell
you WHICH leg is drawing. This prints all 16 channels per board so you can
load one leg at a time and watch which channels answer.

DETERMINING PB1/PB2 -> LEFT/RIGHT (about a minute, robot in the straps):

    1. Bring the robot to Live (STANDBY) so the motors hold.
    2. Run this with --watch.
    3. Push ONE leg against its hold, firmly, for a couple of seconds.
       The motors resisting will draw current.
    4. The channels that light up belong to that leg's board.
    5. Repeat with a leg on the other side to confirm the complement.

Nothing in the banked air-pronk data can substitute for this: in a pronk
all four legs move in phase, so every channel correlates with every leg
(r = 0.2-0.5, and the per-leg argmax disagrees between runs). Correlation
was tried on the 2026-08-31 runs and refused to separate them.

Channel 0 is printed but marked: it is a near-constant ~13.5 A that does not
track load and correlates negatively with torque. It is not load current,
which is why the panel excludes it from the total.

Usage:
    python3 power_channels.py --watch          # live table
    python3 power_channels.py --watch --hz 5
    python3 power_channels.py <capture>.csv    # same table from a recording
"""

import argparse
import sys

LOAD_CHANNELS = tuple(range(1, 8))
BOARDS = ('pb1', 'pb2')


def _bar(value, full=3.0, width=18):
    """A crude bar so a channel waking up is visible at a glance."""
    if value != value:                                   # NaN
        return ' ' * width
    n = int(max(0.0, min(1.0, abs(value) / full)) * width)
    return ('#' * n).ljust(width)


def render(sample, note=''):
    out = []
    if note:
        out.append(note)
    for b in BOARDS:
        total = sum(sample.get('%s_i_%d' % (b, k), 0.0) for k in LOAD_CHANNELS)
        v = sample.get('%s_v_0' % b, float('nan'))
        out.append('%s   bus %.2f V   load %.2f A   %.0f W'
                   % (b.upper(), v, total, v * total if v == v else 0.0))
        for k in range(8):
            i = sample.get('%s_i_%d' % (b, k), float('nan'))
            tag = '  <- not load current, excluded' if k == 0 else ''
            out.append('   i_%d %8.3f A |%s|%s' % (k, i, _bar(i), tag))
    return '\n'.join(out)


def from_csv(path):
    import csv
    import statistics as st
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print('empty capture'); return 1

    def med(col):
        vals = []
        for r in rows:
            try:
                vals.append(float(r[col]))
            except (KeyError, TypeError, ValueError):
                pass
        return st.median(vals) if vals else float('nan')

    sample = {}
    for b in BOARDS:
        sample['%s_v_0' % b] = med('%s_v_0' % b)
        for k in range(8):
            sample['%s_i_%d' % (b, k)] = med('%s_i_%d' % (b, k))
    print(render(sample, 'medians over %d rows of %s' % (len(rows), path)))
    return 0


def watch(hz):
    import rclpy
    from rclpy.node import Node
    from corgi_msgs.msg import PowerStateStamped

    class Watch(Node):
        def __init__(self):
            super().__init__('power_channels')
            self.latest = None
            self.create_subscription(PowerStateStamped, 'power/state',
                                     self._cb, 10)
            self.create_timer(1.0 / max(0.5, hz), self._print)
            self.seen = False

        def _cb(self, msg):
            self.latest = msg

        def _print(self):
            if self.latest is None:
                if not self.seen:
                    print('waiting for power/state ... (is the bridge up?)')
                return
            self.seen = True
            s = {}
            for b in BOARDS:
                s['%s_v_0' % b] = float(getattr(self.latest, '%s_v_0' % b, 0.0))
                for k in range(8):
                    f = '%s_i_%d' % (b, k)
                    s[f] = float(getattr(self.latest, f, 0.0))
            # cheap screen clear so the table stays in place
            sys.stdout.write('\033[H\033[J')
            sys.stdout.write(render(s, 'push ONE leg and watch which channels '
                                       'answer   (ctrl-C to stop)\n'))
            sys.stdout.flush()

    rclpy.init()
    node = Watch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('capture', nargs='?', help='a recorded .csv')
    ap.add_argument('--watch', action='store_true', help='live from power/state')
    ap.add_argument('--hz', type=float, default=4.0)
    a = ap.parse_args()

    if a.watch:
        return watch(a.hz)
    if a.capture:
        return from_csv(a.capture)
    ap.print_help()
    return 2


if __name__ == '__main__':
    sys.exit(main())
