#!/usr/bin/env python3
"""leg_duty.py -- per-leg stance duty against the template's DESIGN value.

The complaint "the front is dragging / the rear is hopping" names one end of a
comparison. The lesson of log 325.38 and the memory
`test-which-pair-is-off-design-not-which-matches-the-complaint` is to tabulate
BOTH ends against the design value before diagnosing either: last time the
front turned out to be at design (0.411 against a design 0.41) and the REAR was
the off-design pair at 0.252.

There is no contact sensor on hardware, so stance is taken from the leg's own
load: a leg is in stance while its |torque_r| + |torque_l| exceeds a fraction
of that leg's own per-stride peak. The threshold is swept so the reader can see
the duty is not an artefact of one choice.

Design duty comes from the template itself (in_stance rows / all rows).

Usage: leg_duty.py [--selftest] [--template CSV] bag.db3 [bag.db3 ...]
Needs ROS sourced. Read-only.
"""
import argparse
import csv
import io
import os
import sqlite3
import sys

import numpy as np

LEGS = ["a", "b", "c", "d"]
NAMES = {"a": "A FL", "b": "B FR", "c": "C RR", "d": "D RL"}
FRACS = [0.15, 0.25, 0.35]          # threshold sweep, fraction of the leg's own peak
DEFAULT_TEMPLATE = ("/home/biorola/corgi_ws/corgi_ros2_ws/src/corgi_force_control/"
                    "config/gslip_pronk_template_v070.csv")


def design_duty(path):
    try:
        with io.open(path) as f:
            rows = list(csv.DictReader(f))
        st = [int(r["in_stance"]) for r in rows]
        return float(sum(st)) / len(st)
    except Exception as e:
        print("  (template unreadable: %s)" % e)
        return float("nan")


def duty_from_load(t, load, frac):
    """Fraction of time above frac * (95th pct of load). Robust to spikes."""
    pk = float(np.percentile(load, 95))
    if pk <= 0:
        return float("nan")
    return float(np.mean(load > frac * pk))


def selftest():
    ok = True

    def chk(name, got, want, tol):
        nonlocal ok
        good = abs(got - want) <= tol
        ok = ok and good
        print("  %-44s got %7.4f  want %7.4f  %s"
              % (name, got, want, "PASS" if good else "FAIL"))

    # a square wave that is "loaded" 40 % of the time, like the template
    t = np.arange(0, 10, 0.001)
    period = 0.25
    phase = (t % period) / period
    load = np.where(phase < 0.40, 10.0, 0.0)
    for f in FRACS:
        chk("square wave, 40 %% duty, thresh %.2f" % f, duty_from_load(t, load, f), 0.40, 0.01)
    load2 = np.where(phase < 0.25, 10.0, 0.0)
    chk("square wave, 25 % duty", duty_from_load(t, load2, 0.25), 0.25, 0.01)
    # a spike must not move the 95th-percentile-based threshold much
    load3 = load.copy()
    load3[5000] = 500.0
    chk("one 50x spike barely moves it", duty_from_load(t, load3, 0.25), 0.40, 0.01)
    print("SELF-TEST %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def load_bag(db):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    con = sqlite3.connect("file:%s?mode=ro" % db, uri=True)
    T = {n: (i, get_message(ty)) for i, n, ty in con.execute("SELECT id,name,type FROM topics")}

    def rd(n):
        i, M = T[n]
        return [(ts * 1e-9, deserialize_message(d, M)) for ts, d in
                con.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
                            (i,))]

    trig = rd("/trigger")
    pairs, on = [], None
    for ts, m in trig:
        if m.enable and on is None:
            on = ts
        elif not m.enable and on is not None:
            pairs.append((on, ts))
            on = None
    if on is not None:
        pairs.append((on, trig[-1][0]))
    if not pairs:
        raise SystemExit("no trigger cycle in %s" % db)
    T0, TOFF = max(pairs, key=lambda p: p[1] - p[0])
    ms = rd("/motor/state")
    ts_ = np.array([x for x, _ in ms])
    w = (ts_ >= T0 + 0.5) & (ts_ <= TOFF)      # skip the launch transient
    out = {"t": ts_[w], "gait": TOFF - T0}
    for l in LEGS:
        mod = [getattr(m, "module_" + l) for _, m in ms]
        tr = np.array([x.torque_r for x in mod])[w]
        tl = np.array([x.torque_l for x in mod])[w]
        out[l] = np.abs(tr) + np.abs(tl)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="*")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--template", default=DEFAULT_TEMPLATE)
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if not a.bags:
        ap.error("give a bag, or --selftest")
    if selftest() != 0:
        print("REFUSING to analyse: self-test failed")
        return 1
    d_duty = design_duty(a.template)
    print("\ndesign stance duty from the template: %.3f\n" % d_duty)
    for db in a.bags:
        try:
            b = load_bag(db)
        except SystemExit as e:
            print("%s: %s" % (os.path.basename(db), e))
            continue
        print("=== %s   (gait %.1f s, %d samples)"
              % (os.path.basename(os.path.dirname(db)), b["gait"], len(b["t"])))
        print("  leg    duty @0.15 @0.25 @0.35   vs design (@0.25)   peak load  mean load")
        for l in LEGS:
            duties = [duty_from_load(b["t"], b[l], f) for f in FRACS]
            pk = float(np.percentile(b[l], 95))
            mn = float(np.mean(b[l]))
            delta = duties[1] - d_duty
            flag = "" if abs(delta) <= 0.05 else ("  <-- OFF DESIGN %+.3f" % delta)
            print("  %-6s  %.3f  %.3f  %.3f       %+.3f            %6.1f     %6.1f%s"
                  % (NAMES[l], duties[0], duties[1], duties[2], delta, pk, mn, flag))
        fr = (np.mean(b["a"]) + np.mean(b["b"])) / 2.0
        re = (np.mean(b["c"]) + np.mean(b["d"])) / 2.0
        print("  front mean load %.1f   rear %.1f   rear/front %.2f" % (fr, re, re / fr if fr else float("nan")))
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
