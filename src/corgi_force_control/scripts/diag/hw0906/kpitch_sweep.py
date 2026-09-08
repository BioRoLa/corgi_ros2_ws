#!/usr/bin/env python3
"""kpitch_sweep.py -- what would a HIGHER k_pitch actually deliver?

Reconstructs the pitch-channel perturbation u = k_pitch*pitch + d_pitch*wy from
a bag's own IMU, exactly as gslip_pronk.cpp:2713-2728 forms it, and reports for
each (k_pitch, d_pitch, pitch_limit) how much of stance is CLAMPED and what the
delivered |u| actually is after clamping. The point: past some gain the channel
stops being a gain at all and becomes a constant offset pinned at the clamp, and
the useful knob becomes pitch_limit instead.

Stance is taken the way pitch_clamp.py takes it: module_a.kp_r < 100 marks the
stance gain set.

Usage: kpitch_sweep.py [--selftest] bag.db3 [bag.db3 ...]
Needs ROS sourced. Read-only.
"""
import argparse
import os
import sqlite3
import sys

import numpy as np

KS = [0.30, 0.45, 0.60, 0.90, 1.20]
DS = [0.10, 0.0, -0.10]
LIMS_DEG = [3.0, 5.0]


def selftest():
    ok = True

    def chk(name, got, want, tol):
        nonlocal ok
        good = abs(got - want) <= tol
        ok = ok and good
        print("  %-46s got %8.4f  want %8.4f  %s"
              % (name, got, want, "PASS" if good else "FAIL"))

    # a known signal: pitch constant at -0.10 rad, wy zero
    pitch = np.full(1000, -0.10)
    wy = np.zeros(1000)
    lim = np.radians(3.0)
    for k in (0.30, 0.60):
        u = np.clip(k * pitch + 0.0 * wy, -lim, lim)
        chk("k=%.2f on a -0.10 rad pitch, clamp 3 deg" % k,
            float(np.degrees(np.abs(u).mean())),
            min(np.degrees(k * 0.10), 3.0), 1e-6)
    # clamp actually binds
    u = np.clip(1.20 * pitch, -lim, lim)
    chk("k=1.20 saturates at the 3 deg clamp",
        float(np.degrees(np.abs(u).mean())), 3.0, 1e-6)
    chk("...and is clamped 100 % of the time",
        float(np.mean(np.abs(1.20 * pitch) >= lim)), 1.0, 1e-9)
    print("SELF-TEST %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def load(db):
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
    T0, TOFF = max(pairs, key=lambda p: p[1] - p[0])
    imu = rd("/imu")
    mc = rd("/motor/command")
    ti = np.array([t - T0 for t, _ in imu])
    tc = np.array([t - T0 for t, _ in mc])
    q = np.array([[m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z]
                  for _, m in imu])
    w_, x_, y_, z_ = q.T
    pitch = np.arcsin(np.clip(2 * (w_ * y_ - z_ * x_), -1, 1))
    wy = np.array([m.angular_velocity.y for _, m in imu])
    kpr = np.array([m.module_a.kp_r for _, m in mc])
    st = np.interp(ti, tc, (kpr < 100).astype(float)) > 0.5
    win = (ti > 0.5) & (ti < (TOFF - T0)) & st
    return pitch[win], wy[win], float(TOFF - T0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="*")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if not a.bags:
        ap.error("give a bag, or --selftest")
    if selftest() != 0:
        print("REFUSING to analyse: self-test failed")
        return 1
    for db in a.bags:
        pitch, wy, gait = load(db)
        print("\n=== %s   (stance samples %d, gait %.1f s, body pitch median %+.2f deg)"
              % (os.path.basename(os.path.dirname(db)), len(pitch), gait,
                 float(np.degrees(np.median(pitch)))))
        for lim_deg in LIMS_DEG:
            lim = np.radians(lim_deg)
            print("  clamp %.1f deg" % lim_deg)
            print("    k_pitch |   d=+0.10 clamp%% / |u|   |   d= 0.00 clamp%% / |u|   |"
                  "   d=-0.10 clamp%% / |u|")
            for k in KS:
                cells = []
                for d in DS:
                    u = k * pitch + d * wy
                    cl = 100.0 * float(np.mean(np.abs(u) >= lim))
                    du = float(np.degrees(np.mean(np.abs(np.clip(u, -lim, lim)))))
                    cells.append("%5.1f %% / %4.2f deg" % (cl, du))
                print("      %.2f  |   %s   |   %s   |   %s"
                      % (k, cells[0], cells[1], cells[2]))
    print("\nReading: once clamp%% approaches 100 the channel is no longer a gain -- it is a")
    print("constant offset pinned at the clamp, identical for every k above that point, and")
    print("the only knob that still does anything is pitch_limit.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
