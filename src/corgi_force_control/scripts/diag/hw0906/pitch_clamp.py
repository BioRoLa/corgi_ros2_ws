#!/usr/bin/env python3
"""Is the pitch channel actually clamped?

pitch_limit_ is NOT a limit on body pitch. It is a clamp, in radians, on the THETA
perturbation u = k_pitch*pitch + d_pitch*wy (gslip_pronk.cpp:2725-2726, and the member
comment at :749 "rad, clamp on the theta perturbation"). So comparing a 4.9 deg body
pitch against a 3.0 deg pitch_limit is a units error. Reconstruct u and measure it.
Usage: pitch_clamp.py bag.db3 [k_pitch] [d_pitch] [pitch_limit_deg]
"""
import sys, sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

DB = sys.argv[1]
KP = float(sys.argv[2]) if len(sys.argv) > 2 else 0.30
DP = float(sys.argv[3]) if len(sys.argv) > 3 else 0.10
LIM_DEG = float(sys.argv[4]) if len(sys.argv) > 4 else 3.0
LIM = np.radians(LIM_DEG)

con = sqlite3.connect("file:%s?mode=ro" % DB, uri=True)
T = {n: (i, get_message(t)) for i, n, t in con.execute("SELECT id,name,type FROM topics")}


def rd(n):
    i, M = T[n]
    return [(ts * 1e-9, deserialize_message(d, M))
            for ts, d in con.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (i,))]


trig = rd("/trigger"); pairs = []; on = None
for ts, m in trig:
    if m.enable and on is None:
        on = ts
    elif not m.enable and on is not None:
        pairs.append((on, ts)); on = None
T0, TOFF = max(pairs, key=lambda p: p[1] - p[0]); gait = TOFF - T0

imu = rd("/imu"); mc = rd("/motor/command")
ti = np.array([t - T0 for t, _ in imu]); tc_ = np.array([t - T0 for t, _ in mc])
q = np.array([[m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z] for _, m in imu])
w_, x_, y_, z_ = q.T
pitch = np.arcsin(np.clip(2 * (w_ * y_ - z_ * x_), -1, 1))            # rad
wy = np.array([m.angular_velocity.y for _, m in imu])                 # rad/s

# stance only, the gate pitch_correction applies: kp_r drops out of the flight set
kpr = np.array([m.module_a.kp_r for _, m in mc])
stance_c = kpr < 100
st = np.interp(ti, tc_, stance_c.astype(float)) > 0.5

w = (ti > 2.6) & (ti < gait)
P = KP * pitch
D = DP * wy
u = P + D

for name, mask in (("whole gait", w), ("stance only", w & st)):
    if mask.sum() < 50:
        continue
    uu = u[mask]
    print("%-12s  n=%6d" % (name, mask.sum()))
    print("   |u| median %5.2f deg   p90 %5.2f   max %5.2f   (clamp %.2f deg)"
          % (np.degrees(np.median(np.abs(uu))), np.degrees(np.percentile(np.abs(uu), 90)),
             np.degrees(np.abs(uu).max()), LIM_DEG))
    print("   CLAMPED (|u| >= limit): %5.1f %% of samples" % (100.0 * np.mean(np.abs(uu) >= LIM)))
    print("   term split: |k_pitch*pitch| median %5.2f deg, |d_pitch*wy| median %5.2f deg"
          % (np.degrees(np.median(np.abs(P[mask]))), np.degrees(np.median(np.abs(D[mask])))))
    print("   body pitch over the window: median %+5.2f deg, range %+5.2f to %+5.2f"
          % (np.degrees(np.median(pitch[mask])), np.degrees(pitch[mask].min()), np.degrees(pitch[mask].max())))
    print("   pitch RATE |wy|: median %5.2f deg/s, p95 %6.2f" %
          (np.degrees(np.median(np.abs(wy[mask]))), np.degrees(np.percentile(np.abs(wy[mask]), 95))))
    print()

# how much would a bigger clamp actually buy?
mask = w & st
for lim_deg in (3.0, 4.0, 5.0, 6.0, 8.0):
    lim = np.radians(lim_deg)
    print("   clamp %4.1f deg -> clamped %5.1f %% of stance, delivered |u| mean %5.2f deg"
          % (lim_deg, 100.0 * np.mean(np.abs(u[mask]) >= lim),
             np.degrees(np.mean(np.minimum(np.abs(u[mask]), lim)))))
