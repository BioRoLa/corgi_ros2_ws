#!/usr/bin/env python3
"""Launch-stride validity check for a cambered arc (registered rule, log 325.43).

Run right after the bag closes, before the next arc:
    python3 launch_check.py <bag>.db3
Prints the flight fraction (IMU a_z < 0.5 g proxy, the same statistic as
l_trend.py) and the per-leg peak loads of the first three complete stride
windows, then a one-word verdict on the REGISTERED rule (log 325.43):
    VALID    - no window among 1-3 has flight fraction > 0.9
    VOID     - ANY window among 1-3 has flight fraction > 0.9 (bad launch: re-run the id)
    UNSCORED - a window lacks IMU samples or fewer than 3 complete windows exist
Until 2026-09-08 the verdict read window 1 only (the superseded 325.40 wording)
and passed s2_ol10_a2 (0.68 / 0.99 / 0.72); see log 325.46. Self-test:
run_launch_check.sh (roll1 VALID, a2 VOID at window 2).
Needs ROS sourced (rclpy + the corgi_msgs). Read-only on the bag.
"""
import sys, sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

RULE = 0.9
L4 = "abcd"
DB = sys.argv[1]
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
if not pairs:
    sys.exit("no complete trigger window in the bag -- was the trigger released before the bag closed?")
T0, TOFF = max(pairs, key=lambda p: p[1] - p[0]); gait = TOFF - T0

ms = rd("/motor/state"); mc = rd("/motor/command"); imu = rd("/imu")
ts_ = np.array([t - T0 for t, _ in ms]); tc_ = np.array([t - T0 for t, _ in mc]); ti = np.array([t - T0 for t, _ in imu])
tr = {l: np.array([getattr(m, "module_" + l).torque_r for _, m in ms]) for l in L4}
tl = {l: np.array([getattr(m, "module_" + l).torque_l for _, m in ms]) for l in L4}
tH = {l: np.array([getattr(m, "module_" + l).torque_h for _, m in ms]) for l in L4}
acc_z = np.array([m.linear_acceleration.z for _, m in imu])
pre = (ti > 0.2) & (ti < 2.0)
az0 = np.median(acc_z[pre])

# stride = window between kp_r edges on module A after the settle (same as l_trend.py)
kpr = np.array([m.module_a.kp_r for _, m in mc]); low = kpr < 100
on_ = tc_[1:][(~low[:-1]) & low[1:]]
on_ = on_[(on_ > 2.6) & (on_ < gait - 0.3)]
n = len(on_) - 1
print("gait %.1f s, %d complete strides (controller count = %d)" % (gait, n, n + 2))
if n < 2:
    sys.exit("too few strides to score the launch")

print(" stride | flight_frac | peak load A/B/C/D (N.m) | |tau_h| C / D")
ffs = []
for i in range(min(3, n)):
    a, b = on_[i], on_[i + 1]
    w = (ts_ >= a) & (ts_ < b); wi = (ti >= a) & (ti < b)
    ff = float(np.mean(acc_z[wi] < 0.5 * az0)) if wi.sum() > 3 else float("nan")
    pk = [(np.abs(tr[l][w]) + np.abs(tl[l][w])).max() for l in L4]
    print("   %d    |    %.2f     | %5.1f %5.1f %5.1f %5.1f      | %5.1f / %5.1f"
          % (i + 1, ff, pk[0], pk[1], pk[2], pk[3], np.abs(tH["c"][w]).max(), np.abs(tH["d"][w]).max()))
    ffs.append(ff)

# Registered rule (log 325.43): VOID if ANY of the first three complete stride
# windows has flight fraction > 0.90. Until 2026-09-08 this verdict was taken
# from window 1 only (the superseded 325.40 wording) and passed s2_ol10_a2
# (0.68 / 0.99 / 0.72) -- the arc the rule was written to catch (log 325.46).
bad = [(k + 1, f) for k, f in enumerate(ffs) if not np.isnan(f) and f > RULE]
unscored = [k + 1 for k, f in enumerate(ffs) if np.isnan(f)]
if bad:
    k, f = max(bad, key=lambda kf: kf[1])
    print("VOID (bad launch: window %d flight fraction %.2f > %.1f -- re-run the id)" % (k, f, RULE))
elif unscored or len(ffs) < 3:
    print("UNSCORED (windows %s lack IMU samples, or fewer than 3 complete windows) -- not a VALID"
          % (unscored if unscored else "n<3"))
else:
    print("VALID (max flight fraction %.2f over windows 1-3 <= %.1f)" % (max(ffs), RULE))
