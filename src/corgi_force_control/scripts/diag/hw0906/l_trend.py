#!/usr/bin/env python3
"""Does the gait DEGRADE within the arc, and what accumulates while it does?

Per stride: commanded and measured camber, front/rear theta sweep, front/rear peak load,
body pitch and roll, ABAD torque, and the bus. If the front drags progressively, one of
these should trend with it.
Usage: l_trend.py bag.db3
"""
import sys, sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

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
T0, TOFF = max(pairs, key=lambda p: p[1] - p[0]); gait = TOFF - T0

ms = rd("/motor/state"); mc = rd("/motor/command"); imu = rd("/imu"); ps = rd("/power/state")
ts_ = np.array([t - T0 for t, _ in ms]); tc_ = np.array([t - T0 for t, _ in mc])
ti = np.array([t - T0 for t, _ in imu]); tp = np.array([t - T0 for t, _ in ps])

gm = {l: np.degrees([getattr(m, "module_" + l).gamma for _, m in ms]) for l in L4}
gc = {l: np.degrees([getattr(m, "module_" + l).gamma for _, m in mc]) for l in L4}
th = {l: np.degrees([getattr(m, "module_" + l).theta for _, m in ms]) for l in L4}
tr = {l: np.array([getattr(m, "module_" + l).torque_r for _, m in ms]) for l in L4}
tl = {l: np.array([getattr(m, "module_" + l).torque_l for _, m in ms]) for l in L4}
tH = {l: np.array([getattr(m, "module_" + l).torque_h for _, m in ms]) for l in L4}

q = np.array([[m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z] for _, m in imu])
w_, x_, y_, z_ = q.T
roll = np.degrees(np.arctan2(2 * (w_ * x_ + y_ * z_), 1 - 2 * (x_ ** 2 + y_ ** 2)))
pitch = np.degrees(np.arcsin(np.clip(2 * (w_ * y_ - z_ * x_), -1, 1)))
gyr = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for _, m in imu])
acc = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for _, m in imu])
pre = (ti > 0.2) & (ti < 2.0)
gb = np.median(gyr[pre], 0); az0 = np.median(acc[pre, 2])

v01 = np.array([m.pb1_v_0 for _, m in ps])
Is = np.array([sum(getattr(m, "pb1_i_%d" % k) + getattr(m, "pb2_i_%d" % k) for k in range(1, 8)) for _, m in ps])

kpr = np.array([m.module_a.kp_r for _, m in mc]); low = kpr < 100
on_ = tc_[1:][(~low[:-1]) & low[1:]]
on_ = on_[(on_ > 2.6) & (on_ < gait - 0.3)]

print("gait %.1f s, %d strides" % (gait, len(on_)))
print(" i   t(s) | cmd|g| meas gA  gD | thA  thB  thC  thD |  pkA   pkB   pkC   pkD | tauH_D | pitch  roll | flight_frac | ISum vmin")
rows = []
for i in range(len(on_) - 1):
    a, b = on_[i], on_[i + 1]
    w = (ts_ >= a) & (ts_ < b); wc = (tc_ >= a) & (tc_ < b)
    wi = (ti >= a) & (ti < b); wp = (tp >= a) & (tp < b)
    if w.sum() < 20:
        continue
    sw = {l: th[l][w].max() - th[l][w].min() for l in L4}
    pk = {l: (np.abs(tr[l][w]) + np.abs(tl[l][w])).max() for l in L4}
    # flight fraction proxy: vertical accel below 1 g means unloaded
    ff = float(np.mean(acc[wi, 2] < 0.5 * az0)) if wi.sum() > 3 else float("nan")
    rows.append((i, a, np.median(np.abs(gc["a"][wc])), np.median(gm["a"][w]), np.median(gm["d"][w]),
                 sw["a"], sw["b"], sw["c"], sw["d"], pk["a"], pk["b"], pk["c"], pk["d"],
                 np.abs(tH["d"][w]).max(), np.mean(pitch[wi]), np.mean(roll[wi]), ff,
                 Is[wp].max() if wp.any() else float("nan"),
                 v01[wp].min() if wp.any() else float("nan")))
    r = rows[-1]
    print("%2d %6.2f | %6.2f %6.2f %6.2f | %4.1f %4.1f %4.1f %4.1f | %5.1f %5.1f %5.1f %5.1f | %6.1f | %+5.2f %+5.2f | %6.2f | %5.1f %6.2f"
          % r)

if len(rows) >= 6:
    n = len(rows); h = n // 2
    A = np.array([r[1:] for r in rows], dtype=float)
    names = ["cmd|g|", "meas gA", "meas gD", "thA", "thB", "thC", "thD",
             "pkA", "pkB", "pkC", "pkD", "tauH_D", "pitch", "roll", "flight_frac", "ISum", "vmin"]
    print("\nFIRST HALF vs LAST HALF (the question is what accumulates)")
    print("  %-12s %8s %8s %8s" % ("quantity", "first", "last", "change"))
    for k, nm in enumerate(names):
        f = np.nanmean(A[:h, k + 1]); l = np.nanmean(A[h:, k + 1])
        print("  %-12s %8.2f %8.2f %+8.2f" % (nm, f, l, l - f))
