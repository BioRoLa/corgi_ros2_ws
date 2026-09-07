#!/usr/bin/env python3
"""Why do the front legs stop hopping when camber is commanded?

Compares a cambered run against a straight one on the things camber could plausibly break:
  - is the camber actually being DELIVERED (cmd gamma vs measured gamma per leg)
  - is the ABAD gain wrapping (#36: kp_h > 500 wraps the 12-bit CAN field)
  - front vs rear load, impulse share and theta amplitude
  - body pitch and roll under camber
  - per-leg vertical reach proxy: theta amplitude and the stance/flight split
Usage: l15_diag.py cambered.db3 [straight_reference.db3]
"""
import sys, sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

L4 = "abcd"
NM = {"a": "A FL", "b": "B FR", "c": "C RR", "d": "D RL"}


def analyse(DB, tag):
    con = sqlite3.connect("file:%s?mode=ro" % DB, uri=True)
    T = {n: (i, get_message(t)) for i, n, t in con.execute("SELECT id,name,type FROM topics")}

    def rd(n):
        i, M = T[n]
        return [(ts * 1e-9, deserialize_message(d, M))
                for ts, d in con.execute(
                    "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (i,))]

    trig = rd("/trigger"); pairs = []; on = None
    for ts, m in trig:
        if m.enable and on is None:
            on = ts
        elif not m.enable and on is not None:
            pairs.append((on, ts)); on = None
    if not pairs:
        print("%s: no trigger cycle" % tag); return
    T0, TOFF = max(pairs, key=lambda p: p[1] - p[0]); gait = TOFF - T0

    ms = rd("/motor/state"); mc = rd("/motor/command"); imu = rd("/imu")
    ts_ = np.array([t - T0 for t, _ in ms]); tc_ = np.array([t - T0 for t, _ in mc])
    ti = np.array([t - T0 for t, _ in imu])

    gm = {l: np.degrees([getattr(m, "module_" + l).gamma for _, m in ms]) for l in L4}
    gc = {l: np.degrees([getattr(m, "module_" + l).gamma for _, m in mc]) for l in L4}
    th = {l: np.degrees([getattr(m, "module_" + l).theta for _, m in ms]) for l in L4}
    thc = {l: np.degrees([getattr(m, "module_" + l).theta for _, m in mc]) for l in L4}
    tr = {l: np.array([getattr(m, "module_" + l).torque_r for _, m in ms]) for l in L4}
    tl = {l: np.array([getattr(m, "module_" + l).torque_l for _, m in ms]) for l in L4}
    tH = {l: np.array([getattr(m, "module_" + l).torque_h for _, m in ms]) for l in L4}
    kph = {l: np.array([getattr(m, "module_" + l).kp_h for _, m in mc]) for l in L4}

    q = np.array([[m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z] for _, m in imu])
    w_, x_, y_, z_ = q.T
    roll = np.degrees(np.arctan2(2 * (w_ * x_ + y_ * z_), 1 - 2 * (x_ ** 2 + y_ ** 2)))
    sinp = np.clip(2 * (w_ * y_ - z_ * x_), -1, 1)
    pitch = np.degrees(np.arcsin(sinp))
    gyr = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for _, m in imu])

    w = (ts_ > 2.6) & (ts_ < gait); wc = (tc_ > 2.6) & (tc_ < gait); wi = (ti > 2.6) & (ti < gait)
    pre = (ti > -2.0) & (ti < -0.2)
    if pre.sum() < 20:
        pre = (ti > 0.2) & (ti < 2.0)
    prem = (ts_ > 0.2) & (ts_ < 2.0)

    print("\n================ %s   (gait %.1f s)" % (tag, gait))
    print("CAMBER: is it delivered?    cmd mean / measured mean / measured range   (deg)")
    for l in L4:
        print("   %-5s cmd %+6.2f   meas %+6.2f   meas [%+6.2f,%+6.2f]   pre-trigger meas %+6.2f"
              % (NM[l], gc[l][wc].mean(), gm[l][w].mean(), gm[l][w].min(), gm[l][w].max(),
                 np.median(gm[l][prem])))

    print("ABAD GAIN (#36: values > 500 wrap the 12-bit CAN field)")
    for l in L4:
        k = kph[l][wc]
        print("   %-5s kp_h cmd  min %6.1f  median %6.1f  max %6.1f   %s"
              % (NM[l], k.min(), np.median(k), k.max(), "*** OVER 500 ***" if k.max() > 500 else "ok"))

    print("PER-LEG: theta sweep, load, impulse share")
    imp = {}
    for l in L4:
        imp[l] = float(np.trapz(np.maximum(tl[l][w] - tr[l][w], 0), ts_[w]))
    tot = sum(imp.values()) or 1.0
    for l in L4:
        amp = th[l][w].max() - th[l][w].min()
        ampc = thc[l][wc].max() - thc[l][wc].min()
        pk = (np.abs(tr[l][w]) + np.abs(tl[l][w])).max()
        print("   %-5s theta meas %6.1f-%6.1f (amp %5.1f, cmd amp %5.1f)  peak %5.1f N.m  |tau_h| max %4.1f  impulse share %4.2f"
              % (NM[l], th[l][w].min(), th[l][w].max(), amp, ampc, pk, np.abs(tH[l][w]).max(), imp[l] / tot))
    f = (imp["a"] + imp["b"]); r = (imp["c"] + imp["d"])
    print("   front impulse %.2f   rear %.2f   rear/front %.2f" % (f, r, r / f if f else float("nan")))

    print("ATTITUDE: pitch %+.2f deg (pre %+.2f), roll %+.2f (pre %+.2f), yaw rate %+.2f deg/s"
          % (pitch[wi].mean(), np.median(pitch[pre]), roll[wi].mean(), np.median(roll[pre]),
             np.degrees(np.mean(gyr[wi, 2] - np.median(gyr[pre, 2])))))
    print("   pitch range over the gait %+.2f to %+.2f deg" % (pitch[wi].min(), pitch[wi].max()))


for k, p in enumerate(sys.argv[1:]):
    analyse(p, p.split("/")[-1].replace("_0.db3", ""))
