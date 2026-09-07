#!/usr/bin/env python3
"""Per-stride ABAD peak |tau_h| (four legs) and per-stride mean IMU roll/pitch from the bags,
for the ABAD-torque and roll per-stride figures the registered protocol owes (log 325.25).

Stride detector exactly as l_trend.py: falling edge of module_a kp_r below 100 on /motor/command
(the flight-set switch), edges between 2.6 s after the longest trigger window's start and 0.3 s
before its end; stride i spans edge i to edge i+1. Stride index printed here is 1-based
(= l_trend.py's i + 1). Roll/pitch from the /imu quaternion as in l_trend.py (roll agrees with
the gyro; pitch is the x-mirror, see memory note) -- mean over the stride.
Also prints the registered N_sat: first stride whose any-leg ABAD peak >= 39 N.m.
Usage: figs0906_bag_strides.py out.json name=bag.db3 [...]
"""
import sys, json, sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

L4 = "abcd"
SAT = 39.0
out_path = sys.argv[1]
result = {}
for arg in sys.argv[2:]:
    name, DB = arg.split("=", 1)
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

    ms = rd("/motor/state"); mc = rd("/motor/command"); imu = rd("/imu")
    ts_ = np.array([t - T0 for t, _ in ms]); tc_ = np.array([t - T0 for t, _ in mc])
    ti = np.array([t - T0 for t, _ in imu])
    tH = {l: np.array([getattr(m, "module_" + l).torque_h for _, m in ms]) for l in L4}
    q = np.array([[m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z] for _, m in imu])
    w_, x_, y_, z_ = q.T
    roll = np.degrees(np.arctan2(2 * (w_ * x_ + y_ * z_), 1 - 2 * (x_ ** 2 + y_ ** 2)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w_ * y_ - z_ * x_), -1, 1)))
    kpr = np.array([m.module_a.kp_r for _, m in mc]); low = kpr < 100
    on_ = tc_[1:][(~low[:-1]) & low[1:]]
    on_ = on_[(on_ > 2.6) & (on_ < gait - 0.3)]

    rows = []
    for i in range(len(on_) - 1):
        a, b = on_[i], on_[i + 1]
        w = (ts_ >= a) & (ts_ < b); wi = (ti >= a) & (ti < b)
        if w.sum() < 20:
            continue
        pk = [float(np.abs(tH[l][w]).max()) for l in L4]
        rows.append(dict(k=len(rows) + 1, t=float(a), dur=float(b - a), tauH=pk,
                         roll=float(np.mean(roll[wi])) if wi.sum() else float("nan"),
                         pitch=float(np.mean(pitch[wi])) if wi.sum() else float("nan")))
    n_sat = next((r["k"] for r in rows if max(r["tauH"]) >= SAT), None)
    n_sat_leg = None
    if n_sat:
        n_sat_leg = "".join(L4[j].upper() for j, v in enumerate(rows[n_sat - 1]["tauH"]) if v >= SAT)
    print("== %-14s gait %.1f s, %d strides   N_sat(>=39 any leg) = %s (%s)   stride period median %.3f s"
          % (name, gait, len(rows), n_sat, n_sat_leg, np.median([r["dur"] for r in rows])))
    print("   k    t(s)  dur  |  |tauH| pk A     B     C     D  | roll  pitch")
    for r in rows:
        print("  %2d %6.2f %.3f | %5.1f %5.1f %5.1f %5.1f | %+5.2f %+5.2f"
              % (r["k"], r["t"], r["dur"], *r["tauH"], r["roll"], r["pitch"]))
    C = np.array([r["tauH"] for r in rows])
    print("   per-leg median peak |tauH| over strides: A %.1f  B %.1f  C %.1f  D %.1f   | max: A %.1f B %.1f C %.1f D %.1f"
          % (*np.median(C, 0), *C.max(0)))
    rr = np.array([r["roll"] for r in rows])
    print("   roll: first %.2f  last %.2f  mean %.2f  min %.2f  max %.2f  | abort |roll|>8: %s"
          % (rr[0], rr[-1], rr.mean(), rr.min(), rr.max(), bool((np.abs(rr) > 8).any())))
    result[name] = dict(gait=round(gait, 2), n=len(rows), n_sat=n_sat, n_sat_leg=n_sat_leg,
                        t=[round(r["t"], 3) for r in rows],
                        tauH={L4[j].upper(): [round(r["tauH"][j], 2) for r in rows] for j in range(4)},
                        roll=[round(r["roll"], 3) for r in rows], pitch=[round(r["pitch"], 3) for r in rows])
    print()

with open(out_path, "w") as fh:
    json.dump(result, fh, indent=1)
print("wrote", out_path)
