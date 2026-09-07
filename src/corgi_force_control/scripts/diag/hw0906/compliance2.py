#!/usr/bin/env python3
"""Is the front/rear compliance split structural, or is it tracking lag?

Lag scales with joint velocity. At the velocity zero-crossing near peak compression the lag
contributes nothing, so any residual error-per-load there is structural give plus backlash.
Compare the full-stance slope against the near-zero-velocity slope: if the split survives,
the legs really are softer at the front.
Also reports the slope inside a load band common to all four legs, so front and rear are
compared at matched load rather than across their different operating ranges.
Usage: compliance2.py bag1.db3 [...]
"""
import sys, sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

L4 = "abcd"
NM = {"a": "A FL", "b": "B FR", "c": "C RR", "d": "D RL"}
allslow = {l: [] for l in L4}
allband = {l: [] for l in L4}

for DB in sys.argv[1:]:
    lbl = DB.split("/")[-1].replace("_0.db3", "")
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
        continue
    T0, TOFF = max(pairs, key=lambda p: p[1] - p[0]); gait = TOFF - T0

    ms = rd("/motor/state"); mc = rd("/motor/command")
    ts_ = np.array([t - T0 for t, _ in ms]); tc_ = np.array([t - T0 for t, _ in mc])
    thm = {l: np.degrees([getattr(m, "module_" + l).theta for _, m in ms]) for l in L4}
    thc = {l: np.degrees([getattr(m, "module_" + l).theta for _, m in mc]) for l in L4}
    tr = {l: np.array([getattr(m, "module_" + l).torque_r for _, m in ms]) for l in L4}
    tl = {l: np.array([getattr(m, "module_" + l).torque_l for _, m in ms]) for l in L4}
    kpr = np.array([m.module_a.kp_r for _, m in mc])
    st = np.interp(ts_, tc_, (kpr < 100).astype(float)) > 0.5
    base = (ts_ > 2.6) & (ts_ < gait) & st
    if base.sum() < 200:
        continue

    for l in L4:
        vel = np.abs(np.gradient(thm[l], ts_))          # deg/s
        cmd_i = np.interp(ts_, tc_, thc[l])
        err = cmd_i - thm[l]
        load = np.abs(tr[l]) + np.abs(tl[l])
        thr = np.percentile(vel[base], 20)              # slowest fifth of stance
        slow = base & (vel <= thr)
        if slow.sum() > 60:
            allslow[l].append(np.polyfit(load[slow], err[slow], 1)[0])
        band = base & (load > 28) & (load < 42)         # load band all four legs visit
        if band.sum() > 60:
            allband[l].append(np.polyfit(load[band], err[band], 1)[0])

print("PER-LEG SLOPE, deg/N.m, median across %d runs" % len(sys.argv[1:]))
print("   leg   | slowest fifth of stance (lag ~ 0) | matched load band 28-42 N.m")
sl = {}
bd = {}
for l in L4:
    sl[l] = float(np.median(allslow[l])) if allslow[l] else float("nan")
    bd[l] = float(np.median(allband[l])) if allband[l] else float("nan")
    print("   %-5s |            %6.3f                 |         %6.3f" % (NM[l], sl[l], bd[l]))

for nm, d in (("slow-sample", sl), ("matched-band", bd)):
    f = (d["a"] + d["b"]) / 2.0
    r = (d["c"] + d["d"]) / 2.0
    lft = (d["a"] + d["d"]) / 2.0
    rgt = (d["b"] + d["c"]) / 2.0
    print()
    print("%s:  front %.3f  rear %.3f  -> front/rear %.2f   |   left %.3f  right %.3f  -> right/left %.2f"
          % (nm, f, r, f / r if r else float("nan"), lft, rgt, rgt / lft if lft else float("nan")))
