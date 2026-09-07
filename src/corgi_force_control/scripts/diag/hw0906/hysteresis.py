#!/usr/bin/env python3
"""Elastic deflection or ground drag? The error-vs-load loop tells them apart.

An ELASTIC leg deflects with the load and recovers as it unloads, so the loading and
unloading branches of error-vs-load lie on top of each other.
A leg being DRAGGED accumulates displacement while it is on the ground, so at the same load
the error is larger on the way out than on the way in: a hysteresis loop, opening in the
direction of travel.

Also reported, because Alex's mechanism predicts them:
  - stance fraction per leg (a leg that is not hopping is on the ground longer)
  - whether per-stride error tracks stance DURATION (drag) or peak LOAD (elastic)
  - whether the front/rear ratio moves with camber, which changes the lateral force
Usage: hysteresis.py bag.db3 [...]
"""
import sys, sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

L4 = "abcd"
NM = {"a": "A FL", "b": "B FR", "c": "C RR", "d": "D RL"}
agg = {l: {"hyst": [], "sf": [], "rd": [], "rl": []} for l in L4}

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
    low = kpr < 100
    on_ = tc_[1:][(~low[:-1]) & low[1:]]
    on_ = on_[(on_ > 2.6) & (on_ < gait - 0.3)]

    for l in L4:
        cmd_i = np.interp(ts_, tc_, thc[l])
        err = cmd_i - thm[l]
        load = np.abs(tr[l]) + np.abs(tl[l])
        hy, sf, per_err, per_dur, per_pk = [], [], [], [], []
        for i in range(len(on_) - 1):
            a, b = on_[i], on_[i + 1]
            w = np.where((ts_ >= a) & (ts_ < b))[0]
            if len(w) < 30:
                continue
            L = load[w]
            # contact = load above a fifth of this stride's peak
            thr = 0.20 * L.max()
            inc = L > thr
            if inc.sum() < 12:
                continue
            idx = w[inc]
            sf.append(len(idx) / float(len(w)))
            pk = int(np.argmax(load[idx]))
            up, dn = idx[:pk + 1], idx[pk:]
            if len(up) < 5 or len(dn) < 5:
                continue
            # compare the two branches at a load both visit
            lo = max(load[up].min(), load[dn].min())
            hi = min(load[up].max(), load[dn].max())
            if hi - lo < 5:
                continue
            probe = np.linspace(lo + 0.1 * (hi - lo), hi - 0.1 * (hi - lo), 7)
            eu = np.interp(probe, load[up], err[up])
            od = np.argsort(load[dn])
            ed = np.interp(probe, load[dn][od], err[dn][od])
            hy.append(float(np.mean(ed - eu)))
            per_err.append(float(np.mean(np.abs(err[idx]))))
            per_dur.append(float(ts_[idx[-1]] - ts_[idx[0]]))
            per_pk.append(float(L.max()))
        if len(hy) > 4:
            agg[l]["hyst"].append(float(np.median(hy)))
            agg[l]["sf"].append(float(np.median(sf)))
            if len(per_err) > 5:
                agg[l]["rd"].append(float(np.corrcoef(per_dur, per_err)[0, 1]))
                agg[l]["rl"].append(float(np.corrcoef(per_pk, per_err)[0, 1]))

print("PER-LEG, median across %d runs" % len(sys.argv[1:]))
print("   leg   | hysteresis (deg)  | stance fraction | corr(err,duration) | corr(err,peak load)")
for l in L4:
    h = np.median(agg[l]["hyst"]) if agg[l]["hyst"] else float("nan")
    s = np.median(agg[l]["sf"]) if agg[l]["sf"] else float("nan")
    rd_ = np.median(agg[l]["rd"]) if agg[l]["rd"] else float("nan")
    rl_ = np.median(agg[l]["rl"]) if agg[l]["rl"] else float("nan")
    print("   %-5s |      %+6.3f       |      %5.3f      |       %+5.2f        |       %+5.2f"
          % (NM[l], h, s, rd_, rl_))

H = {l: (np.median(agg[l]["hyst"]) if agg[l]["hyst"] else np.nan) for l in L4}
S = {l: (np.median(agg[l]["sf"]) if agg[l]["sf"] else np.nan) for l in L4}
print()
print("hysteresis:      front %+.3f deg   rear %+.3f deg" % ((H["a"] + H["b"]) / 2, (H["c"] + H["d"]) / 2))
print("stance fraction: front  %.3f       rear  %.3f" % ((S["a"] + S["b"]) / 2, (S["c"] + S["d"]) / 2))
