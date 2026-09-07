#!/usr/bin/env python3
"""Per-leg apparent compliance, measured from the bags.

For each leg over stance, regress the tracking error (commanded minus measured) against the
load. The slope is deg per N.m. It is an APPARENT compliance: it contains real structural
give plus controller lag plus backlash, and cannot separate them. But the three contaminants
are common to all four legs and the structure is not, so the RANKING across legs is
informative even though the absolute number is not a material property.

theta channel  -> fore/aft leg-train compliance (the printed lower links)
gamma channel  -> ABAD / camber-direction compliance
Usage: compliance.py bag1.db3 [bag2.db3 ...]
"""
import sys, sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

L4 = "abcd"
NM = {"a": "A FL", "b": "B FR", "c": "C RR", "d": "D RL"}
acc = {l: {"th": [], "gm": []} for l in L4}

print("run                | theta compliance deg/N.m        | gamma compliance deg/N.m")
print("                   |   A      B      C      D        |   A      B      C      D")
for DB in sys.argv[1:]:
    lbl = DB.split("/")[-1].replace("_0.db3", "")
    con = sqlite3.connect("file:%s?mode=ro" % DB, uri=True)
    T = {n: (i, get_message(t)) for i, n, t in con.execute("SELECT id,name,type FROM topics")}

    def rd(n):
        i, M = T[n]
        return [(ts * 1e-9, deserialize_message(d, M))
                for ts, d in con.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (i,))]

    need = ("/trigger", "/motor/state", "/motor/command")
    if any(n not in T for n in need):
        continue
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
    gmm = {l: np.degrees([getattr(m, "module_" + l).gamma for _, m in ms]) for l in L4}
    gmc = {l: np.degrees([getattr(m, "module_" + l).gamma for _, m in mc]) for l in L4}
    tr = {l: np.array([getattr(m, "module_" + l).torque_r for _, m in ms]) for l in L4}
    tl = {l: np.array([getattr(m, "module_" + l).torque_l for _, m in ms]) for l in L4}
    tH = {l: np.array([getattr(m, "module_" + l).torque_h for _, m in ms]) for l in L4}

    kpr = np.array([m.module_a.kp_r for _, m in mc])
    stance_c = kpr < 100
    st = np.interp(ts_, tc_, stance_c.astype(float)) > 0.5
    w = (ts_ > 2.6) & (ts_ < gait) & st
    if w.sum() < 200:
        continue

    row_th, row_gm = [], []
    for l in L4:
        cmd_i = np.interp(ts_[w], tc_, thc[l])
        err = cmd_i - thm[l][w]
        load = np.abs(tr[l][w]) + np.abs(tl[l][w])
        A = np.polyfit(load, err, 1)
        row_th.append(A[0]); acc[l]["th"].append(A[0])

        gcmd_i = np.interp(ts_[w], tc_, gmc[l])
        gerr = np.abs(gcmd_i - gmm[l][w])
        gload = np.abs(tH[l][w])
        B = np.polyfit(gload, gerr, 1)
        row_gm.append(B[0]); acc[l]["gm"].append(B[0])

    print("%-18s | %6.3f %6.3f %6.3f %6.3f | %6.3f %6.3f %6.3f %6.3f"
          % (lbl, row_th[0], row_th[1], row_th[2], row_th[3],
             row_gm[0], row_gm[1], row_gm[2], row_gm[3]))

print()
print("MEDIAN ACROSS RUNS (n = %d)" % len(acc["a"]["th"]))
th_med = {l: float(np.median(acc[l]["th"])) for l in L4}
gm_med = {l: float(np.median(acc[l]["gm"])) for l in L4}
for l in L4:
    print("   %-5s theta %6.3f deg/N.m  (spread %6.3f to %6.3f)   gamma %6.3f  (spread %6.3f to %6.3f)"
          % (NM[l], th_med[l], min(acc[l]["th"]), max(acc[l]["th"]),
             gm_med[l], min(acc[l]["gm"]), max(acc[l]["gm"])))
tv = np.array([th_med[l] for l in L4]); gv = np.array([gm_med[l] for l in L4])
print()
print("theta channel: stiffest %s, softest %s, spread %.0f %% of the mean"
      % (NM[L4[int(np.argmin(tv))]], NM[L4[int(np.argmax(tv))]], 100 * (tv.max() - tv.min()) / abs(tv.mean())))
print("gamma channel: stiffest %s, softest %s, spread %.0f %% of the mean"
      % (NM[L4[int(np.argmin(gv))]], NM[L4[int(np.argmax(gv))]], 100 * (gv.max() - gv.min()) / abs(gv.mean())))
print()
print("left pair (A,D) vs right pair (B,C), theta: %.4f vs %.4f  -> right/left %.2f"
      % ((th_med["a"] + th_med["d"]) / 2, (th_med["b"] + th_med["c"]) / 2,
         ((th_med["b"] + th_med["c"]) / 2) / ((th_med["a"] + th_med["d"]) / 2)))
print("front pair (A,B) vs rear pair (C,D), theta: %.4f vs %.4f  -> rear/front %.2f"
      % ((th_med["a"] + th_med["b"]) / 2, (th_med["c"] + th_med["d"]) / 2,
         ((th_med["c"] + th_med["d"]) / 2) / ((th_med["a"] + th_med["b"]) / 2)))
