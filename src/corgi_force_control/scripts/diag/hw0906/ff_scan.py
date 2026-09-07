#!/usr/bin/env python3
"""How often is a stride nearly fully airborne, and where in the arc?

For every scored arc: per-stride flight fraction (a_z < 0.5 g proxy), the max, where it
sits, and how many strides exceed candidate thresholds. This is the false-positive check
for the launch-stride abort rule: a rule that fires on healthy arcs is not a rule.
Usage: ff_scan.py <bag.db3> [<bag.db3> ...]
"""
import sys, os, sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

L4 = "abcd"
print("%-16s %5s | %s" % ("run", "n", "per-stride flight fraction (window 1..n)"))
summary = []
for DB in sys.argv[1:]:
    run = os.path.basename(DB).replace("_0.db3", "")
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
        print("%-16s  no trigger window" % run); continue
    T0, TOFF = max(pairs, key=lambda p: p[1] - p[0]); gait = TOFF - T0
    mc = rd("/motor/command"); imu = rd("/imu"); ms = rd("/motor/state")
    tc_ = np.array([t - T0 for t, _ in mc]); ti = np.array([t - T0 for t, _ in imu]); ts_ = np.array([t - T0 for t, _ in ms])
    acc_z = np.array([m.linear_acceleration.z for _, m in imu])
    az0 = np.median(acc_z[(ti > 0.2) & (ti < 2.0)])
    tr = {l: np.array([getattr(m, "module_" + l).torque_r for _, m in ms]) for l in L4}
    tl = {l: np.array([getattr(m, "module_" + l).torque_l for _, m in ms]) for l in L4}
    kpr = np.array([m.module_a.kp_r for _, m in mc]); low = kpr < 100
    on_ = tc_[1:][(~low[:-1]) & low[1:]]
    on_ = on_[(on_ > 2.6) & (on_ < gait - 0.3)]
    ff, pkmin = [], []
    for i in range(len(on_) - 1):
        a, b = on_[i], on_[i + 1]
        wi = (ti >= a) & (ti < b); w = (ts_ >= a) & (ts_ < b)
        ff.append(float(np.mean(acc_z[wi] < 0.5 * az0)) if wi.sum() > 3 else np.nan)
        pkmin.append(min((np.abs(tr[l][w]) + np.abs(tl[l][w])).max() for l in L4) if w.sum() > 20 else np.nan)
    ff = np.array(ff); pkmin = np.array(pkmin)
    print("%-16s %5d | %s" % (run, len(ff), " ".join("%.2f" % v for v in ff)))
    summary.append((run, ff, pkmin))

print("\n%-16s %6s %6s %8s | %s" % ("run", "max", "where", "min pk", "count over threshold  0.85 / 0.90 / 0.95"))
for run, ff, pk in summary:
    if not len(ff):
        continue
    k = int(np.nanargmax(ff))
    print("%-16s %6.2f %6d %8.1f | %d / %d / %d"
          % (run, ff[k], k + 1, pk[k], (ff > 0.85).sum(), (ff > 0.90).sum(), (ff > 0.95).sum()))

print("\nFIRST THREE WINDOWS ONLY (the launch): max ff, and the min per-leg peak load there")
for run, ff, pk in summary:
    if len(ff) < 3:
        continue
    k = int(np.nanargmax(ff[:3]))
    print("  %-16s max %.2f at window %d, min peak load %.1f N.m  -> %s"
          % (run, ff[k], k + 1, pk[k], "VOID" if ff[k] > 0.90 else "valid"))
