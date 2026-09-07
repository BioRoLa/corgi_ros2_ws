#!/usr/bin/env python3
"""What does the ~40 N.m ABAD ceiling look like in the data? A hard wall at one exact value
(encoding saturation) or a soft ceiling with scatter (a physical or control limit)?
Also: is the SAME value reached on other axes (torque_r, torque_l)? Usage: ceiling_shape.py bag...
"""
import sys, sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

L4 = "abcd"
allmax = {l: [] for l in L4}
top = {l: [] for l in L4}
rl_max = []
vals = []
for DB in sys.argv[1:]:
    try:
        con = sqlite3.connect("file:%s?mode=ro" % DB, uri=True)
        T = {n: (i, get_message(t)) for i, n, t in con.execute("SELECT id,name,type FROM topics")}
        if "/motor/state" not in T:
            continue
        i, M = T["/motor/state"]
        ms = [deserialize_message(d, M) for _, d in con.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (i,))]
    except Exception as e:
        continue
    for l in L4:
        th = np.array([getattr(m, "module_" + l).torque_h for m in ms])
        tr = np.array([getattr(m, "module_" + l).torque_r for m in ms])
        tl = np.array([getattr(m, "module_" + l).torque_l for m in ms])
        a = np.abs(th)
        allmax[l].append(float(a.max()))
        top[l].extend(np.sort(a)[-25:].tolist())
        rl_max.append((float(np.abs(tr).max()), float(np.abs(tl).max())))
        vals.extend(a[a > 30].tolist())

print("per-leg |torque_h| max per bag (N.m), over %d bags" % len(allmax["a"]))
for l in L4:
    v = np.array(allmax[l])
    print("   %s: overall max %.3f   count of bags with max > 39: %d/%d   top values: %s"
          % (l.upper(), v.max(), int((v > 39).sum()), len(v), ", ".join("%.2f" % x for x in sorted(v)[-6:])))
tv = np.array(sum(top.values(), []))
print()
print("all top-25-per-bag |torque_h| samples pooled (n=%d): max %.4f, 99th %.3f, 95th %.3f" % (len(tv), tv.max(), np.percentile(tv, 99), np.percentile(tv, 95)))
u, c = np.unique(np.round(tv[tv > 38.5], 3), return_counts=True)
order = np.argsort(-c)[:12]
print("most frequent exact values above 38.5 (a hard wall shows as one dominant value):")
for k in order:
    print("   %.3f  x%d" % (u[k], c[k]))
print()
v = np.array(vals)
if v.size:
    hist, edges = np.histogram(v, bins=np.arange(30, 46, 0.5))
    print("histogram of |torque_h| > 30 N.m (all legs, all samples):")
    for h, e in zip(hist, edges):
        print("   %5.1f-%5.1f  %6d  %s" % (e, e + 0.5, h, "#" * int(60 * h / max(hist.max(), 1))))
print()
rl = np.array(rl_max)
print("for reference, |torque_r| max overall %.2f, |torque_l| max overall %.2f (do those reach a wall too?)" % (rl[:, 0].max(), rl[:, 1].max()))
