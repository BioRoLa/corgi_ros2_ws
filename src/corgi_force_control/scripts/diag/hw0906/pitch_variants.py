#!/usr/bin/env python3
"""The pitch channel's theta correction u = k_pitch*pitch + d_pitch*wy, reconstructed from the
banked /imu for several (k_pitch, d_pitch) pairs (log 325.46 §2; OI #52).

Same reconstruction as pitch_clamp.py (stance-gated on module_a.kp_r < 100, window
[T0+2.6, TOFF], clamp 3 deg = pitch_limit 0.05236 rad), run over a list of bags and
gain pairs at once. Self-test: (0.30, 0.10) must reproduce log 325.26's stance clamp
duty 52.9 / 51.7 / 37.1 % on s2_l0_a4 / s2_ol10_roll1 / s2_ol10_a1 within 1 point.

Registered (325.46 §2) before this ran: (0.30, 0.0) clamp < 10 % of stance in every bag
with |u| median 1.2-1.7 deg; (0.15, 0.10) 35-45 %; (0.15, 0.0) < 3 %; (0.30, -0.10)
reported, not predicted.

Usage: pitch_variants.py [--out pitch_variants.json] bag.db3 [bag.db3 ...]
Needs ROS sourced. Read-only on the bags.
"""
import sys, os, json, sqlite3, argparse
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

VARIANTS = [(0.30, 0.10), (0.30, 0.0), (0.30, -0.10), (0.15, 0.10), (0.15, 0.0)]
LIM = 0.05236
SELF = {"s2_l0_a4": 52.9, "s2_ol10_roll1": 51.7, "s2_ol10_a1": 37.1}


def load(DB):
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
    ti = np.array([t - T0 for t, _ in imu]); tc = np.array([t - T0 for t, _ in mc])
    q = np.array([[m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z] for _, m in imu])
    w_, x_, y_, z_ = q.T
    pitch = np.arcsin(np.clip(2 * (w_ * y_ - z_ * x_), -1, 1))
    wy = np.array([m.angular_velocity.y for _, m in imu])
    kpr = np.array([m.module_a.kp_r for _, m in mc])
    st = np.interp(ti, tc, (kpr < 100).astype(float)) > 0.5
    win = (ti > 2.6) & (ti < gait)
    return dict(pitch=pitch, wy=wy, mask=win & st, gait=gait)


def stats(b, k, d):
    m = b["mask"]
    P = k * b["pitch"][m]; D = d * b["wy"][m]; u = P + D
    return dict(clamped_pct=100.0 * float(np.mean(np.abs(u) >= LIM)),
                u_med_deg=float(np.degrees(np.median(np.abs(u)))),
                P_med_deg=float(np.degrees(np.median(np.abs(P)))),
                D_med_deg=float(np.degrees(np.median(np.abs(D)))),
                u_delivered_mean_deg=float(np.degrees(np.mean(np.minimum(np.abs(u), LIM)))),
                pitch_med_deg=float(np.degrees(np.median(b["pitch"][m]))),
                wy_rms_degps=float(np.degrees(np.sqrt(np.mean(b["wy"][m] ** 2)))),
                n=int(m.sum()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    out = {}; ok = True
    hdr = "%-16s" % "bag" + "".join("  (%+.2f,%+.2f) clamp%%  |u|  |P|  |D|" % v for v in VARIANTS)
    print(hdr)
    for DB in a.bags:
        name = os.path.basename(DB).replace("_0.db3", "")
        b = load(DB)
        row = {"%.2f,%.2f" % v: stats(b, *v) for v in VARIANTS}
        out[name] = row
        line = "%-16s" % name
        for v in VARIANTS:
            s = row["%.2f,%.2f" % v]
            line += "  %5.1f %4.2f %4.2f %4.2f          " % (s["clamped_pct"], s["u_med_deg"], s["P_med_deg"], s["D_med_deg"])
        print(line)
        if name in SELF:
            got = row["0.30,0.10"]["clamped_pct"]
            flag = abs(got - SELF[name]) <= 1.0
            ok &= flag
            print("   selftest %s: as-flown clamp %.1f %% vs log 325.26 %.1f %% -> %s" % (name, got, SELF[name], "ok" if flag else "MISMATCH"))
    print("SELFTEST", "PASS" if ok else "FAIL")
    if a.out:
        with open(a.out, "w") as fh:
            json.dump(out, fh, indent=1)
        print("wrote", a.out)


if __name__ == "__main__":
    main()
