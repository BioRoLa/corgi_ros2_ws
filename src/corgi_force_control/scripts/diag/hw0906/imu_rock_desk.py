#!/usr/bin/env python3
"""imu_rock_desk.py -- score the registered desk rock/tilt test (log 325.46, OI #52/#47).

The robot sat on its own feet, motors dead, and was hand-tilted through three
named moves and then rocked for a minute and set back down. There is no Vicon:
the truth reference is RETURN-TO-REST CLOSURE, i.e. the body ends at the same
physical attitude it started at, so the true net change over the episode is zero.

What it scores (all registered in the log BEFORE the bag was started):
  P-SIGN-1/2/3  the frame signs on the three named moves;
  P-ROCK-D1     both still windows, |mean wy| and |quaternion pitch rate|;
  P-ROCK-D2     quaternion pitch offset at the end of the rock vs its pre-rock rest;
  P-ROCK-D3     that the offset saturates (2-5 deg) instead of growing linearly;
  P-ROCK-D4     the relaxation to <= 0.3 deg within 40 s, fitted TC 5-20 s.
Plus the frame-closure regression, which was not registered and is reported as
an observation: d(euler)/dt against the published body rate over the rock.

Euler convention is the one the 18-arc analysers use (imu_poststop.py:62,
pitch_variants.py:47): pitch = asin(2(w*y - z*x)); roll and yaw are the matching
ZYX terms. `wy` is the published angular_velocity.y, unrotated, as the
controller consumes it.

Usage: imu_rock_desk.py [--selftest] [--out imu_rock_desk.json] bag.db3
Needs ROS sourced. Read-only.
"""
import argparse
import json
import os
import sqlite3
import sys

import numpy as np

MOVING_DEGPS = 2.0      # |gyro| rolling rms above this = the body is being handled
RMS_WIN_S = 0.25
MIN_STILL_S = 20.0      # a "still window" for scoring
MIN_BURST_S = 0.4
ROCK_MIN_S = 25.0       # the rock is the longest burst


# ---------------------------------------------------------------- maths
def quat_to_euler_deg(w, x, y, z):
    """ZYX euler in degrees; pitch matches imu_poststop.py:62 exactly."""
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def rolling_rms(t, v, win_s):
    """rms of |v| (row norm) in a centred window, via cumulative sums on a uniform grid."""
    n = len(t)
    if n < 3:
        return np.zeros(n)
    dt = float(np.median(np.diff(t)))
    k = max(3, int(round(win_s / dt)) | 1)
    sq = np.sum(v * v, axis=1) if v.ndim == 2 else v * v
    pad = k // 2
    padded = np.concatenate((np.full(pad, sq[0]), sq, np.full(pad, sq[-1])))
    c = np.concatenate(([0.0], np.cumsum(padded)))
    return np.sqrt((c[k:k + n] - c[:n]) / k)


def segments(t, moving):
    """[(t0, t1, is_moving)] runs of the boolean."""
    out = []
    i = 0
    n = len(moving)
    while i < n:
        j = i
        while j + 1 < n and moving[j + 1] == moving[i]:
            j += 1
        out.append((float(t[i]), float(t[j]), bool(moving[i])))
        i = j + 1
    return out


def slope(t, y):
    if len(t) < 20:
        return float("nan")
    return float(np.polyfit(t - t[0], y, 1)[0])


def fit_tc(t, y, y_inf):
    """Fit y - y_inf = A exp(-t/tau); returns tau in s (nan if the fit is unusable)."""
    d = y - y_inf
    s = np.sign(np.median(d[:max(5, len(d) // 10)]))
    d = d * s
    ok = d > 0.02
    if ok.sum() < 50:
        return float("nan")
    tt, dd = t[ok] - t[0], np.log(d[ok])
    a = np.polyfit(tt, dd, 1)[0]
    return float(-1.0 / a) if a < 0 else float("nan")


# ---------------------------------------------------------------- self-test
def selftest():
    ok = True

    def chk(name, got, want, tol):
        nonlocal ok
        good = abs(got - want) <= tol
        ok = ok and good
        print("  %-46s got %9.4f  want %9.4f  %s"
              % (name, got, want, "PASS" if good else "FAIL"))

    print("self-test 1: euler extraction on known quaternions")
    for axis, ang, idx, label in ((1, 10.0, 1, "pitch"), (0, 10.0, 0, "roll"), (2, 10.0, 2, "yaw")):
        h = np.radians(ang) / 2
        q = [np.cos(h), 0.0, 0.0, 0.0]
        q[1 + axis] = np.sin(h)
        e = quat_to_euler_deg(np.array([q[0]]), np.array([q[1]]), np.array([q[2]]), np.array([q[3]]))
        chk("+10 deg about axis %d -> %s" % (axis, label), float(e[idx][0]), 10.0, 1e-6)
        for k in range(3):
            if k != idx:
                chk("  cross term %d is zero" % k, float(e[k][0]), 0.0, 1e-6)

    print("self-test 2: rolling rms of a known square wave")
    t = np.arange(0, 10, 0.001)
    v = np.where((t > 4) & (t < 6), 20.0, 0.0)
    r = rolling_rms(t, v, 0.25)
    chk("rms inside the burst", float(np.median(r[(t > 4.5) & (t < 5.5)])), 20.0, 0.01)
    chk("rms outside the burst", float(np.median(r[t < 3])), 0.0, 0.01)

    print("self-test 3: segmentation finds one burst")
    segs = [s for s in segments(t, r > 2.0) if s[2]]
    chk("burst count", float(len(segs)), 1.0, 0)
    chk("burst start", segs[0][0], 4.0, 0.2)
    chk("burst end", segs[0][1], 6.0, 0.2)

    print("self-test 4: exponential time-constant fit")
    t = np.arange(0, 60, 0.01)
    y = 3.0 * np.exp(-t / 12.0) + 0.5
    chk("fitted tau", fit_tc(t, y, 0.5), 12.0, 0.2)

    print("SELF-TEST %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


# ---------------------------------------------------------------- load
def load(db):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    con = sqlite3.connect("file:%s?mode=ro" % db, uri=True)
    T = {n: (i, get_message(ty)) for i, n, ty in con.execute("SELECT id,name,type FROM topics")}
    i, M = T["/imu"]
    rows = [(ts * 1e-9, deserialize_message(d, M)) for ts, d in
            con.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (i,))]
    t = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, m in rows])
    o = np.argsort(t, kind="stable")
    t = t[o]
    keep = np.concatenate(([True], np.diff(t) > 0))
    t = t[keep] - t[keep][0]
    q = np.array([[m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z]
                  for _, m in rows])[o][keep]
    g = np.degrees(np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]
                             for _, m in rows]))[o][keep]
    roll, pitch, yaw = quat_to_euler_deg(q[:, 0], q[:, 1], q[:, 2], q[:, 3])
    return dict(t=t, q=q, g=g, roll=roll, pitch=pitch, yaw=yaw)


# ---------------------------------------------------------------- scoring
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", nargs="?")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--out", default="imu_rock_desk.json")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if not a.bag:
        ap.error("give a bag, or --selftest")
    if selftest() != 0:
        print("REFUSING to analyse: self-test failed")
        return 1
    print()

    d = load(a.bag)
    t, g, roll, pitch, yaw = d["t"], d["g"], d["roll"], d["pitch"], d["yaw"]
    print("bag: %s  %d samples  %.1f s  %.0f Hz"
          % (os.path.basename(a.bag), len(t), t[-1], len(t) / t[-1]))

    rms = rolling_rms(t, g, RMS_WIN_S)
    segs = segments(t, rms > MOVING_DEGPS)
    segs = [s for s in segs if (s[1] - s[0]) > (MIN_BURST_S if s[2] else 1.0)]
    stills = [s for s in segs if not s[2] and (s[1] - s[0]) >= MIN_STILL_S]
    bursts = [s for s in segs if s[2]]
    print("segments: %d bursts, %d still windows >= %.0f s"
          % (len(bursts), len(stills), MIN_STILL_S))
    for k, (t0, t1, _) in enumerate(bursts):
        m = (t >= t0) & (t <= t1)
        print("  burst %d: %6.1f-%6.1f s (%5.1f s)  gy_rms %6.2f deg/s  "
              "d_roll %+7.2f d_pitch %+7.2f d_yaw %+7.2f"
              % (k, t0, t1, t1 - t0, float(np.sqrt(np.mean(g[m, 1] ** 2))),
                 roll[m][-1] - roll[m][0], pitch[m][-1] - pitch[m][0], yaw[m][-1] - yaw[m][0]))

    res = {"bag": os.path.basename(a.bag), "n": int(len(t)), "dur_s": float(t[-1]),
           "bursts": [], "checks": {}}

    rock = max(bursts, key=lambda s: s[1] - s[0]) if bursts else None

    # Pick each named move by WHICH ANGLE ACTUALLY MOVED, not by burst order.
    # The first attempt took "the first three short bursts" and so scored the
    # roll and yaw predictions on the pitch-return burst and on a 2 deg yaw
    # twitch: the operator's three moves were bursts 0, 3 and 11, not 0, 1, 2.
    def d_ang(b, ang):
        m = (t >= b[0]) & (t <= b[1])
        return float(np.max(ang[m]) - np.min(ang[m])) if m.any() else 0.0

    pre_b = [b for b in bursts if b is not rock and b[1] <= (rock[0] if rock else t[-1])]
    sign_bursts, used = [], []
    for ang in (pitch, roll, yaw):
        cand = [b for b in pre_b if b not in used]
        if not cand:
            sign_bursts.append(None)
            continue
        b = max(cand, key=lambda bb: d_ang(bb, ang))
        used.append(b)
        sign_bursts.append(b)
    print("\nmove bursts chosen by largest excursion in the named channel: %s"
          % ", ".join("%.1f-%.1f s" % (b[0], b[1]) if b else "none" for b in sign_bursts))

    # ---- P-SIGN: the three named moves, in order, outbound leg only
    names = [("P-SIGN-1 nose-up", pitch, g[:, 1], "pitch", "wy", +1, -1),
             ("P-SIGN-2 left-side-down roll", roll, g[:, 0], "roll", "wx", 0, 0),
             ("P-SIGN-3 yaw left", yaw, g[:, 2], "yaw", "wz", -1, +1)]
    print("\nnamed moves (outbound leg = start -> the extremum of the angle):")
    for k, (label, ang, rate, an, rn) in enumerate([(n[0], n[1], n[2], n[3], n[4]) for n in names]):
        if k >= len(sign_bursts) or sign_bursts[k] is None:
            print("  %-30s NOT FOUND" % label)
            res["checks"][label.split()[0]] = "NOT FOUND"
            continue
        t0, t1, _ = sign_bursts[k]
        m = (t >= t0 - 0.2) & (t <= t1 + 0.2)
        aa, rr, tt = ang[m], rate[m], t[m]
        base = aa[0]
        j = int(np.argmax(np.abs(aa - base)))
        out = slice(0, max(j, 1))
        back = slice(max(j, 1), len(aa))
        d_out = float(aa[out][-1] - base)
        r_out = float(np.mean(rr[out]))
        r_back = float(np.mean(rr[back])) if back.stop > back.start + 5 else float("nan")
        print("  %-30s %s %+7.2f deg   mean %s %+8.2f deg/s (return %+8.2f)  peak |%s| %.1f"
              % (label, an, d_out, rn, r_out, r_back, rn, float(np.max(np.abs(rr)))))
        res["bursts"].append({"move": label, "t0": t0, "t1": t1, "d_angle_deg": d_out,
                              "mean_rate_degps": r_out, "return_rate_degps": r_back})
        exp_a, exp_r = names[k][5], names[k][6]
        if exp_a:
            good = (np.sign(d_out) == exp_a) and (np.sign(r_out) == exp_r)
            res["checks"][label.split()[0]] = ("PASS" if good else "FAIL") + \
                " d_%s=%+.2f mean_%s=%+.2f" % (an, d_out, rn, r_out)
        else:
            good = np.sign(d_out) == np.sign(r_out)
            res["checks"][label.split()[0]] = ("PASS (agree)" if good else "FAIL (disagree)") + \
                " d_%s=%+.2f mean_%s=%+.2f" % (an, d_out, rn, r_out)
        print("      -> %s" % res["checks"][label.split()[0]])

    # ---- frame closure regression over the rock (observation, not registered)
    if rock:
        t0, t1, _ = rock
        m = (t >= t0) & (t <= t1)
        small = m & (np.abs(roll) < 15) & (np.abs(pitch) < 15)
        print("\nframe check over the rock (%.1f s, %d samples, |roll|,|pitch| < 15 deg):"
              % (t1 - t0, int(small.sum())))
        res["frame"] = {}
        for an, ang, rn, rate in (("roll", roll, "wx", g[:, 0]),
                                  ("pitch", pitch, "wy", g[:, 1]),
                                  ("yaw", yaw, "wz", g[:, 2])):
            dang = np.gradient(ang, t)
            x, y = rate[small], dang[small]
            sl = float(np.polyfit(x, y, 1)[0])
            r = float(np.corrcoef(x, y)[0, 1])
            res["frame"]["%s_vs_%s" % (an, rn)] = {"slope": sl, "r": r}
            print("  d(%s)/dt vs %-3s : slope %+6.3f   r %+6.3f   -> %s"
                  % (an, rn, sl, r, "AGREE" if sl > 0 else "MIRRORED"))

        # ---- P-ROCK-D2/D3: quaternion pitch offset at the end of the rock
        pre = [s for s in stills if s[1] <= t0] or [s for s in segs if not s[2] and s[1] <= t0]
        post = [s for s in stills if s[0] >= t1] or [s for s in segs if not s[2] and s[0] >= t1]
        gy_rms = float(np.sqrt(np.mean(g[m, 1] ** 2)))
        res["rock"] = {"t0": t0, "t1": t1, "dur_s": t1 - t0, "gy_rms_degps": gy_rms}
        print("\nrock window: %.1f s, gy rms %.1f deg/s (registered >= 20)" % (t1 - t0, gy_rms))
        if pre and post:
            pm = (t >= pre[-1][0]) & (t <= pre[-1][1])
            pitch_pre = float(np.median(pitch[pm][-int(5 / np.median(np.diff(t))):]))
            qm = (t >= post[0][0]) & (t <= min(post[0][0] + 2.0, post[0][1]))
            pitch_end = float(np.median(pitch[qm]))
            fm = (t >= max(post[0][1] - 10, post[0][0])) & (t <= post[0][1])
            pitch_rest = float(np.median(pitch[fm]))
            off = pitch_end - pitch_pre
            res["rock"].update(pitch_pre_deg=pitch_pre, pitch_end_of_rock_deg=pitch_end,
                               pitch_final_rest_deg=pitch_rest, offset_deg=off,
                               closure_deg=pitch_rest - pitch_pre)
            print("  quaternion pitch: pre-rock rest %+7.3f  end of rock %+7.3f  "
                  "final rest %+7.3f deg" % (pitch_pre, pitch_end, pitch_rest))
            print("  OFFSET at end of rock            %+7.3f deg   (P-ROCK-D2: >= 1.0 nose-down; "
                  "<= 0.3 refutes)" % off)
            print("  return-to-rest closure           %+7.3f deg   (the truth reference; "
                  "should be ~0)" % (pitch_rest - pitch_pre))
            lin = abs(0.33 * (t1 - t0))
            print("  linear-accumulation strawman     %7.3f deg   (0.33 deg/s x %.0f s) "
                  "-- P-ROCK-D3 expects 2-5 instead" % (lin, t1 - t0))
            # The registered precondition is gy rms >= 20 deg/s over a 60 s rock.
            # Below it the test is OUT OF ITS REGISTERED CONDITIONS and scores
            # nothing: a weak rock cannot refute a motion-induced error.
            ok_amp = (gy_rms >= 20.0) and ((t1 - t0) >= 45.0)
            if not ok_amp:
                why = ("gy_rms %.1f < 20 deg/s" % gy_rms) if gy_rms < 20 else ""
                if (t1 - t0) < 45:
                    why += ("; " if why else "") + "rock %.0f s < 45 s" % (t1 - t0)
                res["checks"]["P-ROCK-D2"] = ("UNSCORED (precondition: %s) offset=%+.3f deg"
                                              % (why, off))
                res["checks"]["P-ROCK-D3"] = "UNSCORED (same precondition)"
            else:
                res["checks"]["P-ROCK-D2"] = ("PASS" if abs(off) >= 1.0 else
                                              ("REFUTED" if abs(off) <= 0.3 else "UNRESOLVED")
                                              ) + " offset=%+.3f deg" % off
                res["checks"]["P-ROCK-D3"] = ("PASS" if 2.0 <= abs(off) <= 5.0 else "FAIL"
                                              ) + " |offset|=%.3f vs linear %.1f" % (abs(off), lin)

            # ---- P-ROCK-D4: relaxation
            rm = (t >= post[0][0]) & (t <= post[0][1])
            tau = fit_tc(t[rm], pitch[rm], pitch_rest)
            within = np.abs(pitch[rm] - pitch_pre) <= 0.3
            t_in = float(t[rm][np.argmax(within)] - post[0][0]) if within.any() else float("nan")
            res["rock"].update(relax_tau_s=tau, t_within_0p3_s=t_in,
                               post_still_s=post[0][1] - post[0][0])
            print("  relaxation: tau %.1f s, within 0.3 deg of pre-rock after %.1f s "
                  "(window %.0f s)" % (tau, t_in, post[0][1] - post[0][0]))
            if abs(off) < 0.3:
                res["checks"]["P-ROCK-D4"] = ("UNSCORED (no offset to relax: %+.3f deg is "
                                              "already inside the 0.3 deg bar)" % off)
            else:
                res["checks"]["P-ROCK-D4"] = ("PASS" if (t_in == t_in and t_in <= 40) else "FAIL"
                                              ) + " tau=%.1f t=%.1f" % (tau, t_in)

    # ---- P-ROCK-D1: still windows
    print("\nstill windows (P-ROCK-D1: |mean wy| and |pitch rate| <= 0.02 deg/s):")
    d1 = []
    for k, (t0, t1, _) in enumerate(stills):
        m = (t >= t0 + 2) & (t <= t1)
        mw = float(np.mean(g[m, 1]))
        sp = slope(t[m], pitch[m])
        d1.append(max(abs(mw), abs(sp)))
        print("  still %d %6.1f-%6.1f s (%4.0f s): mean wy %+7.4f deg/s   pitch slope %+7.4f deg/s"
              % (k, t0, t1, t1 - t0, mw, sp))
        res.setdefault("still", []).append({"t0": t0, "t1": t1, "mean_wy_degps": mw,
                                            "pitch_slope_degps": sp})
    if d1:
        res["checks"]["P-ROCK-D1"] = ("PASS" if max(d1) <= 0.02 else "FAIL") + \
            " worst=%.4f deg/s" % max(d1)

    print("\n=== CHECKS ===")
    for k in sorted(res["checks"]):
        print("  %-12s %s" % (k, res["checks"][k]))
    with open(a.out, "w") as f:
        json.dump(res, f, indent=1, sort_keys=True)
    print("wrote %s" % a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
