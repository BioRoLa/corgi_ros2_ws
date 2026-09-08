#!/usr/bin/env python3
"""Does the IMU's pitch error persist after the robot stops? (log 325.46 §2; OI #52)

The 18-run Vicon comparison (imu_vs_vicon3.json) says the pitch over-read is
motion-induced, not a static bias, and is carried by the published angular rate --
which is the EKF's bias-COMPENSATED rate (cx5.hpp:161), not the raw gyro. The bags
cannot see the raw gyro, but they can see what happens after the arc: the robot
stands still for 25-54 s after trigger-off, and a stationary body has zero true
pitch rate. So, per bag, in successive windows after the stop:

  * mean published wy (deg/s)          -- a non-zero mean on a static body is the
                                          rate error itself; its decay in time is the
                                          signature of a bias STATE relaxing (EKF bias
                                          estimator), a step to zero is the signature of
                                          a motion-instantaneous error (g-sensitivity,
                                          vibration rectification);
  * quaternion pitch slope (deg/s)      -- what the controller would have consumed;
  * the pre-trigger static window       -- the reference (bias ~0 in 18/18 runs);
  * the in-arc excess from imu_vs_vicon3.json for the same bag, for scale.

'Static' is taken from the IMU itself: |gyro| rms < STATIC_GY deg/s and |a_z - g| small
over 1 s blocks, starting 2 s after trigger-off (the trigger-off hold snap, 325.12).

Usage: imu_poststop.py [--json imu_vs_vicon3.json] [--out imu_poststop.json] bag.db3 [...]
Needs ROS sourced. Read-only.
"""
import os, sys, json, sqlite3, argparse
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

HERE = os.path.dirname(os.path.abspath(__file__))
WINDOWS = [(2, 7), (7, 12), (12, 22), (22, 40)]
STATIC_GY = 2.0   # deg/s rms over 1 s blocks


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
    T0, TOFF = max(pairs, key=lambda p: p[1] - p[0])
    imu = rd("/imu")
    t = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, m in imu])
    o = np.argsort(t, kind="stable"); t = t[o]
    keep = np.concatenate(([True], np.diff(t) > 0))
    q = np.array([[m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z] for _, m in imu])[o][keep]
    g = np.degrees(np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for _, m in imu]))[o][keep]
    az = np.array([m.linear_acceleration.z for _, m in imu])[o][keep]
    t = t[keep]
    w_, x_, y_, z_ = q.T
    pitch = np.degrees(np.arcsin(np.clip(2 * (w_ * y_ - z_ * x_), -1, 1)))
    # header stamps vs bag time: the bag's trigger times are bag-clock; shift by the median offset
    tb = np.array([ts for ts, _ in imu])[o][keep]
    off = float(np.median(tb - t))
    return dict(t=t + off, pitch=pitch, g=g, az=az, T0=T0, TOFF=TOFF, gait=TOFF - T0, bag_end=float(t[-1] + off))


def slope(t, y):
    if len(t) < 20:
        return float("nan")
    A = np.polyfit(t, y, 1)
    return float(A[0])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--json", default=os.path.join(HERE, "imu_vs_vicon3.json"))
    ap.add_argument("--out", default=os.path.join(HERE, "imu_poststop.json"))
    a = ap.parse_args()
    excess = {}
    if os.path.exists(a.json):
        for r in json.load(open(a.json)):
            excess[r["bag"]] = dict(quat=r["axes"]["pitch"]["quat"]["excess_slope"],
                                    gyro=r["axes"]["pitch"]["gyro"]["excess_slope"],
                                    gbias_pre_y=r["gbias_pre"][1])
    out = {}
    print("%-16s %7s | %-8s %-8s %-8s %-8s | %-8s %-8s %-8s %-8s | %s" % (
        "bag", "in-arc", "wy 2-7", "wy 7-12", "wy12-22", "wy22-40", "dp 2-7", "dp 7-12", "dp12-22", "dp22-40", "static blocks / coverage s"))
    print("%-16s %7s | %-35s | %-35s |" % ("", "excess", "mean published wy after stop, deg/s", "quaternion pitch slope after stop, deg/s"))
    for DB in a.bags:
        name = os.path.basename(DB).replace("_0.db3", "")
        b = load(DB)
        ts = b["t"] - b["TOFF"]                       # seconds after trigger-off
        cov = float(b["bag_end"] - b["TOFF"])
        # static blocks (1 s) after the stop, by the IMU's own gyro rms
        blocks = []
        for k in range(2, int(cov)):
            m = (ts >= k) & (ts < k + 1)
            if m.sum() > 100:
                gy_rms = float(np.sqrt(np.mean(np.sum(b["g"][m] ** 2, axis=1))))
                blocks.append((k, gy_rms < STATIC_GY, gy_rms))
        static = {k for k, s, _ in blocks if s}
        row = dict(coverage_s=cov, n_static_blocks=len(static), n_blocks=len(blocks), windows={})
        pre = (ts > -b["gait"] - 2.0) & (ts < -b["gait"] - 0.2)
        row["pre_wy_mean"] = float(np.mean(b["g"][pre, 1])) if pre.sum() > 100 else float("nan")
        row["pre_pitch_slope"] = slope(ts[pre], b["pitch"][pre]) if pre.sum() > 100 else float("nan")
        cells_w, cells_p = [], []
        for (w0, w1) in WINDOWS:
            m = (ts >= w0) & (ts < w1)
            secs = [k for k in range(w0, w1) if k in static]
            if m.sum() > 100 and len(secs) >= 0.6 * (w1 - w0):
                ms = m & np.isin(np.floor(ts).astype(int), secs)
                wy = float(np.mean(b["g"][ms, 1])); dp = slope(ts[ms], b["pitch"][ms])
            else:
                wy = dp = float("nan")
            row["windows"]["%d-%d" % (w0, w1)] = dict(wy_mean=wy, pitch_slope=dp, static_secs=len(secs))
            cells_w.append(wy); cells_p.append(dp)
        row["in_arc_excess"] = excess.get(name)
        out[name] = row
        ex = excess.get(name, {}).get("quat", float("nan"))
        fmt = lambda v: ("%+8.3f" % v) if np.isfinite(v) else "     nan"
        print("%-16s %+7.3f | %s | %s | %d/%d static, %.0f s" % (
            name, ex, " ".join(fmt(v) for v in cells_w), " ".join(fmt(v) for v in cells_p), len(static), len(blocks), cov))
    # pooled reading
    def pooled(key, w):
        v = [r["windows"][w][key] for r in out.values() if np.isfinite(r["windows"][w][key])]
        return (float(np.median(v)), len(v)) if v else (float("nan"), 0)
    print("\nPOOLED medians after the stop (published wy, deg/s | quaternion pitch slope, deg/s):")
    for w in ["%d-%d" % ww for ww in WINDOWS]:
        a1, n1 = pooled("wy_mean", w); a2, n2 = pooled("pitch_slope", w)
        print("   %6s s : wy %+7.3f (n=%2d) | pitch slope %+7.3f (n=%2d)" % (w, a1, n1, a2, n2))
    pre = [r["pre_wy_mean"] for r in out.values() if np.isfinite(r["pre_wy_mean"])]
    exs = [r["in_arc_excess"]["quat"] for r in out.values() if r["in_arc_excess"]]
    print("   pre-trigger wy median %+7.3f deg/s (n=%d); in-arc quaternion excess median %+7.3f deg/s (n=%d)"
          % (np.median(pre) if pre else float("nan"), len(pre), np.median(exs) if exs else float("nan"), len(exs)))
    print("Reading: a non-zero post-stop wy that decays over tens of seconds = a bias STATE relaxing (EKF gyro-bias estimator);")
    print("         wy back at the pre-trigger level within the first window = a motion-instantaneous rate error.")
    with open(a.out, "w") as fh:
        json.dump(out, fh, indent=1)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
