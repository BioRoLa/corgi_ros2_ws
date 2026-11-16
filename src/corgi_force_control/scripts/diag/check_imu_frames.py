#!/usr/bin/env python3
"""Do /imu's quaternion and angular velocity describe the same body frame?

For each bag: the level attitude, and per axis the correlation of the published
body rate with the rate the quaternion implies (2 * q^-1 * dq/dt). In the
convention (CONVENTIONS.md: x forward, y left, z up, orientation and rate
agreeing in sign) all three are positive. Needs a bag with some motion on each
axis -- rock the body by hand in roll, pitch and yaw for a few seconds.

Until 2026-11-16 corgi_imu published the quaternion with pitch and yaw
mirrored: x +0.93..+0.96, y -0.96..-0.99, z -0.98..-1.00 on the 2026-09 and
2026-10-29 hardware bags. --as-fixed applies the node's correction
(w, x, -y, -z) to an old bag to preview it.

Usage: check_imu_frames.py [--as-fixed] <bag> [<bag> ...]
"""
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from corgi_msgs.msg import ImuStamped
def load(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    rows = []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic != "/imu": continue
        m = deserialize_message(data, ImuStamped)
        q, w, a = m.orientation, m.angular_velocity, m.linear_acceleration
        rows.append((t * 1e-9, q.w, q.x, q.y, q.z, w.x, w.y, w.z, a.x, a.y, a.z))
    return np.asarray(rows)
def qmul(a, b):
    w1, x1, y1, z1 = a.T; w2, x2, y2, z2 = b.T
    return np.stack([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2], 1)
AS_FIXED = '--as-fixed' in sys.argv
bad = 0
for path in [a for a in sys.argv[1:] if not a.startswith('--')]:
    d = load(path)
    if len(d) < 100: print(path, "no /imu"); continue
    t, q, w, a = d[:, 0], d[:, 1:5], d[:, 5:8], d[:, 8:11]
    q /= np.linalg.norm(q, axis=1)[:, None]
    if AS_FIXED:
        q = q * np.array([1, 1, -1, -1])
    qw, qx, qy, qz = q.T
    roll = np.degrees(np.arctan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy)))
    pitch = np.degrees(np.arcsin(np.clip(2*(qw*qy-qz*qx), -1, 1)))
    print("\n%s  n=%d  %.0f Hz" % (path.rstrip('/').split('/')[-1], len(d), len(d)/(t[-1]-t[0])))
    print("  mean q (w x y z) %s   roll %.1f pitch %.1f deg" % (np.round(q.mean(0), 3), np.median(roll), np.median(pitch)))
    print("  mean accel %s   mean rate %s" % (np.round(a.mean(0), 2), np.round(w.mean(0), 4)))
    # body rate from the quaternion: 2 * q^-1 * dq/dt, on a 20 ms grid
    step = max(1, int(0.02 * len(d) / (t[-1]-t[0])))
    qs, ts = q[::step], t[::step]
    qc = qs * np.array([1, -1, -1, -1])
    dq = qmul(qc[:-1], qs[1:])
    dq *= np.sign(dq[:, :1])
    wq = 2 * dq[:, 1:] / np.diff(ts)[:, None]
    wm = np.array([w[i:i+step].mean(0) for i in range(0, len(w) - step, step)])[:len(wq)]
    wq = wq[:len(wm)]
    for k, n in enumerate("xyz"):
        if wm[:, k].std() < 0.01: print("  rate %s: too little motion (std %.4f rad/s)" % (n, wm[:, k].std())); continue
        r = np.corrcoef(wm[:, k], wq[:, k])[0, 1]
        bad += r < 0.5
        print("  rate %s: corr(published, from quaternion) %+.3f   std %.3f rad/s%s" % (
            n, r, wm[:, k].std(), "" if r >= 0.5 else "   <-- quaternion and rate disagree"))
print()
print("PASS: quaternion and rate agree on every axis with motion" if not bad
      else "FAIL: %d axis/axes disagree -- is corgi_imu running legacy_orientation, or an old build?" % bad)
sys.exit(1 if bad else 0)
