"""Body attitude and per-leg contact balance.

Roll and pitch are reported SEPARATELY from yaw on purpose. The combined
"tilt" metric 2*acos|w| conflates yaw with tilt and misled this debugging four
separate times -- apparent tilts of 29, 62, 129 and 145 deg all turned out to
be mostly yaw.

Drift = last minus first, which is what tells you the robot is veering rather
than just oscillating.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from corgi_msgs.msg import SimLegContactStamped, ImuStamped


def quat_to_rpy(x, y, z, w):
    """Standard ZYX (yaw-pitch-roll) extraction, vectorised."""
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)


class Rec(Node):
    def __init__(self):
        super().__init__("rpy_check")
        self.imu, self.contact = [], []
        self.create_subscription(ImuStamped, "imu", self.i_cb, 50)
        self.create_subscription(SimLegContactStamped, "sim/leg_contact", self.c_cb, 50)

    def i_cb(self, msg):
        q = msg.orientation
        self.imu.append([q.x, q.y, q.z, q.w])

    def c_cb(self, msg):
        self.contact.append([msg.module_a.contact, msg.module_b.contact,
                             msg.module_c.contact, msg.module_d.contact])


def main():
    rclpy.init()
    n = Rec()
    t_end = n.get_clock().now().nanoseconds + int(12e9)
    while rclpy.ok() and n.get_clock().now().nanoseconds < t_end:
        rclpy.spin_once(n, timeout_sec=0.0)

    q = np.array(n.imu)
    if len(q) < 10:
        print("no imu data")
        return
    print(f"samples: {len(q)}")
    print()

    r, p, yw = quat_to_rpy(q[:, 0], q[:, 1], q[:, 2], q[:, 3])
    yw = np.rad2deg(np.unwrap(np.deg2rad(yw)))   # so drift is not wrapped
    for lbl, a in (("roll ", r), ("pitch", p), ("yaw  ", yw)):
        print(f"  {lbl}: mean {a.mean():+8.2f}  min {a.min():+8.2f}  "
              f"max {a.max():+8.2f}  drift {a[-1]-a[0]:+8.2f} deg")

    print()
    print("  roll  = rolling onto its side")
    print("  pitch = nose up/down")
    print("  yaw   = turning (drift means it is veering)")

    c = np.array(n.contact)
    if len(c):
        per = 100.0 * c.mean(axis=0)
        print()
        print(f"contact % per leg: A {per[0]:.0f}  B {per[1]:.0f}  "
              f"C {per[2]:.0f}  D {per[3]:.0f}")
        print("  (a pronk wants these roughly equal)")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
