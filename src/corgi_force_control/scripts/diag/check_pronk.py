"""Record the running gait and check it against the G-SLIP template.

Reports per-leg joint ranges, inter-leg phase correlation (a pronk wants all
four in phase, so ~+1.0 against leg A), and the flight fraction against the
template's duty factor.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from corgi_msgs.msg import MotorStateStamped, SimLegContactStamped, ImuStamped

DUTY = 0.434   # template duty factor -> expect ~57% airborne


class Rec(Node):
    def __init__(self):
        super().__init__("gslip_check")
        self.motor, self.contact, self.imu = [], [], []
        self.create_subscription(MotorStateStamped, "motor/state", self.m_cb, 50)
        self.create_subscription(SimLegContactStamped, "sim/leg_contact", self.c_cb, 50)
        self.create_subscription(ImuStamped, "imu", self.i_cb, 50)

    def m_cb(self, msg):
        self.motor.append([[m.theta, m.beta, m.gamma] for m in
                           (msg.module_a, msg.module_b, msg.module_c, msg.module_d)])

    def c_cb(self, msg):
        self.contact.append([msg.module_a.contact, msg.module_b.contact,
                             msg.module_c.contact, msg.module_d.contact])

    def i_cb(self, msg):
        q = msg.orientation
        self.imu.append([q.x, q.y, q.z, q.w])


def main():
    rclpy.init()
    n = Rec()
    t_end = n.get_clock().now().nanoseconds + int(12e9)
    while rclpy.ok() and n.get_clock().now().nanoseconds < t_end:
        rclpy.spin_once(n, timeout_sec=0.0)

    m, c = np.array(n.motor), np.array(n.contact)
    print(f"samples: motor={len(m)} contact={len(c)} imu={len(n.imu)}")
    if len(m) < 10:
        print("NOT ENOUGH DATA -- is the gait running?")
        return

    names = "ABCD"
    print()
    print("per-module joint ranges (deg)")
    print(f"{'leg':>4} {'theta min':>10} {'theta max':>10} {'sweep':>7} "
          f"{'beta min':>9} {'beta max':>9} {'gamma max|':>11}")
    for i in range(4):
        th = np.rad2deg(m[:, i, 0])
        be = np.rad2deg(m[:, i, 1])
        ga = np.rad2deg(m[:, i, 2])
        print(f"{names[i]:>4} {th.min():10.2f} {th.max():10.2f} "
              f"{th.max()-th.min():7.2f} {be.min():9.2f} {be.max():9.2f} "
              f"{np.abs(ga).max():11.2f}")

    print()
    print("phase check (correlation of theta against leg A; pronk wants ~+1.0)")
    a = m[:, 0, 0] - m[:, 0, 0].mean()
    for i in range(1, 4):
        b = m[:, i, 0] - m[:, i, 0].mean()
        d = np.linalg.norm(a) * np.linalg.norm(b)
        r = float(a @ b / d) if d > 1e-12 else float("nan")
        print(f"   A vs {names[i]}: {r:+.3f}")

    if len(c):
        air = int((~c.any(axis=1)).sum())
        alld = int(c.all(axis=1).sum())
        print()
        print(f"contact: all four down {100*alld/len(c):.1f}% of samples, "
              f"airborne (flight) {100*air/len(c):.1f}%")
        print(f"  template duty factor is {DUTY}, so expect "
              f"~{100*(1-DUTY):.0f}% airborne")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
