"""Commanded vs measured, per leg -- and the torque saturation check.

This is the script that found the pronk's real problem: all four legs pegged at
the 35 N-m limit against a Phase-2 prediction of 18.7 N-m.

Read the beta ratios with the control law in mind. These are NOT position
servos: gslip_pronk publishes ImpedanceCmd and the CSV row is the virtual
spring's REST POSE, not a setpoint to track. Deviation from it is the spring
deflection -- the mechanism, not an error. So a ratio below 1 is only
interesting when it differs BETWEEN legs, or when the magnitude is far from
what the design compression predicts (100 -> 84.4 deg, about 15.6 deg).

Amplitude comparison needs no time alignment, so it is the primary metric. RMS
error interpolates the command onto the state timestamps.

Torque comes from motor/state, where the driver publishes the COMMANDED torque
post-clamp (corgi_driver.get_states), so it is the right signal for saturation.

Run this EARLY after the trigger: the gait stops roughly a minute in.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from corgi_msgs.msg import MotorCmdStamped, MotorStateStamped

NAMES = "ABCD"
TORQUE_LIMIT = 35.0
SAT = 0.98 * TORQUE_LIMIT


class Rec(Node):
    def __init__(self):
        super().__init__("cmd_vs_state")
        self.cmd_t, self.cmd = [], []
        self.st_t, self.st = [], []
        self.create_subscription(MotorCmdStamped, "motor/command", self.cmd_cb, 100)
        self.create_subscription(MotorStateStamped, "motor/state", self.st_cb, 100)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def cmd_cb(self, msg):
        self.cmd_t.append(self._now())
        self.cmd.append([[m.theta, m.beta, m.gamma] for m in
                         (msg.module_a, msg.module_b, msg.module_c, msg.module_d)])

    def st_cb(self, msg):
        self.st_t.append(self._now())
        self.st.append([[m.theta, m.beta, m.gamma, m.torque_r, m.torque_l] for m in
                        (msg.module_a, msg.module_b, msg.module_c, msg.module_d)])


def main():
    rclpy.init()
    n = Rec()
    t_end = n.get_clock().now().nanoseconds + int(12e9)
    # Tight spin: spin_once handles ONE callback per call, so a 0.05 s timeout
    # round-robins two 1 kHz topics down to ~67 Hz and aliases the stride.
    while rclpy.ok() and n.get_clock().now().nanoseconds < t_end:
        rclpy.spin_once(n, timeout_sec=0.0)

    cmd, st = np.array(n.cmd), np.array(n.st)
    ct, stt = np.array(n.cmd_t), np.array(n.st_t)
    print(f"samples: command={len(cmd)}  state={len(st)}")
    if len(cmd) < 20 or len(st) < 20:
        print("NOT ENOUGH DATA -- is the gait running, and is force_control up?")
        return

    print()
    print("beta amplitude, commanded vs measured (deg)")
    print(f"{'leg':>4} {'cmd sweep':>10} {'meas sweep':>11} {'ratio':>7} "
          f"{'cmd min':>8} {'cmd max':>8} {'meas min':>9} {'meas max':>9}")
    cmd_sweeps, ratios = [], []
    for i in range(4):
        cb, sb = np.rad2deg(cmd[:, i, 1]), np.rad2deg(st[:, i, 1])
        cs, ss = cb.max() - cb.min(), sb.max() - sb.min()
        r = ss / cs if cs > 1e-9 else float("nan")
        cmd_sweeps.append(cs)
        ratios.append(r)
        print(f"{NAMES[i]:>4} {cs:10.2f} {ss:11.2f} {r:7.3f} "
              f"{cb.min():8.2f} {cb.max():8.2f} {sb.min():9.2f} {sb.max():9.2f}")

    print()
    print("theta amplitude, commanded vs measured (deg)")
    print(f"{'leg':>4} {'cmd sweep':>10} {'meas sweep':>11} {'ratio':>7}")
    for i in range(4):
        c_, s_ = np.rad2deg(cmd[:, i, 0]), np.rad2deg(st[:, i, 0])
        cs, ss = c_.max() - c_.min(), s_.max() - s_.min()
        print(f"{NAMES[i]:>4} {cs:10.2f} {ss:11.2f} "
              f"{(ss/cs if cs > 1e-9 else float('nan')):7.3f}")

    print()
    print("beta tracking error (deg), command interpolated onto state clock")
    print(f"{'leg':>4} {'rms':>8} {'max|e|':>8} {'mean e':>8}")
    for i in range(4):
        ci = np.interp(stt, ct, np.rad2deg(cmd[:, i, 1]))
        e = np.rad2deg(st[:, i, 1]) - ci
        print(f"{NAMES[i]:>4} {np.sqrt((e**2).mean()):8.3f} "
              f"{np.abs(e).max():8.3f} {e.mean():+8.3f}")

    print()
    print(f"commanded torque per leg (N-m), limit {TORQUE_LIMIT}")
    print(f"{'leg':>4} {'max|tr|':>9} {'max|tl|':>9} {'% of limit':>11} {'sat samples':>12}")
    for i in range(4):
        tr, tl = st[:, i, 3], st[:, i, 4]
        peak = max(np.abs(tr).max(), np.abs(tl).max())
        sat = int(((np.abs(tr) > SAT) | (np.abs(tl) > SAT)).sum())
        print(f"{NAMES[i]:>4} {np.abs(tr).max():9.2f} {np.abs(tl).max():9.2f} "
              f"{100*peak/TORQUE_LIMIT:10.1f}% {sat:12d}")

    # Separate the two questions properly: is the COMMAND asymmetric, and is the
    # RESPONSE asymmetric? An earlier version conflated them and drew the wrong
    # conclusion from agreeing ratios.
    print()
    cmd_spread = max(cmd_sweeps) - min(cmd_sweeps)
    ratio_spread = max(ratios) - min(ratios)
    print(f"command spread across legs: {cmd_spread:.2f} deg")
    print(f"response ratio spread:      {ratio_spread:.3f}")
    if cmd_spread > 0.5:
        print("-> the COMMAND differs between legs. Look at apply_row and the")
        print("   template mapping before touching gains.")
    elif ratio_spread > 0.10:
        print("-> command is symmetric but the RESPONSE is not. Suspect impedance")
        print("   or torque saturation on the low-ratio legs; check the saturated")
        print("   sample counts above, which usually name the same legs.")
    else:
        print("-> command and response are both symmetric across legs. Any")
        print("   asymmetry seen elsewhere this run is not coming from beta.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
