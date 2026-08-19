#!/usr/bin/env python3
"""record_camber.py's pattern + the IMU stream -- the Stage 1.5 recorder.

The banked Stage 1 corpus has no IMU: record_camber.py never subscribed to
`imu`, and its odom callback drops twist.angular. That makes a full ESEKF
replay impossible on banked data (the filter propagates on the IMU). This
recorder exists to close exactly that gap for the announced Stage 1.5
re-record subset; it lives in corgi_odometry/script (NOT scripts/diag, which
another thread owns).

Everything record_camber.py keeps is kept, for its reasons: sim time
(Webots ~14x slower, varying), spin_once(timeout_sec=0.0) (a timeout
aliases the 1 kHz subs to ~67 Hz), the same npz key names (check_turn.py /
check_camber_turn.py / the stage15 analyser read these dumps unchanged).
Added keys, all trigger-anchored like the rest:

    imu_t     (n,)    sim time
    imu_quat  (n, 4)  orientation [x y z w]   -- InertialUnit
    imu_gyro  (n, 3)  angular velocity, rad/s -- Gyro
    imu_accel (n, 3)  linear acceleration, m/s^2 -- Accelerometer
    odom_ang  (k, 3)  GT twist.angular (record_camber drops this)

CAVEAT carried from recon: corgi_driver.py's Webots device names are
cross-wired (getDevice("imu") is the Accelerometer, "ang_vel" the Gyro,
"gyro" the InertialUnit). The ImuStamped FIELDS are believed to be wired
semantically (orientation/angular_velocity/linear_acceleration); the sweep
runner gates each dump on |mean accel| ~ g and quat norm ~ 1, and the
analysis cross-checks imu_gyro against the GT-quaternion rate. If those
disagree, suspect the device wiring first.

Usage:
    python3 record_camber_imu.py --dump /tmp/x.npz --until 32.5
"""
import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from corgi_msgs.msg import (ImuStamped, MotorCmdStamped, MotorStateStamped,
                            SimLegContactStamped, TriggerStamped)


class Rec(Node):
    def __init__(self):
        super().__init__("camber_imu_recorder",
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 "use_sim_time",
                                 rclpy.Parameter.Type.BOOL, True)])
        self.contact, self.motor, self.cmd, self.odom = [], [], [], []
        self.torque, self.imu, self.odom_ang = [], [], []
        self.t_trigger = None
        self.create_subscription(SimLegContactStamped, "sim/leg_contact",
                                 self.c_cb, 100)
        self.create_subscription(MotorStateStamped, "motor/state",
                                 self.m_cb, 100)
        self.create_subscription(MotorCmdStamped, "motor/command",
                                 self.k_cb, 100)
        self.create_subscription(Odometry, "sim/base_odom", self.o_cb, 100)
        self.create_subscription(ImuStamped, "imu", self.i_cb, 100)
        self.create_subscription(TriggerStamped, "trigger", self.t_cb, 10)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def c_cb(self, msg):
        self.contact.append((self._now(), [
            msg.module_a.contact, msg.module_b.contact,
            msg.module_c.contact, msg.module_d.contact]))

    def m_cb(self, msg):
        now = self._now()
        mods = (msg.module_a, msg.module_b, msg.module_c, msg.module_d)
        self.motor.append((now, [[m.theta, m.beta, m.gamma] for m in mods]))
        # torque kept separate: `motor` is rad2deg'd wholesale downstream
        self.torque.append((now, [[m.torque_r, m.torque_l, m.torque_h]
                                  for m in mods]))

    def k_cb(self, msg):
        mods = (msg.module_a, msg.module_b, msg.module_c, msg.module_d)
        self.cmd.append((self._now(),
                         [[m.theta, m.beta, m.gamma] for m in mods]))

    def o_cb(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        q = msg.pose.pose.orientation
        w = msg.twist.twist.angular
        now = self._now()
        self.odom.append((now,
                          [p.x, p.y, p.z, v.x, v.y, v.z, q.x, q.y, q.z, q.w]))
        self.odom_ang.append((now, [w.x, w.y, w.z]))

    def i_cb(self, msg):
        q = msg.orientation
        g = msg.angular_velocity
        a = msg.linear_acceleration
        self.imu.append((self._now(),
                         [q.x, q.y, q.z, q.w, g.x, g.y, g.z, a.x, a.y, a.z]))

    def t_cb(self, msg):
        if msg.enable and self.t_trigger is None:
            self.t_trigger = self._now()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dump", required=True, help="output .npz")
    ap.add_argument("--until", type=float, default=32.5,
                    help="seconds of SIM time after the trigger (default: "
                         "%(default)s)")
    ap.add_argument("--timeout", type=float, default=1200.0,
                    help="wall-clock give-up, seconds (default: %(default)s)")
    args = ap.parse_args()

    rclpy.init()
    n = Rec()
    wall_end = time.time() + args.timeout
    while rclpy.ok() and time.time() < wall_end:
        rclpy.spin_once(n, timeout_sec=0.0)
        if (n.t_trigger is not None and n.odom
                and n.odom[-1][0] - n.t_trigger > args.until):
            break

    print(f"samples: contact={len(n.contact)} motor={len(n.motor)} "
          f"cmd={len(n.cmd)} odom={len(n.odom)} imu={len(n.imu)}")
    if n.t_trigger is None:
        print("NO TRIGGER SEEN -- nothing is anchored, nothing is written.")
        rclpy.shutdown()
        return
    if not n.odom:
        print("NO sim/base_odom -- no trajectory. Is corgi_sim current?")
        rclpy.shutdown()
        return
    if not n.imu:
        print("NO imu SAMPLES -- the whole point of this recorder. Is the "
              "driver publishing `imu` (ImuStamped)? Nothing is written.")
        rclpy.shutdown()
        return

    anchor = n.t_trigger
    ct = np.array([t for t, _ in n.contact]) - anchor
    cv = np.array([v for _, v in n.contact], dtype=bool)
    mt = np.array([t for t, _ in n.motor]) - anchor
    mv = np.rad2deg(np.array([v for _, v in n.motor]))
    kt = np.array([t for t, _ in n.cmd]) - anchor
    kv = np.rad2deg(np.array([v for _, v in n.cmd]))
    ot = np.array([t for t, _ in n.odom]) - anchor
    ov = np.array([v for _, v in n.odom])
    oa = np.array([v for _, v in n.odom_ang])
    qt = np.array([t for t, _ in n.torque]) - anchor
    qv = np.array([v for _, v in n.torque])
    it_ = np.array([t for t, _ in n.imu]) - anchor
    iv = np.array([v for _, v in n.imu])

    covered = ot.max()
    print(f"anchor: trigger at sim t={anchor:.2f}")
    print(f"recorded {ot.min():.2f} .. {covered:.2f} s after the trigger")
    if covered < args.until - 0.5:
        print(f"  WARNING: {args.until - covered:.2f} s short of --until.")
    a_norm = float(np.linalg.norm(iv[:, 7:10], axis=1).mean())
    print(f"imu sanity: |accel| mean {a_norm:.2f} m/s^2 (expect ~9.8), "
          f"gyro rms {float(np.sqrt((iv[:, 4:7] ** 2).mean())):.3f} rad/s, "
          f"quat norm mean "
          f"{float(np.linalg.norm(iv[:, 0:4], axis=1).mean()):.3f}")

    np.savez_compressed(args.dump, contact_t=ct, contact=cv,
                        motor_t=mt, motor_deg=mv,
                        cmd_t=kt, cmd_deg=kv,
                        odom_t=ot, odom=ov, odom_ang=oa,
                        torque_t=qt, torque_nm=qv,
                        imu_t=it_, imu_quat=iv[:, 0:4], imu_gyro=iv[:, 4:7],
                        imu_accel=iv[:, 7:10],
                        anchor=anchor)
    print(f"raw samples written to {args.dump}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
