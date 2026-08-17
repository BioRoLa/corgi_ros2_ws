#!/usr/bin/env python3
"""Record a camber-roll pass to an .npz. Companion to camber_roll.py.

Deliberately NOT check_ramp.py. That one keys everything to a stride template
and anchors on the commanded-beta fiducial it finds on impedance/command --
neither of which exists here. camber_roll.py is not a gait: it publishes
position commands straight to motor/command and rolls a continuously
increasing beta, so the anchor is simply the trigger.

What it keeps, because the reasons still hold:

  * Sim time, not wall time. Webots runs ~14x slower than real time here and
    the factor varies between runs, so a wall-clock stop lands at a different
    point in the schedule every time.
  * spin_once(timeout_sec=0.0). With a timeout the 1 kHz subscriptions
    round-robin down to ~67 Hz and alias.
  * The same npz key names check_ramp.py writes, so check_turn.py and
    check_camber_turn.py can both read either kind of dump.

It also records the COMMAND stream, which check_ramp does not need but this
does: the whole run is void if the robot is not in the pose it was given, and
the commanded-vs-measured camber is the check for that.

Usage:
    python3 record_camber.py --dump /tmp/camber_lr20.npz --until 30
"""
import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from corgi_msgs.msg import (MotorCmdStamped, MotorStateStamped,
                            SimLegContactStamped, TriggerStamped)


class Rec(Node):
    def __init__(self):
        super().__init__("camber_recorder",
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 "use_sim_time",
                                 rclpy.Parameter.Type.BOOL, True)])
        self.contact, self.motor, self.cmd, self.odom = [], [], [], []
        self.torque = []
        self.t_trigger = None
        self.create_subscription(SimLegContactStamped, "sim/leg_contact",
                                 self.c_cb, 100)
        self.create_subscription(MotorStateStamped, "motor/state",
                                 self.m_cb, 100)
        self.create_subscription(MotorCmdStamped, "motor/command",
                                 self.k_cb, 100)
        self.create_subscription(Odometry, "sim/base_odom", self.o_cb, 100)
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
        # Torque in its own list: `motor` is converted wholesale with
        # np.rad2deg() downstream, which would silently scale every torque by
        # 57.3 -- a wrong answer that looks like a plausible one.
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
        self.odom.append((self._now(),
                          [p.x, p.y, p.z, v.x, v.y, v.z, q.x, q.y, q.z, q.w]))

    def t_cb(self, msg):
        if msg.enable and self.t_trigger is None:
            self.t_trigger = self._now()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dump", required=True, help="output .npz")
    ap.add_argument("--until", type=float, default=30.0,
                    help="seconds of SIM time after the trigger to record. "
                         "camber_cycle.sh waits on this process and drops the "
                         "trigger when it exits, so this ends the whole run. "
                         "Must cover camber_roll.py's whole schedule -- the "
                         "default 20 s roll ends at 27.5 s (default: "
                         "%(default)s)")
    ap.add_argument("--timeout", type=float, default=1200.0,
                    help="wall-clock give-up, seconds. At ~14x slower than "
                         "real time a 30 s pass is ~7 min (default: "
                         "%(default)s)")
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
          f"cmd={len(n.cmd)} odom={len(n.odom)}")
    if n.t_trigger is None:
        print("NO TRIGGER SEEN -- was this recorder started before the "
              "trigger? Nothing is anchored, so nothing is written.")
        rclpy.shutdown()
        return
    if not n.odom:
        print("NO sim/base_odom -- there is no trajectory to fit. Is corgi_sim "
              "built from a revision that publishes it?")
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
    qt = np.array([t for t, _ in n.torque]) - anchor
    qv = np.array([v for _, v in n.torque])

    covered = ot.max()
    print(f"anchor: trigger at sim t={anchor:.2f}")
    print(f"recorded {ot.min():.2f} .. {covered:.2f} s after the trigger")
    if covered < args.until - 0.5:
        print(f"  WARNING: {args.until - covered:.2f} s short of --until. The "
              f"run ended early -- check the controller log before trusting "
              f"the fit; a truncated roll is a short arc.")

    np.savez_compressed(args.dump, contact_t=ct, contact=cv,
                        motor_t=mt, motor_deg=mv,
                        cmd_t=kt, cmd_deg=kv,
                        odom_t=ot, odom=ov,
                        torque_t=qt, torque_nm=qv,
                        anchor=anchor)
    print(f"raw samples written to {args.dump}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
