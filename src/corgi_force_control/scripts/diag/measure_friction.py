#!/usr/bin/env python3
"""Measure each R/L motor's Coulomb (+ viscous) friction with a slow beta roll.

Why: force_control adds a direction-latched friction feedforward, friction_ in
force_control.hpp (AR, AL, BR, BL, CR, CL, DR, DL, N.m in ROS torque units).
Those numbers come from Eason's 2025-03 kt_test (commits 2260d3c / 92843f9,
deleted): stiff PD hold, roll every leg's beta 180 deg forward over 20 s, hold,
roll back, compare the torque in the two directions. They predate the ABAD
hardware and nothing has re-measured them since. This is that test, ported.

What it does: captures the current pose, then commands every module's beta
through a slow trapezoidal roll of --range deg and back, once per --speeds
entry, with theta and gamma held. A and D roll in -beta, B and C in +beta
(same as kt_test, so all four legs turn the same physical way). With theta
fixed, phi_r = beta - theta and phi_l = beta + theta move exactly with beta.

At a constant speed the PD torque is friction + gravity(beta). The gravity part
depends only on position, so at the same beta
    tau(phi rising) - tau(phi falling) = 2 * friction(speed) * s
where s = +/-1 is the motor's torque-vs-phi sign. Per motor and per speed this
takes the median over 5 deg beta bins of half that difference; across speeds it
fits Fc + b*omega. Fc is directly comparable to friction_.

force_control SUBTRACTS friction_ while phi rises, so its feedforward assists
the motion only if tau(phi rising) < tau(phi falling). The SIGN column checks
that, which no earlier measurement did.

Procedure:
  1. Robot on a stand with EVERY foot clear of the ground and the stand -- the
     legs turn a full --range. Robot mode ACTIVE, nothing else on
     motor/command (no gait / force_control / homing).
  2. Run. It asks for confirmation, rolls, and prints the table.
  3. Paste the "friction_ =" line into force_control.hpp only if SIGN agrees
     and the per-speed values are consistent; A/B it on hardware with
     friction_ff_scale (0 = off, 1 = on).

When it exits it STOPS publishing; the driver keeps the last command, so the
robot stays held at the starting pose. Drop to Idle from the panel when done.

Usage:
  measure_friction.py                          # 180 deg at 10, 20, 40 deg/s
  measure_friction.py --range 90 --speeds 15   # quick look
  measure_friction.py --modules bc             # only B and C roll
Needs ROS sourced (rclpy + corgi_msgs).
"""
import argparse
import csv
import json
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from corgi_msgs.msg import MotorCmdStamped, MotorStateStamped, RobotStateStamped

MODULES = "abcd"
ROLL_SIGN = {"a": -1.0, "b": +1.0, "c": +1.0, "d": -1.0}  # kt_test's mirroring
ROBOT_MODE_ACTIVE = 3
# force_control.hpp friction_, for comparison (AR, AL, BR, BL, CR, CL, DR, DL)
FRICTION_CFG = [0.625, 0.44, 0.662, 0.499, 0.623, 0.409, 0.677, 0.356]


def trapezoid(dist, speed, accel_t):
    """Positions (0 -> dist) of a trapezoidal profile sampled at 1 ms, + its velocity."""
    t_ramp = min(accel_t, dist / speed)
    t_cruise = max(0.0, dist / speed - t_ramp)
    t_total = 2 * t_ramp + t_cruise
    t = np.arange(0.0, t_total + 1e-9, 0.001)
    acc = speed / t_ramp
    v = np.where(t < t_ramp, acc * t,
                 np.where(t < t_ramp + t_cruise, speed, np.maximum(0.0, acc * (t_total - t))))
    x = np.concatenate([[0.0], np.cumsum(0.5 * (v[1:] + v[:-1]) * 0.001)])
    return x * (dist / x[-1]), v


class Rig:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("measure_friction")
        self.lock = threading.Lock()
        self.state = None
        self.robot_mode = None
        self.recording = None
        self.hold = None
        self.offset = {n: 0.0 for n in MODULES}  # commanded beta offset from hold, rad
        self.seq = 0
        self.node.create_subscription(MotorStateStamped, "motor/state", self._state_cb, 10)
        self.node.create_subscription(RobotStateStamped, "robot/state", self._robot_cb, 10)
        self.pub = None

    # -- callbacks ---------------------------------------------------------
    def _robot_cb(self, msg):
        self.robot_mode = msg.robot_mode

    def _state_cb(self, msg):
        with self.lock:
            self.state = msg
            if self.recording is not None:
                row = [time.monotonic()]
                for n in MODULES:
                    m = getattr(msg, "module_" + n)
                    row += [m.theta, m.beta, m.velocity_r, m.velocity_l, m.torque_r, m.torque_l]
                self.recording.append(row)

    def _publish_hold(self):
        cmd = MotorCmdStamped()
        self.seq += 1
        cmd.header.seq = self.seq
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        with self.lock:
            offset = dict(self.offset)
        for name in MODULES:
            c = getattr(cmd, "module_" + name)
            theta, beta, gamma = self.hold[name]
            c.theta, c.beta, c.gamma = theta, beta + offset[name], gamma
            c.kp_r = c.kp_l = self.args.kp_rl
            c.kp_h = self.args.kp_h
            c.kd_r = c.kd_l = c.kd_h = self.args.kd
        self.pub.publish(cmd)

    # -- setup ---------------------------------------------------------------
    def precheck(self, spin):
        t_end = time.time() + 3.0
        while time.time() < t_end and (self.state is None or self.robot_mode is None):
            spin(0.05)
        if self.state is None:
            return "no motor/state within 3 s -- is the bridge up?"
        if self.robot_mode is None:
            return "no robot/state within 3 s -- is the bridge up?"
        if self.robot_mode != ROBOT_MODE_ACTIVE:
            return "robot mode is %d, needs %d (ACTIVE)" % (self.robot_mode, ROBOT_MODE_ACTIVE)
        others = self.node.count_publishers("motor/command")
        if others and not self.args.force:
            return ("%d other publisher(s) on motor/command -- stop the gait / "
                    "force_control / homing first (or --force)" % others)
        return None

    def start_hold(self):
        with self.lock:
            self.hold = {n: (getattr(self.state, "module_" + n).theta,
                             getattr(self.state, "module_" + n).beta,
                             getattr(self.state, "module_" + n).gamma) for n in MODULES}
        self.pub = self.node.create_publisher(MotorCmdStamped, "motor/command", 5)
        self.node.create_timer(1.0 / self.args.rate, self._publish_hold)

    def roll(self, x):
        """Step the commanded offset through x (rad, 1 ms apart) on the rolling modules."""
        t0 = time.monotonic()
        for k, xk in enumerate(x):
            with self.lock:
                for n in self.args.modules:
                    self.offset[n] = ROLL_SIGN[n] * xk
            dt = t0 + (k + 1) * 0.001 - time.monotonic()
            if dt > 0:
                time.sleep(dt)

    def run_speed(self, speed_deg):
        dist = math.radians(self.args.range)
        x, _ = trapezoid(dist, math.radians(speed_deg), self.args.accel_time)
        with self.lock:
            self.recording = []
        time.sleep(self.args.pause)
        self.roll(x)
        time.sleep(self.args.pause)
        self.roll(x[::-1])
        time.sleep(self.args.pause)
        with self.lock:
            data, self.recording = self.recording, None
        return np.asarray(data)


def analyse(data, speed_deg, args):
    """Per module/motor friction at one speed. Column layout from Rig._state_cb."""
    out = {}
    speed = math.radians(speed_deg)
    for i, n in enumerate(MODULES):
        if n not in args.modules:
            continue
        c = 1 + 6 * i
        beta = np.unwrap(data[:, c + 1])
        for side, vcol, tcol in (("r", c + 2, c + 4), ("l", c + 3, c + 5)):
            vel, tau = data[:, vcol], data[:, tcol]
            # cruise only: inertia is negligible and the speed is the target
            rising = vel > args.cruise_frac * speed
            falling = vel < -args.cruise_frac * speed
            bins = np.arange(beta.min(), beta.max() + args.bin_deg * math.pi / 180,
                             math.radians(args.bin_deg))
            halves = []
            for lo, hi in zip(bins[:-1], bins[1:]):
                up = tau[rising & (beta >= lo) & (beta < hi)]
                dn = tau[falling & (beta >= lo) & (beta < hi)]
                if len(up) >= 5 and len(dn) >= 5:
                    halves.append(0.5 * (up.mean() - dn.mean()))
            h = np.asarray(halves)
            out[n + side] = {
                "speed_deg_s": speed_deg,
                "bins": int(len(h)),
                "half_diff_median": float(np.median(h)) if len(h) else None,
                "half_diff_iqr": float(np.subtract(*np.percentile(h, [75, 25]))) if len(h) else None,
                "n_rising": int(rising.sum()), "n_falling": int(falling.sum()),
            }
    return out


def summarise(per_speed, args):
    rows = {}
    for key in [n + s for n in args.modules for s in "rl"]:
        pts = [(r[key]["speed_deg_s"], r[key]["half_diff_median"]) for r in per_speed
               if r[key]["half_diff_median"] is not None]
        if not pts:
            rows[key] = None
            continue
        w = np.radians([p[0] for p in pts])
        hd = np.array([p[1] for p in pts])
        sign = float(np.sign(np.median(hd)))
        mag = np.abs(hd)
        if len(pts) >= 2 and np.ptp(w) > 0:
            b, fc = np.polyfit(w, mag, 1)
        else:
            b, fc = 0.0, float(mag[0])
        idx = MODULES.index(key[0]) * 2 + (0 if key[1] == "r" else 1)
        rows[key] = {"fc": float(fc), "b": float(b), "sign": sign,
                     "per_speed": {"%g" % p[0]: float(abs(p[1])) for p in pts},
                     "consistent_sign": bool(np.all(np.sign(hd) == sign)),
                     "cfg": FRICTION_CFG[idx]}
    return rows


def print_table(rows, args):
    speeds = ["%g" % s for s in args.speeds]
    print("\nmotor   Fc N.m   b N.m.s/rad  cfg    Fc/cfg  " + "  ".join("%5s" % ("@" + s) for s in speeds) + "   SIGN")
    for key, r in rows.items():
        label = key[0].upper() + key[1].upper()
        if r is None:
            print("%-6s  no cruise samples in both directions -- increase --range or lower --cruise-frac" % label)
            continue
        # force_control subtracts friction_ while phi rises: assists iff tau(rise) < tau(fall)
        agrees = r["sign"] < 0
        sign_txt = ("agrees" if agrees else "OPPOSES force_control") + ("" if r["consistent_sign"] else ", mixed")
        print("%-6s  %6.3f   %8.4f   %5.3f  %5.2f   " % (label, r["fc"], r["b"], r["cfg"], r["fc"] / r["cfg"])
              + "  ".join("%5.3f" % r["per_speed"].get(s, float("nan")) for s in speeds) + "   " + sign_txt)
    vals = []
    for n in MODULES:
        for s in "rl":
            r = rows.get(n + s)
            vals.append("%.3f" % r["fc"] if r else "%.3f" % FRICTION_CFG[MODULES.index(n) * 2 + (s == "l")])
    print("\nfriction_ = {%s};  // AR, AL, BR, BL, CR, CL, DR, DL; unmeasured motors keep cfg" % ", ".join(vals))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--modules", default=MODULES, help="modules to roll, e.g. 'bc' (default all)")
    ap.add_argument("--range", type=float, default=180.0, help="roll distance, deg (kt_test: 180)")
    ap.add_argument("--speeds", type=float, nargs="+", default=[10.0, 20.0, 40.0], help="deg/s (kt_test: 9)")
    ap.add_argument("--accel-time", type=float, default=0.5, help="trapezoid ramp, s")
    ap.add_argument("--pause", type=float, default=1.5, help="hold between moves, s")
    ap.add_argument("--cruise-frac", type=float, default=0.8, help="keep samples above this fraction of target speed")
    ap.add_argument("--bin-deg", type=float, default=5.0)
    ap.add_argument("--kp-h", type=float, default=150.0)
    ap.add_argument("--kp-rl", type=float, default=150.0, help="kt_test: 150")
    ap.add_argument("--kd", type=float, default=1.75, help="kt_test: 1.75")
    ap.add_argument("--rate", type=float, default=500.0, help="command rate, Hz")
    ap.add_argument("--out", default="~/corgi_runs/friction_measure")
    ap.add_argument("--force", action="store_true", help="skip the other-publisher check")
    ap.add_argument("--yes", action="store_true", help="skip the feet-clear confirmation")
    args = ap.parse_args()
    args.modules = "".join(n for n in MODULES if n in args.modules.lower())
    if not args.modules:
        ap.error("--modules names none of a, b, c, d")

    rclpy.init()
    rig = Rig(args)
    ex = SingleThreadedExecutor()
    ex.add_node(rig.node)

    err = rig.precheck(lambda t: ex.spin_once(timeout_sec=t))
    if err:
        print("REFUSED: " + err + ". Nothing was sent.")
        return 2
    total = sum(2 * args.range / s + 2 * args.accel_time + 3 * args.pause for s in args.speeds)
    print("Will roll beta %g deg and back on module(s) %s at %s deg/s (~%.0f s), theta and gamma held."
          % (args.range, args.modules.upper(), ", ".join("%g" % s for s in args.speeds), total))
    if not args.yes and input("Robot on a stand, every foot clear of the ground and the stand? [y/N] ").strip().lower() != "y":
        print("Aborted. Nothing was sent.")
        return 1
    rig.start_hold()
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(1.0)  # settle into the hold before the first roll

    per_speed, raw = [], {}
    try:
        for s in args.speeds:
            print("  rolling at %g deg/s ..." % s, flush=True)
            data = rig.run_speed(s)
            if len(data) < 100:
                print("  only %d motor/state samples -- bridge still up? stopping." % len(data))
                break
            raw["%g" % s] = data
            per_speed.append(analyse(data, s, args))
    except KeyboardInterrupt:
        print("\n  interrupted; returning to the start pose")
        with rig.lock:
            back = {n: rig.offset[n] for n in args.modules}
        steps = int(max(abs(v) for v in back.values()) / math.radians(10.0) * 1000) + 1
        for k in range(1, steps + 1):
            with rig.lock:
                for n in back:
                    rig.offset[n] = back[n] * (1 - k / steps)
            time.sleep(0.001)

    rows = summarise(per_speed, args) if per_speed else {}
    if rows:
        print_table(rows, args)
        out = os.path.join(os.path.expanduser(args.out), time.strftime("%Y%m%d_%H%M%S"))
        os.makedirs(out, exist_ok=True)
        header = ["t"] + ["%s_%s" % (f, n) for n in MODULES
                          for f in ("theta", "beta", "vel_r", "vel_l", "tau_r", "tau_l")]
        for s, data in raw.items():
            with open(os.path.join(out, "roll_%sdeg_s.csv" % s), "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(header)
                w.writerows(data.tolist())
        with open(os.path.join(out, "result.json"), "w") as fh:
            json.dump({"args": vars(args), "hold": rig.hold, "per_speed": per_speed, "summary": rows}, fh, indent=2)
        print("saved %s" % out)
    print("Stopped publishing. The driver keeps the last hold command -- drop to Idle from the panel.")
    rig.node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
