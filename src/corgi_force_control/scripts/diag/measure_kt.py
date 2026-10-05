#!/usr/bin/env python3
"""Measure a motor's KT (true torque / firmware torque) with a lever and known masses.

Why: fpga_driver divides torque AND kp/ki/kd by each motor's KT (config.yaml),
so a wrong KT is a wrong stiffness. The R/L values were measured; every
Motor_H KT is an unmeasured copy of LF Motor_L (2.148).

What it does: holds the robot at its current pose (publishes motor/command),
and on each keypress averages the reported torque for --window seconds. Hang
masses on a horizontal bar at --radius from the joint axis; the applied torque
is m*g*r. Fit reported vs applied:

  --joint h     reported torque_h = firmware torque * KT_cfg, so
                KT_true = KT_cfg / slope.
  --joint beta  rig check. A weight on the leg is a pure beta torque, which is
                torque_l + torque_r in ROS units (phi_l = beta+theta,
                phi_r = beta-theta). Those are already scaled by the measured
                R/L KTs, so slope ~= 1.00 means the rig matches whatever
                produced them. Do this once before trusting --joint h.

Procedure (per module, per joint):
  1. Robot on a stand, robot mode ACTIVE, nothing else publishing
     motor/command (no gait / force_control / homing).
  2. Clamp the bar, level it, measure r from the joint axis to the hook.
  3. Run, then:  t = tare with only the bar (and hook) on.
     Hang masses in steps and enter the TOTAL hanging mass each time, e.g.
     0.5, 1, 2, 3, then unload 2, 1, 0.5 (loading + unloading averages out
     friction). Move the bar to the other side, re-tare, and enter the
     masses as negatives: -0.5, -1, ...
  4. f = fit,  u = undo last point,  q = fit, save, quit.

Units: masses in kg, radius in m. Results go to --out as JSON + CSV.
When it exits it STOPS publishing; the driver keeps the last command, so the
robot stays held. Drop to Idle from the panel when done.

Usage:
  measure_kt.py --module a --joint h    --radius 0.250
  measure_kt.py --module a --joint beta --radius 0.300
Needs ROS sourced (rclpy + corgi_msgs).
"""
import argparse
import csv
import json
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from corgi_msgs.msg import MotorCmdStamped, MotorStateStamped, RobotStateStamped

MODULES = "abcd"
ROBOT_MODE_ACTIVE = 3


class Rig:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("measure_kt")
        self.lock = threading.Lock()
        self.state = None
        self.robot_mode = None
        self.recording = None  # list while sampling
        self.hold = None       # captured pose per module
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
                m = getattr(msg, "module_" + self.args.module)
                if self.args.joint == "h":
                    tau, vel = m.torque_h, m.velocity_h
                else:
                    tau, vel = m.torque_l + m.torque_r, 0.5 * (m.velocity_l + m.velocity_r)
                self.recording.append((tau, vel))

    def _publish_hold(self):
        cmd = MotorCmdStamped()
        self.seq += 1
        cmd.header.seq = self.seq
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        for name in MODULES:
            c = getattr(cmd, "module_" + name)
            theta, beta, gamma = self.hold[name]
            c.theta, c.beta, c.gamma = theta, beta, gamma
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

    def sample(self, seconds):
        with self.lock:
            self.recording = []
        time.sleep(seconds)
        with self.lock:
            data, self.recording = self.recording, None
        if len(data) < 10:
            return None
        a = np.asarray(data)
        return {"mean": float(a[:, 0].mean()), "std": float(a[:, 0].std()),
                "n": int(len(a)), "vel_mean": float(a[:, 1].mean())}


def fit(points, joint, kt_cfg):
    x = np.array([p["applied"] for p in points])
    y = np.array([p["reported"] for p in points])
    if len(points) < 2 or np.ptp(x) == 0:
        return None
    s, c = np.polyfit(x, y, 1)
    res = y - (s * x + c)
    out = {"n": len(points), "slope": float(s), "intercept": float(c),
           "resid_rms": float(np.sqrt(np.mean(res ** 2))),
           "r2": float(1 - res.var() / y.var()) if y.var() > 0 else float("nan")}
    for side, mask in (("pos", x > 0), ("neg", x < 0)):
        xs = np.concatenate([[0.0], x[mask]])  # each side also passes the tare
        ys = np.concatenate([[0.0], y[mask]])
        out["slope_" + side] = float(np.polyfit(xs, ys, 1)[0]) if mask.sum() >= 2 else None
    if joint == "h":
        out["kt_true"] = float(kt_cfg / abs(s))
        for side in ("pos", "neg"):
            v = out["slope_" + side]
            out["kt_true_" + side] = float(kt_cfg / abs(v)) if v else None
    return out


def print_fit(f, joint, kt_cfg):
    if f is None:
        print("  need >= 2 points spanning different torques")
        return
    print("  %d points  slope %.4f  intercept %+.3f N.m  resid rms %.3f N.m  R^2 %.5f"
          % (f["n"], f["slope"], f["intercept"], f["resid_rms"], f["r2"]))
    if joint == "h":
        print("  KT_true = %.3f   (config KT %.3f)" % (f["kt_true"], kt_cfg))
        for side in ("pos", "neg"):
            if f.get("kt_true_" + side):
                print("    %s side only: %.3f" % (side, f["kt_true_" + side]))
        if f.get("kt_true_pos") and f.get("kt_true_neg"):
            d = abs(f["kt_true_pos"] - f["kt_true_neg"]) / f["kt_true"]
            print("    side-to-side spread %.1f %%%s" % (100 * d, "  (> 3 %: check level / r / friction)" if d > 0.03 else ""))
    else:
        print("  |slope| = %.3f  -> R/L config KTs %s the rig (1.00 = match; %.1f %% off)"
              % (abs(f["slope"]), "AGREE with" if abs(abs(f["slope"]) - 1) < 0.03 else "DISAGREE with",
                 100 * (abs(f["slope"]) - 1)))
    if abs(f["slope"]) > 0 and f["slope"] < 0:
        print("  (negative slope: the '+' side you chose is the motor's negative direction -- harmless)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--module", required=True, choices=list(MODULES))
    ap.add_argument("--joint", default="h", choices=["h", "beta"])
    ap.add_argument("--radius", type=float, required=True, help="joint axis to hook, m (bar level)")
    ap.add_argument("--kt-cfg", type=float, default=2.148,
                    help="KT in the sbRIO config.yaml for this motor (h only; default 2.148)")
    ap.add_argument("--window", type=float, default=3.0, help="averaging window per point, s")
    ap.add_argument("--kp-h", type=float, default=150.0)
    ap.add_argument("--kp-rl", type=float, default=90.0)
    ap.add_argument("--kd", type=float, default=1.75)
    ap.add_argument("--rate", type=float, default=500.0, help="hold command rate, Hz")
    ap.add_argument("--g", type=float, default=9.80665)
    ap.add_argument("--out", default="~/corgi_runs/kt_measure")
    ap.add_argument("--force", action="store_true", help="skip the other-publisher check")
    args = ap.parse_args()

    rclpy.init()
    rig = Rig(args)
    ex = SingleThreadedExecutor()
    ex.add_node(rig.node)

    err = rig.precheck(lambda t: ex.spin_once(timeout_sec=t))
    if err:
        print("REFUSED: " + err + ". Nothing was sent.")
        return 2
    rig.start_hold()
    threading.Thread(target=ex.spin, daemon=True).start()
    print("Holding the current pose (kp_h %.0f, kp_r/l %.0f, kd %.2f) at %.0f Hz."
          % (args.kp_h, args.kp_rl, args.kd, args.rate))
    print("module %s  joint %s  r = %.4f m  window %.1f s" % (args.module.upper(), args.joint, args.radius, args.window))
    print("commands: t = tare | <kg> / -<kg> = point | u = undo | f = fit | q = save & quit")

    points, tare = [], None
    try:
        while True:
            line = input("> ").strip().lower()
            if not line:
                continue
            if line == "q":
                break
            if line == "f":
                print_fit(fit(points, args.joint, args.kt_cfg), args.joint, args.kt_cfg)
                continue
            if line == "u":
                print("  removed %s" % (points.pop(),) if points else "  nothing to undo")
                continue
            if line == "t":
                s = rig.sample(args.window)
                if s is None:
                    print("  no motor/state samples -- bridge still up?")
                    continue
                tare = s
                print("  tare %+.3f N.m (std %.3f, n %d)" % (s["mean"], s["std"], s["n"]))
                continue
            try:
                mass = float(line)
            except ValueError:
                print("  ? (t / <kg> / -<kg> / u / f / q)")
                continue
            if tare is None:
                print("  tare first (t, bar on, no mass)")
                continue
            s = rig.sample(args.window)
            if s is None:
                print("  no motor/state samples -- bridge still up?")
                continue
            applied = mass * args.g * args.radius
            p = {"mass_kg": mass, "applied": applied, "reported": s["mean"] - tare["mean"],
                 "raw": s["mean"], "tare": tare["mean"], "std": s["std"], "n": s["n"],
                 "vel_mean": s["vel_mean"], "t": time.time()}
            points.append(p)
            warn = "  <-- noisy, re-take?" if s["std"] > 0.05 * max(abs(p["reported"]), 0.5) else ""
            print("  m %+.3f kg  applied %+.3f N.m  reported %+.3f N.m  (std %.3f)  ratio %.3f%s"
                  % (mass, applied, p["reported"], s["std"],
                     p["reported"] / applied if applied else float("nan"), warn))
    except (EOFError, KeyboardInterrupt):
        print()

    f = fit(points, args.joint, args.kt_cfg)
    print_fit(f, args.joint, args.kt_cfg)
    if points:
        out = os.path.join(os.path.expanduser(args.out),
                           "%s_%s%s" % (time.strftime("%Y%m%d_%H%M%S"), args.module, args.joint))
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, "points.csv"), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(points[0].keys()))
            w.writeheader()
            w.writerows(points)
        with open(os.path.join(out, "result.json"), "w") as fh:
            json.dump({"args": vars(args), "tare": tare, "fit": f, "points": points}, fh, indent=2)
        print("saved %s" % out)
    print("Stopped publishing. The driver keeps the last hold command -- drop to Idle from the panel.")
    rig.node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
