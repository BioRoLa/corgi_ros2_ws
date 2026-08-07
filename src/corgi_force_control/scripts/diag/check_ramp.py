"""Per-segment flight measurement for the G-SLIP speed-ramp template.

Why this exists: sim_cycle.sh samples once, on a fixed 20 s WALL delay, and
reports a single averaged flight fraction. That is wrong for the ramp in two
separate ways.

  1. The ramp changes speed across 9.58 s, so one average over the whole thing
     cannot show whether the ladder worked. Each rung needs its own number.

  2. Webots runs well below real time (measured ~14x slower), and the factor
     varies between runs. A fixed wall delay therefore lands at a different
     *gait age* every run -- which is what produced 68.2% and 9.3% flight from
     two identical hop invocations. Everything here is keyed to SIM time.

Anchoring. gslip_pronk plays the CSV one row per 1 ms tick and wraps at the
end, so template time must be recovered, not assumed. Commanded beta is exactly
zero through the hop segment and steps to +-7 deg when the first forward
segment starts, which is an unambiguous fiducial at a known template time. We
latch that instant and derive t_template from it. If beta never leaves zero
(e.g. a hop-only template, or a robot that never got that far) we fall back to
the trigger instant plus the controller's settle_ticks.

Only the FIRST pass is reported. At the wrap the template snaps from v~1.20
back to the in-place hop while the robot is still carrying 2.035 m/s, so
anything after that is measuring a different experiment.

Usage:
    python3 check_ramp.py <template.csv> [--settle 1.0] [--timeout 400]
"""
import argparse
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node

from corgi_msgs.msg import (ImpedanceCmdStamped, ImuStamped, MotorStateStamped,
                            SimLegContactStamped, TriggerStamped)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ramp_segments import segment_template, stride_boundaries  # noqa: E402

# Template time at which the first forward segment begins, i.e. the first row
# with non-zero commanded beta. Recovered from the CSV, not hard-coded.
BETA_FIDUCIAL_RAD = np.deg2rad(0.5)


class Rec(Node):
    def __init__(self):
        super().__init__("gslip_ramp_check",
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 "use_sim_time",
                                 rclpy.Parameter.Type.BOOL, True)])
        self.contact, self.motor, self.cmd, self.imu = [], [], [], []
        self.t_trigger = None
        # Spin with timeout_sec=0.0 (see README): with a timeout, three 1 kHz
        # subscriptions round-robin down to ~67 Hz and alias the stride.
        self.create_subscription(SimLegContactStamped, "sim/leg_contact",
                                 self.c_cb, 100)
        self.create_subscription(MotorStateStamped, "motor/state",
                                 self.m_cb, 100)
        self.create_subscription(ImpedanceCmdStamped, "impedance/command",
                                 self.k_cb, 100)
        self.create_subscription(ImuStamped, "imu", self.i_cb, 100)
        self.create_subscription(TriggerStamped, "trigger", self.t_cb, 10)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def c_cb(self, msg):
        self.contact.append((self._now(), [
            msg.module_a.contact, msg.module_b.contact,
            msg.module_c.contact, msg.module_d.contact]))

    def m_cb(self, msg):
        self.motor.append((self._now(), [
            [m.theta, m.beta, m.gamma] for m in
            (msg.module_a, msg.module_b, msg.module_c, msg.module_d)]))

    def k_cb(self, msg):
        self.cmd.append((self._now(), [
            [m.theta, m.beta] for m in
            (msg.module_a, msg.module_b, msg.module_c, msg.module_d)]))

    def i_cb(self, msg):
        q = msg.orientation
        self.imu.append((self._now(), [q.x, q.y, q.z, q.w]))

    def t_cb(self, msg):
        if msg.enable and self.t_trigger is None:
            self.t_trigger = self._now()


def find_anchor(cmd, fiducial_t, settle, t_trigger):
    """-> (t_sim of template t=0, how it was found)."""
    for t, mods in cmd:
        if max(abs(m[1]) for m in mods) > BETA_FIDUCIAL_RAD:
            return t - fiducial_t, f"commanded beta fiducial at t={fiducial_t:.3f}s"
    if t_trigger is not None:
        return t_trigger + settle, "trigger + settle (beta fiducial never seen)"
    return None, "no anchor"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("template")
    ap.add_argument("--settle", type=float, default=1.0,
                    help="controller settle_ticks, in seconds")
    ap.add_argument("--timeout", type=float, default=400.0,
                    help="wall-clock give-up, seconds")
    ap.add_argument("--dump", default=None,
                    help="write raw samples to this .npz for offline analysis")
    args = ap.parse_args()

    segs = segment_template(args.template)
    duration = segs[-1]["t_end"]
    # The fiducial is the start of the first segment with non-zero beta.
    fiducial_t = next((s["t_start"] for s in segs
                       if max(abs(v) for v in s["beta_deg"]) > 0.5), None)

    rclpy.init()
    n = Rec()
    import time
    wall_end = time.time() + args.timeout
    anchor = None
    while rclpy.ok() and time.time() < wall_end:
        rclpy.spin_once(n, timeout_sec=0.0)
        if anchor is None and fiducial_t is not None and len(n.cmd) > 50:
            anchor, _ = find_anchor(n.cmd, fiducial_t, args.settle, n.t_trigger)
        if anchor is not None and n.contact and n.contact[-1][0] - anchor > duration:
            break

    if anchor is None:
        anchor, how = find_anchor(n.cmd, fiducial_t or 0.0, args.settle,
                                  n.t_trigger)
    else:
        _, how = find_anchor(n.cmd, fiducial_t or 0.0, args.settle, n.t_trigger)

    print(f"samples: contact={len(n.contact)} motor={len(n.motor)} "
          f"cmd={len(n.cmd)} imu={len(n.imu)}")
    if anchor is None:
        print("NO ANCHOR -- never saw the trigger or a non-zero commanded beta.")
        print("Is the gait running? Was this recorder started before the trigger?")
        rclpy.shutdown()
        return
    print(f"anchor: {how}")

    ct = np.array([t for t, _ in n.contact]) - anchor
    cv = np.array([v for _, v in n.contact], dtype=bool)
    mt = np.array([t for t, _ in n.motor]) - anchor
    mv = np.rad2deg(np.array([v for _, v in n.motor]))

    covered = ct.max() if len(ct) else -1
    print(f"template time covered: {ct.min():.2f} .. {covered:.2f} s "
          f"(one pass is {duration:.2f} s)")
    if covered < duration:
        print(f"  WARNING: recording ended {duration-covered:.2f} s short of a "
              f"full pass; late segments are missing or partial")

    print()
    print(f"{'segment':<12} {'t range':>14} {'flight':>8} {'design':>8} "
          f"{'all-down':>9} {'theta max':>10} {'|beta| max':>11} {'n':>6}")
    for s in segs:
        m = (ct >= s["t_start"]) & (ct <= s["t_end"])
        mm = (mt >= s["t_start"]) & (mt <= s["t_end"])
        n_c = int(m.sum())
        if n_c < 20:
            print(f"{s['name']:<12} {s['t_start']:6.2f}..{s['t_end']:<6.2f} "
                  f"{'--':>8} {100*(1-s['duty']):7.1f}% {'--':>9} "
                  f"{'--':>10} {'--':>11} {n_c:6d}")
            continue
        seg = cv[m]
        air = 100.0 * float((~seg.any(axis=1)).sum()) / n_c
        alld = 100.0 * float(seg.all(axis=1).sum()) / n_c
        th = f"{mv[mm][:, :, 0].max():10.2f}" if mm.sum() else f"{'--':>10}"
        be = f"{np.abs(mv[mm][:, :, 1]).max():11.2f}" if mm.sum() else f"{'--':>11}"
        print(f"{s['name']:<12} {s['t_start']:6.2f}..{s['t_end']:<6.2f} "
              f"{air:7.1f}% {100*(1-s['duty']):7.1f}% {alld:8.1f}% "
              f"{th} {be} {n_c:6d}")

    print()
    print("flight is the fraction of samples with no leg in contact; design is")
    print("1 - duty from the template. A working ramp holds flight near design")
    print("on every rung, not just the hop.")

    # Per stride. A rung that starts poor and climbs is an unconverged
    # transient (the rung is too short); a rung that is flat and poor is a
    # broken rung. The segment average cannot tell those apart.
    print()
    print("per stride (flight %, and theta max in deg)")
    strides = stride_boundaries(args.template)
    row = []
    for k, (t0, t1, name) in enumerate(strides):
        m = (ct >= t0) & (ct <= t1)
        mm = (mt >= t0) & (mt <= t1)
        n_c = int(m.sum())
        if n_c < 10:
            row.append(f"{k:>3} {name:<9}    --      --")
            continue
        seg = cv[m]
        air = 100.0 * float((~seg.any(axis=1)).sum()) / n_c
        th = mv[mm][:, :, 0].max() if mm.sum() else float("nan")
        row.append(f"{k:>3} {name:<9} {air:5.1f}%  {th:6.2f}")
    for line in row:
        print("  " + line)

    if args.dump:
        np.savez_compressed(args.dump, contact_t=ct, contact=cv,
                            motor_t=mt, motor_deg=mv, anchor=anchor,
                            template=args.template)
        print(f"\nraw samples written to {args.dump}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
