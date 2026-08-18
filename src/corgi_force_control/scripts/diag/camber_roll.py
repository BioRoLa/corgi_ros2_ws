#!/usr/bin/env python3
"""The camber-command confirming run: does uniform camber steer a ROLLING leg?

Stage 0 handed forward one untested assumption, and every cambered stage rests
on it: uniform camber must be commanded as

    gamma = lambda * {+1, -1, -1, +1}   (A, B, C, D -- the LEFT/RIGHT selector)

and NOT as the front/rear couple {+1, +1, -1, -1} that gamma_correction() uses.
That pattern was derived from the sign conventions and has never been run. It
has to be settled before Stage 1, because Stage 1's own sweep COMMANDS camber:
validating the contact model against a robot that is not in the commanded pose
would produce a confident wrong answer.

The test. Fold to WHEELED MODE (theta = 17 deg), which is the one pose where
every rim centre coincides with the hip and the leg-wheel is a true wheel of
radius 0.145 m. Hold a steady camber. Roll. A cambered rolling wheel turns
because its contact circle is centred where the spin axis meets the ground,

    R_turn = rho * cos^2(lambda) / sin(lambda)

-- 0.809 m at 10 deg, 0.372 m at 20 deg for this tread (Stage 0). Run the
front/rear pattern as the negative control: it should produce no net path
curvature, which is a RETRODICTION -- it is precisely why gamma_correction()
never curved the path -- so reproducing it confirms the convention twice over.

Two cautions from Stage 0, both binding, both handled by the phase sequence
below. Do NOT test at small camber: the tread is flat for |w| <= 5 mm, so
contact jumps 10 mm across lambda = 0. And do NOT reverse camber sign within a
run -- each lambda is its own run, held steady.

Companion pieces: record_camber.py writes the dump, check_camber_turn.py fits
the circle, camber_cycle.sh runs one pass from a fresh simulator.

Usage:
    python3 camber_roll.py --lam-deg 20 --pattern lr
    python3 camber_roll.py --lam-deg 20 --pattern lr --dry-run
"""
import argparse
import math
import sys

# Module order throughout this workspace: A = front-left, B = front-right,
# C = rear-right, D = rear-left.
LEGS = ("A", "B", "C", "D")

# Per-leg camber sign.
#
# `lr` is the pattern under test. gamma > 0 abducts OUTWARD on all four legs
# (motor_dir.ABAD = {+1,-1,+1,-1} in motor_config.yaml already compensates the
# front/rear ABAD axis flip), so {+1,-1,-1,+1} drives all four contacts to the
# same side while the hips stay put -- which is a uniform lean. It is the same
# left/right partition as gslip_pronk's steer_sign and roll_sign.
#
# `fr` is the NEGATIVE CONTROL, the front/rear couple gamma_correction() uses.
# Front and rear lean opposite ways, so whatever path curvature each pair
# generates cancels by construction.
#
# `all` was added 2026-08-13, after the first sweep. `lr` produced 9.2 deg of
# BODY ROLL and no path curvature at all, which is what a uniform lean should
# look like from the body's point of view -- but the run cannot distinguish
# "the pattern is right and camber does not steer" from "the pattern is wrong".
# `all` is the other reading of the convention: if gamma > 0 does NOT abduct
# outward on all four legs but tilts them all the same physical way, then this
# is the uniform lean and `lr` was a splay. Only one of the two can curve.
PATTERNS = {
    "lr": (+1.0, -1.0, -1.0, +1.0),
    "fr": (+1.0, +1.0, -1.0, -1.0),
    "all": (+1.0, +1.0, +1.0, +1.0),
    "none": (0.0, 0.0, 0.0, 0.0),
}

# Wheeled mode. 17 deg is the low end of the theta workspace and the pose where
# the leg-wheel is genuinely a wheel on its own axle -- R(alpha) is constant at
# 0.145 m across the whole rim, rather than the piecewise 0.145-0.388 m it
# takes anywhere else. It is also the one place the old 0.145 m rolling-radius
# constant is correct; see implementation log section 21.
THETA_WHEEL_DEG = 17.0

# Standing pose used before the fold, matching the G-SLIP pronk's nominal
# stance. The robot starts folded on SUPPORT_BOX, so it must stand up to clear
# the box before the trigger removes it -- the same sequence every existing run
# uses (gslip_pronk execute_standup_phase).
THETA_STAND_DEG = 100.0

# Gains, matching corgi_csv_control so this is the same position-control regime
# the transform CSVs replay under. No force control here: the question is
# geometric, and an impedance law would put the leg somewhere other than where
# it was commanded, which is the one thing this run cannot tolerate.
KP, KD = 90.0, 1.75
# AB/AD proportional gain. Separate from KP because it is the only
# lateral-stiffness knob this rig has: k_lateral belongs to gslip_pronk
# -> force_control -> K_joint(2,2) -> kp_h, and this rig launches neither
# node. Overridden by --kp-h; None means "use KP", i.e. unchanged.
KP_H = None


# Which legs a --lift option takes off the ground. A=FL B=FR C=RR D=RL.
LIFT_PAIRS = {
    "none": (),
    "rear": (2, 3),    # C, D -- keeps the FRONT lateral pair down
    "front": (0, 1),   # A, B -- keeps the REAR lateral pair down
    "diag": (1, 3),    # B, D -- keeps the A/C diagonal down
}

# Stage 0 contact geometry, for the Ackermann solve. TRACK is the CONTACT
# track: all four wheel planes sit outboard of the hips by 0.091675 m.
TRACK_M = 0.4234


def ride_height(lam_deg):
    """Hub height of a wheel cambered by lambda, metres (Stage 0 tread form)."""
    a = math.radians(abs(lam_deg))
    return 0.145 * math.cos(a) - 0.005 * math.sin(a)


def ackermann_outer(lam_inner_deg):
    """-> the outer wheel's camber that puts both apexes on one point.

    A cambered wheel rolls without drilling only about the point where its spin
    axis meets the ground. For a lateral pair those apexes share an x, so they
    can be made to coincide -- but only at DIFFERENT camber angles:

        cot(lam_outer) - cot(lam_inner) = track / h

    which is Ackermann geometry written in camber. The inner wheel leans more
    because it runs the tighter circle. See LegWheel
    examples/gslip/camber_apex_geometry.py, which also shows why no camber
    distribution can do this with all four wheels down: the apex inherits its
    hub's x, so front and rear are stuck 0.510 m apart whatever you command.
    """
    h = ride_height(lam_inner_deg)
    cot_out = 1.0 / math.tan(math.radians(lam_inner_deg)) + TRACK_M / h
    return math.degrees(math.atan(1.0 / cot_out))


def ackermann_radius(lam_inner_deg):
    """-> the body centreline's turn radius about the shared apex, metres."""
    h = ride_height(lam_inner_deg)
    return abs(-0.5 * TRACK_M - h / math.tan(math.radians(lam_inner_deg)))


def lift_camber_for(clearance_m, lam_down_deg):
    """-> the camber that raises a wheel `clearance_m` above the ground.

    A cambered wheel's hub-to-contact drop is ride_height(lambda), so a leg
    cambered harder than its neighbours hangs clear by the difference. Solved
    by bisection because ride_height mixes cos and sin.

    Sizing matters more than it looks. 45 deg lifts 42 mm, and 42 mm is enough
    for the body to rotate a long way before the lifted pair can catch it --
    which is how the first attempt ended up on its back. A small clearance
    keeps the pair OFF the ground for normal motion but lets it arrest a pitch
    almost immediately, so the rear rides light instead of absent.
    """
    target = ride_height(lam_down_deg) - clearance_m
    lo, hi = abs(lam_down_deg), 89.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if ride_height(mid) > target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def camber_target(lam_deg, pattern, lift="none", lift_camber=45.0,
                  acker=False):
    """-> the held camber for each of A, B, C, D, in radians.

    Lifted legs get `lift_camber` with the SPLAY sign (outward on both), which
    is roll-neutral -- an lr-signed lift would add a roll couple through legs
    that are supposed to be doing nothing. Cambering lifts a wheel because its
    hub-to-contact drop is 0.145*cos(lambda): at 45 deg that is 42 mm shorter
    than an upright wheel, so the wheel clears the ground the others stand on.
    """
    lifted = LIFT_PAIRS[lift]
    if acker:
        # The down pair only. lam_deg is the INNER wheel; the outer follows.
        lam_out = ackermann_outer(abs(lam_deg))
        mags = []
        for i in range(4):
            sign = PATTERNS["lr"][i]
            # Inner is the side the turn centre is on. With the lr convention
            # and lambda > 0 the robot leans right, so B and C (right) are
            # inner and take the larger angle.
            inner = sign < 0
            mags.append(sign * math.radians(abs(lam_deg) if inner else lam_out))
        gammas = mags
    else:
        gammas = [s * math.radians(lam_deg) for s in PATTERNS[pattern]]

    for i in lifted:
        gammas[i] = math.radians(lift_camber)  # splay sign: outward, both legs
    return gammas


def smoothstep(s):
    """Cosine ease over s in [0, 1]. Ramps start and end at zero velocity.

    A linear ramp steps the commanded velocity at both ends, and at 1 kHz into
    a stiff position loop that shows up as a torque spike. Only the ROLL phase
    is deliberately linear -- constant beta_dot is the whole point there.
    """
    s = min(1.0, max(0.0, s))
    return 0.5 * (1.0 - math.cos(math.pi * s))


class Schedule:
    """The commanded pose as a function of time since the trigger.

    Pure function of time, with no ROS in it, so --dry-run exercises exactly
    the trajectory the node publishes rather than a re-derivation of it.

    Phase order is deliberate. The robot LEANS LAST, after it is already
    standing on its wheels, so the camber that is under test is the only thing
    that changes between the settled wheel-mode pose and the roll. Folding
    while already cambered would drag each contact across the tread through the
    fold, which is the same scrub the flat band is known to cause.
    """

    def __init__(self, lam_deg, pattern, beta_rate, settle=1.0, fold_time=3.0,
                 lean_time=2.0, lean_settle=1.5, roll_time=20.0,
                 lift="none", lift_camber=45.0, acker=False, beta_ramp=2.0):
        self.lam = math.radians(lam_deg)
        self.signs = PATTERNS[pattern]
        self.pattern = pattern
        self.beta_rate = beta_rate
        self.beta_ramp = beta_ramp
        self.lift = lift
        self.acker = acker
        self.gamma_target = camber_target(lam_deg, pattern, lift, lift_camber,
                                          acker)

        self.t_settle = settle
        self.t_fold = self.t_settle + fold_time
        self.t_lean = self.t_fold + lean_time
        self.t_lean_settle = self.t_lean + lean_settle
        self.t_roll = self.t_lean_settle + roll_time
        self.roll_time = roll_time

    @property
    def duration(self):
        return self.t_roll

    def phase(self, t):
        if t < self.t_settle:
            return "settle"
        if t < self.t_fold:
            return "fold"
        if t < self.t_lean:
            return "lean"
        if t < self.t_lean_settle:
            return "lean_settle"
        if t < self.t_roll:
            return "roll"
        return "stop"

    def __call__(self, t):
        """-> (theta, beta, [gamma_A..gamma_D]), all radians."""
        stand = math.radians(THETA_STAND_DEG)
        wheel = math.radians(THETA_WHEEL_DEG)

        # theta: stand -> wheel mode, over the fold window.
        if t < self.t_settle:
            theta = stand
        elif t < self.t_fold:
            s = smoothstep((t - self.t_settle) / (self.t_fold - self.t_settle))
            theta = stand + s * (wheel - stand)
        else:
            theta = wheel

        # gamma: 0 -> the per-leg target, over the lean window, then held.
        # Scaled as a vector rather than a single lambda times a sign pattern,
        # because the Ackermann and lifted-pair poses give each leg its own
        # magnitude.
        if t < self.t_fold:
            s = 0.0
        elif t < self.t_lean:
            s = smoothstep((t - self.t_fold) / (self.t_lean - self.t_fold))
        else:
            s = 1.0
        gammas = [s * g for g in self.gamma_target]

        # beta: held at zero until the roll, then eased up to a constant rate
        # and held there.
        #
        # The ease-in is NOT cosmetic. Stepping beta_dot from 0 to 2.07 rad/s
        # is an impulsive reaction torque on the body, and with a lifted pair
        # there is nothing behind the loaded axle to absorb it: the first
        # lifted-pair attempt (2026-08-14) wheelied the robot straight over.
        # Four wheels down hid this completely.
        #
        # beta_dot(tau) = rate * smoothstep(tau/T), integrated in closed form so
        # beta stays exactly monotonic:
        #     beta(tau) = rate/2 * (tau - (T/pi) sin(pi tau/T))
        T = self.beta_ramp
        if t < self.t_lean_settle:
            beta = 0.0
        else:
            tau = min(t, self.t_roll) - self.t_lean_settle
            if T > 0.0 and tau < T:
                beta = 0.5 * self.beta_rate * (
                    tau - (T / math.pi) * math.sin(math.pi * tau / T))
            else:
                ramp_beta = 0.5 * self.beta_rate * T
                beta = ramp_beta + self.beta_rate * (tau - T)

        return theta, beta, gammas


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lam-deg", type=float, required=True,
                    help="camber angle in degrees. Steady 10-20 deg holds are "
                         "well clear of the 10 mm contact discontinuity the "
                         "flat tread band puts either side of zero")
    ap.add_argument("--pattern", choices=sorted(PATTERNS), default="lr",
                    help="lr = the left/right selector under test; fr = the "
                         "front/rear couple, as the negative control; none = "
                         "the zero-camber baseline")
    ap.add_argument("--beta-rate", type=float, default=2.07,
                    help="rolling rate, rad/s. 2.07 is about 0.30 m/s at the "
                         "0.145 m wheel-mode rolling radius, which covers a "
                         "full lap in the 20 s roll window at both 10 and "
                         "20 deg (default: %(default)s)")
    ap.add_argument("--roll-time", type=float, default=20.0,
                    help="seconds of rolling. The circle fit needs at least a "
                         "quarter turn and is badly conditioned below it "
                         "(default: %(default)s)")
    ap.add_argument("--fold-time", type=float, default=3.0,
                    help="seconds to fold from the standing pose to wheeled "
                         "mode (default: %(default)s)")
    ap.add_argument("--lean-time", type=float, default=2.0,
                    help="seconds to ramp camber in (default: %(default)s)")
    ap.add_argument("--settle", type=float, default=1.0,
                    help="seconds standing after the support box is removed, "
                         "before the fold (default: %(default)s)")
    ap.add_argument("--standup-time", type=float, default=2.5,
                    help="seconds to ramp from the measured pose to the "
                         "standing pose, BEFORE the trigger (default: "
                         "%(default)s)")
    ap.add_argument("--lift", choices=sorted(LIFT_PAIRS), default="none",
                    help="take a pair of legs off the ground by cambering them "
                         "hard, leaving the others to carry the robot. `rear` "
                         "keeps the FRONT lateral pair down, which is the only "
                         "configuration whose camber apexes can be made to "
                         "coincide (default: %(default)s)")
    ap.add_argument("--lift-clearance", type=float, default=0.015,
                    help="how far above the ground to hold the lifted pair, "
                         "metres. The camber that achieves it is solved from "
                         "the tread geometry. Keep this SMALL: the pair has to "
                         "be clear for normal rolling but close enough to "
                         "arrest a pitch before it becomes a tumble "
                         "(default: %(default)s)")
    ap.add_argument("--lift-camber", type=float, default=None,
                    help="force the lifted pair's camber in degrees instead of "
                         "solving it from --lift-clearance")
    ap.add_argument("--beta-ramp", type=float, default=2.0,
                    help="seconds to ease beta_dot up to --beta-rate. Stepping "
                         "it is an impulsive reaction torque on the body, "
                         "which wheelies a lifted-pair run straight over "
                         "(default: %(default)s)")
    ap.add_argument("--acker", action="store_true",
                    help="give the down pair ACKERMANN camber instead of "
                         "uniform: --lam-deg is then the inner wheel and the "
                         "outer one is solved so both apexes coincide. Uniform "
                         "camber puts them exactly one track width apart at "
                         "every angle, which is why it cannot roll a curve.")
    ap.add_argument("--kp-h", type=float, default=None,
                    help="AB/AD proportional gain, default = KP (90). "
                         "The lateral-stiffness knob for this rig; "
                         "k_lateral does not reach it.")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the commanded trajectory and its invariants "
                         "with no ROS traffic and no simulator")
    return ap.parse_args(argv)


def resolve_lift_camber(args):
    """-> the lifted pair's camber, forced or solved from the clearance."""
    if args.lift == "none":
        return 0.0
    if args.lift_camber is not None:
        return args.lift_camber
    return lift_camber_for(args.lift_clearance, abs(args.lam_deg))


def dry_run(args):
    """Print the trajectory and check the invariants that matter.

    Everything here is a property the run is void without, so it is asserted
    rather than eyeballed: the camber pattern, the wheel-mode theta, and a beta
    that only ever increases (a non-monotonic beta is not rolling, it is
    rocking).
    """
    sched = Schedule(args.lam_deg, args.pattern, args.beta_rate,
                     settle=args.settle, fold_time=args.fold_time,
                     lean_time=args.lean_time, roll_time=args.roll_time,
                     lift=args.lift, lift_camber=resolve_lift_camber(args),
                     acker=args.acker, beta_ramp=args.beta_ramp)

    print(f"lambda {args.lam_deg:+.1f} deg   pattern "
          f"{'ACKERMANN' if args.acker else args.pattern}   "
          f"lift {args.lift}   beta_rate {args.beta_rate} rad/s")
    print("held camber per leg: " + "  ".join(
        f"{leg} {math.degrees(g):+6.2f}"
        + ("*" if i in LIFT_PAIRS[args.lift] else " ")
        for i, (leg, g) in enumerate(zip(LEGS, sched.gamma_target)))
        + "   (* = lifted)")
    if args.acker:
        print(f"  Ackermann: inner {abs(args.lam_deg):.2f} deg, outer "
              f"{ackermann_outer(abs(args.lam_deg)):.2f} deg "
              f"-> shared apex, R_turn {ackermann_radius(abs(args.lam_deg)):.3f} m")
    if args.lift != "none":
        lc = resolve_lift_camber(args)
        clear = ride_height(abs(args.lam_deg)) - ride_height(lc)
        print(f"  lift: {args.lift} pair at {lc:.2f} deg -> "
              f"{1000*clear:.1f} mm of clearance")
    print(f"  beta_dot eased in over {args.beta_ramp:.1f} s")
    print(f"phases: settle {sched.t_settle:.1f} | fold {sched.t_fold:.1f} | "
          f"lean {sched.t_lean:.1f} | settle {sched.t_lean_settle:.1f} | "
          f"roll ends {sched.t_roll:.1f}  (seconds after trigger)")
    print()
    print(f"{'t':>6} {'phase':>12} {'theta':>8} {'beta':>9} "
          + " ".join(f"{'g' + leg:>7}" for leg in LEGS))

    n = 200
    prev_beta = -1e9
    betas, thetas = [], []
    for i in range(n + 1):
        t = sched.duration * i / n
        theta, beta, gammas = sched(t)
        betas.append(beta)
        thetas.append(theta)
        assert beta >= prev_beta - 1e-12, f"beta went backwards at t={t:.3f}"
        prev_beta = beta
        if i % 10 == 0:
            print(f"{t:6.2f} {sched.phase(t):>12} "
                  f"{math.degrees(theta):8.2f} {math.degrees(beta):9.2f} "
                  + " ".join(f"{math.degrees(g):7.2f}" for g in gammas))

    theta_end, _, gam_end = sched(sched.duration)

    assert abs(math.degrees(theta_end) - THETA_WHEEL_DEG) < 1e-9, \
        "final theta is not wheeled mode"
    for leg, got, want in zip(LEGS, gam_end, sched.gamma_target):
        assert abs(got - want) < 1e-12, f"leg {leg} camber {got} != {want}"

    swept = betas[-1]
    print()
    print(f"  theta ends at {math.degrees(theta_end):.2f} deg (wheeled mode)  OK")
    print(f"  camber pattern {[round(math.degrees(g), 3) for g in gam_end]} deg  OK")
    print(f"  beta monotonic, sweeps {math.degrees(swept):.0f} deg "
          f"= {swept / (2 * math.pi):.2f} revolutions  OK")
    print(f"  rolling distance if it does not slip: "
          f"{0.145 * swept:.2f} m at 0.145 m rolling radius")
    if args.pattern != "none" and abs(args.lam_deg) > 1e-9:
        r = predicted_turn_radius(abs(args.lam_deg))
        print(f"  predicted single-wheel turn radius at "
              f"{abs(args.lam_deg):.0f} deg: {r:.3f} m "
              f"-> {0.145 * swept / (2 * math.pi * r):.2f} laps")
    return 0


# Stage 0's tread model, restated here so --dry-run can print the prediction
# without the legwheel package (which is not installed in the ROS workspace --
# the same reason check_turn.py embeds its own _OOR_POLY). The authority is
# LegWheel examples/gslip/crown_effect.py; check_camber_turn.py carries the
# same constants and the two are checked against each other.
R_OUT, R_CORNER, W_FLAT = 0.145, 0.015, 0.005


def predicted_turn_radius(lam_deg):
    a = math.radians(abs(lam_deg))
    if a < 1e-9:
        return float("inf")
    rho = (R_OUT - R_CORNER) + R_CORNER * math.cos(a)
    return rho * math.cos(a) ** 2 / math.sin(a)


def main(argv=None):
    args = parse_args(argv)
    global KP_H
    KP_H = args.kp_h
    print(f"AB/AD kp_h = {KP if KP_H is None else KP_H:.1f} "
          f"(leg kp = {KP:.1f})", flush=True)
    if args.dry_run:
        return dry_run(args)

    import rclpy
    from rclpy.node import Node
    from corgi_msgs.msg import MotorCmdStamped, MotorStateStamped, TriggerStamped

    sched = Schedule(args.lam_deg, args.pattern, args.beta_rate,
                     settle=args.settle, fold_time=args.fold_time,
                     lean_time=args.lean_time, roll_time=args.roll_time,
                     lift=args.lift, lift_camber=resolve_lift_camber(args),
                     acker=args.acker, beta_ramp=args.beta_ramp)

    class CamberRoll(Node):
        def __init__(self):
            super().__init__(
                "camber_roll",
                parameter_overrides=[rclpy.parameter.Parameter(
                    "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
            self.pub = self.create_publisher(MotorCmdStamped, "motor/command",
                                             1000)
            self.create_subscription(MotorStateStamped, "motor/state",
                                     self.state_cb, 100)
            self.create_subscription(TriggerStamped, "trigger", self.trig_cb, 10)
            self.state = None
            self.trigger = False
            self.t_trigger = None
            self.seq = 0
            self.phase = None
            self.done = False
            self.standup_done = False
            # Schedule position, kept explicitly so a sim-clock jump can be
            # absorbed without replaying phases. See tick().
            self.t_elapsed = 0.0
            # The measured pose the standup ramp starts from. Latched once, on
            # the first real motor/state -- ramping from a pose that is being
            # re-read every tick chases the robot instead of leading it.
            self.theta0 = None
            self.beta0 = None

        def state_cb(self, msg):
            self.state = msg

        def trig_cb(self, msg):
            if msg.enable and not self.trigger:
                self.trigger = True
                self.t_trigger = self.now()
                self.get_logger().info(
                    f"Trigger at sim t={self.t_trigger:.2f}; support box is "
                    f"being removed. lambda={args.lam_deg:+.1f} deg "
                    f"pattern={args.pattern}")

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def publish(self, theta, beta, gammas):
            msg = MotorCmdStamped()
            msg.header.seq = self.seq
            msg.header.stamp = self.get_clock().now().to_msg()
            self.seq += 1
            for leg, gamma in zip(("a", "b", "c", "d"), gammas):
                cmd = getattr(msg, f"module_{leg}")
                cmd.theta, cmd.beta, cmd.gamma = theta, beta, gamma
                cmd.kp_r = cmd.kp_l = KP
                cmd.kp_h = KP if KP_H is None else KP_H
                cmd.ki_r = cmd.ki_l = cmd.ki_h = 0.0
                cmd.kd_r = cmd.kd_l = cmd.kd_h = KD
                cmd.torque_r = cmd.torque_l = cmd.torque_h = 0.0
            self.pub.publish(msg)

        def log_pose(self, tag):
            """Measured theta/beta/gamma per leg, at every phase boundary.

            The run is void if the robot is not in the pose it was given, and
            the ABAD has a known 1.5-2 deg left/right split (B, C above A, D),
            so the achieved camber is recorded rather than assumed.
            """
            if self.state is None:
                return
            mods = [getattr(self.state, f"module_{m}") for m in "abcd"]
            self.get_logger().info(
                f"[{tag}] " + "  ".join(
                    f"{leg}: th={math.degrees(m.theta):6.2f} "
                    f"be={math.degrees(m.beta):+7.2f} "
                    f"ga={math.degrees(m.gamma):+6.2f}"
                    for leg, m in zip(LEGS, mods)))

        def tick(self):
            if self.done:
                return

            # Phase -1: wait for a real motor/state before ramping anywhere.
            # One spin is not enough -- the subscription has not discovered the
            # publisher yet at construction, so the state is still
            # zero-initialised and a ramp from it would command a collapse.
            if self.state is None:
                return
            if self.theta0 is None:
                self.theta0 = self.state.module_a.theta
                self.beta0 = self.state.module_a.beta
                self.t_start = self.now()
                self.get_logger().info(
                    f"Got motor/state: theta={math.degrees(self.theta0):.2f} "
                    f"beta={math.degrees(self.beta0):+.2f} deg. Standing up "
                    f"to {THETA_STAND_DEG:.0f} deg over "
                    f"{args.standup_time:.1f} s.")

            if not self.trigger:
                # Standup, then hold. Same shape as gslip_pronk's standup: ramp
                # from where the robot ACTUALLY is, so a robot already standing
                # is not commanded to collapse.
                s = smoothstep((self.now() - self.t_start) / args.standup_time)
                stand = math.radians(THETA_STAND_DEG)
                self.publish(self.theta0 + s * (stand - self.theta0),
                             self.beta0 * (1.0 - s), [0.0] * 4)
                # Announce readiness only when the ramp has actually finished.
                # camber_cycle.sh greps for this line before it starts the
                # recorder and fires the trigger, and triggering mid-standup
                # would remove the support box from under a robot that is not
                # yet standing on its legs.
                if s >= 1.0 and not self.standup_done:
                    self.standup_done = True
                    self.log_pose("standing")
                    self.get_logger().info(
                        "Standup complete; holding for the trigger.")
                return

            t = self.now() - self.t_trigger
            # The sim clock can jump backwards under us -- Webots settling at
            # startup, or the trigger latching against a stale epoch. Rebase so
            # the schedule CONTINUES from where it had got to, rather than
            # restarting it: an earlier version re-latched to now(), which sent
            # the robot back through standup, fold and lean and gave the run
            # two roll phases.
            if t < self.t_elapsed - 0.5:
                self.get_logger().warn(
                    f"sim clock jumped backwards ({t:.2f} s vs "
                    f"{self.t_elapsed:.2f}); rebasing to keep the schedule "
                    f"continuous")
                self.t_trigger = self.now() - self.t_elapsed
                t = self.t_elapsed
            self.t_elapsed = t
            phase = sched.phase(t)
            if phase != self.phase:
                self.log_pose(f"enter {phase}")
                self.get_logger().info(f"--- {phase} at t={t:.2f} s")
                self.phase = phase
            theta, beta, gammas = sched(t)
            self.publish(theta, beta, gammas)

            if phase == "stop":
                self.log_pose("final")
                self.get_logger().info(
                    f"Roll complete: beta swept "
                    f"{math.degrees(beta):.0f} deg "
                    f"({0.145 * beta:.2f} m of rolling if it did not slip)")
                self.done = True

    rclpy.init()
    node = CamberRoll()
    # 1 kHz on the SIM clock. Webots runs well below real time and the factor
    # varies, so anything keyed to wall time lands at a different point in the
    # schedule every run.
    node.create_timer(0.001, node.tick)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
