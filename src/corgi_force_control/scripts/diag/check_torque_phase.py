"""Torque attributed to gait phase.

Peak torque alone cannot say WHY the legs saturate. Binning by position within
stance separates the candidates:

  saturating early in stance    -> touchdown transient (impedance applies K to
                                   the full instantaneous error, so a position
                                   error at contact spikes the force in a way
                                   the smooth fixed-point trajectory never does)
  saturating through mid-stance -> the spring is genuinely too stiff for 35 N-m
                                   at the compression actually reached
  saturating in flight          -> k_flight / swing tracking, not the spring

Result when this was first run on the pronk: flight 4.4-5.6 N-m and 0-1%
saturated (innocent); first stance bin 3-14% (not a touchdown transient);
0.20-0.60 stance progress 45-57% saturated. Mid-stance, at peak compression.

Phase comes from CONTACT, not from the commanded beta. Beta is a TRIANGLE --
it sweeps back through stance and forward through flight -- so there is no
discontinuity to key a stride boundary on. An earlier version tried to detect a
sawtooth wrap and found zero boundaries.

Run this EARLY after the trigger: the gait stops roughly a minute in, and a late
run reads a static robot.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from corgi_msgs.msg import MotorStateStamped, SimLegContactStamped

NAMES = "ABCD"
LIMIT = 35.0
SAT = 0.98 * LIMIT
NSTANCE_BINS = 5


class Rec(Node):
    def __init__(self):
        super().__init__("torque_phase")
        self.st, self.state = [], []
        self.kt, self.contact = [], []
        self.create_subscription(MotorStateStamped, "motor/state", self.st_cb, 100)
        self.create_subscription(SimLegContactStamped, "sim/leg_contact", self.k_cb, 100)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def st_cb(self, msg):
        self.st.append(self._now())
        self.state.append([[m.torque_r, m.torque_l, m.theta] for m in
                           (msg.module_a, msg.module_b, msg.module_c, msg.module_d)])

    def k_cb(self, msg):
        self.kt.append(self._now())
        self.contact.append([float(m.contact) for m in
                             (msg.module_a, msg.module_b, msg.module_c, msg.module_d)])


def stance_progress(down):
    """0 at touchdown, ->1 at liftoff, NaN in flight, per contiguous stance run."""
    prog = np.full(len(down), np.nan)
    edges = np.diff(down.astype(int))
    starts = list(np.where(edges == 1)[0] + 1)
    ends = list(np.where(edges == -1)[0] + 1)
    if down[0]:
        starts = [0] + starts
    if down[-1]:
        ends = ends + [len(down)]
    for a, b in zip(starts, ends):
        if b > a:
            prog[a:b] = np.linspace(0.0, 1.0, b - a, endpoint=False)
    return prog


def main():
    rclpy.init()
    n = Rec()
    t_end = n.get_clock().now().nanoseconds + int(14e9)
    # Tight spin -- see check_cmd_vs_state.py for why a timeout aliases this.
    while rclpy.ok() and n.get_clock().now().nanoseconds < t_end:
        rclpy.spin_once(n, timeout_sec=0.0)

    st, state = np.array(n.st), np.array(n.state)
    kt, contact = np.array(n.kt), np.array(n.contact)
    print(f"samples: state={len(state)} contact={len(contact)}")
    if len(state) < 50 or len(contact) < 50:
        print("NOT ENOUGH DATA -- is the gait running?")
        return

    # Contact interpolated onto the state clock (nearest, since it is boolean).
    con = np.stack([np.interp(st, kt, contact[:, i]) for i in range(4)], axis=1)
    tau = np.abs(state[:, :, :2]).max(axis=2)   # peak |torque| per leg

    print()
    print("split by contact state (each leg uses its own contact)")
    print(f"{'leg':>4} {'stance tau':>11} {'stance sat%':>12} "
          f"{'flight tau':>11} {'flight sat%':>12}")
    for i in range(4):
        down = con[:, i] > 0.5
        up = ~down
        s_tau = tau[down, i].mean() if down.any() else float("nan")
        s_sat = 100.0 * (tau[down, i] > SAT).mean() if down.any() else float("nan")
        f_tau = tau[up, i].mean() if up.any() else float("nan")
        f_sat = 100.0 * (tau[up, i] > SAT).mean() if up.any() else float("nan")
        print(f"{NAMES[i]:>4} {s_tau:11.1f} {s_sat:11.0f}% "
              f"{f_tau:11.1f} {f_sat:11.0f}%")

    print()
    print("within stance only, normalised stance progress (0 = touchdown)")
    print(f"{'progress':>11} " + " ".join(f"{'tau'+c:>7}" for c in NAMES)
          + " | " + " ".join(f"{'sat'+c:>6}" for c in NAMES))
    progs = [stance_progress(con[:, i] > 0.5) for i in range(4)]
    peak_bin = [0.0] * 4
    for b in range(NSTANCE_BINS):
        lo, hi = b / NSTANCE_BINS, (b + 1) / NSTANCE_BINS
        taus, sats = [], []
        for i in range(4):
            sel = (~np.isnan(progs[i])) & (progs[i] >= lo) & (progs[i] < hi)
            taus.append(tau[sel, i].mean() if sel.any() else float("nan"))
            s = 100.0 * (tau[sel, i] > SAT).mean() if sel.any() else 0.0
            sats.append(s)
            if s > peak_bin[i]:
                peak_bin[i] = s
        print(f"{lo:5.2f}-{hi:4.2f} " + " ".join(f"{t:7.1f}" for t in taus)
              + " | " + " ".join(f"{s:6.0f}" for s in sats))

    # Attribution, stated rather than left to the reader.
    print()
    early = []
    mid = []
    for i in range(4):
        p = progs[i]
        e = (~np.isnan(p)) & (p < 0.2)
        m = (~np.isnan(p)) & (p >= 0.2) & (p < 0.6)
        early.append(100.0 * (tau[e, i] > SAT).mean() if e.any() else 0.0)
        mid.append(100.0 * (tau[m, i] > SAT).mean() if m.any() else 0.0)
    fl = []
    for i in range(4):
        up = con[:, i] <= 0.5
        fl.append(100.0 * (tau[up, i] > SAT).mean() if up.any() else 0.0)
    print(f"saturation: flight {max(fl):.0f}%  early stance {max(early):.0f}%  "
          f"mid stance {max(mid):.0f}%  (worst leg in each)")
    if max(fl) > 10:
        print("-> FLIGHT gains are saturating; look at k_flight, not the spring.")
    elif max(early) > max(mid):
        print("-> TOUCHDOWN transient dominates; the reference pose at contact is")
        print("   too far from where the leg actually is.")
    elif max(mid) > 10:
        print("-> MID-STANCE dominates: the spring is too stiff for 35 N-m at the")
        print("   compression reached. Check the theta ratio in check_cmd_vs_state:")
        print("   if it is >>1 the leg is running past its designed range, and")
        print("   softening k_radial will make it travel further, not less.")
    else:
        print("-> no significant saturation this run.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
