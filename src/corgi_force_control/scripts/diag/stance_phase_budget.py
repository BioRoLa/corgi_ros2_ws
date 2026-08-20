"""WHERE inside stance is the torque spent, and WHERE is the distance made?

Alex, 2026-08-20: "is there a way to get the benefits of the phase shift
while still having forward motion? is this an issue with the sequencing
within the gait?"

The shift halves torque and costs 40-85% of the speed because, per S51
and S55, the propulsion IS the gain-schedule fault: real stance spent
under the stiff FLIGHT gain paddles the robot forward ("no rolling fixed
point exists; the ~30% stance slip IS the propulsion mechanism"). A
uniform +65 ms rotation of the labels removes the paddle everywhere.

That is only an unavoidable trade if torque and thrust are made at the
SAME point in stance. If they are made at different points, a uniform
shift is too blunt an instrument and a phase-DEPENDENT gain schedule
could keep the thrust while dropping the spike.

So bin both, per debounced stance episode, against stance progress:

  cost    mean |tau_demand| over leg motors in the bin
  thrust  the forward roll made in the bin, r * delta_beta with
          r = 0.145 m (the foot arc radius the differential-drive model
          in S19/S22 uses). Stance is rolling contact, so a sweep of
          d_beta while down advances that side by ~r*d_beta. Actual
          advance is ~70% of this (the 30% slip) -- the SHAPE across
          bins is what matters here, not the absolute value.

Read-only. Offline sibling of check_torque_phase.py, which is a live ROS
node and cannot be pointed at banked captures.

Usage:
    stance_phase_budget.py --dir ~/corgi_runs/.../cell [--dir ...] [--label X]
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_menger import debounce, DEBOUNCE  # noqa: E402

LEGS = "ABCD"
R_FOOT = 0.145          # foot arc radius, m (S21: constant only at theta=17,
                        # but it is the model's rolling radius and a shape
                        # comparison across bins does not depend on it)
NBINS = 5
TAIL_S = 20.0
MIN_STANCE_SAMPLES = 4


class Unfit(Exception):
    pass


def load(path):
    per = {}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            if r["motor"] not in ("L_Motor", "R_Motor"):
                continue
            try:
                t = float(r["t"])
                d = per.setdefault((t, r["leg"]), {})
                d["c"] = int(r["in_contact"])
                d["b"] = float(r["beta"])
                d["tau"] = max(abs(float(r["tau_demand"])), d.get("tau", 0.0))
            except (ValueError, KeyError):
                continue
    if len(per) < 2000:
        raise Unfit(f"only {len(per)} (t,leg) rows")
    out = {}
    for leg in LEGS:
        ts = sorted(t for (t, L) in per if L == leg)
        if len(ts) < 500:
            continue
        out[leg] = (np.array(ts),
                    np.array([per[(t, leg)]["c"] for t in ts], dtype=bool),
                    np.array([per[(t, leg)]["b"] for t in ts]),
                    np.array([per[(t, leg)]["tau"] for t in ts]))
    if not out:
        raise Unfit("no usable legs")
    return out


def budget(path):
    """-> (cost[NBINS], thrust[NBINS], n_episodes)."""
    legs = load(path)
    cost = [[] for _ in range(NBINS)]
    thrust = [[] for _ in range(NBINS)]
    n_ep = 0
    for leg, (t, c, b, tau) in legs.items():
        m = t >= (t.max() - TAIL_S)
        t, c, b, tau = t[m], c[m], b[m], tau[m]
        d = debounce(c, DEBOUNCE)
        edges = np.flatnonzero(d[1:].astype(int) - d[:-1].astype(int))
        rise = [i + 1 for i in edges if d[i + 1]]
        fall = [i + 1 for i in edges if not d[i + 1]]
        for r0 in rise:
            nxt = [f for f in fall if f > r0]
            if not nxt:
                continue
            f0 = nxt[0]
            if f0 - r0 < MIN_STANCE_SAMPLES:
                continue
            n_ep += 1
            idx = np.arange(r0, f0 + 1)
            prog = (idx - r0) / max(1, (f0 - r0))
            for k in range(NBINS):
                sel = idx[(prog >= k / NBINS) & (prog < (k + 1) / NBINS)]
                if len(sel) < 2:
                    continue
                cost[k].append(float(tau[sel].mean()))
                # forward roll contributed inside this bin
                thrust[k].append(R_FOOT * float(b[sel[-1]] - b[sel[0]]))
    if n_ep < 20:
        raise Unfit(f"only {n_ep} stance episodes")
    return (np.array([np.mean(v) if v else np.nan for v in cost]),
            np.array([np.sum(v) if v else np.nan for v in thrust]),
            n_ep)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", action="append", required=True)
    ap.add_argument("--label", action="append", default=[])
    args = ap.parse_args()
    print(f"stance progress bins, {NBINS} equal slices of each debounced "
          f"stance episode, last {TAIL_S:.0f}s\n")
    for i, d in enumerate(args.dir):
        lab = args.label[i] if i < len(args.label) else os.path.basename(
            d.rstrip("/"))
        cs, ts, ns = [], [], 0
        for p in sorted(glob.glob(os.path.join(os.path.expanduser(d),
                                               "run[0-9].csv"))):
            try:
                c, t, n = budget(p)
            except Unfit as e:
                print(f"  {p}: REFUSED -- {e}")
                continue
            cs.append(c); ts.append(t); ns += n
        if not cs:
            continue
        c = np.nanmean(np.array(cs), axis=0)
        t = np.nanmean(np.array(ts), axis=0)
        tt = np.nansum(t)
        print(f"== {lab}   ({len(cs)} runs, {ns} stance episodes)")
        print(f"   {'bin':>10} " + " ".join(f"{k/NBINS:.1f}-{(k+1)/NBINS:.1f}"
                                            for k in range(NBINS)))
        print(f"   {'mean |tau|':>10} " + " ".join(f"{v:7.1f}" for v in c))
        print(f"   {'roll (mm)':>10} " + " ".join(f"{1000*v:7.1f}" for v in t))
        if abs(tt) > 1e-9:
            print(f"   {'roll %':>10} " + " ".join(f"{100*v/tt:6.0f}%"
                                                   for v in t))
        # where the cost is vs where the distance is
        if np.nansum(c) > 0 and abs(tt) > 1e-9:
            cshare = c / np.nansum(c)
            tshare = t / tt
            k_cost = int(np.nanargmax(cshare))
            k_thrust = int(np.nanargmax(tshare))
            print(f"   peak COST in bin {k_cost} "
                  f"({k_cost/NBINS:.1f}-{(k_cost+1)/NBINS:.1f}), "
                  f"peak ROLL in bin {k_thrust} "
                  f"({k_thrust/NBINS:.1f}-{(k_thrust+1)/NBINS:.1f})"
                  + ("   -> SAME phase: inseparable"
                     if k_cost == k_thrust else
                     "   -> DIFFERENT phase: separable in principle"))
        print()


if __name__ == "__main__":
    main()
