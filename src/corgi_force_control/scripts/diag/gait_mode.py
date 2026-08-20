"""Which GAIT is this, not how well is it doing.

Added 2026-08-20 for the k_flight sweep (log S103) after Alex, watching
the render at k_flight = 2500: "the robot isn't really pronking anymore,
it's just lifting the front legs, then lifting the rear, but never
together and no real airtime."

Flight fraction alone cannot say that. A pronk with less flight and an
alternating front/rear rock both read as "low flight", but they are
different gaits, and a torque curve that silently crosses between them
is the "valid within a condition, meaningless across" trap in its purest
form -- the cheap cells would be cheap because they are not running the
gait being priced.

Contact-state occupancy over the steady tail separates them:

    PRONK      four legs together AND leaving the ground -> real ALL-AIR
    ROCK/BOUND front pair and rear pair alternate -> time on ONE PAIR,
               little or no all-air
    STAND      everything down, nearly all the time

    air_index = air / (air + front_only + rear_only)

1.0 is a clean pronk; 0 is a pure alternating gait or a stand.

⚠ FIRST ATTEMPT WAS WRONG, and wrong in the direction that flatters a
dead gait: `(all4 + air) / (all4 + air + F + R)` scored the k1500 STAND
at 0.79 (76% all-four-down, 0.3% air) and the real design-gain PRONK at
0.63 (22% all-four, 37% air) -- because "all four down" is exactly what
standing is. Counting all4 as pronk-like rewards not moving. Air is the
discriminator that cannot be faked: a pronk must leave the ground.
Kept as a comment because the broken version looked entirely reasonable.

Usage:
    gait_mode.py run1.csv [run2.csv ...]
    gait_mode.py --dir ~/corgi_runs/kflight/k2500/lam0_default
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
FRONT, REAR = (0, 1), (2, 3)      # A,B front; C,D rear
TAIL_S = 20.0


class Unfit(Exception):
    pass


def load_contact(path):
    per = {}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            if r["motor"] != "L_Motor":
                continue
            try:
                per.setdefault(float(r["t"]), {})[r["leg"]] = \
                    int(r["in_contact"])
            except (ValueError, KeyError):
                continue
    ts = sorted(t for t, v in per.items() if len(v) == 4)
    if len(ts) < 500:
        raise Unfit(f"only {len(ts)} complete samples")
    t = np.array(ts)
    c = np.array([[per[k][l] for l in LEGS] for k in ts], dtype=bool)
    return t, c


def stats(path, tail_s=TAIL_S):
    t, c = load_contact(path)
    m = t >= (t.max() - tail_s)
    t, c = t[m], c[m]
    if len(t) < 200:
        raise Unfit(f"only {len(t)} samples in the tail")
    # Debounce per leg, exactly as the touchdown detectors do, so a
    # single chattering sample cannot invent an alternation.
    d = np.column_stack([debounce(c[:, i], DEBOUNCE) for i in range(4)])
    n_down = d.sum(axis=1)
    f = d[:, list(FRONT)].any(axis=1)
    r = d[:, list(REAR)].any(axis=1)
    none = float((n_down == 0).mean())
    all4 = float((n_down == 4).mean())
    front_only = float((f & ~r).mean())
    rear_only = float((r & ~f).mean())
    denom = none + front_only + rear_only
    return {
        "none": none, "all4": all4,
        "front_only": front_only, "rear_only": rear_only,
        "air_index": none / denom if denom > 1e-9 else float("nan"),
        "occ": [float((n_down == k).mean()) for k in range(5)],
        "span": float(t[-1] - t[0]),
    }


def report(path, s):
    occ = " ".join(f"{100*v:4.1f}" for v in s["occ"])
    print(f"{'/'.join(path.split('/')[-3:-1]):22} "
          f"{os.path.basename(path):12} "
          f"air_idx {s['air_index']:5.2f}   "
          f"air {100*s['none']:4.1f}%  all4 {100*s['all4']:4.1f}%  "
          f"F-only {100*s['front_only']:4.1f}%  R-only {100*s['rear_only']:4.1f}%"
          f"   occ0-4 [{occ}]")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="*")
    ap.add_argument("--dir", action="append", default=[])
    ap.add_argument("--tail", type=float, default=TAIL_S)
    args = ap.parse_args()
    paths = list(args.runs)
    for d in args.dir:
        paths += sorted(glob.glob(os.path.join(
            os.path.expanduser(d), "run[0-9].csv")))
    if not paths:
        ap.error("give run CSVs or --dir")
    vals = []
    for p in paths:
        try:
            s = stats(p, args.tail)
        except Unfit as e:
            print(f"{p}: REFUSED -- {e}")
            continue
        report(p, s)
        vals.append(s)
    if len(vals) > 1:
        pi = np.array([v["air_index"] for v in vals])
        print(f"{'POOLED (n=' + str(len(pi)) + ')':35} "
              f"air_idx median {np.median(pi):5.2f} "
              f"[{pi.min():.2f}, {pi.max():.2f}]")


if __name__ == "__main__":
    main()
