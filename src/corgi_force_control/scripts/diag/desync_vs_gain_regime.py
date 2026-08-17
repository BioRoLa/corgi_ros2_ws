"""Measure leg desynchronisation, and test whether it drives the gain-regime fault.

THE CHAIN UNDER TEST. Legs desync -> one global gain schedule cannot track four
independent contact events -> a loaded leg is held at the stiff flight gain
instead of the virtual spring. The last link is confirmed (n = 4, ratio
0.76-0.78). This measures the FIRST link on the same captures, so both ends of
the chain come from one dataset.

DESYNC, defined as in the earlier analysis: the fraction of samples where the
four legs disagree about being down. Broken out by grouping, because the earlier
work found the split is always FRONT/REAR and never diagonal -- which is the
signature of a pitch moment, not of random jitter:

    front {A,B}  vs  rear {C,D}      <- expected to dominate
    left  {A,D}  vs  right {B,C}
    diag  {A,C}  vs  {B,D}

WHY BOTH NUMBERS TOGETHER. If desync is what breaks the gain schedule, runs with
more desync should show a worse gain-regime ratio. Three runs is far too few to
fit anything, but the pair should at least move together -- and if they move
OPPOSITE ways, the proposed chain is wrong and the k_tangential lever is aimed
at nothing.

Run:
    python3 desync_vs_gain_regime.py run1.csv [run2.csv ...]
"""
import csv
import sys

import numpy as np

LEGS = "ABCD"
FRONT, REAR = (0, 1), (2, 3)      # A,B front; C,D rear  (module x = +/-0.255)
LEFT, RIGHT = (0, 3), (1, 2)      # A,D left;  B,C right (module y = +/-0.12)
DIAG_A, DIAG_B = (0, 2), (1, 3)
KP_THRESH = 100.0                 # between the 36 and 289 gain modes


def load(path):
    """(t, contact[4], kp) sampled once per timestamp, from the L_Motor rows."""
    per_t = {}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            if r["motor"] != "L_Motor":
                continue
            t = float(r["t"])
            d = per_t.setdefault(t, {"c": [None] * 4, "kp": None,
                                     "tf": 0.0})
            i = LEGS.index(r["leg"])
            d["c"][i] = int(r["in_contact"])
            if d["kp"] is None:
                d["kp"] = float(r["kp"])
            d["tf"] = max(d["tf"], abs(float(r["t_ff"])))
    ts = sorted(k for k, v in per_t.items() if all(x is not None for x in v["c"]))
    t = np.array(ts)
    c = np.array([per_t[k]["c"] for k in ts], dtype=bool)
    kp = np.array([per_t[k]["kp"] for k in ts])
    tf = np.array([per_t[k]["tf"] for k in ts])
    return t, c, kp, tf


def split_frac(c, ga, gb):
    """Fraction of samples where group a and group b disagree about being down.

    Each group is scored by majority-down, so a single chattering leg does not
    register as a whole-group split.
    """
    a = c[:, list(ga)].mean(axis=1) > 0.5
    b = c[:, list(gb)].mean(axis=1) > 0.5
    return float((a != b).mean())


def main():
    paths = sys.argv[1:]
    if not paths:
        print("usage: desync_vs_gain_regime.py run1.csv [run2.csv ...]")
        return

    print()
    print("=" * 78)
    print("LEG DESYNCHRONISATION, AND THE GAIN-REGIME FAULT, FROM ONE DATASET")
    print("=" * 78)
    print(f"{'run':>10} {'n':>7} {'any split':>10} {'front/rear':>11} "
          f"{'left/right':>11} {'diagonal':>9} {'regime ratio':>13}")

    rows = []
    for p in paths:
        t, c, kp, tf = load(p)
        nz = np.flatnonzero(tf > 1e-9)
        if not nz.size:
            print(f"{p.split('/')[-1]:>10}   no gait window")
            continue
        sel = t >= t[nz[0]]
        t, c, kp = t[sel], c[sel], kp[sel]
        if len(t) < 500:
            print(f"{p.split('/')[-1]:>10} {len(t):7d}   too few samples")
            continue

        any_split = float((c.any(axis=1) & ~c.all(axis=1)).mean())
        fr = split_frac(c, FRONT, REAR)
        lr = split_frac(c, LEFT, RIGHT)
        dg = split_frac(c, DIAG_A, DIAG_B)

        # Gain-regime ratio on the SAME samples, so the two are commensurable.
        stance_g = kp < KP_THRESH
        down = c.mean(axis=1) > 0.5
        ratio = (stance_g[down].mean() / stance_g.mean()
                 if down.sum() > 50 and stance_g.mean() > 0 else float("nan"))

        rows.append((any_split, fr, lr, dg, ratio))
        print(f"{p.split('/')[-1]:>10} {len(t):7d} {100*any_split:9.1f}% "
              f"{100*fr:10.1f}% {100*lr:10.1f}% {100*dg:8.1f}% {ratio:13.2f}")

    if not rows:
        return
    a = np.array(rows)
    print()
    print("=" * 78)
    print(f"  any split     {100*a[:,0].mean():5.1f}%  "
          f"[{100*a[:,0].min():.1f}-{100*a[:,0].max():.1f}]")
    print(f"  front/rear    {100*a[:,1].mean():5.1f}%  "
          f"[{100*a[:,1].min():.1f}-{100*a[:,1].max():.1f}]   <- the pitch-moment signature")
    print(f"  left/right    {100*a[:,2].mean():5.1f}%")
    print(f"  diagonal      {100*a[:,3].mean():5.1f}%")
    print(f"  regime ratio  {a[:,4].mean():5.2f}")
    print()
    if a[:, 1].mean() > a[:, 2].mean() and a[:, 1].mean() > a[:, 3].mean():
        print("  Front/rear dominates, as the pitch-moment account predicts.")
        print("  k_tangential is the lever: the moment is LINEAR in it, and it")
        print("  has never been tuned against this.")
    else:
        print("  Front/rear does NOT dominate here. The pitch-moment account")
        print("  does not fit this data -- do not sweep k_tangential on it.")
    print("=" * 78)
    print()
    print("  NOTE: with n = 3 the desync/ratio pair cannot be fitted. What it")
    print("  can do is falsify: if a run with MORE desync shows a BETTER regime")
    print("  ratio, the proposed chain is wrong.")
    print()


if __name__ == "__main__":
    main()
