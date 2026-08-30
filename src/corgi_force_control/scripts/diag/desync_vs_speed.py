"""Do desynchronised legs cost speed? Pooled across every campaign on disk.

The torque ceiling ruled itself out: 6x the clamp, no change in speed. The
energy audit says ~40 W of net motor work is leaving the system without coming
back through the motors. And the friction bracket runs BACKWARDS -- mu 0.6
gives 0.79 m/s, mu 1.6 gives 0.40 -- so grip is costing speed, which is the
signature of legs fighting the ground rather than rolling along it.

A pronk reduces four legs to one virtual leg ONLY if they act together. Section
17 measured 32-45% desynchronisation on every forward rung. If desynchronised
legs are in contact at different points of their own sweep, they push against
each other through the ground; high friction makes that binding expensive and
low friction lets it slide, which is exactly the observed friction ordering.

This measures desync directly -- the spread of touchdown instants across the
four legs, as a fraction of a stride -- and pools it against achieved speed over
every run recorded, so the correlation is across ~30 runs rather than within one
campaign.

    python3 desync_vs_speed.py <dir> [<dir> ...]
"""
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_yaw_phase import dedupe_time  # noqa: E402
from ramp_segments import load as load_template  # noqa: E402

START, END = 4.0, 9.5


def touchdown_spread(ct, cv, stride_s, t0, t1):
    """-> mean spread of per-leg touchdown instants, as a fraction of a stride.

    For each leg, the rising edges of its own contact signal. Legs acting as one
    virtual leg touch down together and the spread is ~0; a spread approaching
    0.5 means legs are landing in antiphase and the pronk reduction is void.
    """
    m = (ct >= t0) & (ct <= t1)
    t, c = ct[m], cv[m]
    if len(t) < 100:
        return float("nan")
    edges = []
    for i in range(4):
        e = t[np.flatnonzero(c[1:, i] & ~c[:-1, i]) + 1]
        edges.append(e)
    if min(len(e) for e in edges) < 3:
        return float("nan")
    # Phase of each leg's touchdowns within the stride cycle, as a unit vector;
    # circular spread is the right statistic because phase wraps.
    spreads = []
    ref = edges[0]
    for k in range(min(len(e) for e in edges)):
        ph = np.array([(edges[i][k] % stride_s) / stride_s for i in range(4)])
        z = np.exp(2j * np.pi * ph).mean()
        spreads.append(1.0 - abs(z))          # 0 = perfectly in phase
    return float(np.mean(spreads))


def one(path):
    d = np.load(path, allow_pickle=True)
    ot, ov = dedupe_time(d["odom_t"], d["odom"])
    ct, cv = d["contact_t"], d["contact"]
    m = (ot >= START) & (ot <= END)
    if int(m.sum()) < 50:
        return None
    t, seg = ot[m], ov[m]
    span = t[-1] - t[0]
    v = float(np.hypot(np.diff(seg[:, 0]), np.diff(seg[:, 1])).sum() / span)
    tt, _, _, _, st = load_template(str(d["template"]))
    stride_s = float(tt[-1])
    cm = (ct >= t[0]) & (ct <= t[-1])
    air = 100.0 * float((~cv[cm].any(axis=1)).mean())
    return dict(v=v, air=air,
                desync=touchdown_spread(ct, cv, stride_s, t[0], t[-1]))


def main():
    rows, names = [], []
    for root in sys.argv[1:]:
        for p in sorted(glob.glob(os.path.join(root, "*.npz"))):
            r = one(p)
            if r and np.isfinite(r["desync"]):
                rows.append(r)
                names.append(os.path.basename(p).replace(".npz", ""))
    if len(rows) < 5:
        print("too few runs")
        return

    v = np.array([r["v"] for r in rows])
    ds = np.array([r["desync"] for r in rows])
    air = np.array([r["air"] for r in rows])

    print(f"pooled over {len(rows)} runs")
    print()
    print(f"  {'run':>22} {'v m/s':>7} {'desync':>8} {'flight%':>8}")
    for n, r in sorted(zip(names, rows), key=lambda x: x[1]["desync"]):
        print(f"  {n:>22} {r['v']:7.3f} {r['desync']:8.3f} {r['air']:7.1f}%")
    print()
    print(f"  corr(desync, speed)   {np.corrcoef(ds, v)[0, 1]:+.3f}")
    print(f"  corr(desync, flight)  {np.corrcoef(ds, air)[0, 1]:+.3f}")
    print()
    print("  desync = circular spread of the four legs' touchdown phases.")
    print("           0 = one virtual leg (the pronk reduction holds);")
    print("           rising values mean the legs are landing apart, and the")
    print("           single-virtual-leg model the template rests on is void.")


if __name__ == "__main__":
    main()
