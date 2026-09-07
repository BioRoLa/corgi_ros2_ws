#!/usr/bin/env python3
"""How much does the machine sink under its own weight, and does it sink evenly?

The joint encoders cannot see the tread: theta is measured at the joint and the TPU is
downstream of it, so tread compression appears nowhere in a joint tracking error. Vicon can
see it, because it watches the body.

Method: split the six body markers into a front group and a rear group along the body's own
fore/aft axis, then compare each group's height at the top of flight against its height at
the bottom of stance. That difference is everything in series below the body at that end --
commanded impedance, leg structure, and tread.
Usage: ride_height.py file.c3d [...]
"""
import sys
import numpy as np
import c3d

BODY = ["B1", "B2", "B3", "B4", "B5", "B6"]


def load(path):
    with open(path, "rb") as fh:
        r = c3d.Reader(fh)
        rate = float(r.point_rate)
        labels = [l.strip() for l in r.point_labels]
        frames = [np.array(p, dtype=float) for _, p, _ in r.read_frames()]
    P = np.stack(frames)
    return rate, labels, P[:, :, 0:3], P[:, :, 3] >= 0


print("run          | body sink, top of flight -> bottom of stance (mm)      | fore/aft")
print("             |   whole body      front end       rear end             | difference")
for path in sys.argv[1:]:
    name = path.split("/")[-1].replace(".c3d", "")
    rate, labels, xyz, valid = load(path)
    T = xyz.shape[0]
    t = np.arange(T) / rate
    idx = [labels.index(b) for b in BODY if b in labels]
    if len(idx) < 5:
        print("%-12s only %d body markers" % (name, len(idx))); continue
    spread = [np.nanmean(np.nanstd(np.where(valid[:, idx], xyz[:, idx, a], np.nan), axis=1)) for a in range(3)]
    up = int(np.argmin(spread))
    horiz = [a for a in (0, 1, 2) if a != up]

    cen = np.full((T, 3), np.nan)
    for a in range(3):
        cen[:, a] = np.nanmean(np.where(valid[:, idx], xyz[:, idx, a], np.nan), axis=1)
    ok = ~np.isnan(cen).any(1)
    dz = np.gradient(np.where(ok, cen[:, up], np.nan)) * rate
    win = int(0.5 * rate)
    rms = np.sqrt(np.convolve(np.nan_to_num(dz) ** 2, np.ones(win) / win, mode="same"))
    hop = rms > 0.35 * np.nanmax(rms)
    d = np.diff(np.concatenate(([0], hop.view(np.int8), [0])))
    st, en = np.where(d == 1)[0], np.where(d == -1)[0]
    gap = int(0.4 * rate)
    merged = []
    for s, e in zip(st, en):
        if merged and s - merged[-1][1] <= gap:
            merged[-1][1] = e
        else:
            merged.append([s, e])
    i0, i1 = max(merged, key=lambda p: p[1] - p[0])
    w = slice(i0, i1)

    # fore/aft axis of the marker cloud, from the frame where all markers are seen
    nval = valid[:, idx].sum(1)
    ref = int(np.argmax(nval))
    P = xyz[ref][idx]
    Pc = P - P.mean(0)
    # principal horizontal direction = body long axis
    H = Pc[:, horiz]
    u, s, vt = np.linalg.svd(H - H.mean(0))
    axis = vt[0]
    proj = (H - H.mean(0)) @ axis
    order = np.argsort(proj)
    nf = max(2, len(idx) // 3)
    rear_m = [idx[k] for k in order[:nf]]
    front_m = [idx[k] for k in order[-nf:]]

    def group_h(members):
        v = np.where(valid[:, members], xyz[:, members, up], np.nan)
        return np.nanmean(v, axis=1)

    hf, hr, hb = group_h(front_m), group_h(rear_m), cen[:, up]
    res = []
    for h in (hb, hf, hr):
        hh = h[w]
        g = ~np.isnan(hh)
        if g.sum() < 100:
            res.append(float("nan")); continue
        hh = hh[g]
        res.append(float(np.percentile(hh, 95) - np.percentile(hh, 5)))
    print("%-12s |    %6.1f          %6.1f          %6.1f            |  %+6.1f"
          % (name, res[0], res[1], res[2], res[1] - res[2]))

print()
print("Note: this is the TOTAL series give below the body -- commanded impedance, leg")
print("structure and tread together. It cannot separate them, and it is a peak-to-peak")
print("excursion of a hopping machine, not a static sink. A static hold measurement,")
print("or a bench load test, is what separates the tread from the rest.")
