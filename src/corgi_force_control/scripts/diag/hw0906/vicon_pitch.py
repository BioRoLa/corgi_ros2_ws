#!/usr/bin/env python3
"""Is the growing body pitch real, or is the IMU estimate drifting?

The pitch controller acts on the IMU's quaternion pitch. If that estimate drifts, the
controller is chasing a lie and will progressively extend one pair of legs. Vicon gives the
true rigid-body attitude from the marker cloud, so the two can be compared directly.
Usage: vicon_pitch.py file.c3d [...]
"""
import sys
import numpy as np
import c3d

BODY = ["B1", "B2", "B3", "B4", "B5"]   # B6 dropped 2026-09-08: it IS rigid on the body
# (|B6-B1| 346.8 +- 0.4 mm) but is visible in only ~95% of in-window frames, and the
# varying-subset nanmean stepped the centroid by (B6-c5)/6 = 3.7 mm. B1-B5 are ~100%.


def load(path):
    with open(path, "rb") as fh:
        r = c3d.Reader(fh)
        rate = float(r.point_rate)
        labels = [l.strip() for l in r.point_labels]
        frames = [np.array(p, dtype=float) for _, p, _ in r.read_frames()]
    P = np.stack(frames)
    return rate, labels, P[:, :, 0:3], P[:, :, 3] >= 0


for path in sys.argv[1:]:
    name = path.split("/")[-1].replace(".c3d", "")
    rate, labels, xyz, valid = load(path)
    T = xyz.shape[0]
    t = np.arange(T) / rate
    idx = [labels.index(b) for b in BODY if b in labels]
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
    starts, ends = np.where(d == 1)[0], np.where(d == -1)[0]
    gap = int(0.4 * rate)
    merged = []
    for s, e in zip(starts, ends):
        if merged and s - merged[-1][1] <= gap:
            merged[-1][1] = e
        else:
            merged.append([s, e])
    i0, i1 = max(merged, key=lambda p: p[1] - p[0])

    # rigid-body attitude by Kabsch against the best-seen frame
    nval = valid[:, idx].sum(1)
    ref_t = int(np.argmax(nval))
    ref_mask = valid[ref_t, idx]
    tilt = np.full(T, np.nan)      # rotation of the body's long axis out of horizontal
    for tt in range(i0, i1):
        m = valid[tt, idx] & ref_mask
        if m.sum() < 4:
            continue
        A = xyz[tt][idx][m]; B = xyz[ref_t][idx][m]
        A = A - A.mean(0); B = B - B.mean(0)
        U, S, Vt = np.linalg.svd(B.T @ A)
        dd = np.sign(np.linalg.det(Vt.T @ U.T))
        R = Vt.T @ np.diag([1, 1, dd]) @ U.T
        # pitch = rotation that tips the horizontal fore/aft axis toward vertical.
        # horiz[0] is taken as the fore/aft axis; report the vertical component it acquires.
        tilt[tt] = np.degrees(np.arcsin(np.clip(R[up, horiz[0]], -1, 1)))

    good = ~np.isnan(tilt)
    tt = t[good]; v = tilt[good]
    if len(v) < 100:
        print("%-12s not enough attitude frames" % name); continue
    n = len(v); h3 = n // 3
    A = np.polyfit(tt, v, 1)
    print("== %-12s hop window %.2f-%.2f s" % (name, t[i0], t[i1 - 1]))
    print("   Vicon body tilt about the lateral axis (deg, referenced to the arc's own start)")
    print("      first third %+6.2f   middle %+6.2f   last third %+6.2f   |  drift %+5.2f deg over the arc, slope %+5.2f deg/s"
          % (v[:h3].mean() - v[:h3].mean(), v[h3:2 * h3].mean() - v[:h3].mean(),
             v[2 * h3:].mean() - v[:h3].mean(), v[2 * h3:].mean() - v[:h3].mean(), A[0]))
    print()
