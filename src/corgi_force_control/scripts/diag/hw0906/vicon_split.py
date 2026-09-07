#!/usr/bin/env python3
"""Does the cambered arc start valid and decay? Speed and heading rate in successive
2 s slices of the hopping window, so a short arc can be judged separately from a long one.
Usage: vicon_split.py file.c3d [...]
"""
import sys
import numpy as np
import c3d

BODY = ["B1", "B2", "B3", "B4", "B5", "B6"]
FLOOR = 0.252          # registered 0.85x speed-validity floor, log 325.20


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

    print("== %s   hop window %.2f-%.2f s (%.1f s)" % (name, t[i0], t[i1 - 1], (i1 - i0) / rate))
    print("   slice(s)      net speed (m/s)   vs floor 0.252")
    step = int(2.0 * rate)
    for a in range(i0, i1 - int(0.8 * rate), step):
        b = min(a + step, i1)
        p = cen[a:b][:, horiz] / 1000.0
        g = ~np.isnan(p).any(1)
        if g.sum() < 20:
            continue
        p = p[g]
        dur = (g.sum() - 1) / rate
        v = float(np.linalg.norm(p[-1] - p[0]) / dur) if dur > 0 else float("nan")
        bar = "PASS" if v >= FLOOR else "fail"
        print("   %5.2f-%5.2f      %6.3f            %s" % (t[a], t[b - 1], v, bar))
    print()
