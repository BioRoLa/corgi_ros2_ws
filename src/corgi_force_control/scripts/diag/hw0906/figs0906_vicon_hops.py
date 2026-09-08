#!/usr/bin/env python3
"""Per-HOP forward speed from Vicon, for the per-stride speed figure (log 325.25 owes it).

Hop window exactly as vicon_split.py / vicon_l0.py: body centroid of B1..B5, vertical axis =
smallest marker spread, rolling 0.5 s RMS of vertical velocity > 0.35 x max, longest contiguous
span with 0.4 s gap bridging. Inside the window the arc is split into individual hops at the
UPWARD zero crossings of the (lightly smoothed) vertical velocity, i.e. at the bottom of each
stance; per hop, speed = net horizontal displacement / hop duration.
The whole-arc registered estimator (net displacement / window duration) is recomputed as a
self-check against the numbers in the log; it must agree to 3 digits or the run is flagged.
Usage: figs0906_vicon_hops.py out.json name=file.c3d [...]
"""
import sys, json
import numpy as np
import c3d

BODY = ["B1", "B2", "B3", "B4", "B5"]   # B6 dropped 2026-09-08: it IS rigid on the body
# (|B6-B1| 346.8 +- 0.4 mm) but is visible in only ~95% of in-window frames, and the
# varying-subset nanmean stepped the centroid by (B6-c5)/6 = 3.7 mm. B1-B5 are ~100%.
FLOOR = 0.257            # registered 0.85 x baseline at k_roll 0.60 (log 325.37)
SMOOTH_S = 0.04          # smoothing of vertical velocity before zero-crossing detection
MIN_HOP_S = 0.20         # crossings closer than this are the same stance bottom
FULL_LO, FULL_HI = 0.22, 0.32   # template-like hop period (0.2642 s design); outside = launch/stop segment
# whole-arc values from the log (headline table in the brief), for the self-check only
LOG_V = {"L0_RA1": 0.285, "L0RA2": 0.320, "OL10_A1": 0.157, "OL10_R1": 0.262, "OL10A2": 0.168,
         "OL10_NA1": 0.212, "OL10_NA2": 0.309, "OL15_A1": 0.232, "OL15_NA1": 0.273}


def load(path):
    with open(path, "rb") as fh:
        r = c3d.Reader(fh)
        rate = float(r.point_rate)
        labels = [l.strip() for l in r.point_labels]
        frames = [np.array(p, dtype=float) for _, p, _ in r.read_frames()]
    P = np.stack(frames)
    return rate, labels, P[:, :, 0:3], P[:, :, 3] >= 0


def interp_nan(x):
    x = x.copy()
    bad = np.isnan(x)
    if bad.all():
        return x
    idx = np.arange(len(x))
    x[bad] = np.interp(idx[bad], idx[~bad], x[~bad])
    return x


out_path = sys.argv[1]
result = {}
for arg in sys.argv[2:]:
    name, path = arg.split("=", 1)
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

    # --- whole-arc registered estimator, as vicon_l0.py (net displacement / duration) ---
    pos = cen[i0:i1][:, horiz] / 1000.0
    tw = t[i0:i1]
    good = ~np.isnan(pos).any(1)
    pw = pos[good]; tg = tw[good]
    v_arc = float(np.linalg.norm(pw[-1] - pw[0]) / (tg[-1] - tg[0]))
    flag = ""
    if name in LOG_V and abs(v_arc - LOG_V[name]) > 0.0015:
        flag = "   ** DISAGREES with log %.3f **" % LOG_V[name]

    # --- split into hops at upward zero crossings of smoothed vertical velocity ---
    z = interp_nan(cen[i0:i1, up])
    dzw = np.gradient(z) * rate / 1000.0                       # m/s
    k = max(1, int(SMOOTH_S * rate))
    dzs = np.convolve(dzw, np.ones(k) / k, mode="same")
    cross = np.where((dzs[:-1] <= 0) & (dzs[1:] > 0))[0] + 1
    keep = []
    for c in cross:
        if keep and (c - keep[-1]) < MIN_HOP_S * rate:
            continue
        keep.append(int(c))
    ph = np.stack([interp_nan(cen[i0:i1, a]) for a in horiz], 1) / 1000.0
    hops = []
    for a, b in zip(keep[:-1], keep[1:]):
        dur = (b - a) / rate
        disp = float(np.linalg.norm(ph[b] - ph[a]))
        apex = float((z[a:b].max() - z[a]) / 1000.0)
        hops.append(dict(t=float(t[i0 + a] - t[i0]), dur=dur, v=disp / dur, rise=apex))
    v = np.array([h["v"] for h in hops])
    # a "full" hop has a template-like period; the launch segment (~0.45 s, half-height apex) and
    # any zero-rise stub at the window edge are not strides in the bag's sense and are excluded
    # from the medians and from N_floor, but still plotted (hollow) so nothing is hidden.
    full = np.array([FULL_LO <= h["dur"] <= FULL_HI for h in hops])
    vf = v[full]
    below = [i + 1 for i, h in enumerate(hops) if full[i] and h["v"] < FLOOR]
    n_floor = below[0] if below else None            # registered: first (full) hop below the floor
    n_floor3 = None                                    # exploratory: first hop of >=3 consecutive full hops below
    for i in range(len(hops) - 2):
        if all(full[i + j] and hops[i + j]["v"] < FLOOR for j in range(3)):
            n_floor3 = i + 1; break
    print("== %-9s window %6.2f-%6.2f s (%.2f s)  whole-arc v %.3f m/s%s" % (name, t[i0], t[i1 - 1], (i1 - i0) / rate, v_arc, flag))
    print("   hops %d (full %d)   median per-hop v (full) %.3f   min %.3f   max %.3f   N_floor reg (first full hop < %.3f): %s   3-consecutive: %s   hop period median %.3f s"
          % (len(hops), int(full.sum()), np.median(vf), vf.min(), vf.max(), FLOOR, n_floor, n_floor3, np.median([h["dur"] for h in hops])))
    print("   hop:  " + " ".join("%5d" % (i + 1) for i in range(len(hops))))
    print("   v  :  " + " ".join("%5.3f" % h["v"] for h in hops))
    print("   dur:  " + " ".join("%5.3f" % h["dur"] for h in hops))
    print("   rise: " + " ".join("%5.3f" % h["rise"] for h in hops))
    result[name] = dict(v_arc=round(v_arc, 4), window=[round(float(t[i0]), 3), round(float(t[i1 - 1]), 3)],
                        n_hops=len(hops), n_full=int(full.sum()), median_v_full=round(float(np.median(vf)), 4),
                        n_floor=n_floor, n_floor3=n_floor3,
                        v=[round(h["v"], 4) for h in hops], dur=[round(h["dur"], 4) for h in hops],
                        full=[bool(f) for f in full], t=[round(h["t"], 3) for h in hops])
    print()

with open(out_path, "w") as fh:
    json.dump(result, fh, indent=1)
print("wrote", out_path)
