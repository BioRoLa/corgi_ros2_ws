#!/usr/bin/env python3
"""lambda-0 cell from Vicon: forward speed, heading rate, curvature, and marker dropout.

Heading comes from a Kabsch fit of the five body markers against a reference frame, so it is
the rigid-body yaw and not the direction of travel -- the two differ when the robot crabs.
The gait window is found from the body's own vertical oscillation, so no trigger alignment
with the bag is needed.
Usage: vicon_l0.py file1.c3d [file2.c3d ...]
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
        frames = []
        for _, pts, _ in r.read_frames():
            frames.append(np.array(pts, dtype=float))
    P = np.stack(frames)                     # (T, N, >=4)
    xyz = P[:, :, 0:3]
    resid = P[:, :, 3]
    valid = resid >= 0
    return rate, labels, xyz, valid


def yaw_series(xyz, valid, idx, up):
    """Kabsch yaw about the vertical axis, per frame, against the first well-seen frame."""
    horiz = [a for a in (0, 1, 2) if a != up]
    T = xyz.shape[0]
    nval = valid[:, idx].sum(1)
    ref_t = int(np.argmax(nval))
    ref_mask = valid[ref_t, idx]
    ref = xyz[ref_t][idx][ref_mask]
    ref_c = ref - ref.mean(0)
    yaw = np.full(T, np.nan)
    for t in range(T):
        m = valid[t, idx] & ref_mask
        if m.sum() < 3:
            continue
        A = xyz[t][idx][m[ref_mask] if False else m]
        B = xyz[ref_t][idx][m]
        A = A - A.mean(0)
        B = B - B.mean(0)
        H = B.T @ A
        U, S, Vt = np.linalg.svd(H)
        d = np.sign(np.linalg.det(Vt.T @ U.T))
        R = Vt.T @ np.diag([1, 1, d]) @ U.T
        yaw[t] = np.degrees(np.arctan2(R[horiz[1], horiz[0]], R[horiz[0], horiz[0]]))
    return np.unwrap(np.radians(yaw[~np.isnan(yaw)])) * 180 / np.pi, ~np.isnan(yaw)


print("run          dur(s) | gait window (s) | v_fwd (m/s) | yaw rate (deg/s) | kappa (1/m) | radius (m) | dropout")
summary = []
for path in sys.argv[1:]:
    name = path.split("/")[-1].replace(".c3d", "")
    rate, labels, xyz, valid = load(path)
    T = xyz.shape[0]
    t = np.arange(T) / rate

    idx = [labels.index(b) for b in BODY if b in labels]
    if len(idx) < 4:
        print("%-12s only %d body markers found (%s)" % (name, len(idx), labels[:8])); continue

    # which axis is up: the one whose per-frame spread across body markers is smallest
    spread = [np.nanmean(np.nanstd(np.where(valid[:, idx], xyz[:, idx, a], np.nan), axis=1))
              for a in range(3)]
    up = int(np.argmin(spread))
    horiz = [a for a in (0, 1, 2) if a != up]

    cen = np.full((T, 3), np.nan)
    for a in range(3):
        v = np.where(valid[:, idx], xyz[:, idx, a], np.nan)
        cen[:, a] = np.nanmean(v, axis=1)
    ok = ~np.isnan(cen).any(1)

    # gait window: rolling RMS of vertical velocity
    dz = np.gradient(np.where(ok, cen[:, up], np.nan)) * rate
    win = int(0.5 * rate)
    k = np.ones(win) / win
    rms = np.sqrt(np.convolve(np.nan_to_num(dz) ** 2, k, mode="same"))
    thr = 0.35 * np.nanmax(rms)
    hop = rms > thr
    if hop.sum() < rate:
        print("%-12s no clear hopping window" % name); continue
    # LONGEST CONTIGUOUS hopping span, not first-to-last: one capture ran 76 s and held the
    # arc plus the walk back to the start, and first-to-last swallowed both.
    d = np.diff(np.concatenate(([0], hop.view(np.int8), [0])))
    starts = np.where(d == 1)[0]
    ends = np.where(d == -1)[0]
    gap = int(0.4 * rate)                    # bridge brief dropouts inside one arc
    merged = []
    for s, e in zip(starts, ends):
        if merged and s - merged[-1][1] <= gap:
            merged[-1][1] = e
        else:
            merged.append([s, e])
    i0, i1 = max(merged, key=lambda p: p[1] - p[0])
    # trim the first 2.5 s of settle inside the window, as the bag scoring does
    i0b = min(i1 - int(rate), i0 + int(0.0 * rate))
    w = slice(i0b, i1)

    yaw_u, yok = yaw_series(xyz, valid, idx, up)
    tt = t[yok]
    sel = (tt >= t[i0b]) & (tt <= t[i1 - 1])
    if sel.sum() > 10:
        A = np.polyfit(tt[sel], yaw_u[sel], 1)
        yaw_rate = float(A[0])
    else:
        yaw_rate = float("nan")

    pos = cen[w][:, horiz] / 1000.0          # mm -> m
    tw = t[w]
    good = ~np.isnan(pos).any(1)
    pos = pos[good]; tw = tw[good]
    step = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    dt = np.diff(tw)
    v = float(np.median(step / dt))
    net = float(np.linalg.norm(pos[-1] - pos[0]))
    dur = float(tw[-1] - tw[0])
    v_net = net / dur if dur else float("nan")

    kappa = np.radians(yaw_rate) / v_net if v_net > 1e-6 else float("nan")
    radius = 1.0 / kappa if abs(kappa) > 1e-9 else float("inf")

    lab_idx = [i for i, l in enumerate(labels) if not l.startswith("*")]
    drop = 100.0 * (1.0 - valid[w, :][:, lab_idx].mean())

    print("%-12s %6.1f | %6.2f-%6.2f | net %5.3f  step %5.3f | %+8.2f | %+9.4f | %8.2f | %5.2f%%"
          % (name, T / rate, t[i0b], t[i1 - 1], v_net, v, yaw_rate, kappa, radius, drop))
    summary.append((name, v_net, yaw_rate, kappa, drop, dur, net))

if summary:
    v = np.array([s[1] for s in summary]); yr = np.array([s[2] for s in summary])
    kp = np.array([s[3] for s in summary]); dr = np.array([s[4] for s in summary])
    print()
    print("v_fwd (net displacement / time): median %.3f m/s   spread %.3f to %.3f" % (np.median(v), v.min(), v.max()))
    print("   -> 0.85x speed-validity floor = %.3f m/s" % (0.85 * np.median(v)))
    print("yaw rate : median %+.2f deg/s    spread %+.2f to %+.2f" % (np.median(yr), yr.min(), yr.max()))
    print("kappa    : median %+.4f 1/m      spread %+.4f to %+.4f   (registered bar |kappa| <= 0.05)" % (np.median(kp), kp.min(), kp.max()))
    print("           n over the bar: %d/%d" % (int((np.abs(kp) > 0.05).sum()), len(kp)))
    print("dropout  : median %.2f %%          worst %.2f %%   (labelled markers, gait window)" % (np.median(dr), dr.max()))
    print()
    print("per-run distance and duration:")
    for s in summary:
        print("   %-12s net %5.2f m over %5.2f s" % (s[0], s[6], s[5]))
