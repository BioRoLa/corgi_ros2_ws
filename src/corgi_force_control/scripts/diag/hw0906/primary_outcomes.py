#!/usr/bin/env python3
"""Registered primary outcomes of the 325.25 protocol, computed for the first time.

Per run, in the machine's own stride clock (bag), with Vicon aligned onto it:
  N_sat   first complete stride whose per-stride |tau_h| peak on ANY leg is >= 39 N.m
  N_floor first stride whose Vicon net-displacement speed over the BAG's stride window is
          below the 0.257 m/s floor (325.37)
  droll/dstride, dpitch/dstride  IMU quaternion attitude, stride means, slope over 1..N_sat
  lambda_sustain per sign  largest |lambda| whose WHOLE-ARC speed clears the floor

Stride detector = l_trend.py's: kp_r on module_a falling below 100 in /motor/command,
edges inside (2.6 s, gait - 0.3 s); stride k (1-based) = [edge_k, edge_{k+1}).
Clock alignment: the IMU vertical acceleration (bag clock) is cross-correlated with the
second derivative of the Vicon body-centroid height (Vicon clock); a 0.5 s RMS envelope
gives the coarse lag without periodic ambiguity, the raw signals refine it within half a
stride. The peak is checked against the best peak one stride period away.
Hop split (vicon_split.py's window, cut at the bottoms of the vertical trajectory) is kept
as a cross-check only. Whole-arc speed is recomputed with vicon_l0.py's estimator as a
SELF-TEST against the headline numbers already in the log.

Usage: primary_outcomes.py <data root> <out.json>
"""
import sys, os, json, sqlite3
import numpy as np
import c3d
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
try:
    from scipy.signal import correlate as _corr
    def xcorr(a, b):
        return _corr(a, b, mode="full", method="fft")
except Exception:
    def xcorr(a, b):
        return np.correlate(a, b, mode="full")

ROOT = sys.argv[1]
OUT_JSON = sys.argv[2]
FLOOR = 0.257            # 0.85 x mean(0.285, 0.320), log 325.37
TAU_SAT = 39.0           # N.m, registered 325.25
T_FIRST = 2.6            # strides start 2.6 s after trigger-on (l_trend.py)
MIN_HOP_S = 0.15
L4 = "abcd"
NM = {"a": "A FL", "b": "B FR", "c": "C RR", "d": "D RL"}
BODY = ["B1", "B2", "B3", "B4", "B5", "B6"]

HEADLINE = {"L0_RA1": 0.285, "L0RA2": 0.320, "OL10_R1": 0.262, "OL10A2": 0.168,
            "OL10_NA1": 0.212, "OL10_NA2": 0.309, "OL15_A1": 0.232, "OL15_NA1": 0.273}

RUNS = [  # bag, vicon, lambda, ctl-log stride count, note
    ("s2_l0r_a1",    "L0_RA1",   0,  24, ""),
    ("s2_l0r_a2",    "L0RA2",    0,  25, ""),
    ("s2_ol10_roll1", "OL10_R1", 10,  21, ""),
    ("s2_ol10_a2",   "OL10A2",  10,  15, ""),
    ("s2_ol10n_a1",  "OL10_NA1", -10, 18, "rope catch, contaminated (325.28) -- shown, not scored"),
    ("s2_ol10n_a2",  "OL10_NA2", -10, 19, ""),
    ("s2_ol15_a1",   "OL15_A1", 15,  12, ""),
    ("s2_ol15n_a1",  "OL15_NA1", -15, 13, ""),
]


# ----------------------------------------------------------------------------- bag side
def bag_strides(db):
    con = sqlite3.connect("file:%s?mode=ro" % db, uri=True)
    T = {n: (i, get_message(t)) for i, n, t in con.execute("SELECT id,name,type FROM topics")}

    def rd(n):
        i, M = T[n]
        return [(ts * 1e-9, deserialize_message(d, M))
                for ts, d in con.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (i,))]

    trig = rd("/trigger"); pairs = []; on = None
    for ts, m in trig:
        if m.enable and on is None:
            on = ts
        elif not m.enable and on is not None:
            pairs.append((on, ts)); on = None
    assert pairs, "no trigger pair in %s" % db
    T0, TOFF = max(pairs, key=lambda p: p[1] - p[0]); gait = TOFF - T0

    ms = rd("/motor/state"); mc = rd("/motor/command"); imu = rd("/imu")
    ts_ = np.array([t - T0 for t, _ in ms]); tc_ = np.array([t - T0 for t, _ in mc])
    ti = np.array([t - T0 for t, _ in imu])
    tH = {l: np.array([getattr(m, "module_" + l).torque_h for _, m in ms]) for l in L4}
    q = np.array([[m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z] for _, m in imu])
    w_, x_, y_, z_ = q.T
    roll = np.degrees(np.arctan2(2 * (w_ * x_ + y_ * z_), 1 - 2 * (x_ ** 2 + y_ ** 2)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w_ * y_ - z_ * x_), -1, 1)))
    az = np.array([m.linear_acceleration.z for _, m in imu])
    pre = (ti > 0.2) & (ti < 2.0)
    az = az - np.median(az[pre])

    kpr = np.array([m.module_a.kp_r for _, m in mc]); low = kpr < 100
    on_ = tc_[1:][(~low[:-1]) & low[1:]]
    on_all = on_.copy()
    on_ = on_[(on_ > T_FIRST) & (on_ < gait - 0.3)]

    rows = []
    for i in range(len(on_) - 1):
        a, b = on_[i], on_[i + 1]
        w = (ts_ >= a) & (ts_ < b); wi = (ti >= a) & (ti < b)
        assert w.sum() >= 20, "stride %d in %s has only %d motor samples" % (i + 1, db, w.sum())
        pk = {l: float(np.abs(tH[l][w]).max()) for l in L4}
        rows.append(dict(k=i + 1, t0=float(a), dur=float(b - a), tauH=pk,
                         roll=float(np.mean(roll[wi])) if wi.sum() > 3 else float("nan"),
                         pitch=float(np.mean(pitch[wi])) if wi.sum() > 3 else float("nan"),
                         roll_max=float(np.abs(roll[wi]).max()) if wi.sum() > 3 else float("nan")))
    edges_before = on_all[on_all <= T_FIRST]
    return dict(gait=float(gait), n_edges=int(len(on_)), strides=rows, ti=ti, az=az,
                imu_rate=float(1.0 / np.median(np.diff(ti))),
                first_edge=float(on_[0]), last_edge=float(on_[-1]),
                edge_before_first=float(edges_before[-1]) if len(edges_before) else float("nan"),
                edge_after_last=float(on_all[on_all >= gait - 0.3][0]) if (on_all >= gait - 0.3).any() else float("nan"))


def slope(xs, ys):
    xs = np.asarray(xs, float); ys = np.asarray(ys, float)
    g = ~np.isnan(ys)
    if g.sum() < 3:
        return float("nan")
    return float(np.polyfit(xs[g], ys[g], 1)[0])


# --------------------------------------------------------------------------- vicon side
def load_c3d(path):
    with open(path, "rb") as fh:
        r = c3d.Reader(fh)
        rate = float(r.point_rate)
        labels = [l.strip() for l in r.point_labels]
        frames = [np.array(p, dtype=float) for _, p, _ in r.read_frames()]
    P = np.stack(frames)
    return rate, labels, P[:, :, 0:3], P[:, :, 3] >= 0


def vicon_load(path):
    rate, labels, xyz, valid = load_c3d(path)
    T = xyz.shape[0]; t = np.arange(T) / rate
    idx = [labels.index(b) for b in BODY if b in labels]
    assert len(idx) >= 4, "only %d body markers in %s" % (len(idx), path)
    spread = [np.nanmean(np.nanstd(np.where(valid[:, idx], xyz[:, idx, a], np.nan), axis=1)) for a in range(3)]
    up = int(np.argmin(spread)); horiz = [a for a in (0, 1, 2) if a != up]
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
    gap = int(0.4 * rate); merged = []
    for s, e in zip(starts, ends):
        if merged and s - merged[-1][1] <= gap:
            merged[-1][1] = e
        else:
            merged.append([s, e])
    i0, i1 = max(merged, key=lambda p: p[1] - p[0])

    pos = cen[i0:i1][:, horiz] / 1000.0; tw = t[i0:i1]
    good = ~np.isnan(pos).any(1); pos = pos[good]; tw = tw[good]
    v_whole = float(np.linalg.norm(pos[-1] - pos[0]) / (tw[-1] - tw[0]))

    z = cen[:, up].copy() / 1000.0; bad = np.isnan(z)
    z[bad] = np.interp(t[bad], t[~bad], z[~bad])
    k = max(3, int(0.04 * rate))
    zs = np.convolve(z, np.ones(k) / k, mode="same")
    dzs = np.gradient(zs) * rate
    seg = dzs[i0:i1]
    cross = np.where((seg[:-1] < 0) & (seg[1:] >= 0))[0] + i0
    kept = []
    for c in cross:
        if kept and (c - kept[-1]) < MIN_HOP_S * rate:
            continue
        kept.append(int(c))
    hops = []
    for j, (a, b) in enumerate(zip(kept[:-1], kept[1:])):
        p = cen[a:b][:, horiz] / 1000.0; g = ~np.isnan(p).any(1)
        ii = np.where(g)[0]
        if len(ii) < 10:
            hops.append(dict(k=j + 1, t0=float(t[a]), dur=float((b - a) / rate), v=float("nan")))
            continue
        disp = float(np.linalg.norm(p[ii[-1]] - p[ii[0]])); dur = (ii[-1] - ii[0]) / rate
        hops.append(dict(k=j + 1, t0=float(t[a]), dur=float((b - a) / rate), v=float(disp / dur)))
    return dict(rate=rate, t=t, z=z, cen_h=cen[:, horiz] / 1000.0, t_win=(float(t[i0]), float(t[i1 - 1])),
                win_s=float((i1 - i0) / rate), v_whole=v_whole, hops=hops)


def scan_lags(ta, a, tb, b, lags, min_overlap):
    """Normalised correlation of a(t_a) against b(t_b) with t_a = t_b + lag, for each lag,
    by direct interpolation -- no FFT index convention to get wrong."""
    out = np.full(len(lags), np.nan)
    for i, L in enumerate(lags):
        tq = tb + L
        m = (tq >= ta[0]) & (tq <= ta[-1])
        if m.sum() < min_overlap:
            continue
        aa = np.interp(tq[m], ta, a); bb = b[m]
        aa = aa - aa.mean(); bb = bb - bb.mean()
        out[i] = float(np.dot(aa, bb) / (np.linalg.norm(aa) * np.linalg.norm(bb) + 1e-12))
    return out


def align_clocks(B, V, period=0.265):
    """lag_t such that t_bag = t_vicon + lag_t.

    Both signals are restricted to their own gait windows first (the captures hold the
    walk-back, the bags hold handling). A 0.25 s RMS envelope scanned at 20 ms gives the
    coarse lag with no periodic ambiguity; the raw signals scanned at 2 ms within half a
    stride refine it. The IMU sign is pinned: standing reads +g (l_trend's flight test
    relies on it), so az - az0 has the sign of the Vicon second derivative.
    Returns lag, coarse lag, the raw peak's ratio to the best peak one period away, and
    the normalised correlation r at the chosen lag.
    """
    rate = V["rate"]
    k = max(3, int(0.02 * rate))
    box = np.ones(k) / k
    tg = np.arange(0.0, B["gait"] + 1.5, 1.0 / rate)
    azg = np.interp(tg, B["ti"], B["az"])
    azg = np.convolve(np.convolve(azg, box, mode="same"), box, mode="same")   # band-match the Vicon path
    # smooth and differentiate the FULL capture, then cut: 'same' convolution zero-pads, and a
    # step at a windowed segment's end becomes a spurious spike in the second derivative that
    # an envelope will lock onto (this bit the synthetic self-test by exactly one period)
    t = V["t"]
    zs = np.convolve(V["z"] - np.nanmean(V["z"]), box, mode="same")
    vz = np.convolve(np.gradient(zs) * rate, box, mode="same")
    azv_full = np.gradient(vz) * rate
    m = (t >= V["t_win"][0] - 1.5) & (t <= V["t_win"][1] + 1.5)
    tv = t[m]; azv = azv_full[m]

    def env(x):
        w = int(0.25 * rate)
        return np.sqrt(np.convolve(x ** 2, np.ones(w) / w, mode="same"))
    ea, ev = env(azg), env(azv)
    n_min = int(0.5 * min(len(tg), len(tv)))
    # Two non-periodic physical priors on the lag: the template starts at T_FIRST in every run
    # (every first edge is at 2.76-2.79 s), so the Vicon hop window must start in
    # [T_FIRST-0.6, T_FIRST+0.3] of bag time; and hopping cannot end before trigger-off nor
    # more than 1.2 s after it. Both are enforced on the coarse grid; a periodic side lobe
    # cannot satisfy them.
    lo = max(T_FIRST - 0.6 - V["t_win"][0], B["gait"] - 0.1 - V["t_win"][1])
    hi = min(T_FIRST + 0.3 - V["t_win"][0], B["gait"] + 1.2 - V["t_win"][1])
    assert hi > lo, "no lag satisfies both the gait-start and trigger-off priors (lo %.2f hi %.2f)" % (lo, hi)
    coarse_grid = np.arange(lo, hi, 0.01)
    rc = scan_lags(tg, ea, tv, ev, coarse_grid, n_min)
    assert np.isfinite(rc).any(), "no overlap on the coarse grid"
    coarse = float(coarse_grid[int(np.nanargmax(rc))])
    # coarse margin: envelope correlation at the chosen lag vs the best one period either side
    cbest = float(np.nanmax(rc)); cs = []
    for off in (-period, period):
        ms = np.abs(coarse_grid - (coarse + off)) <= 0.03
        cs.append(float(np.nanmax(rc[ms])) if ms.any() and np.isfinite(rc[ms]).any() else float("nan"))
    coarse_margin = cbest - np.nanmax(cs) if np.isfinite(cs).any() else float("nan")
    fine_grid = np.arange(coarse - period / 2.0, coarse + period / 2.0, 0.002)
    rf = scan_lags(tg, azg, tv, azv, fine_grid, n_min)
    j = int(np.nanargmax(rf)); lag = float(fine_grid[j]); r = float(rf[j])
    side_grid = np.concatenate([np.arange(lag - period - 0.03, lag - period + 0.03, 0.002),
                                np.arange(lag + period - 0.03, lag + period + 0.03, 0.002)])
    rs = scan_lags(tg, azg, tv, azv, side_grid, n_min)
    side = float(np.nanmax(rs)) if np.isfinite(rs).any() else float("-inf")
    return dict(lag=lag, coarse=coarse, ratio=r / side if side > 0 else float("inf"), r=r,
                coarse_margin=float(coarse_margin), lo=float(lo), hi=float(hi))


def _selftest_align():
    """A known shift must be recovered: pulse train b, a = same train seen 0.371 s later."""
    rate = 500.0
    tb = np.arange(0, 12.0, 1 / rate)
    b = np.zeros_like(tb)
    for c in np.arange(2.2, 8.7, 0.265):
        b += 8.0 * np.exp(-0.5 * ((tb - c) / 0.012) ** 2) - 9.8 * ((tb > c + 0.11) & (tb < c + 0.265))
    ta = np.arange(0, 14.0, 1 / rate); a = np.interp(ta - 0.371, tb, b)
    Bf = dict(gait=9.1, ti=ta, az=a)                      # trigger-off 9.1 s: last pulse lands at 9.07
    Vf = dict(rate=rate, t=tb, z=None, t_win=(2.2, 8.7))  # window starts 2.571 s bag time, inside the prior
    # feed the second derivative path a signal whose 2nd derivative is b: integrate twice
    v = np.cumsum(b) / rate; z = np.cumsum(v) / rate
    Vf["z"] = z
    got = align_clocks(Bf, Vf)["lag"]
    assert abs(got - 0.371) < 0.004, "alignment self-test failed: got %.3f, expected 0.371" % got
    return got


print("alignment self-test: recovered lag %.3f s (expected 0.371)" % _selftest_align())


def edge_phase(B):
    """IMU vertical acceleration (minus standing) in the 30 ms before and after each stride edge:
    flight reads about -9.8, stance impact reads positive -- says whether the kp_r edge is
    touchdown (flight -> stance) or liftoff (stance -> flight). Bag-only, no Vicon needed."""
    ti, az = B["ti"], B["az"]; before = []; after = []
    for r in B["strides"]:
        e = r["t0"]
        wb = (ti >= e - 0.03) & (ti < e); wa = (ti >= e) & (ti < e + 0.03)
        if wb.sum() > 3 and wa.sum() > 3:
            before.append(float(np.mean(az[wb]))); after.append(float(np.mean(az[wa])))
    return float(np.median(before)), float(np.median(after))


def vicon_speed_over(V, ta, tb):
    """net horizontal displacement / duration of the Vicon centroid over [ta, tb) in VICON time."""
    t = V["t"]; m = (t >= ta) & (t < tb)
    p = V["cen_h"][m]; tt = t[m]; g = ~np.isnan(p).any(1)
    if g.sum() < 10:
        return float("nan")
    ii = np.where(g)[0]
    return float(np.linalg.norm(p[ii[-1]] - p[ii[0]]) / (tt[ii[-1]] - tt[ii[0]]))


# ----------------------------------------------------------------------------- main
results = []
for bag, vic, lam, n_ctl, note in RUNS:
    db = os.path.join(ROOT, "bags", bag, bag + "_0.db3")
    c3 = os.path.join(ROOT, "vicon", vic + ".c3d")
    B = bag_strides(db); V = vicon_load(c3)
    AL = align_clocks(B, V)
    lag = AL["lag"]
    S = B["strides"]; n = len(S)
    az_before, az_after = edge_phase(B)
    # after alignment: offset from each stride edge to the nearest Vicon trajectory bottom
    bott = np.array([h["t0"] + lag for h in V["hops"]])
    offs = [float(np.min(np.abs(bott - r["t0"]))) for r in S] if len(bott) else []
    bott_off_med = float(np.median(offs)) if offs else float("nan")
    bott_off_max = float(np.max(offs)) if offs else float("nan")

    # per-stride Vicon speed over the bag's own windows (machine clock)
    for r in S:
        r["v"] = vicon_speed_over(V, r["t0"] - lag, r["t0"] + r["dur"] - lag)
    # also the partial first window [2.6 s, first edge) and the last [last edge, TOFF)
    v_pre = vicon_speed_over(V, T_FIRST - lag, B["first_edge"] - lag)
    v_post = vicon_speed_over(V, B["last_edge"] - lag, B["gait"] - lag)

    N_sat = None; sat_leg = None; sat_val = None
    for r in S:
        l = max(L4, key=lambda q: r["tauH"][q])
        if r["tauH"][l] >= TAU_SAT:
            N_sat, sat_leg, sat_val = r["k"], l, r["tauH"][l]; break
    n_sat_strides = sum(1 for r in S if max(r["tauH"].values()) >= TAU_SAT)
    legmax = {l: max(r["tauH"][l] for r in S) for l in L4}
    N_sat_sust = None
    for r in S:
        if all(max(rr["tauH"].values()) >= TAU_SAT for rr in S if rr["k"] >= r["k"]):
            N_sat_sust = r["k"]; break

    ks = [r["k"] for r in S]; ro = [r["roll"] for r in S]; pi = [r["pitch"] for r in S]
    pre = S if N_sat is None else [r for r in S if r["k"] <= N_sat]
    sl_roll_pre = slope([r["k"] for r in pre], [r["roll"] for r in pre])
    sl_pitch_pre = slope([r["k"] for r in pre], [r["pitch"] for r in pre])
    sl_roll_all = slope(ks, ro); sl_pitch_all = slope(ks, pi)
    roll_abs_max = max(r["roll_max"] for r in S)

    vs = np.array([r["v"] for r in S])
    N_floor = None
    for r in S:
        if not np.isnan(r["v"]) and r["v"] < FLOOR:
            N_floor = r["k"]; break
    n_below = int(np.sum(vs[~np.isnan(vs)] < FLOOR))
    med_v = float(np.nanmedian(vs))
    # sustained-below variant (exploratory): first stride from which every later stride is below
    N_floor_sust = None
    for r in S:
        if all((not np.isnan(rr["v"])) and rr["v"] < FLOOR for rr in S if rr["k"] >= r["k"]):
            N_floor_sust = r["k"]; break

    H = V["hops"]; hv = np.array([h["v"] for h in H])
    for h in H:
        h["t_bag"] = h["t0"] + lag

    R = dict(bag=bag, vicon=vic, lam=lam, note=note, n_ctl=n_ctl,
             n_strides=n, gait_s=B["gait"], gait_after_first=B["gait"] - T_FIRST,
             first_edge=B["first_edge"], last_edge=B["last_edge"], edge_before_first=B["edge_before_first"],
             edge_after_last=B["edge_after_last"], imu_rate=B["imu_rate"],
             stride_dur_med=float(np.median([r["dur"] for r in S])),
             lag=lag, lag_coarse=AL["coarse"], lag_ratio=AL["ratio"], lag_r=AL["r"],
             coarse_margin=AL["coarse_margin"], lag_lo=AL["lo"], lag_hi=AL["hi"],
             az_before=az_before, az_after=az_after, bott_off_med=bott_off_med, bott_off_max=bott_off_max,
             win_bag=(V["t_win"][0] + lag, V["t_win"][1] + lag),
             N_sat=N_sat, sat_leg=sat_leg, sat_val=sat_val, n_sat_strides=n_sat_strides,
             N_sat_sust=N_sat_sust, legmax=legmax,
             sl_roll_pre=sl_roll_pre, sl_pitch_pre=sl_pitch_pre, n_pre=len(pre),
             sl_roll_all=sl_roll_all, sl_pitch_all=sl_pitch_all, roll_abs_max=roll_abs_max,
             roll_first=S[0]["roll"], roll_last=S[-1]["roll"], pitch_first=S[0]["pitch"], pitch_last=S[-1]["pitch"],
             v_whole=V["v_whole"], v_head=HEADLINE[vic], win_s=V["win_s"],
             v_pre=v_pre, v_post=v_post,
             N_floor=N_floor, N_floor_sust=N_floor_sust, n_below=n_below, med_v=med_v,
             n_hops=len(H), hop_med_v=float(np.nanmedian(hv)), hop_n_below=int(np.sum(hv[~np.isnan(hv)] < FLOOR)),
             strides=S, hops=H)
    results.append(R)

with open(OUT_JSON, "w") as fh:
    json.dump(results, fh, indent=1)

# ----------------------------------------------------------------------------- report
print("SELF-TEST: whole-arc speed recomputed with vicon_l0's estimator vs the logged headline")
for R in results:
    ok = abs(R["v_whole"] - R["v_head"]) < 0.002
    print("  %-9s recomputed %.3f  logged %.3f  %s" % (R["vicon"], R["v_whole"], R["v_head"], "ok" if ok else "MISMATCH"))
assert all(abs(R["v_whole"] - R["v_head"]) < 0.002 for R in results), "whole-arc self-test failed"

print("\nCLOCK ALIGNMENT (t_bag = t_vicon + lag; ratio = raw peak / best peak one stride away; r = normalised correlation at the lag)")
print("  run           ctl  windows  first edge  last edge  TOFF  | lag(s)  coarse [prior lo..hi] c-margin ratio    r  | hop window in bag time (end-TOFF) | edge-to-bottom med/max | az before/after edge")
for R in results:
    print("  %-13s %3d   %3d    %6.3f    %6.3f  %6.2f | %7.3f %7.3f [%6.2f..%6.2f] %+6.3f %5.2f  %5.2f | %6.2f .. %6.2f (%+5.2f) | %.3f / %.3f s | %+5.1f / %+5.1f"
          % (R["bag"], R["n_ctl"], R["n_strides"], R["first_edge"], R["last_edge"], R["gait_s"],
             R["lag"], R["lag_coarse"], R["lag_lo"], R["lag_hi"], R["coarse_margin"], R["lag_ratio"], R["lag_r"],
             R["win_bag"][0], R["win_bag"][1],
             R["win_bag"][1] - R["gait_s"], R["bott_off_med"], R["bott_off_max"], R["az_before"], R["az_after"]))
for R in results:
    assert T_FIRST - 0.6 <= R["win_bag"][0] <= T_FIRST + 0.3, "%s: hop window starts at %.2f s bag time" % (R["bag"], R["win_bag"][0])
    assert -0.1 <= R["win_bag"][1] - R["gait_s"] <= 1.2, "%s: hop window ends %.2f s after trigger-off" % (R["bag"], R["win_bag"][1] - R["gait_s"])

for R in results:
    print("\n=== %s / %s  lambda %+d  %s" % (R["bag"], R["vicon"], R["lam"], R["note"]))
    print("  partial window [2.6 s, first edge): v %.3f ;  [last edge, TOFF): v %.3f" % (R["v_pre"], R["v_post"]))
    print("  stride  t0(s)  |  tauH A    B     C     D  | roll   pitch  | v(m/s) machine clock")
    for s in R["strides"]:
        print("  %4d   %6.3f | %5.1f %5.1f %5.1f %5.1f | %+5.2f %+5.2f | %6.3f %s"
              % (s["k"], s["t0"], s["tauH"]["a"], s["tauH"]["b"], s["tauH"]["c"], s["tauH"]["d"], s["roll"], s["pitch"],
                 s["v"], "" if np.isnan(s["v"]) or s["v"] >= FLOOR else "<floor"))
    print("  hop split cross-check (bottom-to-bottom, t in bag clock): " +
          " ".join("%d@%.2f:%.3f" % (h["k"], h["t_bag"], h["v"]) for h in R["hops"]))
    print("  N_sat = %s (leg %s, %.1f N.m); strides >= 39: %d/%d; sustained from %s; per-leg max A %.1f B %.1f C %.1f D %.1f"
          % (R["N_sat"], NM.get(R["sat_leg"], "-"), R["sat_val"] or float("nan"), R["n_sat_strides"], R["n_strides"],
             R["N_sat_sust"], R["legmax"]["a"], R["legmax"]["b"], R["legmax"]["c"], R["legmax"]["d"]))
    print("  slopes over strides 1..N_sat (n=%d): droll %+.3f  dpitch %+.3f deg/stride | whole arc (n=%d): droll %+.3f dpitch %+.3f | |roll|max %.2f"
          % (R["n_pre"], R["sl_roll_pre"], R["sl_pitch_pre"], R["n_strides"], R["sl_roll_all"], R["sl_pitch_all"], R["roll_abs_max"]))
    print("  N_floor = %s (sustained from %s); strides below %.3f: %d/%d; median per-stride v %.3f; whole-arc %.3f | hop split: median %.3f, %d/%d below"
          % (R["N_floor"], R["N_floor_sust"], FLOOR, R["n_below"], R["n_strides"], R["med_v"], R["v_whole"],
             R["hop_med_v"], R["hop_n_below"], R["n_hops"]))

print("\nlambda_sustain per sign (whole-arc v >= %.3f), contaminated arc excluded:" % FLOOR)
for sign, name in ((1, "+lambda"), (-1, "-lambda")):
    cells = {}
    for R in results:
        if R["lam"] == 0 or np.sign(R["lam"]) != sign or R["note"]:
            continue
        cells.setdefault(abs(R["lam"]), []).append(R["v_whole"] >= FLOOR)
    for L in sorted(cells):
        print("  %s %2d deg: %d/%d arcs clear the floor" % (name, L, sum(cells[L]), len(cells[L])))
