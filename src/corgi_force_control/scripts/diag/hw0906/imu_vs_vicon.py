#!/usr/bin/env python3
"""IMU attitude estimator against Vicon truth, every run tonight with both a bag and a c3d.

Vicon: rigid-body attitude by Kabsch fit of B1..B6 against a reference frame near the hop
start. Body axes at the reference: fore = PCA long axis of the marker cloud, sign-aligned to
the direction of early travel (falls back to the travel direction itself if the cloud is
not elongated); up = lab vertical; left = up x fore. Reported in PHYSICAL terms:
   roll  + = left side up          (right-handed about the fore axis)
   pitch + = nose up
   yaw   + = left turn (CCW from above)
IMU: quaternion roll/pitch/yaw (ZYX, the same formulas l15_diag/l_trend/pitch_clamp use) and
the integrated gyro (cumtrapz of angular_velocity), raw sign, no bias removal.

The two clocks are not shared: the bag is aligned to Vicon by cross-correlating the body's
vertical velocity (Vicon centroid dz vs cumtrapz of IMU a_z, both high-passed). Once aligned,
the per-hop attitude oscillation (~10 deg p-p in pitch) gives an unambiguous SIGN and SCALE
test for each IMU channel against Vicon, independent of the small drifts.

Drift = mean(last third) - mean(first third) over the common hop window (vicon_pitch.py's
definition). Excess = IMU - Vicon pointwise on the aligned time base, smoothed 1 s, linear fit.
Usage: imu_vs_vicon.py out.json  (run list is hardcoded below)
"""
import sys, json, sqlite3
import numpy as np
import c3d
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

D = "/home/alexc/corgi_runs/hw_2026-09-06/bags"
V = "/home/alexc/corgi_runs/hw_2026-09-06/vicon"
BODY = ["B1", "B2", "B3", "B4", "B5", "B6"]

# bag, vicon, lambda (deg, signed; CL = closed loop), k_roll, note
RUNS = [
    ("s2_l0_a1", "L0_ramp_1", "0", 0.25, "λ0 baseline set"),
    ("s2_l0_a2", "L0_ramp_2", "0", 0.25, "λ0 baseline set"),
    ("s2_l0_a3", "L0_ramp_3", "0", 0.25, "λ0 baseline set"),
    ("s2_l0_a4", "L0_ramp_4", "0", 0.25, "λ0 baseline set"),
    ("s2_l0_a5", "L0_ramp_5", "0", 0.25, "λ0 baseline set"),
    ("s2_l15_a1", "L15_1", "CL~13.4", 0.25, "closed loop, fails floor"),
    ("s2_l15_a2", "L15_2", "CL~13.4", 0.25, "closed loop, fails floor"),
    ("s2_l15_b1", "L15_B1", "CL~13.4", 0.25, "closed loop, k_lateral 3500"),
    ("s2_l10_a1", "L10_A1", "CL~13.4", 0.25, "closed loop, 8 strides"),
    ("s2_ol10_a1", "OL10_A1", "+10", 0.25, "open loop, decays"),
    ("s2_ol10_roll1", "OL10_R1", "+10", 0.60, "passes old floor"),
    ("s2_l0r_a1", "L0_RA1", "0", 0.60, "baseline 1"),
    ("s2_ol10n_a1", "OL10_NA1", "-10", 0.60, "rope catch"),
    ("s2_ol10n_a2", "OL10_NA2", "-10", 0.60, "clean"),
    ("s2_ol15_a1", "OL15_A1", "+15", 0.60, "fails floor"),
    ("s2_ol15n_a1", "OL15_NA1", "-15", 0.60, "passes"),
    ("s2_l0r_a2", "L0RA2", "0", 0.60, "baseline 2"),
    ("s2_ol10_a2", "OL10A2", "+10", 0.60, "repeat, fails"),
]


# ----------------------------------------------------------------------------- helpers
def movmean(x, n):
    n = max(int(n), 1)
    k = np.ones(n) / n
    xp = np.pad(x, (n // 2, n - 1 - n // 2), mode="edge")
    return np.convolve(xp, k, mode="valid")


def nan_interp(t, x):
    x = x.copy(); g = ~np.isnan(x)
    if g.sum() < 2:
        return x
    x[~g] = np.interp(t[~g], t[g], x[g])
    return x


def cumtrapz(y, t):
    out = np.zeros_like(y)
    out[1:] = np.cumsum(0.5 * (y[1:] + y[:-1]) * np.diff(t))
    return out


def thirds(v):
    n = len(v); h = n // 3
    if h < 5:
        return float("nan"), float("nan")
    f = v[:h].mean(); m = v[h:2 * h].mean(); l = v[2 * h:].mean()
    return float(l - f), float(m - f)


def hop_window(t, dz, rate):
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
    return max(merged, key=lambda p: p[1] - p[0])


# ----------------------------------------------------------------------------- Vicon
def vicon_attitude(path):
    with open(path, "rb") as fh:
        r = c3d.Reader(fh)
        rate = float(r.point_rate)
        labels = [l.strip() for l in r.point_labels]
        frames = [np.array(p, dtype=float) for _, p, _ in r.read_frames()]
    P = np.stack(frames); xyz = P[:, :, 0:3]; valid = P[:, :, 3] >= 0
    T = xyz.shape[0]; t = np.arange(T) / rate
    idx = [labels.index(b) for b in BODY if b in labels]
    spread = [np.nanmean(np.nanstd(np.where(valid[:, idx], xyz[:, idx, a], np.nan), axis=1)) for a in range(3)]
    up = int(np.argmin(spread))
    horiz = [a for a in (0, 1, 2) if a != up]
    cen = np.full((T, 3), np.nan)
    for a in range(3):
        cen[:, a] = np.nanmean(np.where(valid[:, idx], xyz[:, idx, a], np.nan), axis=1)
    ok = ~np.isnan(cen).any(1)
    assert np.nanmean(cen[:, up]) > 0, "expected the up axis to be positive (mm above floor)"
    dz = np.gradient(np.where(ok, cen[:, up], np.nan)) * rate / 1000.0     # m/s, NaN where missing
    i0, i1 = hop_window(t, dz, rate)

    # reference frame: best-seen frame within [hop start - 1 s, hop start + 0.5 s]
    nval = valid[:, idx].sum(1)
    lo, hi = max(0, i0 - int(rate)), min(T, i0 + int(0.5 * rate))
    ref_t = lo + int(np.argmax(nval[lo:hi]))
    ref_mask = valid[ref_t, idx]
    ref = xyz[ref_t][idx][ref_mask]
    # body axes at the reference
    upv = np.zeros(3); upv[up] = 1.0
    Hc = ref[:, horiz] - ref[:, horiz].mean(0)
    w, vec = np.linalg.eigh(Hc.T @ Hc)
    elong = float(np.sqrt(w[1] / max(w[0], 1e-9)))
    pca = np.zeros(3); pca[horiz[0]] = vec[0, 1]; pca[horiz[1]] = vec[1, 1]
    # early travel direction: centroid displacement over the first 1.5 s of hopping
    j0, j1 = i0, min(i1, i0 + int(1.5 * rate))
    seg = cen[j0:j1]; g = ~np.isnan(seg).any(1)
    trav = seg[g][-1] - seg[g][0]; trav[up] = 0.0; trav /= np.linalg.norm(trav)
    if elong >= 1.3:
        fore = pca if pca @ trav > 0 else -pca
        fore_method = "pca"
    else:
        fore = trav.copy(); fore_method = "travel"
    fore /= np.linalg.norm(fore)
    left = np.cross(upv, fore)
    ang_pca_travel = float(np.degrees(np.arccos(np.clip(abs(pca @ trav), -1, 1))))

    a0, a1 = max(0, i0 - int(1.5 * rate)), min(T, i1 + int(1.5 * rate))
    roll = np.full(T, np.nan); pitch = np.full(T, np.nan); yaw = np.full(T, np.nan); tilt_old = np.full(T, np.nan)
    B0 = ref - ref.mean(0)
    for tt in range(a0, a1):
        m = valid[tt, idx] & ref_mask
        if m.sum() < 4:
            continue
        A = xyz[tt][idx][m]; B = xyz[ref_t][idx][m]
        A = A - A.mean(0); B = B - B.mean(0)
        U, S, Vt = np.linalg.svd(B.T @ A)
        dd = np.sign(np.linalg.det(Vt.T @ U.T))
        R = Vt.T @ np.diag([1, 1, dd]) @ U.T          # maps reference cloud -> current cloud
        ex = R @ fore; ey = R @ left
        pitch[tt] = np.degrees(np.arcsin(np.clip(ex @ upv, -1, 1)))
        roll[tt] = np.degrees(np.arcsin(np.clip(ey @ upv, -1, 1)))
        exh = ex.copy(); exh[up] = 0.0
        yaw[tt] = np.degrees(np.arctan2(np.cross(fore, exh) @ upv, fore @ exh))
        tilt_old[tt] = np.degrees(np.arcsin(np.clip(R[up, horiz[0]], -1, 1)))   # vicon_pitch.py's quantity
    g = ~np.isnan(yaw)
    yaw[g] = np.degrees(np.unwrap(np.radians(yaw[g])))
    return dict(t=t, rate=rate, i0=i0, i1=i1, roll=roll, pitch=pitch, yaw=yaw, tilt_old=tilt_old,
                dz=dz, up=up, fore_method=fore_method, elong=elong, ang_pca_travel=ang_pca_travel,
                nfit=int((~np.isnan(pitch[i0:i1])).sum()), nwin=int(i1 - i0), ref_t=ref_t)


# ----------------------------------------------------------------------------- bag
def bag_imu(path):
    con = sqlite3.connect("file:%s?mode=ro" % path, uri=True)
    T = {n: (i, get_message(ty)) for i, n, ty in con.execute("SELECT id,name,type FROM topics")}

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
    T0, TOFF = max(pairs, key=lambda p: p[1] - p[0]); gait = TOFF - T0
    imu = rd("/imu")
    t = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, m in imu])
    q = np.array([[m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z] for _, m in imu])
    g = np.degrees(np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for _, m in imu]))
    a = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for _, m in imu])
    w_, x_, y_, z_ = q.T
    roll = np.degrees(np.arctan2(2 * (w_ * x_ + y_ * z_), 1 - 2 * (x_ ** 2 + y_ ** 2)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w_ * y_ - z_ * x_), -1, 1)))
    yaw = np.degrees(np.unwrap(np.arctan2(2 * (w_ * z_ + x_ * y_), 1 - 2 * (y_ ** 2 + z_ ** 2))))
    mc = rd("/motor/command")
    tc = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, m in mc])
    kpr = np.array([m.module_a.kp_r for _, m in mc])
    if not np.all(np.diff(tc) >= 0):
        o = np.argsort(tc); tc = tc[o]; kpr = kpr[o]
    low = kpr < 100
    onsets = tc[1:][(~low[:-1]) & low[1:]]                       # stance onsets (l_trend's stride clock)
    pre = (t - T0 > -2.0) & (t - T0 < -0.2)
    return dict(t=t, T0=T0, gait=gait, roll=roll, pitch=pitch, yaw=yaw, g=g, az=a[:, 2],
                gbias_pre=g[pre].mean(0).tolist() if pre.sum() > 100 else [float("nan")] * 3,
                gstd_pre=g[pre].std(0).tolist() if pre.sum() > 100 else [float("nan")] * 3,
                onsets=onsets)


# ----------------------------------------------------------------------------- align
def align(vc, bg):
    """Return lag such that t_bag = t_vicon + lag, plus diagnostics."""
    rate = vc["rate"]; tv = vc["t"]
    v_hp = nan_interp(tv, vc["dz"]); v_hp = v_hp - movmean(v_hp, rate)
    tb = bg["t"]; vz = cumtrapz(bg["az"], tb)
    grid = np.arange(tb[0], tb[-1], 1.0 / rate)
    b = np.interp(grid, tb, vz); b_hp = b - movmean(b, rate)
    i0, i1 = vc["i0"], vc["i1"]
    a0, a1 = max(0, i0 - int(1.0 * rate)), min(len(tv), i1 + int(1.0 * rate))
    x = v_hp[a0:a1]; x = (x - x.mean()) / (x.std() + 1e-12)
    lag0 = bg["T0"] - tv[i0]                                   # coarse: trigger vs Vicon hop onset
    lags = np.arange(lag0 - 2.0, lag0 + 2.0, 1.0 / rate)
    rr = np.full(len(lags), -np.inf)
    for k, L in enumerate(lags):
        j0 = int(round((tv[a0] + L - grid[0]) * rate))
        if j0 < 0 or j0 + len(x) > len(b_hp):
            continue
        y = b_hp[j0:j0 + len(x)]; y = (y - y.mean()) / (y.std() + 1e-12)
        rr[k] = float(np.mean(x * y))
    k = int(np.argmax(rr)); lag = float(lags[k]); rpk = float(rr[k])
    # second best at least 0.15 s away (one stride is 0.264 s)
    far = np.abs(lags - lag) > 0.15
    r2 = float(rr[far].max()) if far.any() else float("nan")
    return lag, rpk, r2


# ----------------------------------------------------------------------------- per run
def hp(x, n):
    return x - movmean(x, n)


def analyse(bag, vic, lam, kroll, note):
    vc = vicon_attitude("%s/%s.c3d" % (V, vic))
    bg = bag_imu("%s/%s/%s_0.db3" % (D, bag, bag))
    lag, rpk, r2 = align(vc, bg)
    rate = vc["rate"]; tv = vc["t"]; i0, i1 = vc["i0"], vc["i1"]
    # common window in Vicon time: Vicon hop window ∩ bag trigger window
    w0 = max(tv[i0], bg["T0"] - lag); w1 = min(tv[i1 - 1], bg["T0"] + bg["gait"] - lag)
    dur = w1 - w0
    out = dict(bag=bag, vicon=vic, lam=lam, kroll=kroll, note=note, lag=lag, r_align=rpk, r_align_2nd=r2,
               vicon_window=[float(tv[i0]), float(tv[i1 - 1])], common_window_s=float(dur),
               fore_method=vc["fore_method"], elong=vc["elong"], ang_pca_travel=vc["ang_pca_travel"],
               vicon_fit_frac=vc["nfit"] / max(vc["nwin"], 1), gbias_pre=vc and bg["gbias_pre"], gstd_pre=bg["gstd_pre"],
               imu_drop_frac=float(np.mean(np.diff(bg["t"]) > 0.0015)))
    # Vicon series in the common window
    mv = (tv >= w0) & (tv <= w1)
    tvw = tv[mv]
    # IMU series in the common window (bag time -> Vicon time)
    tb = bg["t"] - lag
    mb = (tb >= w0) & (tb <= w1)
    tbw = tb[mb]
    gint = {ax: cumtrapz(bg["g"][:, k], bg["t"]) for k, ax in enumerate(("roll", "pitch", "yaw"))}
    res = {}
    for ax in ("roll", "pitch", "yaw"):
        vser = vc[ax][mv]; gv = ~np.isnan(vser)
        v_third, v_mid = thirds(vser[gv])
        v_slope = float(np.polyfit(tvw[gv], vser[gv], 1)[0]) if gv.sum() > 100 else float("nan")
        qser = bg[ax][mb]
        q_third, q_mid = thirds(qser)
        q_slope = float(np.polyfit(tbw, qser, 1)[0])
        gser = gint[ax][mb]
        g_third, g_mid = thirds(gser)
        g_slope = float(np.polyfit(tbw, gser, 1)[0])
        # aligned per-hop comparison: interpolate Vicon onto the IMU times, high-pass 0.5 s (hop 0.264 s)
        vi = np.interp(tbw, tvw[gv], vser[gv])
        nb = int(0.5 * 1000)     # IMU ~1 kHz
        v_h = hp(vi, nb); q_h = hp(qser, nb); g_h = hp(gser, nb)
        def rs(a, b):
            r = float(np.corrcoef(a, b)[0, 1]); s = float(np.polyfit(a, b, 1)[0]); return r, s
        rq, sq = rs(v_h, q_h); rg, sg = rs(v_h, g_h)
        # excess IMU - Vicon pointwise (raw IMU sign), smoothed 1 s, linear fit
        def excess(iser, sgn):
            e = sgn * iser - vi; e = e - e[:nb].mean()
            es = movmean(e, 1000)
            A = np.polyfit(tbw, es, 1); fit = np.polyval(A, tbw)
            ss = np.sum((es - es.mean()) ** 2)
            r2 = 1 - np.sum((es - fit) ** 2) / ss if ss > 0 else float("nan")
            return float(A[0]), float(r2), float(es[-1] - es[0])
        sgn_q = 1.0 if rq >= 0 else -1.0
        sgn_g = 1.0 if rg >= 0 else -1.0
        eq_slope, eq_r2, eq_tot = excess(qser, sgn_q)
        eg_slope, eg_r2, eg_tot = excess(gser, sgn_g)
        res[ax] = dict(vicon_drift=v_third, vicon_mid=v_mid, vicon_slope=v_slope,
                       quat_drift=q_third, quat_mid=q_mid, quat_slope=q_slope,
                       gyro_drift=g_third, gyro_mid=g_mid, gyro_slope=g_slope,
                       hop_r_quat=rq, hop_scale_quat=sq, hop_r_gyro=rg, hop_scale_gyro=sg,
                       hop_amp_vicon=float(v_h.std()), hop_amp_quat=float(q_h.std()), hop_amp_gyro=float(g_h.std()),
                       excess_quat_slope=eq_slope, excess_quat_r2=eq_r2, excess_quat_total=eq_tot,
                       excess_gyro_slope=eg_slope, excess_gyro_r2=eg_r2, excess_gyro_total=eg_tot)
    # reproduction checks: vicon_pitch.py's tilt over ITS window, l_trend's stride window for the quaternion pitch
    to = vc["tilt_old"][i0:i1]; go = ~np.isnan(to)
    out["tilt_old_drift"] = thirds(to[go])[0]
    on = bg["onsets"]; on = on[(on - bg["T0"] > 2.6) & (on - bg["T0"] < bg["gait"] - 0.3)]
    if len(on) >= 6:
        tt = bg["t"]; per = []
        for i in range(len(on) - 1):
            wi = (tt >= on[i]) & (tt < on[i + 1])
            if wi.sum() > 3:
                per.append(bg["pitch"][wi].mean())
        per = np.array(per); n = len(per); h3 = n // 3; h2 = n // 2
        out["ltrend_pitch_thirds"] = float(per[2 * h3:].mean() - per[:h3].mean())
        out["ltrend_pitch_halves"] = float(per[h2:].mean() - per[:h2].mean())
        out["ltrend_pitch_lastfirst"] = float(per[-1] - per[0])
        out["n_strides"] = int(n)
    else:
        out["ltrend_pitch_thirds"] = float("nan"); out["ltrend_pitch_halves"] = float("nan")
        out["ltrend_pitch_lastfirst"] = float("nan"); out["n_strides"] = int(len(on))
    out["axes"] = res
    return out


def fmt(x, w=6, p=2):
    return ("%" + str(w) + "." + str(p) + "f") % x if np.isfinite(x) else "%*s" % (w, "nan")


results = []
for bag, vic, lam, kroll, note in RUNS:
    try:
        r = analyse(bag, vic, lam, kroll, note)
    except Exception as e:
        print("!! %s / %s failed: %s: %s" % (bag, vic, type(e).__name__, e)); continue
    results.append(r)
    print("== %-14s %-9s λ %-7s k_roll %.2f  %s" % (bag, vic, lam, kroll, note))
    print("   align lag %+8.3f s  r %.3f (2nd-best off-stride %.3f)   common window %.2f s (Vicon %.2f-%.2f)   fore=%s elong %.2f pca-vs-travel %.1f°  fit %.0f%%  imu drop %.1f%%"
          % (r["lag"], r["r_align"], r["r_align_2nd"], r["common_window_s"], r["vicon_window"][0], r["vicon_window"][1],
             r["fore_method"], r["elong"], r["ang_pca_travel"], 100 * r["vicon_fit_frac"], 100 * r["imu_drop_frac"]))
    print("   pre-trigger gyro mean deg/s  x %+.3f y %+.3f z %+.3f   (std %.2f %.2f %.2f)" % tuple(r["gbias_pre"] + r["gstd_pre"]))
    print("   reproduction: vicon_pitch tilt_old drift %+.2f | l_trend quat pitch thirds %+.2f halves %+.2f last-first %+.2f (%d strides)"
          % (r["tilt_old_drift"], r["ltrend_pitch_thirds"], r["ltrend_pitch_halves"], r["ltrend_pitch_lastfirst"], r["n_strides"]))
    print("   axis   Vicon drift/slope    quat drift/slope    ∫gyro drift/slope   | hop r/scale quat   hop r/scale ∫gyro  amp V/Q/G | excess(quat) slope R² | excess(∫g) slope R²")
    for ax in ("roll", "pitch", "yaw"):
        a = r["axes"][ax]
        print("   %-6s %s° %s°/s   %s° %s°/s   %s° %s°/s  | %s %s        %s %s        %s/%s/%s | %s °/s %s   | %s °/s %s"
              % (ax, fmt(a["vicon_drift"]), fmt(a["vicon_slope"], 6, 3), fmt(a["quat_drift"]), fmt(a["quat_slope"], 6, 3),
                 fmt(a["gyro_drift"]), fmt(a["gyro_slope"], 6, 3),
                 fmt(a["hop_r_quat"], 5), fmt(a["hop_scale_quat"], 5), fmt(a["hop_r_gyro"], 5), fmt(a["hop_scale_gyro"], 5),
                 fmt(a["hop_amp_vicon"], 4, 1), fmt(a["hop_amp_quat"], 4, 1), fmt(a["hop_amp_gyro"], 4, 1),
                 fmt(a["excess_quat_slope"], 6, 3), fmt(a["excess_quat_r2"], 4), fmt(a["excess_gyro_slope"], 6, 3), fmt(a["excess_gyro_r2"], 4)))
    print()

with open(sys.argv[1], "w") as fh:
    json.dump(results, fh, indent=1, default=float)
print("wrote", sys.argv[1], len(results), "runs")
