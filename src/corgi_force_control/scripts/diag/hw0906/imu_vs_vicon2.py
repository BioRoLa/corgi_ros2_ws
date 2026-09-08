#!/usr/bin/env python3
"""IMU attitude estimator against Vicon truth, every run tonight with both a bag and a c3d.
Second pass. Changes from imu_vs_vicon.py:
  * a gyro-only attitude track: the FLU body rates integrated as a quaternion (seeded with
    the IMU's own roll/pitch at the window start), Euler angles extracted in the same
    physical convention as Vicon. Raw per-axis integrals of body rates are NOT Euler-angle
    changes once the body yaws while pitched/rolled; this track is. It is what the fusion
    filter would output with the accelerometer correction switched off, so
       quaternion - gyro_only = what the fusion adds, and gyro_only - Vicon = the gyro's own error.
  * yaw sign taken from the drift level (|Vicon yaw| > 2 deg) rather than the per-hop wiggle.
  * diagnostics: lag vs coarse lag, whole-window yaw on each instrument's own clock, mean
    attitude and rate levels in the window, so the excess can be regressed on them.

Conventions (physical): roll + = left side up, pitch + = nose up, yaw + = left turn (CCW).
Vicon: Kabsch fit of B1..B5 vs a reference frame near the hop start; fore = PCA long axis
sign-aligned to early travel; up = lab vertical; left = up x fore.
IMU quaternion: ZYX Euler with the formulas l15_diag/l_trend/pitch_clamp use (raw sign kept;
the per-hop correlation with Vicon reports its physical sense).
Drift = mean(last third) - mean(first third) over the common hop window.
Usage: imu_vs_vicon2.py out.json
"""
import sys, json, sqlite3
import numpy as np
import c3d
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

D = "/home/alexc/corgi_runs/hw_2026-09-06/bags"
V = "/home/alexc/corgi_runs/hw_2026-09-06/vicon"
BODY = ["B1", "B2", "B3", "B4", "B5"]   # B6 dropped 2026-09-08: it IS rigid on the body
# (|B6-B1| 346.8 +- 0.4 mm) but is visible in only ~95% of in-window frames, and the
# varying-subset nanmean stepped the centroid by (B6-c5)/6 = 3.7 mm. B1-B5 are ~100%.

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


def hop_window(dz, rate):
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


def euler_zyx(q):
    """q = (w,x,y,z) rows, world<-body. Returns roll, pitch, yaw in deg (yaw unwrapped)."""
    w_, x_, y_, z_ = q.T
    roll = np.degrees(np.arctan2(2 * (w_ * x_ + y_ * z_), 1 - 2 * (x_ ** 2 + y_ ** 2)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w_ * y_ - z_ * x_), -1, 1)))
    yaw = np.degrees(np.unwrap(np.arctan2(2 * (w_ * z_ + x_ * y_), 1 - 2 * (y_ ** 2 + z_ ** 2))))
    return roll, pitch, yaw


def quat_from_euler(roll, pitch, yaw):
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])


def quat_mul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def integrate_gyro(t, g_rad, q0):
    """Body-rate quaternion integration, body FLU, q world<-body. g_rad (N,3) rad/s."""
    N = len(t); Q = np.zeros((N, 4)); q = q0 / np.linalg.norm(q0); Q[0] = q
    for k in range(1, N):
        dt = t[k] - t[k - 1]
        w = 0.5 * (g_rad[k] + g_rad[k - 1])
        n = np.linalg.norm(w)
        if n * dt > 1e-12:
            ax = w / n; h = 0.5 * n * dt
            dq = np.array([np.cos(h), ax[0] * np.sin(h), ax[1] * np.sin(h), ax[2] * np.sin(h)])
            q = quat_mul(q, dq); q /= np.linalg.norm(q)
        Q[k] = q
    return Q


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
    dz = np.gradient(np.where(ok, cen[:, up], np.nan)) * rate / 1000.0
    i0, i1 = hop_window(dz, rate)
    nval = valid[:, idx].sum(1)
    lo, hi = max(0, i0 - int(rate)), min(T, i0 + int(0.5 * rate))
    ref_t = lo + int(np.argmax(nval[lo:hi]))
    ref_mask = valid[ref_t, idx]
    ref = xyz[ref_t][idx][ref_mask]
    upv = np.zeros(3); upv[up] = 1.0
    Hc = ref[:, horiz] - ref[:, horiz].mean(0)
    w, vec = np.linalg.eigh(Hc.T @ Hc)
    elong = float(np.sqrt(w[1] / max(w[0], 1e-9)))
    pca = np.zeros(3); pca[horiz[0]] = vec[0, 1]; pca[horiz[1]] = vec[1, 1]
    j0, j1 = i0, min(i1, i0 + int(1.5 * rate))
    seg = cen[j0:j1]; g = ~np.isnan(seg).any(1)
    trav = seg[g][-1] - seg[g][0]; trav[up] = 0.0; trav /= np.linalg.norm(trav)
    if elong >= 1.3:
        fore = pca if pca @ trav > 0 else -pca; fore_method = "pca"
    else:
        fore = trav.copy(); fore_method = "travel"
    fore /= np.linalg.norm(fore)
    left = np.cross(upv, fore)
    ang_pca_travel = float(np.degrees(np.arccos(np.clip(abs(pca @ trav), -1, 1))))
    a0, a1 = max(0, i0 - int(1.5 * rate)), min(T, i1 + int(1.5 * rate))
    roll = np.full(T, np.nan); pitch = np.full(T, np.nan); yaw = np.full(T, np.nan); tilt_old = np.full(T, np.nan)
    for tt in range(a0, a1):
        m = valid[tt, idx] & ref_mask
        if m.sum() < 4:
            continue
        A = xyz[tt][idx][m]; B = xyz[ref_t][idx][m]
        A = A - A.mean(0); B = B - B.mean(0)
        U, S, Vt = np.linalg.svd(B.T @ A)
        dd = np.sign(np.linalg.det(Vt.T @ U.T))
        R = Vt.T @ np.diag([1, 1, dd]) @ U.T
        ex = R @ fore; ey = R @ left
        pitch[tt] = np.degrees(np.arcsin(np.clip(ex @ upv, -1, 1)))
        roll[tt] = np.degrees(np.arcsin(np.clip(ey @ upv, -1, 1)))
        exh = ex.copy(); exh[up] = 0.0
        yaw[tt] = np.degrees(np.arctan2(np.cross(fore, exh) @ upv, fore @ exh))
        tilt_old[tt] = np.degrees(np.arcsin(np.clip(R[up, horiz[0]], -1, 1)))
    g = ~np.isnan(yaw)
    yaw[g] = np.degrees(np.unwrap(np.radians(yaw[g])))
    return dict(t=t, rate=rate, i0=i0, i1=i1, roll=roll, pitch=pitch, yaw=yaw, tilt_old=tilt_old, dz=dz, up=up,
                fore_method=fore_method, elong=elong, ang_pca_travel=ang_pca_travel,
                nfit=int((~np.isnan(pitch[i0:i1])).sum()), nwin=int(i1 - i0))


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
    o = np.argsort(t, kind="stable"); t = t[o]; q = q[o]; g = g[o]; a = a[o]
    keep = np.concatenate(([True], np.diff(t) > 0)); t = t[keep]; q = q[keep]; g = g[keep]; a = a[keep]
    roll, pitch, yaw = euler_zyx(q)
    mc = rd("/motor/command")
    tc = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, m in mc])
    kpr = np.array([m.module_a.kp_r for _, m in mc])
    oc = np.argsort(tc, kind="stable"); tc = tc[oc]; kpr = kpr[oc]
    low = kpr < 100
    onsets = tc[1:][(~low[:-1]) & low[1:]]
    pre = (t - T0 > -2.0) & (t - T0 < -0.2)
    return dict(t=t, T0=T0, gait=gait, roll=roll, pitch=pitch, yaw=yaw, g=g, az=a[:, 2], acc=a,
                gbias_pre=g[pre].mean(0).tolist() if pre.sum() > 100 else [float("nan")] * 3,
                gstd_pre=g[pre].std(0).tolist() if pre.sum() > 100 else [float("nan")] * 3,
                onsets=onsets)


# ----------------------------------------------------------------------------- align
def align(vc, bg):
    """Coarse lag from the NON-PERIODIC launch edge (both instruments see the body go from
    static to hopping; the robot is static until ~2.5 s after the trigger), fine lag from the
    vertical-velocity cross-correlation within +-0.15 s of it. A plain +-2 s correlation search
    (first pass, 2026-09-06) locked onto a stride alias two strides early in every run because
    the true lag sat outside its range: r 0.78-0.95 at the alias vs 0.91-0.995 at the truth."""
    rate = vc["rate"]; tv = vc["t"]
    dz = nan_interp(tv, vc["dz"]); v_hp = dz - movmean(dz, rate)
    tb = bg["t"]; vz = cumtrapz(bg["az"], tb)
    grid = np.arange(tb[0], tb[-1], 1.0 / rate)
    b = np.interp(grid, tb, vz); b_hp = b - movmean(b, rate)
    i0, i1 = vc["i0"], vc["i1"]
    # Vicon sustained-motion onset: walk back from i0 while |dz| (0.1 s mean) stays above 0.05 m/s, then forward
    dzs = movmean(np.abs(dz), int(0.1 * rate)); j = i0
    while j > 0 and dzs[j] > 0.05:
        j -= 1
    while j < len(tv) - 1 and dzs[j] <= 0.05:
        j += 1
    t_v_on = tv[j]
    # IMU sustained-motion onset after the 20 ms jolt at the trigger
    m = (tb > bg["T0"] + 1.0) & (tb < bg["T0"] + 6.0)
    aa = movmean(np.abs(bg["az"][m]), 50)
    t_b_on = tb[m][int(np.argmax(aa > 1.0))]
    lag_edge = t_b_on - t_v_on
    a0, a1 = max(0, i0 - int(1.0 * rate)), min(len(tv), i1 + int(1.0 * rate))
    x = v_hp[a0:a1]; x = (x - x.mean()) / (x.std() + 1e-12)
    lags = np.arange(lag_edge - 0.15, lag_edge + 0.15, 1.0 / rate)
    rr = np.full(len(lags), -np.inf)
    for k, L in enumerate(lags):
        j0 = int(round((tv[a0] + L - grid[0]) * rate))
        if j0 < 0 or j0 + len(x) > len(b_hp):
            continue
        y = b_hp[j0:j0 + len(x)]; y = (y - y.mean()) / (y.std() + 1e-12)
        rr[k] = float(np.mean(x * y))
    k = int(np.argmax(rr)); lag = float(lags[k]); rpk = float(rr[k])
    # alias check: r one stride away on either side (should be clearly lower)
    def r_at(L):
        j0 = int(round((tv[a0] + L - grid[0]) * rate))
        if j0 < 0 or j0 + len(x) > len(b_hp):
            return float("nan")
        y = b_hp[j0:j0 + len(x)]; y = (y - y.mean()) / (y.std() + 1e-12)
        return float(np.mean(x * y))
    r2 = max(r_at(lag - 0.265), r_at(lag + 0.265))
    lag0 = bg["T0"] - tv[i0]
    return lag, rpk, r2, lag - lag0


def hp(x, n):
    return x - movmean(x, n)


# ----------------------------------------------------------------------------- per run
def analyse(bag, vic, lam, kroll, note):
    vc = vicon_attitude("%s/%s.c3d" % (V, vic))
    bg = bag_imu("%s/%s/%s_0.db3" % (D, bag, bag))
    lag, rpk, r2, dlag = align(vc, bg)
    tv = vc["t"]; i0, i1 = vc["i0"], vc["i1"]
    w0 = max(tv[i0], bg["T0"] - lag); w1 = min(tv[i1 - 1], bg["T0"] + bg["gait"] - lag)
    dur = w1 - w0
    out = dict(bag=bag, vicon=vic, lam=lam, kroll=kroll, note=note, lag=lag, dlag=float(dlag), r_align=rpk, r_align_2nd=r2,
               vicon_window=[float(tv[i0]), float(tv[i1 - 1])], bag_gait_s=float(bg["gait"]), common_window_s=float(dur),
               fore_method=vc["fore_method"], elong=vc["elong"], ang_pca_travel=vc["ang_pca_travel"],
               vicon_fit_frac=vc["nfit"] / max(vc["nwin"], 1), gbias_pre=bg["gbias_pre"], gstd_pre=bg["gstd_pre"],
               imu_drop_frac=float(np.mean(np.diff(bg["t"]) > 0.0015)))
    mv = (tv >= w0) & (tv <= w1); tvw = tv[mv]
    tb = bg["t"] - lag
    mb = (tb >= w0) & (tb <= w1); tbw = tb[mb]
    # gyro-only attitude, seeded with the IMU quaternion's own roll/pitch over the first 0.3 s of the window
    # (quaternion pitch is nose-up + as measured against Vicon; FLU pitch is nose-down +, so negate)
    seed = mb & (tb < w0 + 0.3)
    r0 = np.radians(bg["roll"][seed].mean()); p0 = -np.radians(bg["pitch"][seed].mean())
    q0 = quat_from_euler(r0, p0, 0.0)
    Qg = integrate_gyro(bg["t"][mb], np.radians(bg["g"][mb]), q0)
    g_roll, g_pitch_flu, g_yaw = euler_zyx(Qg)
    gyro_only = dict(roll=g_roll, pitch=-g_pitch_flu, yaw=g_yaw)          # physical convention
    raw_int = {ax: cumtrapz(bg["g"][mb, k], bg["t"][mb]) for k, ax in enumerate(("roll", "pitch", "yaw"))}
    # window levels for regression
    out["gy_rms"] = float(np.sqrt(np.mean(bg["g"][mb, 1] ** 2))); out["gx_rms"] = float(np.sqrt(np.mean(bg["g"][mb, 0] ** 2)))
    out["gz_mean"] = float(bg["g"][mb, 2].mean()); out["gy_mean"] = float(bg["g"][mb, 1].mean())
    out["quat_pitch_mean"] = float(bg["pitch"][mb].mean()); out["quat_roll_mean"] = float(bg["roll"][mb].mean())
    out["az_p99"] = float(np.percentile(bg["az"][mb], 99)); out["az_p01"] = float(np.percentile(bg["az"][mb], 1))
    out["ax_mean"] = float(bg["acc"][mb, 0].mean()); out["ay_mean"] = float(bg["acc"][mb, 1].mean())
    res = {}
    for ax in ("roll", "pitch", "yaw"):
        vser = vc[ax][mv]; gv = ~np.isnan(vser)
        v_third, v_mid = thirds(vser[gv])
        v_slope = float(np.polyfit(tvw[gv], vser[gv], 1)[0]) if gv.sum() > 100 else float("nan")
        vi = np.interp(tbw, tvw[gv], vser[gv])
        nb = 500
        v_h = hp(vi, nb)
        tracks = {"quat": bg[ax][mb], "gyro": gyro_only[ax], "rawint": raw_int[ax]}
        row = dict(vicon_drift=v_third, vicon_mid=v_mid, vicon_slope=v_slope)
        for nm, ser in tracks.items():
            d3, dmid = thirds(ser)
            slope = float(np.polyfit(tbw, ser, 1)[0])
            s_h = hp(ser, nb)
            r = float(np.corrcoef(v_h, s_h)[0, 1]); sc = float(np.polyfit(v_h, s_h, 1)[0])
            # physical sign: per-hop correlation, except yaw where the drift level decides when large
            if ax == "yaw" and abs(v_third) > 2.0 and np.isfinite(d3):
                sgn = 1.0 if d3 * v_third >= 0 else -1.0
            else:
                sgn = 1.0 if r >= 0 else -1.0
            e = sgn * ser - vi; e = e - e[:nb].mean(); es = movmean(e, 1000)
            A = np.polyfit(tbw, es, 1); fit = np.polyval(A, tbw)
            ss = np.sum((es - es.mean()) ** 2)
            r2 = float(1 - np.sum((es - fit) ** 2) / ss) if ss > 0 else float("nan")
            row[nm] = dict(drift=d3, mid=dmid, slope=slope, hop_r=r, hop_scale=sc, sgn=sgn,
                           excess_slope=float(A[0]), excess_r2=r2, excess_total=float(es[-1] - es[0]),
                           drift_phys=float(sgn * d3) if np.isfinite(d3) else float("nan"))
        row["hop_amp_vicon"] = float(v_h.std())
        res[ax] = row
    # whole-window yaw on each instrument's own clock (alignment sanity)
    yv = vc["yaw"][i0:i1]; gyv = ~np.isnan(yv)
    out["vicon_yaw_own_window"] = float(yv[gyv][-100:].mean() - yv[gyv][:100].mean())
    mbg = (bg["t"] >= bg["T0"]) & (bg["t"] <= bg["T0"] + bg["gait"])
    gz_int = cumtrapz(bg["g"][mbg, 2], bg["t"][mbg])
    out["gyro_yaw_own_window"] = float(gz_int[-100:].mean() - gz_int[:100].mean())
    # reproduction of the earlier numbers
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
        out["n_strides"] = int(n)
    else:
        out["ltrend_pitch_thirds"] = float("nan"); out["ltrend_pitch_halves"] = float("nan"); out["n_strides"] = int(len(on))
    out["axes"] = res
    return out


def fmt(x, w=6, p=2):
    return ("%" + str(w) + "." + str(p) + "f") % x if np.isfinite(x) else "%*s" % (w, "nan")


if __name__ != "__main__":
    RUNS = []            # imported as a library: run nothing
results = []
for bag, vic, lam, kroll, note in RUNS:
    try:
        r = analyse(bag, vic, lam, kroll, note)
    except Exception as e:
        import traceback; traceback.print_exc()
        print("!! %s / %s failed: %s: %s" % (bag, vic, type(e).__name__, e)); continue
    results.append(r)
    print("== %-14s %-9s λ %-7s k_roll %.2f  %s" % (bag, vic, lam, kroll, note))
    print("   align r %.3f (±1 stride %.3f)  hop onset %+.2f s after trigger  common %.2f s (Vicon %.2f-%.2f, bag gait %.2f)  fore=%s pca-vs-travel %.1f°  fit %.0f%%  drop %.1f%%"
          % (r["r_align"], r["r_align_2nd"], r["dlag"], r["common_window_s"], r["vicon_window"][0], r["vicon_window"][1], r["bag_gait_s"],
             r["fore_method"], r["ang_pca_travel"], 100 * r["vicon_fit_frac"], 100 * r["imu_drop_frac"]))
    print("   own-window yaw: Vicon %+.2f  ∫gz %+.2f | pre-trigger gyro bias x %+.3f y %+.3f z %+.3f °/s | window: gy rms %.1f gx rms %.1f gz mean %+.2f, quat pitch mean %+.2f roll mean %+.2f, az p1/p99 %.1f/%.1f, ax/ay mean %+.2f/%+.2f"
          % (r["vicon_yaw_own_window"], r["gyro_yaw_own_window"], r["gbias_pre"][0], r["gbias_pre"][1], r["gbias_pre"][2],
             r["gy_rms"], r["gx_rms"], r["gz_mean"], r["quat_pitch_mean"], r["quat_roll_mean"], r["az_p01"], r["az_p99"], r["ax_mean"], r["ay_mean"]))
    print("   reproduction: tilt_old %+.2f | l_trend pitch thirds %+.2f halves %+.2f (%d strides)"
          % (r["tilt_old_drift"], r["ltrend_pitch_thirds"], r["ltrend_pitch_halves"], r["n_strides"]))
    print("   axis    Vicon drift  | quat drift(raw) sgn hop_r  excess °/s R² | gyro-only drift sgn hop_r  excess °/s R² | raw∫g drift sgn  excess °/s R²")
    for ax in ("roll", "pitch", "yaw"):
        a = r["axes"][ax]
        print("   %-6s %s°  | %s° %+d %s  %s %s | %s° %+d %s  %s %s | %s° %+d  %s %s"
              % (ax, fmt(a["vicon_drift"]),
                 fmt(a["quat"]["drift"]), int(a["quat"]["sgn"]), fmt(a["quat"]["hop_r"], 5), fmt(a["quat"]["excess_slope"], 6, 3), fmt(a["quat"]["excess_r2"], 4),
                 fmt(a["gyro"]["drift"]), int(a["gyro"]["sgn"]), fmt(a["gyro"]["hop_r"], 5), fmt(a["gyro"]["excess_slope"], 6, 3), fmt(a["gyro"]["excess_r2"], 4),
                 fmt(a["rawint"]["drift"]), int(a["rawint"]["sgn"]), fmt(a["rawint"]["excess_slope"], 6, 3), fmt(a["rawint"]["excess_r2"], 4)))
    print()

if __name__ == "__main__":
    with open(sys.argv[1], "w") as fh:
        json.dump(results, fh, indent=1, default=float)
    print("wrote", sys.argv[1], len(results), "runs")
