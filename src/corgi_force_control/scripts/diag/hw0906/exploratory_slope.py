#!/usr/bin/env python3
"""Log 325.25 SECONDARY, EXPLORATORY analysis: per-stride curvature vs MEASURED camber.

kappa_i = dpsi_i / ds_i per bag stride i.
  dpsi_i : IMU gyro z integrated over the stride, bias = median of the pre-trigger window
           (-2.0 .. -0.2 s), fallback 0.2 .. 2.0 s, exactly as l0_cell.py.
  ds_i   : Vicon body-centroid horizontal displacement over the SAME window, after the bag
           clock has been mapped onto the Vicon clock (see align()).
Stride windows are the kp_r-drop stance entries of module A, > 2.6 s and < gait - 0.3 s
(l0_cell.py / l_trend.py detector), so "stride i" means what the log's stride counts mean.

lambda_eff_i = mean over the stride of (commanded gamma + IMU roll + per-leg tracking error),
lean-sense-signed and averaged over the four legs. Because err = meas - cmd this collapses to
mean_legs(s_leg * gamma_meas) + sigma * (roll - roll_pre). The roll sign sigma relative to the
camber lean is NOT determinable from the data; both hypotheses are reported.

N_sat = first stride (1-based) with any leg max|tau_h| >= 39 N.m; survivors are strides < N_sat.
Fit kappa = a*lambda_eff + b*i + c on pooled survivors (OLS, plus run-cluster and jackknife SE).
Usage: exploratory_slope.py  (paths are hardcoded below)   -> prints report, writes JSON
"""
import sys, sqlite3, json
import numpy as np
import c3d
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAGDIR = "/home/alexc/corgi_runs/hw_2026-09-06/bags"
VICDIR = "/home/alexc/corgi_runs/hw_2026-09-06/vicon"
OUT = ("/mnt/c/Users/alexc/AppData/Local/Temp/claude/C--Users-alexc-Documents-Obsidian-Vault/"
       "fefde951-4709-4df4-a73b-a67aab98cd92/scratchpad/exploratory_slope_strides.json")
RUNS = [  # bag, vicon, cell label, commanded lambda (deg)
    ("s2_l0r_a1", "L0_RA1", "L0", 0),
    ("s2_l0r_a2", "L0RA2", "L0", 0),
    ("s2_ol10_roll1", "OL10_R1", "+10", +10),
    ("s2_ol10_a2", "OL10A2", "+10", +10),
    ("s2_ol10n_a2", "OL10_NA2", "-10", -10),
    ("s2_ol15_a1", "OL15_A1", "+15", +15),
    ("s2_ol15n_a1", "OL15_NA1", "-15", -15),
]
SAT = 39.0
FLOOR = 0.257
MIRROR_PER_DEG = 0.0232     # whole-arc mirror-contrast camber term, log 325.31: 0.2318 /m per 10 deg
L4 = "abcd"
BODY = ["B1", "B2", "B3", "B4", "B5"]   # B6 dropped 2026-09-08: it IS rigid on the body
# (|B6-B1| 346.8 +- 0.4 mm) but is visible in only ~95% of in-window frames, and the
# varying-subset nanmean stepped the centroid by (B6-c5)/6 = 3.7 mm. B1-B5 are ~100%.
GRID = 200.0                # Hz, common grid for the alignment cross-correlation


# ----------------------------------------------------------------------------- bag
def read_bag(path):
    con = sqlite3.connect("file:%s?mode=ro" % path, uri=True)
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
    T0, TOFF = max(pairs, key=lambda p: p[1] - p[0]); gait = TOFF - T0

    ms = rd("/motor/state"); mc = rd("/motor/command"); imu = rd("/imu")
    B = dict(T0=T0, gait=gait)
    B["ts"] = np.array([t - T0 for t, _ in ms]); B["tc"] = np.array([t - T0 for t, _ in mc])
    B["ti"] = np.array([t - T0 for t, _ in imu])
    B["gm"] = {l: np.degrees([getattr(m, "module_" + l).gamma for _, m in ms]) for l in L4}
    B["gc"] = {l: np.degrees([getattr(m, "module_" + l).gamma for _, m in mc]) for l in L4}
    B["tH"] = {l: np.array([getattr(m, "module_" + l).torque_h for _, m in ms]) for l in L4}
    q = np.array([[m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z] for _, m in imu])
    w_, x_, y_, z_ = q.T
    B["roll"] = np.degrees(np.arctan2(2 * (w_ * x_ + y_ * z_), 1 - 2 * (x_ ** 2 + y_ ** 2)))
    B["gyr"] = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for _, m in imu])
    B["acc"] = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for _, m in imu])
    ti = B["ti"]
    pre = (ti > -2.0) & (ti < -0.2); B["pre_window"] = "(-2.0,-0.2)"
    if pre.sum() < 20:
        pre = (ti > 0.2) & (ti < 2.0); B["pre_window"] = "(0.2,2.0) FALLBACK"
    B["gb"] = np.median(B["gyr"][pre], 0)
    B["roll0"] = float(np.median(B["roll"][pre]))
    B["az0"] = float(np.median(B["acc"][pre, 2]))
    B["imu_hz"] = float(1.0 / np.median(np.diff(ti)))
    # stance entries from the kp_r drop, as every other analyser tonight
    kpr = np.array([m.module_a.kp_r for _, m in mc]); low = kpr < 100
    ev = B["tc"][1:][(~low[:-1]) & low[1:]]
    B["ev_all"] = ev
    B["on"] = ev[(ev > 2.6) & (ev < gait - 0.3)]
    # physical hops from the IMU: flight = specific force z below half of rest, >= 60 ms long
    fl = B["acc"][:, 2] < 0.5 * B["az0"]
    seg = segment_ends(ti, fl, 0.03)
    B["td_imu"] = seg[:, 0]; B["fl_dur"] = seg[:, 1]
    B["td_imu_full"] = seg[seg[:, 1] >= 0.10, 0]     # full hops: flight >= 100 ms (full hops fly ~0.19 s)
    B["flight"] = fl
    return B


def segment_ends(t, flag, minlen):
    """(end time, duration) of True segments of flag lasting >= minlen seconds"""
    d = np.diff(np.concatenate(([0], flag.astype(np.int8), [0])))
    s, e = np.where(d == 1)[0], np.where(d == -1)[0]
    out = []
    for a, b in zip(s, e):
        dur = t[min(b, len(t) - 1)] - t[a]
        if dur >= minlen:
            out.append((float(t[min(b, len(t) - 1)]), float(dur)))
    return np.array(out).reshape(-1, 2)


# ----------------------------------------------------------------------------- vicon
def load_c3d(path):
    with open(path, "rb") as fh:
        r = c3d.Reader(fh)
        rate = float(r.point_rate)
        labels = [l.strip() for l in r.point_labels]
        frames = [np.array(p, dtype=float) for _, p, _ in r.read_frames()]
    P = np.stack(frames)
    return rate, labels, P[:, :, 0:3], P[:, :, 3] >= 0


def smooth(x, n):
    """moving average with EDGE padding (zero padding put a 12 m/s spike at t=0 and broke the hop window)"""
    n = max(int(n), 1)
    xp = np.pad(x, (n // 2, n - 1 - n // 2), mode="edge")
    return np.convolve(xp, np.ones(n) / n, mode="valid")


def yaw_series(xyz, valid, idx, up):
    horiz = [a for a in (0, 1, 2) if a != up]
    T = xyz.shape[0]
    nval = valid[:, idx].sum(1)
    ref_t = int(np.argmax(nval))
    ref_mask = valid[ref_t, idx]
    yaw = np.full(T, np.nan)
    for t in range(T):
        m = valid[t, idx] & ref_mask
        if m.sum() < 3:
            continue
        A = xyz[t][idx][m]; Bm = xyz[ref_t][idx][m]
        A = A - A.mean(0); Bm = Bm - Bm.mean(0)
        H = Bm.T @ A
        U, S, Vt = np.linalg.svd(H)
        d = np.sign(np.linalg.det(Vt.T @ U.T))
        R = Vt.T @ np.diag([1, 1, d]) @ U.T
        yaw[t] = np.degrees(np.arctan2(R[horiz[1], horiz[0]], R[horiz[0], horiz[0]]))
    ok = ~np.isnan(yaw)
    out = np.full(T, np.nan)
    out[ok] = np.degrees(np.unwrap(np.radians(yaw[ok])))
    return out


def read_vicon(path):
    rate, labels, xyz, valid = load_c3d(path)
    T = xyz.shape[0]; t = np.arange(T) / rate
    idx = [labels.index(b) for b in BODY if b in labels]
    spread = [np.nanmean(np.nanstd(np.where(valid[:, idx], xyz[:, idx, a], np.nan), axis=1)) for a in range(3)]
    up = int(np.argmin(spread)); horiz = [a for a in (0, 1, 2) if a != up]
    cen = np.full((T, 3), np.nan)
    for a in range(3):
        cen[:, a] = np.nanmean(np.where(valid[:, idx], xyz[:, idx, a], np.nan), axis=1)
    ok = ~np.isnan(cen).any(1)
    z = np.where(ok, cen[:, up], np.nan) / 1000.0
    # fill short gaps by interpolation so the derivatives are defined
    zi = np.interp(t, t[ok], z[ok])
    dz = np.gradient(smooth(zi, int(0.03 * rate))) * rate          # m/s, 30 ms smoothing
    # hop window: longest contiguous span of vertical activity (vicon_l0.py logic)
    win = int(0.5 * rate)
    rms = np.sqrt(np.convolve(dz ** 2, np.ones(win) / win, mode="same"))
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
    # touchdowns: local minima of dz (fastest descent) inside the window, >= 0.15 s apart
    # (hop period is 0.265 s; full hops reach -0.6..-0.7 m/s, the ramp-in half-hops -0.33)
    sep = int(0.15 * rate)
    cand = [i for i in range(i0 + 1, i1 - 1) if dz[i] < -0.20 and dz[i] <= dz[i - 1] and dz[i] < dz[i + 1]]
    td = []
    for i in cand:
        if td and i - td[-1] < sep:
            if dz[i] < dz[td[-1]]:
                td[-1] = i
        else:
            td.append(i)
    lo = [i for i in range(i0 + 1, i1 - 1) if dz[i] > 0.20 and dz[i] >= dz[i - 1] and dz[i] > dz[i + 1]]
    lof = []
    for i in lo:
        if lof and i - lof[-1] < sep:
            if dz[i] > dz[lof[-1]]:
                lof[-1] = i
        else:
            lof.append(i)
    # binary flight indicator: from each liftoff (dz max) to the next touchdown (dz min)
    fl = np.zeros(T, bool)
    for a in lof:
        nxt = [b for b in td if b > a]
        if nxt:
            fl[a:nxt[0]] = True
    xy = cen[:, horiz] / 1000.0
    yaw = yaw_series(xyz, valid, idx, up)
    td = np.array(td, int); lof = np.array(lof, int)
    return dict(rate=rate, t=t, xy=xy, z=zi, dz=dz, win=(t[i0], t[i1 - 1]), td=t[td], td_dz=dz[td],
                td_full=t[td[dz[td] <= -0.5]],            # full-height hops only
                lo=t[lof], flight=fl, yaw=yaw, ok=ok, labels=labels)


# ----------------------------------------------------------------------------- alignment
def align(B, V):
    """Map bag time (s after trigger) to Vicon time: t_v = t_b + off.
    Method of record = COUNT: first Vicon touchdown <-> first bag IMU touchdown (first full hop).
    Check = cross-correlation of the two binary flight indicators on a 200 Hz grid."""
    tdb = B["td_imu_full"]; tdv = V["td_full"]
    off_count = float(tdv[0] - tdb[0])
    # residuals of every bag full-hop touchdown against its nearest Vicon touchdown under the count offset
    res = np.array([tdv[np.argmin(np.abs(tdv - (x + off_count)))] - (x + off_count) for x in tdb])
    # cross-correlation
    tb = np.arange(-3.0, B["gait"] + 3.0, 1 / GRID)
    fb = np.interp(tb, B["ti"], B["flight"].astype(float)) > 0.5
    tv = np.arange(0.0, V["t"][-1], 1 / GRID)
    fv = np.interp(tv, V["t"], V["flight"].astype(float)) > 0.5
    c = np.correlate(fv.astype(float), fb.astype(float), mode="full")
    lags = (np.arange(len(c)) - (len(fb) - 1)) / GRID + (tv[0] - tb[0])
    k = int(np.argmax(c)); off_xc = float(lags[k])
    # next-best peak at least 0.2 s away from the best
    far = np.abs(lags - off_xc) > 0.2
    second = float(c[far].max() / c[k]) if far.any() else float("nan")
    return dict(off_count=off_count, off_xc=off_xc, res_med=float(np.median(res)), res_sd=float(np.std(res)),
                res_max=float(np.abs(res).max()), n_td_bag=len(tdb), n_td_vic=len(tdv),
                xc_second_ratio=second, delta=off_xc - off_count, res_first=[float(x) for x in res[:6]])


# ----------------------------------------------------------------------------- per stride
def strides(B, V, off, sgn, lam_cmd):
    rows = []
    on = B["on"]; ti = B["ti"]; ts = B["ts"]; tc = B["tc"]
    gz = B["gyr"][:, 2] - B["gb"][2]
    for i in range(len(on) - 1):
        a, b = on[i], on[i + 1]
        wi = (ti >= a) & (ti < b); w = (ts >= a) & (ts < b); wc = (tc >= a) & (tc < b)
        if w.sum() < 20 or wi.sum() < 5:
            continue
        dpsi = float(np.trapz(gz[wi], ti[wi]))                       # rad, IMU
        pa = np.array([np.interp(a + off, V["t"], V["xy"][:, k]) for k in range(2)])
        pb = np.array([np.interp(b + off, V["t"], V["xy"][:, k]) for k in range(2)])
        ds = float(np.linalg.norm(pb - pa))                          # m, Vicon
        okv = V["ok"] & ~np.isnan(V["yaw"])
        ya = np.interp(a + off, V["t"][okv], V["yaw"][okv]); yb = np.interp(b + off, V["t"][okv], V["yaw"][okv])
        dpsi_v = float(np.radians(yb - ya))
        gcm = float(np.mean([sgn[l] * np.mean(B["gc"][l][wc]) for l in L4]))
        gmm = float(np.mean([sgn[l] * np.mean(B["gm"][l][w]) for l in L4]))
        per_leg_err = {l: float(sgn[l] * (np.mean(B["gm"][l][w]) - np.mean(B["gc"][l][wc]))) for l in L4}
        roll = float(np.mean(B["roll"][wi]) - B["roll0"])
        tH = {l: float(np.abs(B["tH"][l][w]).max()) for l in L4}
        rows.append(dict(i=i + 1, t0=float(a), dt=float(b - a), dpsi_deg=np.degrees(dpsi), ds=ds,
                         kappa=dpsi / ds if ds > 1e-3 else float("nan"),
                         kappa_vicon=dpsi_v / ds if ds > 1e-3 else float("nan"),
                         v=ds / (b - a), lam_cmd=gcm, lam_meas=gmm, err=gmm - gcm, roll=roll,
                         err_legs=per_leg_err, tauH=tH, tauH_max=max(tH.values()),
                         sat=max(tH.values()) >= SAT))
    nsat = next((r["i"] for r in rows if r["sat"]), None)
    for r in rows:
        r["survives"] = (nsat is None) or (r["i"] < nsat)
    return rows, nsat


# ----------------------------------------------------------------------------- fits
def ols(X, y):
    X = np.asarray(X, float); y = np.asarray(y, float)
    n, k = X.shape
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    e = y - X @ beta
    s2 = float(e @ e) / max(n - k, 1)
    XtXi = np.linalg.inv(X.T @ X)
    se = np.sqrt(np.diag(XtXi) * s2)
    r2 = 1 - float(e @ e) / float(((y - y.mean()) ** 2).sum()) if n > 1 else float("nan")
    return beta, se, r2, e, XtXi


def cluster_se(X, e, XtXi, groups):
    X = np.asarray(X, float); groups = np.asarray(groups)
    G = np.unique(groups); n, k = X.shape
    meat = np.zeros((k, k))
    for g in G:
        m = groups == g
        u = X[m].T @ e[m]
        meat += np.outer(u, u)
    fac = (len(G) / (len(G) - 1)) * ((n - 1) / (n - k)) if len(G) > 1 else float("nan")
    V = XtXi @ meat @ XtXi * fac
    return np.sqrt(np.diag(V))


def fit(rows, xkey, label, with_index=True):
    if len(rows) < 4:
        print("   %-46s n=%d  -> too few strides, not estimable" % (label, len(rows))); return None
    y = [r["kappa"] for r in rows]
    X = [[r[xkey], r["i"], 1.0] if with_index else [r[xkey], 1.0] for r in rows]
    beta, se, r2, e, XtXi = ols(X, y)
    grp = [r["run"] for r in rows]
    cse = cluster_se(X, e, XtXi, grp)
    # leave-one-run-out jackknife of a
    runs = sorted(set(grp)); jk = []
    for g in runs:
        sub = [r for r in rows if r["run"] != g]
        if len(sub) >= 4 and len(set(r[xkey] for r in sub)) > 2:
            b2, *_ = ols([[r[xkey], r["i"], 1.0] if with_index else [r[xkey], 1.0] for r in sub],
                         [r["kappa"] for r in sub])
            jk.append(b2[0])
    out = dict(label=label, n=len(rows), a=float(beta[0]), se_a=float(se[0]), cse_a=float(cse[0]),
               b=float(beta[1]) if with_index else None, se_b=float(se[1]) if with_index else None,
               c=float(beta[-1]), r2=float(r2), jk_min=float(min(jk)) if jk else None,
               jk_max=float(max(jk)) if jk else None, runs=len(runs))
    print("   %-46s n=%3d runs=%d | a = %+.5f /m/deg  SE %.5f (cluster %.5f)  jackknife [%+.4f,%+.4f] | b = %s | c = %+.4f | R2 %.3f | a/0.0232 = %.2f"
          % (label, out["n"], out["runs"], out["a"], out["se_a"], out["cse_a"],
             out["jk_min"] if jk else float("nan"), out["jk_max"] if jk else float("nan"),
             ("%+.5f (SE %.5f)" % (out["b"], out["se_b"])) if with_index else "  --  ",
             out["c"], out["r2"], out["a"] / MIRROR_PER_DEG))
    return out


# ----------------------------------------------------------------------------- main
allrows = []; runinfo = []; sgn_ref = None
print("=" * 100)
print("PER-RUN: alignment, sign map, N_sat")
for bag, vic, cell, lam in RUNS:
    B = read_bag("%s/%s/%s_0.db3" % (BAGDIR, bag, bag))
    V = read_vicon("%s/%s.c3d" % (VICDIR, vic))
    A = align(B, V)
    # lean-sense sign per leg from the commanded gamma of the cambered runs
    wc = (B["tc"] > 2.6) & (B["tc"] < B["gait"])
    med = {l: float(np.median(B["gc"][l][wc])) for l in L4}
    if lam != 0:
        sgn = {l: float(np.sign(med[l]) * np.sign(lam)) for l in L4}
        if sgn_ref is None:
            sgn_ref = sgn
        elif sgn != sgn_ref:
            print("   !!! sign map differs from the first cambered run:", sgn, "vs", sgn_ref)
    else:
        sgn = None
    runinfo.append(dict(bag=bag, vic=vic, cell=cell, lam=lam, B=B, V=V, A=A, med=med, sgn=sgn))
    print("%-14s %-9s gait %5.1f s | strides %2d (kp_r events all %2d) | IMU td %2d  Vicon td %2d | off count %+8.3f  xcorr %+8.3f  (d %+.3f s) | td residual med %+.3f sd %.3f max %.3f | xc 2nd/1st %.2f | IMU %.0f Hz az0 %+.2f pre %s"
          % (bag, vic, B["gait"], len(B["on"]), len(B["ev_all"]), A["n_td_bag"], A["n_td_vic"],
             A["off_count"], A["off_xc"], A["delta"], A["res_med"], A["res_sd"], A["res_max"],
             A["xc_second_ratio"], B["imu_hz"], B["az0"], B["pre_window"]))
    print("      cmd gamma median per leg (deg): " + "  ".join("%s %+6.2f" % (l, med[l]) for l in L4)
          + " | Vicon hop window %.2f-%.2f s" % V["win"])
    print("      bag  kp_r events (first 6, s after trigger): " + " ".join("%.2f" % x for x in B["ev_all"][:6])
          + "  | IMU touchdowns (t, flight s) first 6: " + " ".join("%.2f(%.2f)" % (a, b) for a, b in zip(B["td_imu"][:6], B["fl_dur"][:6])))
    print("      Vicon touchdowns (t, dz) first 6: " + " ".join("%.2f(%.2f)" % (a, b) for a, b in zip(V["td"][:6], V["td_dz"][:6]))
          + "  | first full: bag %.2f  vicon %.2f" % (B["td_imu_full"][0], V["td_full"][0])
          + "  | td residuals under count offset, first 6: " + " ".join("%+.3f" % x for x in A["res_first"]))

# fallback sign map for the lambda-0 runs
for R in runinfo:
    if R["sgn"] is None:
        R["sgn"] = sgn_ref
print("lean-sense sign map (from the cambered commands):", sgn_ref)

print()
print("=" * 100)
print("PER-STRIDE TABLE (alignment of record = COUNT; xcorr used only as a check)")
for R in runinfo:
    B, V, A = R["B"], R["V"], R["A"]
    off = A["off_count"]
    rows, nsat = strides(B, V, off, R["sgn"], R["lam"])
    R["rows"] = rows; R["nsat"] = nsat
    nsurv = sum(r["survives"] for r in rows)
    R["nsurv"] = nsurv
    print("\n--- %s / %s  cell %s  N_sat = %s  survivors = %d  (roll0 %+.2f deg)" % (R["bag"], R["vic"], R["cell"], nsat, nsurv, B["roll0"]))
    print("  i   t0    dt   | dpsi(deg)  ds(m)   kappa   kap_vic |  v(m/s) | lam_cmd lam_meas  err   roll | tauH A    B    C    D  | sat surv")
    for r in rows:
        r["run"] = R["bag"]; r["cell"] = R["cell"]; r["lam_cell"] = R["lam"]
        print(" %2d %6.2f %5.3f | %+8.2f %6.3f %+8.3f %+8.3f | %6.3f%s | %+6.2f %+6.2f %+6.2f %+6.2f | %5.1f %5.1f %5.1f %5.1f |  %s   %s"
              % (r["i"], r["t0"], r["dt"], r["dpsi_deg"], r["ds"], r["kappa"], r["kappa_vicon"], r["v"],
                 "*" if r["v"] < FLOOR else " ", r["lam_cmd"], r["lam_meas"], r["err"], r["roll"],
                 r["tauH"]["a"], r["tauH"]["b"], r["tauH"]["c"], r["tauH"]["d"],
                 "Y" if r["sat"] else ".", "Y" if r["survives"] else "."))
        allrows.append(r)
    if rows:
        k = np.array([r["kappa"] for r in rows]); kv = np.array([r["kappa_vicon"] for r in rows])
        print("  whole-run: mean kappa(IMU) %+.4f  mean kappa(Vicon yaw) %+.4f  | sum dpsi / sum ds = %+.4f (IMU) %+.4f (Vicon)"
              % (np.nanmean(k), np.nanmean(kv),
                 np.radians(sum(r["dpsi_deg"] for r in rows)) / sum(r["ds"] for r in rows),
                 sum(r["kappa_vicon"] * r["ds"] for r in rows) / sum(r["ds"] for r in rows)))

# roll-sign hypothesis: sigma=+1 means IMU roll (rel. pre-trigger) is ADDED to lean-sense camber as read
print()
print("=" * 100)
print("ROLL SIGN: mean roll (rel. pre-trigger) per run, and its product with the commanded sign")
prod = []
for R in runinfo:
    rr = np.array([r["roll"] for r in R["rows"]]);
    print("   %-14s lam %+3d  mean roll %+6.2f deg  (first 3 strides %+6.2f, last 3 %+6.2f)" % (R["bag"], R["lam"], rr.mean(), rr[:3].mean(), rr[-3:].mean()))
    if R["lam"] != 0:
        prod.append(np.sign(R["lam"]) * rr.mean())
sigma_lean = float(np.sign(np.mean(prod)))
print("   -> IMU roll has sign %+d relative to the commanded lean; 'roll adds to the lean' therefore means sigma = %+d" % (sigma_lean, sigma_lean))

for r in allrows:
    r["lam_eff"] = r["lam_meas"] + sigma_lean * r["roll"]        # ASSUMPTION A: roll adds to the lean
    r["lam_eff_alt"] = r["lam_meas"] - sigma_lean * r["roll"]    # ALTERNATIVE: roll subtracts

# per-cell / per-run medians over survivors
print()
print("=" * 100)
print("PER-CELL medians over SURVIVING strides (protocol: < 5 survivors -> not estimable)")
cells = []
for c in ["L0", "+10", "-10", "+15", "-15"]:
    rs = [r for r in allrows if r["cell"] == c and r["survives"]]
    tot = [r for r in allrows if r["cell"] == c]
    runs = sorted(set(r["run"] for r in tot))
    ns = {g: next(R["nsat"] for R in runinfo if R["bag"] == g) for g in runs}
    if len(rs) >= 5:
        k = np.array([r["kappa"] for r in rs]); le = np.array([r["lam_eff"] for r in rs]); lc = np.array([r["lam_cmd"] for r in rs])
        print("   cell %-4s runs %s N_sat %s | survivors %2d/%2d | median kappa %+.4f  (IQR %+.4f..%+.4f) | median lam_eff %+.2f  lam_cmd %+.2f"
              % (c, runs, ns, len(rs), len(tot), np.median(k), np.percentile(k, 25), np.percentile(k, 75), np.median(le), np.median(lc)))
        cells.append(dict(cell=c, n=len(rs), ntot=len(tot), med_kappa=float(np.median(k)), q1=float(np.percentile(k, 25)),
                          q3=float(np.percentile(k, 75)), med_lam_eff=float(np.median(le)), med_lam_cmd=float(np.median(lc)), estimable=True, nsat=ns))
    else:
        print("   cell %-4s runs %s N_sat %s | survivors %2d/%2d | pre-saturation window too short, slope not estimable" % (c, runs, ns, len(rs), len(tot)))
        cells.append(dict(cell=c, n=len(rs), ntot=len(tot), estimable=False, nsat=ns))
print("   per run:")
for R in runinfo:
    rs = [r for r in R["rows"] if r["survives"]]
    if len(rs) >= 5:
        k = np.array([r["kappa"] for r in rs])
        print("      %-14s N_sat %-4s survivors %2d/%2d | median kappa %+.4f | median lam_eff %+.2f" % (R["bag"], R["nsat"], len(rs), len(R["rows"]), np.median(k), np.median([r["lam_eff"] for r in rs])))
    else:
        print("      %-14s N_sat %-4s survivors %2d/%2d | not estimable" % (R["bag"], R["nsat"], len(rs), len(R["rows"])))

print()
print("=" * 100)
print("FITS  kappa_i = a*lambda + b*i + c   (a in 1/m per deg; mirror-contrast reference 0.0232 per COMMANDED deg)")
surv = [r for r in allrows if r["survives"]]
fits = {}
fits["main"] = fit(surv, "lam_eff", "survivors, lam_eff (roll ADDS, assumption A)")
fits["alt"] = fit(surv, "lam_eff_alt", "survivors, lam_eff (roll SUBTRACTS, alternative)")
fits["meas"] = fit(surv, "lam_meas", "survivors, lam_meas only (no roll term)")
fits["cmd"] = fit(surv, "lam_cmd", "survivors, COMMANDED camber (apples-to-apples)")
fits["noidx"] = fit(surv, "lam_eff", "survivors, lam_eff, NO stride covariate", with_index=False)
est_cells = [c["cell"] for c in cells if c["estimable"]]
surv5 = [r for r in surv if r["cell"] in est_cells]
fits["cells5"] = fit(surv5, "lam_eff", "survivors in cells with >=5 survivors only")
runs5 = [R["bag"] for R in runinfo if R["nsurv"] >= 5]
fits["runs5"] = fit([r for r in surv if r["run"] in runs5], "lam_eff", "survivors in RUNS with >=5 survivors (%s)" % ",".join(runs5))
fits["runs5_cmd"] = fit([r for r in surv if r["run"] in runs5], "lam_cmd", "  same, commanded camber")
fits["no1"] = fit([r for r in surv if r["i"] > 1], "lam_eff", "survivors minus stride 1 (launch transient)")
fits["no12"] = fit([r for r in surv if r["i"] > 2], "lam_eff", "survivors minus strides 1-2")
fits["all"] = fit(allrows, "lam_eff", "ALL strides incl. saturated (NOT protocol, reference)")
fits["all_ds"] = fit([r for r in allrows if r["ds"] >= 0.04], "lam_eff", "ALL strides with ds >= 4 cm (drops stalled OL10A2 hops)")
fits["all_cmd"] = fit(allrows, "lam_cmd", "ALL strides, commanded (NOT protocol, reference)")
vrows = [dict(r, kappa=r["kappa_vicon"]) for r in surv if not np.isnan(r["kappa_vicon"])]
fits["vicon"] = fit(vrows, "lam_eff", "survivors, kappa from VICON yaw (cross-check)")

# sensitivity: same strides, Vicon displacement taken under the CROSS-CORRELATION offset instead of the count offset
xrows = []
for R in runinfo:
    rows_x, _ = strides(R["B"], R["V"], R["A"]["off_xc"], R["sgn"], R["lam"])
    for r, r0 in zip(rows_x, R["rows"]):
        assert r["i"] == r0["i"]
        r["run"] = R["bag"]; r["cell"] = R["cell"]; r["lam_eff"] = r["lam_meas"] + sigma_lean * r["roll"]
        r["survives"] = r0["survives"]
        xrows.append(r)
fits["main_xc"] = fit([r for r in xrows if r["survives"]], "lam_eff", "survivors, lam_eff, XCORR alignment (sensitivity)")
dk = np.array([rx["kappa"] - r0["kappa"] for rx, r0 in zip(xrows, allrows)])
print("   kappa_i(xcorr) - kappa_i(count): median %+.4f  sd %.4f  max|.| %.4f  (n=%d)" % (np.median(dk), dk.std(), np.abs(dk).max(), len(dk)))

# ---- literal block for the figure generator (numbers are pasted, never re-derived there)
print()
print("=" * 100)
print("FIGURE DATA (paste into make_figures_2026-09-06_exploratory_slope.py)")
print("RUNS = [")
for R in runinfo:
    print("  dict(run=%r, vic=%r, cell=%r, lam=%d, nsat=%r," % (R["bag"], R["vic"], R["cell"], R["lam"], R["nsat"]))
    print("       i=%s," % [r["i"] for r in R["rows"]])
    print("       kappa=%s," % [round(r["kappa"], 4) for r in R["rows"]])
    print("       lam_eff=%s," % [round(r["lam_eff"], 2) for r in R["rows"]])
    print("       lam_cmd=%s," % [round(r["lam_cmd"], 2) for r in R["rows"]])
    print("       tauH=%s," % [round(r["tauH_max"], 1) for r in R["rows"]])
    print("       surv=%s)," % [bool(r["survives"]) for r in R["rows"]])
print("]")
m = fits["main"]
print("FIT_MAIN = dict(a=%.5f, se=%.5f, b=%.5f, c=%.4f, n=%d, i_mean=%.2f)" % (m["a"], m["se_a"], m["b"], m["c"], m["n"], np.mean([r["i"] for r in surv])))
m = fits["cmd"]
print("FIT_CMD = dict(a=%.5f, se=%.5f, b=%.5f, c=%.4f, n=%d)" % (m["a"], m["se_a"], m["b"], m["c"], m["n"]))
# mirror-contrast reference expressed per lam_eff degree of the pair it came from (OL10_R1 / OL10_NA2)
le = {R["bag"]: np.mean([abs(r["lam_eff"]) for r in R["rows"]]) for R in runinfo}
lc = {R["bag"]: np.mean([abs(r["lam_cmd"]) for r in R["rows"]]) for R in runinfo}
print("MIRROR: 0.2318 /m at nominal 10 deg | mean |lam_cmd| R1 %.2f NA2 %.2f -> per commanded deg %.4f | mean |lam_eff| (A) R1 %.2f NA2 %.2f -> per lam_eff deg %.4f"
      % (lc["s2_ol10_roll1"], lc["s2_ol10n_a2"], 0.2318 / np.mean([lc["s2_ol10_roll1"], lc["s2_ol10n_a2"]]),
         le["s2_ol10_roll1"], le["s2_ol10n_a2"], 0.2318 / np.mean([le["s2_ol10_roll1"], le["s2_ol10n_a2"]])))
print("15 deg pair: 0.5104 /m | mean |lam_cmd| A1 %.2f NA1 %.2f -> per commanded deg %.4f | |lam_eff| A1 %.2f NA1 %.2f -> per lam_eff deg %.4f"
      % (lc["s2_ol15_a1"], lc["s2_ol15n_a1"], 0.5104 / np.mean([lc["s2_ol15_a1"], lc["s2_ol15n_a1"]]),
         le["s2_ol15_a1"], le["s2_ol15n_a1"], 0.5104 / np.mean([le["s2_ol15_a1"], le["s2_ol15n_a1"]])))

json.dump(dict(rows=allrows, cells=cells, fits=fits, sigma_lean=sigma_lean, sign_map=sgn_ref,
               runs=[dict(bag=R["bag"], vic=R["vic"], cell=R["cell"], lam=R["lam"], nsat=R["nsat"], nsurv=R["nsurv"],
                          nstrides=len(R["rows"]), align=R["A"], roll0=R["B"]["roll0"]) for R in runinfo]),
          open(OUT, "w"), indent=1, default=float)
print("\nwrote", OUT)
