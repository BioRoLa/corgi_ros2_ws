#!/usr/bin/env python3
"""Tier 0 of the event-driven gait scheduler ladder (fix-order step 4):
time-resolve HOW coordination decays on banked captures, and emit the one
number Tier 1 needs -- `event_arm_strides`. Offline only, zero sim.

Per run, on a sliding window (~5 strides wide, 1-stride step) over the gait
band:
  - per-leg debounced touchdowns (DEBOUNCE mirrors the controller's
    contact_debounce_ = 3 messages at the ~100 Hz capture rate);
  - pooled circular touchdown-phase spread (time-resolving the 0.85-0.99
    end-state of the S89 audits, lean thread);
  - per-leg phase offsets from the circular mean, grouped front/rear and
    left/right -- the front/rear split is the S17 (desync thread)
    pitch-pump signature;
  - flight fraction (no leg in debounced contact);
  - stance-gain-on-real-contact fraction. The regime is classified from
    the L_Motor `kd`: stance and flight damping separate across a genuine
    empty histogram gap (verified 2026-08-20: stance 0.31-0.81, gap to
    ~1.0, flight 1.0-4.2), whereas kp = J^T K J is pose-dependent. The
    gap is located per run; a `kd` with no gap REFUSES the gain metrics
    (unimodal = unfit input). Crosschecked per run against
    desync_vs_gain_regime.py's kp < 100 rule on the same rows.

Series verdicts on the spread trajectory:
  - trend: Spearman rho vs window index with a circular-shift permutation
    p (ported from audit_kappa_structure -- shifts respect
    autocorrelation, a plain shuffle would overclaim);
  - changepoint: max-|CUSUM| location, significance from a shuffle null;
    the largest single-window jump must sit at the changepoint and carry
    > JUMP_FRAC of the run's range to call a collapse;
  - shape: stable / monotone drift / saturating plateau / event-driven
    collapse.

Decision output (rule pre-committed in log S91, event-scheduler thread):
if >= 50% of analysable runs show event-driven collapse OR an entry
transient (spread settling only after stride k > 2), event_arm_strides =
ceil(median stabilisation stride over those runs); else 0.

Self-tests (all must pass before any real capture is scored): planted
linear desync ramp -> monotone drift; planted step collapse -> collapse
with the changepoint within +-2 windows of the plant; planted duty-0.40
synchronized gait -> flight fraction 0.60 +- 0.02 and stable; fooling case
(white phase jitter, no drift) -> stable, NOT collapse; planted bimodal
`kd` recovers a known honesty fraction; planted unimodal `kd` is REFUSED.

Usage:
    python3 audit_degradation.py --selftest
    python3 audit_degradation.py --campaign ~/corgi_runs/menger_acker \
        [--campaign ~/corgi_runs/shift_duty_sweep ...]
    python3 audit_degradation.py --run path/to/runN.csv
"""
import argparse
import csv
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_menger import LEGS, DEBOUNCE, debounce, touchdowns  # noqa: E402
from audit_contact_structure import circular_spread  # noqa: E402
from audit_kappa_structure import spearman, shift_perm_p  # noqa: E402

FRONT, REAR = (0, 1), (2, 3)          # A,B front; C,D rear
LEFT, RIGHT = (0, 3), (1, 2)
WINDOW_STRIDES = 5
MIN_WINDOWS = 8                        # below this, no shape verdict
MIN_TD_PER_LEG = 8                     # below this, not a sustained gait
KP_THRESH = 100.0                      # desync_vs_gain_regime's rule
JUMP_FRAC = 0.4
N_PERM = 1000
RUN_RE = re.compile(r"^run(\d+)\.csv$")  # quarantined = renamed = excluded


class Unfit(Exception):
    """Input cannot support the question -- refuse, don't guess."""


# ---------------------------------------------------------------- loading

def load_run(path):
    """-> (t, contact (n,4) bool, kd (n,4), kp (n,4), tff (n,4)) from the
    L_Motor rows (one per (t, leg))."""
    per = {leg: {"t": [], "c": [], "kd": [], "kp": [], "tf": []}
           for leg in LEGS}
    with open(path, newline="") as fh:
        rdr = csv.reader(fh)
        header = next(rdr)
        col = {n: i for i, n in enumerate(header)}
        need = ("t", "leg", "motor", "in_contact", "kd", "kp", "t_ff")
        missing = [n for n in need if n not in col]
        if missing:
            raise Unfit(f"missing columns {missing} -- not a "
                        f"corgi_torque_terms capture")
        it, ileg, imot = col["t"], col["leg"], col["motor"]
        ic, ikd, ikp, itf = col["in_contact"], col["kd"], col["kp"], col["t_ff"]
        for row in rdr:
            if row[imot] != "L_Motor" or row[ileg] not in per:
                continue
            d = per[row[ileg]]
            d["t"].append(float(row[it]))
            d["c"].append(row[ic] not in ("0", "0.0", ""))
            d["kd"].append(float(row[ikd]))
            d["kp"].append(float(row[ikp]))
            d["tf"].append(abs(float(row[itf])))
    n = min(len(per[leg]["t"]) for leg in LEGS)
    if n < 1000:
        raise Unfit(f"only {n} samples for the thinnest leg")
    t = np.array(per["A"]["t"][:n])
    for leg in LEGS[1:]:
        if np.max(np.abs(np.array(per[leg]["t"][:n]) - t)) > 5e-4:
            raise Unfit(f"leg-{leg} timestamps misaligned with leg A")
    stack = lambda k: np.column_stack([per[leg][k][:n] for leg in LEGS])
    return (t, stack("c").astype(bool), stack("kd"), stack("kp"),
            stack("tf"))


def gait_band(t, tff):
    """-> (t0, t1): first to last feed-forward activity. A pre-trigger
    hold analysed as a gait was a real 2026-08-17 failure -- band first."""
    nz = np.flatnonzero(tff.max(axis=1) > 1e-9)
    if nz.size < 100:
        raise Unfit("no feed-forward activity -- not a gait run")
    return t[nz[0]], t[nz[-1]]


# ---------------------------------------------------- gain-regime classifier

def kd_gap_threshold(kd):
    """-> threshold inside the widest interior empty histogram gap, with
    >= 10% of mass on each side. No gap -> the regime question cannot be
    answered from this run's kd -> refuse."""
    kd = np.asarray(kd).ravel()
    h, edges = np.histogram(kd, bins=60)
    nz = np.flatnonzero(h)
    if nz.size == 0:
        raise Unfit("empty kd histogram")
    best = None  # (width, lo, hi, i0, i1)
    i = nz[0]
    while i <= nz[-1]:
        if h[i] == 0:
            j = i
            while h[j] == 0:
                j += 1
            width = edges[j] - edges[i]
            lo_mass = h[:i].sum() / h.sum()
            hi_mass = h[j:].sum() / h.sum()
            if lo_mass >= 0.10 and hi_mass >= 0.10:
                if best is None or width > best[0]:
                    best = (width, edges[i], edges[j])
            i = j
        else:
            i += 1
    if best is None:
        raise Unfit("kd has no interior empty gap with >=10% mass on each "
                    "side -- unimodal, gain-regime classifier refused")
    return 0.5 * (best[1] + best[2])


# ------------------------------------------------------------ window series

def circ_mean_phase(ph):
    z = np.exp(2j * np.pi * np.asarray(ph)).mean()
    return (np.angle(z) / (2 * np.pi)) % 1.0


def circ_dist(a, b):
    d = abs(a - b) % 1.0
    return min(d, 1.0 - d)


def window_series(t, contact, t0, t1, stride, kd=None, kd_thr=None):
    """-> dict of per-window series over [t0, t1]."""
    deb = np.column_stack([debounce(contact[:, i], DEBOUNCE)
                           for i in range(4)])
    td = []
    for i in range(4):
        e = touchdowns(t, contact[:, i])
        td.append(e[(e >= t0) & (e <= t1)])
    W = WINDOW_STRIDES * stride
    starts = np.arange(t0, t1 - W, stride)
    out = {"w_t": [], "spread": [], "fr": [], "lr": [], "flight": [],
           "honesty": []}
    for a in starts:
        b = a + W
        legs_ph = []
        ok = True
        for i in range(4):
            e = td[i][(td[i] >= a) & (td[i] < b)]
            if len(e) < 3:
                ok = False
                break
            legs_ph.append(((e - t0) % stride) / stride)
        m = (t >= a) & (t < b)
        flight = float((~deb[m].any(axis=1)).mean()) if m.any() else np.nan
        if kd is not None and m.any():
            c = deb[m]
            stance_g = kd[m] < kd_thr
            honesty = (float(stance_g[c].mean()) if c.sum() > 50
                       else np.nan)
        else:
            honesty = np.nan
        if ok:
            pooled = np.concatenate(legs_ph)
            spread = circular_spread(pooled)
            mph = [circ_mean_phase(p) for p in legs_ph]
            fr = circ_dist(circ_mean_phase(np.concatenate(
                     [legs_ph[i] for i in FRONT])),
                 circ_mean_phase(np.concatenate(
                     [legs_ph[i] for i in REAR])))
            lr = circ_dist(circ_mean_phase(np.concatenate(
                     [legs_ph[i] for i in LEFT])),
                 circ_mean_phase(np.concatenate(
                     [legs_ph[i] for i in RIGHT])))
        else:
            spread, fr, lr = np.nan, np.nan, np.nan
        out["w_t"].append(0.5 * (a + b))
        out["spread"].append(spread)
        out["fr"].append(fr)
        out["lr"].append(lr)
        out["flight"].append(flight)
        out["honesty"].append(honesty)
    return {k: np.array(v, dtype=float) for k, v in out.items()}, td


# ------------------------------------------------------------ shape verdict

def cusum_changepoint(s, rng):
    """-> (index of max |CUSUM|, permutation p for the CUSUM peak)."""
    c = np.cumsum(s - s.mean())
    obs = float(np.max(np.abs(c)))
    k = int(np.argmax(np.abs(c)))
    hits = 0
    for _ in range(N_PERM):
        sp = rng.permutation(s)
        if float(np.max(np.abs(np.cumsum(sp - sp.mean())))) >= obs - 1e-12:
            hits += 1
    return k, (hits + 1) / (N_PERM + 1)


def classify(s, rng):
    """-> dict(shape, rho, p_trend, cp_idx, p_cp, jumpfrac, stab_idx,
    entry_transient) for a spread series (NaNs already dropped)."""
    n = len(s)
    idx = np.arange(n, dtype=float)
    rho = spearman(idx, s)
    p_trend = shift_perm_p(idx, s, rng)
    cp, p_cp = cusum_changepoint(s, rng)
    rng_s = float(s.max() - s.min())
    d = np.abs(np.diff(s))
    jumpfrac = float(d.max() / rng_s) if rng_s > 1e-9 and len(d) else 0.0
    # Transition width: the sliding window smears any step over up to
    # WINDOW_STRIDES windows, so a single-window jump criterion is blind
    # to real collapses. A genuine step's mid-band occupancy cannot
    # exceed the smearing width (+ slack); a drift's occupies most of
    # the series.
    if rng_s > 1e-9:
        mid = (s > s.min() + 0.2 * rng_s) & (s < s.min() + 0.8 * rng_s)
        width = int(mid.sum())
    else:
        width = n
    # stabilisation: first index with 3 consecutive windows inside a band
    # around the late-third median
    late = s[-max(3, n // 3):]
    end_med = float(np.median(late))
    band = 0.15 * rng_s if rng_s > 1e-9 else 0.05
    stab = n - 1
    for j in range(n - 2):
        if np.all(np.abs(s[j:j + 3] - end_med) <= band):
            stab = j
            break
    third = s[-max(5, n // 3):]
    p_late = shift_perm_p(np.arange(len(third), dtype=float), third, rng) \
        if len(third) >= 5 else 1.0
    if p_trend > 0.05 and p_cp > 0.05:
        shape = "stable"
    elif (p_cp < 0.05 and width <= WINDOW_STRIDES + 3
          and rng_s > 0.05):
        shape = "event-driven collapse"
    elif p_late > 0.05 and end_med > s.min() + 0.7 * rng_s:
        shape = "saturating plateau"
    else:
        shape = "monotone drift"
    entry = bool(stab > 2 and p_late > 0.05
                 and abs(s[0] - end_med) > 0.3 * rng_s
                 and s[0] > end_med)
    return {"shape": shape, "rho": rho, "p_trend": p_trend, "cp_idx": cp,
            "p_cp": p_cp, "jumpfrac": jumpfrac, "width": width,
            "stab_idx": stab, "entry_transient": entry}


# --------------------------------------------------------------- run driver

def analyse_arrays(t, contact, kd, kp, tff, rng, label=""):
    """Core analysis on already-loaded arrays. -> report dict."""
    t0, t1 = gait_band(t, tff)
    td_counts = []
    strides = []
    for i in range(4):
        e = touchdowns(t, contact[:, i])
        e = e[(e >= t0) & (e <= t1)]
        td_counts.append(len(e))
        if len(e) >= MIN_TD_PER_LEG:
            strides.append(float(np.median(np.diff(e))))
    if min(td_counts) < MIN_TD_PER_LEG:
        raise Unfit(f"touchdown counts {td_counts} -- not a sustained gait")
    stride = strides[0]  # leg A, consistent with the existing tools
    if max(strides) / min(strides) > 1.2:
        print(f"  {label} WARN: per-leg stride medians disagree "
              f"{np.round(strides, 3)}")
    kd_thr, kd_refused, agree = None, None, np.nan
    try:
        kd_thr = kd_gap_threshold(kd[(t >= t0) & (t <= t1)])
        m = (t >= t0) & (t <= t1)
        agree = float(((kd[m] < kd_thr) == (kp[m] < KP_THRESH)).mean())
    except Unfit as e:
        kd_refused = str(e)
    series, _ = window_series(t, contact, t0, t1, stride,
                              kd if kd_thr is not None else None, kd_thr)
    s = series["spread"]
    valid = ~np.isnan(s)
    if valid.sum() < MIN_WINDOWS:
        raise Unfit(f"only {int(valid.sum())} usable windows -- too short "
                    f"for a shape verdict")
    sv = s[valid]
    verdict = classify(sv, rng)
    hon = series["honesty"][valid]
    fl = series["flight"][valid]
    frv, lrv = series["fr"][valid], series["lr"][valid]
    nl = max(3, len(sv) // 3)
    rep = {
        "label": label, "stride": stride, "n_windows": int(valid.sum()),
        "band": (t0, t1),
        "spread_first": float(np.mean(sv[:3])),
        "spread_last": float(np.mean(sv[-3:])),
        "fr_last": float(np.nanmean(frv[-nl:])),
        "lr_last": float(np.nanmean(lrv[-nl:])),
        "flight_last": float(np.nanmean(fl[-nl:])),
        "honesty_first": (float(np.nanmean(hon[:3]))
                          if np.isfinite(hon[:3]).any() else np.nan),
        "honesty_last": (float(np.nanmean(hon[-nl:]))
                         if np.isfinite(hon[-nl:]).any() else np.nan),
        "kd_thr": kd_thr, "kd_refused": kd_refused, "kp_agree": agree,
        "stab_stride": verdict["stab_idx"] + WINDOW_STRIDES,
        **verdict,
    }
    return rep


def analyse_file(path, rng):
    t, contact, kd, kp, tff = load_run(path)
    return analyse_arrays(t, contact, kd, kp, tff, rng,
                          label=os.path.basename(path))


def print_report(r, indent="  "):
    hon = ("REFUSED: " + r["kd_refused"] if r["kd_refused"]
           else f"{r['honesty_first']:.2f} -> {r['honesty_last']:.2f} "
                f"(kd_thr {r['kd_thr']:.2f}, kp-rule agree "
                f"{r['kp_agree']:.3f})")
    print(f"{indent}{r['label']}: {r['n_windows']} windows, stride "
          f"{r['stride']:.3f} s")
    print(f"{indent}  spread {r['spread_first']:.2f} -> "
          f"{r['spread_last']:.2f}   rho {r['rho']:+.2f} p {r['p_trend']:.3f}"
          f"   shape: {r['shape'].upper()} (p_cp {r['p_cp']:.3f}, "
          f"width {r['width']}, cp win {r['cp_idx']})")
    print(f"{indent}  split f/r {r['fr_last']:.3f} vs l/r {r['lr_last']:.3f}"
          f"   flight {r['flight_last']:.2f}   gain honesty {hon}")
    entry = (f"YES, stabilises ~stride {r['stab_stride']}"
             if r["entry_transient"] else "no")
    print(f"{indent}  entry transient: {entry}")


def decide_arm(reports):
    """The S91 pre-committed rule."""
    triggering = [r for r in reports
                  if r["shape"] == "event-driven collapse"
                  or r["entry_transient"]]
    frac = len(triggering) / len(reports) if reports else 0.0
    if frac >= 0.5:
        arm = int(math.ceil(float(np.median(
            [r["stab_stride"] for r in triggering]))))
    else:
        arm = 0
    return arm, frac, len(triggering)


# ------------------------------------------------------------------ selftest

def synth_gait(K, stride, duty, phase_fn, dt=0.01, seed=0):
    """Synthetic 100 Hz contact matrix: leg i, stride k touches down at
    (k + phase_fn(i, k)) * stride for duty*stride."""
    T = (K + 1) * stride
    t = np.arange(0.0, T, dt)
    contact = np.zeros((len(t), 4), dtype=bool)
    for i in range(4):
        for k in range(K):
            a = (k + phase_fn(i, k)) * stride
            m = (t >= a) & (t < a + duty * stride)
            contact[m, i] = True
    tff = np.ones((len(t), 4))
    return t, contact, tff


def selftest():
    rng = np.random.default_rng(91)
    ok = True
    stride, K = 0.265, 60

    def check(name, cond, detail):
        nonlocal ok
        if cond:
            print(f"selftest {name}: {detail}  OK")
        else:
            print(f"SELFTEST FAIL {name}: {detail}")
            ok = False

    # kd arrays: planted honesty via a regime lagging contact
    def synth_kd(t, contact, lag_samples, bimodal=True):
        r = np.random.default_rng(7)
        kd = np.empty_like(contact, dtype=float)
        kp = np.empty_like(kd)
        for i in range(4):
            reg = np.roll(contact[:, i], lag_samples)
            if bimodal:
                kd[:, i] = np.where(reg, 0.4, 3.0) + 0.1 * r.standard_normal(len(t))
            else:
                kd[:, i] = 2.0 + 0.3 * r.standard_normal(len(t))
            kp[:, i] = np.where(reg, 30.0, 320.0)
        return kd, kp

    # 1. linear desync ramp -> monotone drift
    ramp = lambda i, k: (+0.5 if i in FRONT else -0.5) * 0.15 * k / K
    t, c, tff = synth_gait(K, stride, 0.4, ramp)
    kd, kp = synth_kd(t, c, 0)
    r1 = analyse_arrays(t, c, kd, kp, tff, rng, "ramp")
    check("ramp", r1["shape"] in ("monotone drift", "saturating plateau")
          and r1["shape"] != "event-driven collapse"
          and r1["spread_last"] > r1["spread_first"] + 0.05,
          f"shape {r1['shape']}, spread {r1['spread_first']:.2f}->"
          f"{r1['spread_last']:.2f}")
    check("ramp-signature", r1["fr_last"] > 2 * max(r1["lr_last"], 1e-3)
          or r1["lr_last"] < 0.01,
          f"f/r {r1['fr_last']:.3f} vs l/r {r1['lr_last']:.3f}")

    # 2. step collapse at stride 20 -> collapse, changepoint near plant
    step = lambda i, k: (0.25 if (i in FRONT and k >= 20) else 0.0)
    t, c, tff = synth_gait(K, stride, 0.4, step)
    kd, kp = synth_kd(t, c, 0)
    r2 = analyse_arrays(t, c, kd, kp, tff, rng, "step")
    # the CUSUM peak sits at the transition midpoint; the smearing window
    # puts that at plant - WINDOW_STRIDES/2 in window indices
    cp_expect = 20 - WINDOW_STRIDES / 2
    check("step", r2["shape"] == "event-driven collapse"
          and abs(r2["cp_idx"] - cp_expect) <= 2,
          f"shape {r2['shape']}, cp window {r2['cp_idx']} "
          f"(expect ~{cp_expect:.1f} for a plant at stride 20)")

    # 3. synchronized duty-0.40 -> flight 0.60 +- 0.02, stable
    sync = lambda i, k: 0.0
    t, c, tff = synth_gait(K, stride, 0.4, sync)
    kd, kp = synth_kd(t, c, 0)
    r3 = analyse_arrays(t, c, kd, kp, tff, rng, "sync")
    check("flight", abs(r3["flight_last"] - 0.6) <= 0.02
          and r3["shape"] == "stable",
          f"flight {r3['flight_last']:.3f} (plant 0.600), "
          f"shape {r3['shape']}")

    # 4. fooling case: white phase jitter, no drift -> stable, NOT collapse
    jr = np.random.default_rng(3)
    jit = {(i, k): float(0.05 * jr.standard_normal())
           for i in range(4) for k in range(K)}
    fool = lambda i, k: jit[(i, k)]
    t, c, tff = synth_gait(K, stride, 0.4, fool)
    kd, kp = synth_kd(t, c, 0)
    r4 = analyse_arrays(t, c, kd, kp, tff, rng, "jitter")
    check("fooling", r4["shape"] != "event-driven collapse",
          f"shape {r4['shape']} (must not be collapse)")

    # 5. gain honesty: regime lagging contact by 30% of a stride
    lag_samples = int(0.3 * stride / 0.01)
    t, c, tff = synth_gait(K, stride, 0.4, sync)
    kd, kp = synth_kd(t, c, 0)
    for i in range(4):
        reg = np.roll(c[:, i], lag_samples)
        kd[:, i] = np.where(reg, 0.4, 3.0)
        kp[:, i] = np.where(reg, 30.0, 320.0)
    r5 = analyse_arrays(t, c, kd, kp, tff, rng, "lagged-gains")
    # planted from the ACTUAL integer lag (int() truncation matters:
    # 7 samples of 0.07 s against a 0.106 s contact -> ~0.34, not 0.25)
    planted = 1.0 - (lag_samples * 0.01) / (0.4 * stride)
    check("honesty", r5["kd_refused"] is None
          and abs(r5["honesty_last"] - planted) <= 0.05
          and r5["kp_agree"] > 0.99,
          f"honesty {r5['honesty_last']:.3f} (plant {planted:.3f}), "
          f"agree {r5['kp_agree']:.3f}")

    # 6. unimodal kd must be REFUSED
    kd6, kp6 = synth_kd(t, c, 0, bimodal=False)
    r6 = analyse_arrays(t, c, kd6, kp6, tff, rng, "unimodal")
    check("refusal", r6["kd_refused"] is not None,
          f"kd_refused = {r6['kd_refused']}")

    print("SELFTEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# ----------------------------------------------------- scheduler CSV audit

SCHED_COLS = ("t_node", "leg", "leg_index", "master_index",
              "phase_err_rows", "snap_rows_pending", "in_stance_row",
              "contact_stable", "event")


def load_sched(path):
    """-> list of dict rows from a controller sched-events CSV."""
    rows = []
    with open(path, newline="") as fh:
        rdr = csv.DictReader(fh)
        missing = [c for c in SCHED_COLS if c not in (rdr.fieldnames or [])]
        if missing:
            raise Unfit(f"sched csv missing columns {missing}")
        for r in rdr:
            try:
                rows.append({
                    "t": float(r["t_node"]), "leg": r["leg"],
                    "idx": int(r["leg_index"]),
                    "master": int(r["master_index"]),
                    "pend": float(r["snap_rows_pending"]),
                    "in_st": r["in_stance_row"] == "1",
                    "c": r["contact_stable"] == "1",
                    "ev": r["event"],
                })
            except (ValueError, KeyError):
                continue  # torn tail line from a kill -9 is expected
    if len(rows) < 100:
        raise Unfit(f"only {len(rows)} sched rows")
    return rows


def sched_clock_spread(rows, n_rows):
    """-> (t[], spread_rows[]): max pairwise circular clock distance over
    the 4 legs, from the periodic sample blocks."""
    by_t = {}
    for r in rows:
        if r["ev"] == "":
            by_t.setdefault(round(r["t"], 3), {})[r["leg"]] = r["idx"]
    ts, spread = [], []
    for t in sorted(by_t):
        d = by_t[t]
        if len(d) < 4:
            continue
        vals = [d[l] for l in "ABCD"]
        worst = 0.0
        for a in range(4):
            for b in range(a + 1, 4):
                dd = abs(vals[a] - vals[b]) % n_rows
                worst = max(worst, min(dd, n_rows - dd))
        ts.append(t)
        spread.append(worst)
    return np.array(ts), np.array(spread)


def audit_sched_run(sched_path, torque_path, rng, n_rows=265):
    """The S8 cross-audit for one run. -> report dict."""
    rows = load_sched(sched_path)
    snaps = [r for r in rows if r["ev"] in ("SNAP_START", "SNAP_CLAMPED")]
    tds = [r for r in rows if r["ev"] == "TD"]
    if not tds:
        raise Unfit("no TD events in sched csv -- scheduler never saw a "
                    "touchdown (not an armed run?)")

    # 1. Every snap must ride a touchdown: same leg, same tick (the
    # controller emits TD then SNAP in one sched_tick call).
    orphan = 0
    td_ts = {}
    for r in tds:
        td_ts.setdefault(r["leg"], []).append(r["t"])
    for s in snaps:
        cand = td_ts.get(s["leg"], [])
        if not cand or min(abs(s["t"] - t) for t in cand) > 0.05:
            orphan += 1

    # 2. Snap-magnitude series per leg (P93-5): |pending| at the snap event
    # (the correction was just added; residual from a prior snap is ~0
    # given the 1/stride latch).
    mags = {}
    for s in snaps:
        mags.setdefault(s["leg"], []).append(abs(s["pend"]))
    trend = {}
    for leg, m in mags.items():
        m = np.array(m)
        if len(m) >= 8:
            rho = spearman(np.arange(len(m), dtype=float), m)
            p = shift_perm_p(np.arange(len(m), dtype=float), m, rng)
        else:
            rho, p = np.nan, np.nan
        trend[leg] = {"n": len(m), "mean_first5": float(m[:5].mean()),
                      "mean_last5": float(m[-5:].mean()),
                      "rho": rho, "p": p,
                      "clamped": sum(1 for s in snaps
                                     if s["leg"] == leg
                                     and s["ev"] == "SNAP_CLAMPED")}

    # 3. Clock spread (Tier 2 skip criterion): < 15 rows, no growth, 60 s.
    ts, spread = sched_clock_spread(rows, n_rows)
    if len(spread) >= 10:
        span = float(ts[-1] - ts[0])
        sp_rho = spearman(ts, spread)
        sp_p = shift_perm_p(ts, spread, rng)
        skip_ok = bool(spread.max() < 15 and
                       (sp_rho <= 0 or sp_p > 0.05) and span >= 60)
    else:
        span, sp_rho, sp_p, skip_ok = 0.0, np.nan, np.nan, False

    # 4. S2 recomputed INDEPENDENTLY: the torque CSV's measured contact
    # against the leg's OWN row phase from the sched csv. Clocks differ
    # (node vs webots) -- align per leg on first touchdown edges.
    honesty = np.nan
    if torque_path and os.path.exists(torque_path):
        t, contact, kd, kp, tff = load_run(torque_path)
        t0, t1 = gait_band(t, tff)
        deb = np.column_stack([debounce(contact[:, i], DEBOUNCE)
                               for i in range(4)])
        offs = []
        for i, leg in enumerate(LEGS):
            e = touchdowns(t, contact[:, i])
            e = e[(e >= t0) & (e <= t1)]
            if len(e) and leg in td_ts and td_ts[leg]:
                offs.append(td_ts[leg][0] - e[0])
        if offs:
            off = float(np.median(offs))
            num = den = 0
            per_leg = {l: ([], []) for l in LEGS}  # (t_node, in_st)
            for r in rows:
                if r["ev"] == "":
                    per_leg[r["leg"]][0].append(r["t"])
                    per_leg[r["leg"]][1].append(r["in_st"])
            for i, leg in enumerate(LEGS):
                st, sv = per_leg[leg]
                if len(st) < 10:
                    continue
                st = np.array(st)
                sv = np.array(sv, dtype=bool)
                m = (t >= t0) & (t <= t1) & deb[:, i]
                tq_t = t[m] + off
                j = np.searchsorted(st, tq_t)
                j = np.clip(j, 1, len(st) - 1)
                near = np.where(np.abs(st[j] - tq_t)
                                < np.abs(st[j - 1] - tq_t), j, j - 1)
                ok = np.abs(st[near] - tq_t) <= 0.02
                num += int(sv[near][ok].sum())
                den += int(ok.sum())
            honesty = num / den if den > 500 else np.nan

    return {"n_snaps": len(snaps), "orphan_snaps": orphan,
            "trend": trend, "clock_spread_max": (float(spread.max())
                                                 if len(spread) else np.nan),
            "clock_span_s": span, "clock_rho": sp_rho, "clock_p": sp_p,
            "tier2_skip_ok": skip_ok, "honesty_vs_own_row": honesty}


def print_sched_report(r, label):
    print(f"  {label}: {r['n_snaps']} snaps, {r['orphan_snaps']} orphaned")
    for leg in sorted(r["trend"]):
        t = r["trend"][leg]
        print(f"    leg {leg}: n {t['n']:3d}  first5 {t['mean_first5']:5.1f} "
              f"-> last5 {t['mean_last5']:5.1f} rows  rho {t['rho']:+.2f} "
              f"p {t['p']:.3f}  clamped {t['clamped']}")
    print(f"    clock spread max {r['clock_spread_max']:.1f} rows over "
          f"{r['clock_span_s']:.0f} s (rho {r['clock_rho']:+.2f} "
          f"p {r['clock_p']:.3f})  tier2-skip "
          f"{'OK' if r['tier2_skip_ok'] else 'NOT met'}")
    print(f"    S2 vs own row (independent): "
          f"{r['honesty_vs_own_row']:.2f}"
          if np.isfinite(r["honesty_vs_own_row"]) else
          "    S2 vs own row: no torque join")


def sched_selftest():
    """Planted synthetic snap audit; returns 0/1."""
    rng = np.random.default_rng(93)
    ok = True
    rows = []
    n_rows = 265
    stride = 0.265
    # 4 legs, 60 strides, snap magnitude decaying 20 -> 6 rows for A/B,
    # persistent 12 for C/D; periodic blocks every 10 ms; one orphan snap.
    t = 0.0
    idx = {l: 0 for l in "ABCD"}
    planted = {"A": np.linspace(20, 6, 40), "B": np.linspace(20, 6, 40),
               "C": np.full(40, 12.0), "D": np.full(40, 12.0)}
    for k in range(40):
        t = k * stride
        for leg in "ABCD":
            rows.append({"t": t, "leg": leg, "idx": idx[leg], "master": 0,
                         "pend": 0.0, "in_st": True, "c": True, "ev": "TD"})
            rows.append({"t": t, "leg": leg, "idx": idx[leg], "master": 0,
                         "pend": planted[leg][k], "in_st": True, "c": True,
                         "ev": "SNAP_START"})
        for j in range(20):
            tt = t + 0.01 * j
            for leg in "ABCD":
                rows.append({"t": tt, "leg": leg,
                             "idx": (idx[leg] + j) % n_rows, "master": j,
                             "pend": 0.0, "in_st": j < 10, "c": j < 10,
                             "ev": ""})
    rows.append({"t": t + 1.0, "leg": "A", "idx": 0, "master": 0,
                 "pend": 33.0, "in_st": False, "c": False,
                 "ev": "SNAP_START"})  # the orphan (1.0 s after any TD)
    # run the pieces directly (bypass load; same dict shape)
    snaps = [r for r in rows if r["ev"].startswith("SNAP")]
    tds = [r for r in rows if r["ev"] == "TD"]
    td_ts = {}
    for r in tds:
        td_ts.setdefault(r["leg"], []).append(r["t"])
    orphan = 0
    for s in snaps:
        cand = td_ts.get(s["leg"], [])
        if not cand or min(abs(s["t"] - tt) for tt in cand) > 0.05:
            orphan += 1
    if orphan != 1:
        print(f"SELFTEST FAIL sched-orphan: {orphan} (planted 1)")
        ok = False
    else:
        print("selftest sched-orphan: planted 1 -> 1  OK")
    magsA = np.array([abs(s["pend"]) for s in snaps if s["leg"] == "A"
                      and abs(s["pend"]) < 30])
    magsC = np.array([abs(s["pend"]) for s in snaps if s["leg"] == "C"])
    rhoA = spearman(np.arange(len(magsA), dtype=float), magsA)
    pA = shift_perm_p(np.arange(len(magsA), dtype=float), magsA, rng)
    pC = shift_perm_p(np.arange(len(magsC), dtype=float), magsC, rng)
    if not (rhoA < -0.9 and pA < 0.05 and pC > 0.05):
        print(f"SELFTEST FAIL sched-trend: A rho {rhoA:.2f} p {pA:.3f}, "
              f"C p {pC:.3f}")
        ok = False
    else:
        print(f"selftest sched-trend: decaying A rho {rhoA:.2f} p {pA:.3f}, "
              f"persistent C p {pC:.3f}  OK")
    ts, spread = sched_clock_spread(rows, n_rows)
    if len(spread) == 0 or spread.max() != 0:
        print(f"SELFTEST FAIL sched-spread: max "
              f"{spread.max() if len(spread) else 'none'} (planted 0)")
        ok = False
    else:
        print("selftest sched-spread: synced clocks -> 0 rows  OK")
    print("SCHED SELFTEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# ---------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--campaign", action="append", default=[])
    ap.add_argument("--run")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--sched", action="append", default=[],
                    help="campaign base dir holding cond/sched_runN.csv "
                         "(+ runN.csv for the independent S2 join)")
    ap.add_argument("--sched-selftest", action="store_true")
    ap.add_argument("--template-rows", type=int, default=265)
    args = ap.parse_args()
    rng = np.random.default_rng(88)
    if args.selftest:
        sys.exit(selftest())
    if args.sched_selftest:
        sys.exit(sched_selftest())

    if args.sched:
        for base in args.sched:
            base = os.path.expanduser(base)
            for cond in sorted(os.listdir(base)):
                cdir = os.path.join(base, cond)
                if not os.path.isdir(cdir):
                    continue
                files = sorted(f for f in os.listdir(cdir)
                               if re.match(r"^sched_run(\d+)\.csv$", f))
                if not files:
                    continue
                print(f"== {os.path.basename(base)}/{cond}")
                for f in files:
                    n = re.match(r"^sched_run(\d+)\.csv$", f).group(1)
                    tq = os.path.join(cdir, f"run{n}.csv")
                    try:
                        rep = audit_sched_run(
                            os.path.join(cdir, f),
                            tq if os.path.exists(tq) else None,
                            rng, n_rows=args.template_rows)
                        print_sched_report(rep, f)
                    except Unfit as e:
                        print(f"  {f} REFUSED: {e}")
        return

    if args.run:
        rep = analyse_file(os.path.expanduser(args.run), rng)
        print_report(rep, indent="")
        return

    if not args.campaign:
        ap.error("need --campaign, --run, or --selftest")

    reports, refused = [], []
    for base in args.campaign:
        base = os.path.expanduser(base)
        for cond in sorted(os.listdir(base)):
            cdir = os.path.join(base, cond)
            if not os.path.isdir(cdir) or cond.startswith("replay"):
                continue
            files = sorted(f for f in os.listdir(cdir) if RUN_RE.match(f))
            if not files:
                continue
            print(f"== {os.path.basename(base)}/{cond}")
            for f in files:
                try:
                    rep = analyse_file(os.path.join(cdir, f), rng)
                    rep["cond"] = f"{os.path.basename(base)}/{cond}"
                    reports.append(rep)
                    print_report(rep)
                except Unfit as e:
                    refused.append((cond, f, str(e)))
                    print(f"  {f} REFUSED: {e}")

    if not reports:
        print("no analysable runs.")
        return
    print()
    print("=" * 72)
    shapes = {}
    for r in reports:
        shapes[r["shape"]] = shapes.get(r["shape"], 0) + 1
    print(f"analysable runs: {len(reports)}   refused: {len(refused)}")
    print(f"shapes: {shapes}")
    fr_dom = sum(1 for r in reports
                 if r["fr_last"] > 2 * max(r["lr_last"], 1e-3))
    print(f"front/rear-dominant end state: {fr_dom}/{len(reports)}")
    hon = [r["honesty_last"] for r in reports
           if r["kd_refused"] is None and np.isfinite(r["honesty_last"])]
    if hon:
        print(f"stance-gain-on-contact, end state: median "
              f"{np.median(hon):.2f}  range [{min(hon):.2f}, {max(hon):.2f}]"
              f"  (n {len(hon)})")
    agree = [r["kp_agree"] for r in reports if np.isfinite(r["kp_agree"])]
    if agree:
        print(f"kd-vs-kp classifier agreement: min {min(agree):.3f} "
              f"median {np.median(agree):.3f} (crosscheck on {len(agree)} "
              f"runs)")
    ends = [r["spread_last"] for r in reports]
    print(f"end-state spread: median {np.median(ends):.2f}  "
          f"range [{min(ends):.2f}, {max(ends):.2f}]")
    entry = sum(1 for r in reports if r["entry_transient"])
    print(f"entry transients: {entry}/{len(reports)}")
    arm, frac, ntrig = decide_arm(reports)
    print()
    print(f"DECISION (S91 pre-committed rule): triggering runs "
          f"{ntrig}/{len(reports)} = {frac:.0%} -> "
          f"event_arm_strides = {arm}")


if __name__ == "__main__":
    main()
