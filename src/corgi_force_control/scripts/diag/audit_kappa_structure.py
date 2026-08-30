#!/usr/bin/env python3
"""Kappa structure -- audits 1.3 (wander vs desync) and 1.4 (stance-gated
headline) of the camber-authority consistency handover (log S89, lean
thread). Offline only.

1.3 WANDER (baseline conditions): the per-triple signed Menger kappa series
(check_menger's construction, verbatim) paired with the touchdown desync
over each triple's own time span. If the lam0 wander tracks desync
(Spearman rho, circular-shift permutation p -- shifts respect the series'
autocorrelation, a plain shuffle would overclaim), the 5-degree "threshold"
is presumptively a floor problem (S89 P89-4) and variance reduction, not
more lean, is the cure.

1.4 STANCE-GATE (all conditions): every leg-A touchdown is tagged with its
loading -- the mean number of legs in contact over +-quarter-stride around
it. The touchdown grid is split at the median loading and the Menger
triples are rebuilt FROM EACH SUBGRID, so all three points of a "loaded"
triple are loaded-phase samples (gating only the centre would not bite: a
triple's kappa is set by its whole +-1 m span). Camber's authority lives in
stance; if the loaded-subgrid 5-degree headline climbs out of the equally
gated floor while the full-band one stays dead, phase error is implicated
(S89 P89-5). A near-constant loading (range < 0.1 legs) flags the split as
noise, not physics.

The triple construction is aggregate_menger.run_kappa's, re-derived here
only to keep per-triple metadata and to allow subgrids; --crosscheck
asserts the full-grid re-derivation reproduces run_kappa's median on the
same files to 1e-9 and refuses to report otherwise (run once on a real S88
pair before trusting anything).

Self-tests: a synthetic circle (kappa 0.5 planted) with lateral odom noise
injected only during 4-stride low-load blocks -- the loaded-subgrid IQR
must stay tight while the light-subgrid IQR blows up (symmetric noise
cannot bias a median, so spread is the honest discriminator); a planted
kappa-desync correlation must be recovered with small p and an independent
pair must not.

Usage:
    python3 audit_kappa_structure.py --campaign ~/corgi_runs/menger_acker
    python3 audit_kappa_structure.py --odom-csv o.csv --torque-csv r.csv --crosscheck
    python3 audit_kappa_structure.py --selftest
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_menger import (LEGS, load_odom_csv, load_torque_csv,  # noqa: E402
                          touchdowns, menger)
from check_yaw_phase import dedupe_time  # noqa: E402
from aggregate_menger import run_kappa  # noqa: E402
from audit_contact_structure import circular_spread  # noqa: E402

N_PERM = 1000
MIN_TRIPLES = 6


def spearman(a, b):
    ra = np.argsort(np.argsort(a)).astype(float)
    rb = np.argsort(np.argsort(b)).astype(float)
    ra -= ra.mean()
    rb -= rb.mean()
    den = np.sqrt((ra ** 2).sum() * (rb ** 2).sum())
    return float((ra * rb).sum() / den) if den > 0 else 0.0


def shift_perm_p(a, b, rng):
    """-> p for |rho| under circular shifts of b against a."""
    obs = abs(spearman(a, b))
    n = len(a)
    hits = 0
    for _ in range(N_PERM):
        k = int(rng.integers(1, n))
        if abs(spearman(a, np.roll(b, k))) >= obs - 1e-12:
            hits += 1
    return (hits + 1) / (N_PERM + 1)


def prepare(ot, xy, ct, contact, start, chord, end=30.0):
    """Band, odom segment, leg-A touchdown grid and arc-length basis."""
    t0, t1 = max(ot[0], ct[0]), min(ot[-1], ct[-1])
    t1 = min(t1, t0 + end)
    a0 = t0 + start
    om = (ot >= a0) & (ot <= t1)
    if int(om.sum()) < 100:
        raise SystemExit(f"only {int(om.sum())} odom samples in the band.")
    t, seg = ot[om], xy[om]
    td_all = touchdowns(ct, contact[:, LEGS.index("A")])
    td = td_all[(td_all >= a0) & (td_all <= t1)]
    if len(td) < 5:
        raise SystemExit(f"only {len(td)} leg-A touchdowns in the band.")
    stride = float(np.median(np.diff(td)))
    step = np.hypot(np.diff(seg[:, 0]), np.diff(seg[:, 1]))
    s_cum = np.concatenate(([0.0], np.cumsum(step)))
    if s_cum[-1] < 2.5 * chord:
        raise SystemExit(f"only {s_cum[-1]:.2f} m of path in the band -- too "
                         f"short for {chord:.2f} m half-chords.")
    return {"t": t, "seg": seg, "td": td, "stride": stride,
            "s_cum": s_cum, "band": (a0, t1), "chord": chord}


def triples_from(td, pre, floor=2e-3):
    """run_kappa's triple loop on an arbitrary touchdown subgrid."""
    t, seg, s_cum, chord = pre["t"], pre["seg"], pre["s_cum"], pre["chord"]
    px = np.interp(td, t, seg[:, 0])
    py = np.interp(td, t, seg[:, 1])
    pts = np.column_stack((px, py))
    s_td = np.interp(td, t, s_cum)
    rows = []
    for k in range(len(td)):
        i = int(np.argmin(np.abs(s_td - (s_td[k] - chord))))
        j = int(np.argmin(np.abs(s_td - (s_td[k] + chord))))
        if i >= k or j <= k:
            continue
        if (s_td[k] - s_td[i] < 0.6 * chord
                or s_td[j] - s_td[k] < 0.6 * chord):
            continue
        kap, height = menger(pts[i], pts[k], pts[j])
        if height >= floor:
            rows.append({"tk": td[k], "ti": td[i], "tj": td[j], "kappa": kap})
    return rows


def desync_in(ct, contact, ta, tb):
    """-> pooled circular touchdown spread within [ta, tb], or None."""
    tds = []
    stride = None
    for i, leg in enumerate(LEGS):
        e = touchdowns(ct, contact[:, i])
        e = e[(e >= ta) & (e <= tb)]
        if leg == "A":
            if len(e) < 4:
                return None
            stride = float(np.median(np.diff(e)))
        tds.append(e)
    phases = np.concatenate([(e % stride) / stride for e in tds])
    return circular_spread(phases)


def loading_of(td, ct, contact, stride):
    """-> per-touchdown mean contact count over +-quarter-stride."""
    out = np.full(len(td), np.nan)
    counts = contact.sum(axis=1).astype(float)
    for n, tk in enumerate(td):
        m = (ct >= tk - 0.25 * stride) & (ct <= tk + 0.25 * stride)
        if int(m.sum()) >= 10:
            out[n] = counts[m].mean()
    return out


def iqr(a):
    return float(np.subtract(*np.percentile(a, [75, 25])))


def gated_split(pre, ct, contact):
    """Rebuild triples from the loaded / light touchdown subgrids."""
    td = pre["td"]
    load = loading_of(td, ct, contact, pre["stride"])
    ok = np.isfinite(load)
    if int(ok.sum()) < 10:
        return None
    td, load = td[ok], load[ok]
    thr = float(np.median(load))
    flat = bool(load.max() - load.min() < 0.1)
    top_rows = triples_from(td[load >= thr], pre)
    bot_rows = triples_from(td[load < thr], pre)
    if len(top_rows) < 3 or len(bot_rows) < 3:
        return None
    tk = np.array([r["kappa"] for r in top_rows])
    bk = np.array([r["kappa"] for r in bot_rows])
    return {"top": float(np.median(tk)), "bot": float(np.median(bk)),
            "top_iqr": iqr(tk), "bot_iqr": iqr(bk),
            "ntop": len(tk), "nbot": len(bk),
            "load_lo": float(load.min()), "load_hi": float(load.max()),
            "flat_loading": flat}


def run_structure(odom_csv, torque_csv, start, chord, crosscheck=False):
    ot, xy, quat = load_odom_csv(odom_csv)
    ot, xy, quat = dedupe_time(ot, xy, quat)
    ct, contact, gamma, theta = load_torque_csv(torque_csv)
    pre = prepare(ot, xy, ct, contact, start, chord)
    rows = triples_from(pre["td"], pre)
    if len(rows) < 3:
        raise SystemExit(f"only {len(rows)} valid triples.")
    med = float(np.median([r["kappa"] for r in rows]))
    if crosscheck:
        ref, n_ref = run_kappa(odom_csv, torque_csv, start, chord)
        if not (np.isfinite(ref) and abs(med - ref) < 1e-9
                and n_ref == len(rows)):
            raise SystemExit(
                f"CROSSCHECK FAIL: this median {med:+.6f} ({len(rows)} "
                f"triples) vs run_kappa {ref:+.6f} ({n_ref}) -- refusing to "
                f"report from a construction that disagrees with the S88 "
                f"pipeline.")
        print(f"  crosscheck vs aggregate_menger.run_kappa: "
              f"{med:+.5f} == {ref:+.5f}, {len(rows)} triples  OK")
    for r in rows:
        r["desync"] = desync_in(ct, contact, r["ti"], r["tj"])
    return rows, med, pre, ct, contact


def wander_stats(rows, rng):
    ok = [r for r in rows if r["desync"] is not None]
    if len(ok) < MIN_TRIPLES:
        raise SystemExit(f"only {len(ok)} triples with a desync estimate.")
    k = np.array([abs(r["kappa"]) for r in ok])
    d = np.array([r["desync"] for r in ok])
    return spearman(k, d), shift_perm_p(k, d, rng), len(ok)


def selftest():
    rng = np.random.default_rng(1204)
    ok = True
    # --- Circle kappa=0.5, lateral noise only during 4-stride low blocks ----
    dt, T, stride, v, R = 5e-3, 40.0, 0.26, 0.25, 2.0
    t = np.arange(0.0, T, dt)
    th = v * t / R
    xy = np.column_stack((R * np.sin(th), R * (1 - np.cos(th))))
    ct = np.arange(0.0, T, 1e-3)
    contact = np.zeros((len(ct), 4), dtype=bool)
    phase = (ct / stride) % 1.0
    lowblock_c = (((ct / stride).astype(int) // 4) % 2 == 1)
    for i in range(4):
        contact[:, i] = phase < 0.4
    for i in range(1, 4):
        contact[lowblock_c & (contact[:, i]), i] = False
    lowblock_t = (((t / stride).astype(int) // 4) % 2 == 1)
    noisy = xy.copy()
    # True radial of this circle (centre (0, R)): displacement must be
    # LATERAL to the path -- an along-path slide leaves kappa untouched
    # and the test would pass vacuously.
    normal = np.column_stack((np.sin(th), -np.cos(th)))
    # Smooth lateral wobble (white noise at 5 ms would inflate the shared
    # arc-length basis and starve every subgrid of chord-length triples --
    # real odom noise is band-limited, so the plant should be too).
    wobble = (0.010 * np.sin(2 * np.pi * 1.3 * t + 0.7)
              + 0.006 * np.sin(2 * np.pi * 0.7 * t))
    noisy[lowblock_t] += normal[lowblock_t] * wobble[lowblock_t, None]
    pre = prepare(t, noisy, ct, contact, 2.0, 1.0, end=38.0)
    g = gated_split(pre, ct, contact)
    if g is None or g["flat_loading"]:
        print(f"SELFTEST FAIL stancegate: no usable split ({g})")
        ok = False
    elif abs(g["top"] - 0.5) > 0.05 or g["bot_iqr"] < 2.0 * g["top_iqr"]:
        print(f"SELFTEST FAIL stancegate: top {g['top']:+.4f} "
              f"IQR {g['top_iqr']:.4f} vs bot {g['bot']:+.4f} "
              f"IQR {g['bot_iqr']:.4f}")
        ok = False
    else:
        print(f"selftest stancegate: planted +0.500 -> loaded-grid "
              f"{g['top']:+.4f} IQR {g['top_iqr']:.4f} vs light-grid "
              f"{g['bot']:+.4f} IQR {g['bot_iqr']:.4f} "
              f"({g['ntop']}/{g['nbot']}, load {g['load_lo']:.2f}.."
              f"{g['load_hi']:.2f})  OK")
    # --- Clean circle sanity: full grid must recover 0.5 exactly ------------
    pre_clean = prepare(t, xy, ct, contact, 2.0, 1.0, end=38.0)
    rows_clean = triples_from(pre_clean["td"], pre_clean)
    med_clean = float(np.median([r["kappa"] for r in rows_clean]))
    if abs(med_clean - 0.5) > 0.01:
        print(f"SELFTEST FAIL circle: median {med_clean:+.4f} (want +0.500)")
        ok = False
    else:
        print(f"selftest circle: clean full-grid median {med_clean:+.4f}  OK")
    # --- Correlation recovery ------------------------------------------------
    n = 40
    d = np.abs(np.cumsum(rng.standard_normal(n))) / 5 + 0.2
    k = 0.05 + 0.3 * d + 0.02 * rng.standard_normal(n)
    rho = spearman(k, d)
    p = shift_perm_p(k, d, rng)
    if rho < 0.6 or p > 0.05:
        print(f"SELFTEST FAIL corr: planted rho -> {rho:.2f}, p {p:.3f}")
        ok = False
    else:
        print(f"selftest corr: planted -> rho {rho:.2f}, p {p:.3f}  OK")
    k_ind = 0.05 + 0.02 * np.abs(rng.standard_normal(n))
    p_ind = shift_perm_p(k_ind, d, rng)
    if p_ind < 0.05:
        print(f"SELFTEST FAIL corr-null: independent pair got p {p_ind:.3f}")
        ok = False
    else:
        print(f"selftest corr-null: independent -> p {p_ind:.3f}  OK")
    print("SELFTEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--odom-csv")
    ap.add_argument("--torque-csv")
    ap.add_argument("--campaign")
    ap.add_argument("--start", type=float, default=12.0)
    ap.add_argument("--chord", type=float, default=1.0)
    ap.add_argument("--crosscheck", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    rng = np.random.default_rng(88)
    if args.selftest:
        sys.exit(selftest())

    if args.campaign:
        base = os.path.expanduser(args.campaign)
        full, gated = {}, {}
        for cond in sorted(os.listdir(base)):
            cdir = os.path.join(base, cond)
            if not os.path.isdir(cdir) or cond.startswith("replay"):
                continue
            printed = False
            for n in range(1, 10):
                tq = os.path.join(cdir, f"run{n}.csv")
                od = os.path.join(cdir, f"odom_run{n}.csv")
                if not (os.path.exists(tq) and os.path.exists(od)):
                    continue
                if not printed:
                    print(f"== {cond}")
                    printed = True
                try:
                    rows, med, pre, ct, contact = run_structure(
                        od, tq, args.start, args.chord)
                    line = f"  run{n}: median {med:+.5f} ({len(rows)} triples)"
                    g = gated_split(pre, ct, contact)
                    if g:
                        line += (f"   loaded {g['top']:+.5f} "
                                 f"light {g['bot']:+.5f} "
                                 f"({g['ntop']}/{g['nbot']}, load "
                                 f"{g['load_lo']:.2f}..{g['load_hi']:.2f}"
                                 + (", FLAT" if g["flat_loading"] else "")
                                 + ")")
                        if not g["flat_loading"]:
                            gated.setdefault(cond, []).append(g["top"])
                    if cond in ("lam0", "lam0_default"):
                        rho, p, nn = wander_stats(rows, rng)
                        line += f"   |k|~desync rho {rho:+.2f} p {p:.3f} (n {nn})"
                    full.setdefault(cond, []).append(med)
                    print(line)
                except SystemExit as e:
                    print(f"  run{n} SKIPPED: {e}")
        print()
        print("== stance-gate summary: headline h = (pos - neg)/2, 1/m")
        for lam in ("lam5", "lam10", "lam15"):
            pos, neg = f"{lam}_pos", f"{lam}_neg"
            if pos in full and neg in full:
                h_full = 0.5 * (np.median(full[pos]) - np.median(full[neg]))
                line = f"   {lam:6}: full {h_full:+.4f}"
                if pos in gated and neg in gated:
                    h_gate = 0.5 * (np.median(gated[pos]) - np.median(gated[neg]))
                    line += f"   loaded-grid {h_gate:+.4f}"
                print(line)
        if "lam0" in full:
            arr = np.array(full["lam0"])
            print(f"   lam0 floor: run medians {np.round(arr, 3)}  "
                  f"3sigma {3*float(arr.std(ddof=1) if len(arr) > 1 else 0):.3f}")
            if "lam0" in gated:
                gar = np.array(gated["lam0"])
                print(f"   lam0 loaded-grid: {np.round(gar, 3)}  "
                      f"3sigma {3*float(gar.std(ddof=1) if len(gar) > 1 else 0):.3f}")
        return

    if not (args.odom_csv and args.torque_csv):
        ap.error("need --odom-csv AND --torque-csv, --campaign, or --selftest")
    rows, med, pre, ct, contact = run_structure(
        args.odom_csv, args.torque_csv, args.start, args.chord,
        crosscheck=args.crosscheck)
    print(f"{os.path.basename(args.odom_csv)}: median {med:+.5f} "
          f"({len(rows)} triples)")
    g = gated_split(pre, ct, contact)
    if g:
        print(f"  loaded-grid {g['top']:+.5f} IQR {g['top_iqr']:.4f}  "
              f"light-grid {g['bot']:+.5f} IQR {g['bot_iqr']:.4f} "
              f"({g['ntop']}/{g['nbot']}, load "
              f"{g['load_lo']:.2f}..{g['load_hi']:.2f}"
              + (", FLAT" if g["flat_loading"] else "") + ")")
    rho, p, nn = wander_stats(rows, rng)
    print(f"  |kappa| ~ desync: Spearman rho {rho:+.2f}, "
          f"circular-shift p {p:.3f} (n {nn})")


if __name__ == "__main__":
    main()
