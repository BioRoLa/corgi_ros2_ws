#!/usr/bin/env python3
"""Per-leg achieved-gamma decomposition -- audit 1.1 of the camber-authority
consistency handover (log S89, lean thread). Offline only, no simulator time.

For each run: the per-leg achieved gamma over the steady band, its error
against the commanded Ackermann pattern, and the four orthogonal projections
of the error vector that map onto the named suspects:

    common   (eA+eB+eC+eD)/4   shared bias (h-trap class errors)
    L/R      ((eA+eD)-(eB+eC))/4   the kp_h split (S39/S47) -- {A,D}=left
    F/R      ((eA+eB)-(eC+eD))/4   nose-heavy trim class (S57 d-r thread)
    diag     ((eA+eC)-(eB+eD))/4   the dir_abad partition -- the frame
                                   fingerprint (registered NULL, S89 P89-1)

plus each leg's stride-phase gamma ripple (folded on that leg's own
debounced touchdowns): peak-to-peak of 20 binned means and the first
harmonic's amplitude/phase. A wrong-frame gamma-dot would have corrupted
damping in proportion to gamma-dot, i.e. shown in ripple amp/phase on the
diagonal partition; a gain split shows in the means.

Loaders, debounce and the leg conventions are check_menger.py's own
(imported, not re-derived). The steady band is anchored exactly like
aggregate_menger.py: [overlap_start + start, overlap_start + end] where the
overlap is odom vs torque clock -- pass --odom-csv so the band matches the
S88 headline band; without it the band anchors on the torque clock and the
report says so.

Self-test (house rule: no analyser is trusted before it has failed to fool
itself): synthetic runs with planted per-leg mean offsets on ONE pure
partition at a time must recover that projection and put ~0 in the other
two structured ones (orthogonality -- the fooling case), and a planted
sinusoidal ripple on one leg must come back with its amplitude and phase.

Refusals (gate-analysers-against-invalid-input): missing columns, thin
band, fewer than 8 touchdowns on any leg, dead gait (flight/all-down/theta
gate from check_menger's thresholds), and achieved gamma ~= 0 under a
nonzero command (wrong file pairing).

Usage:
    python3 audit_gamma_decomp.py --torque-csv run1.csv --odom-csv odom_run1.csv \
        --gamma-in 10.0 --gamma-out 7.887 --gamma-dir 1
    python3 audit_gamma_decomp.py --campaign ~/corgi_runs/menger_acker
    python3 audit_gamma_decomp.py --selftest
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_menger import (LEGS, LR_SIGN, load_odom_csv, load_torque_csv,  # noqa: E402
                          touchdowns)

# Commanded magnitudes per campaign condition, DEGREES (aggregate_menger's
# REGISTERED lam_out values; in = the lambda_in the condition is named for).
COND_CMD = {
    "lam0": (0.0, 0.0, 0.0), "lam0_default": (0.0, 0.0, 0.0),
    "lam5_pos": (5.0, 4.406, +1.0), "lam5_neg": (5.0, 4.406, -1.0),
    "lam10_pos": (10.0, 7.887, +1.0), "lam10_neg": (10.0, 7.887, -1.0),
    "lam15_pos": (15.0, 10.725, +1.0), "lam15_neg": (15.0, 10.725, -1.0),
}

RIPPLE_BINS = 20
MIN_TD = 8


def commanded(gin, gout, gdir):
    """-> per-leg commanded gamma, deg (check_menger's pose-gate pattern)."""
    if gdir == 0.0:
        return np.zeros(4)
    out = []
    for leg in LEGS:
        mag = gin if gdir * LR_SIGN[leg] < 0 else gout
        out.append(gdir * LR_SIGN[leg] * mag)
    return np.array(out)


def projections(e):
    """-> dict of the four orthogonal projections of a per-leg 4-vector."""
    eA, eB, eC, eD = e
    return {
        "common": (eA + eB + eC + eD) / 4.0,
        "LR": ((eA + eD) - (eB + eC)) / 4.0,
        "FR": ((eA + eB) - (eC + eD)) / 4.0,
        "diag": ((eA + eC) - (eB + eD)) / 4.0,
    }


def ripple(t, g, td):
    """-> (p2p_deg, amp1_deg, peak_frac) of gamma folded on own touchdowns.

    Phase 0 = this leg's touchdown. First harmonic via the complex mean of
    the demeaned signal against exp(-2j pi phi); amp = 2|z| is the peak
    amplitude of the fitted cosine and peak_frac in [0,1) is the stride
    fraction where it peaks (for A cos(2 pi (phi-f0)), z = A/2 e^{-i2pi f0},
    so the peak is at -angle(z)/2pi).
    """
    if len(td) < MIN_TD:
        return None
    stride = float(np.median(np.diff(td)))
    m = (t >= td[0]) & (t <= td[-1])
    tt, gg = t[m], g[m]
    prev = td[np.searchsorted(td, tt, side="right") - 1]
    phi = ((tt - prev) / stride) % 1.0
    gd = gg - gg.mean()
    binned = np.array([gd[(phi >= b / RIPPLE_BINS) & (phi < (b + 1) / RIPPLE_BINS)].mean()
                       if ((phi >= b / RIPPLE_BINS) & (phi < (b + 1) / RIPPLE_BINS)).any()
                       else np.nan for b in range(RIPPLE_BINS)])
    z = np.mean(gd * np.exp(-2j * np.pi * phi))
    return (float(np.nanmax(binned) - np.nanmin(binned)),
            float(2.0 * np.abs(z)),
            float((-np.angle(z) / (2.0 * np.pi)) % 1.0))


def analyse(t, contact, gamma, theta, band, cmd, label, quiet=False):
    """Core analysis on already-loaded arrays. -> result dict or SystemExit."""
    a0, a1 = band
    m = (t >= a0) & (t <= a1)
    if int(m.sum()) < 1000:
        raise SystemExit(f"{label}: only {int(m.sum())} samples in the band "
                         f"-- nothing to measure.")
    tseg, cseg, gseg = t[m], contact[m], gamma[m]
    air = 100.0 * float((~cseg.any(axis=1)).mean())
    alld = 100.0 * float(cseg.all(axis=1).mean())
    th_max = float(theta[m].max()) if theta is not None else float("nan")
    th_ok = (100.0 < th_max < 165.0) if np.isfinite(th_max) else True
    if not (air > 25.0 and alld < 60.0 and th_ok):
        raise SystemExit(f"{label}: gait gate FAIL (flight {air:.1f}%, "
                         f"all-down {alld:.1f}%, theta max {th_max:.1f}) -- "
                         f"re-run material, not audit input.")
    ach = gseg.mean(axis=0)
    if np.abs(cmd).max() > 1.0 and np.abs(ach).max() < 0.2 * np.abs(cmd).max():
        raise SystemExit(f"{label}: commanded |gamma| up to "
                         f"{np.abs(cmd).max():.1f} deg but achieved "
                         f"{np.abs(ach).max():.2f} deg -- wrong file pairing "
                         f"or channel never engaged. Refusing.")
    err = ach - cmd
    rip = {}
    for i, leg in enumerate(LEGS):
        td = touchdowns(t, contact[:, i])
        td = td[(td >= a0) & (td <= a1)]
        if len(td) < MIN_TD:
            raise SystemExit(f"{label}: only {len(td)} leg-{leg} touchdowns "
                             f"in the band -- too few to fold a ripple.")
        rip[leg] = ripple(t, gamma[:, i], td)
    res = {"ach": ach, "err": err, "proj": projections(err),
           "rip_p2p": np.array([rip[leg][0] for leg in LEGS]),
           "rip_amp": np.array([rip[leg][1] for leg in LEGS]),
           "rip_ph": np.array([rip[leg][2] for leg in LEGS])}
    res["proj_ripamp"] = projections(res["rip_amp"])
    if not quiet:
        p = res["proj"]
        print(f"  {label}")
        print("    achieved " + " ".join(f"{leg} {ach[i]:+6.2f}" for i, leg in enumerate(LEGS))
              + "   cmd " + " ".join(f"{cmd[i]:+6.2f}" for i in range(4)))
        print("    error    " + " ".join(f"{leg} {err[i]:+6.2f}" for i, leg in enumerate(LEGS)))
        print(f"    proj(mean err)  common {p['common']:+6.3f}  L/R {p['LR']:+6.3f}"
              f"  F/R {p['FR']:+6.3f}  diag {p['diag']:+6.3f}   (deg)")
        pr = res["proj_ripamp"]
        print("    ripple amp1     " + " ".join(f"{leg} {res['rip_amp'][i]:5.3f}" for i, leg in enumerate(LEGS))
              + f"   proj: L/R {pr['LR']:+6.3f}  F/R {pr['FR']:+6.3f}  diag {pr['diag']:+6.3f}")
        print("    ripple peak at  " + " ".join(f"{leg} {res['rip_ph'][i]:5.2f}" for i, leg in enumerate(LEGS))
              + "   (stride fraction, 0 = own touchdown)")
    return res


def band_for(torque_csv, odom_csv, start, end):
    t, contact, gamma, theta = load_torque_csv(torque_csv)
    if odom_csv and os.path.exists(odom_csv):
        ot, _, _ = load_odom_csv(odom_csv)
        t0 = max(ot[0], t[0])
        t1 = min(ot[-1], t[-1])
        if t1 - t0 < 5.0:
            raise SystemExit(f"{torque_csv}: odom/torque overlap only "
                             f"{t1-t0:.2f} s -- streams from different runs?")
        anchor = "overlap (S88-matched)"
    else:
        t0, t1 = t[0], t[-1]
        anchor = "TORQUE CLOCK ONLY -- band not S88-comparable"
    a0, a1 = t0 + start, min(t1, t0 + end)
    return t, contact, gamma, theta, (a0, a1), anchor


def selftest():
    rng = np.random.default_rng(89)
    dt, T, stride, duty = 1e-3, 30.0, 0.26, 0.4
    t = np.arange(0.0, T, dt)
    contact = np.zeros((len(t), 4), dtype=bool)
    for i in range(4):
        ph = (t / stride) % 1.0
        contact[:, i] = ph < duty
    cmd = commanded(10.0, 7.887, +1.0)
    cases = {
        "LR": np.array([+0.5, -0.5, -0.5, +0.5]),
        "FR": np.array([+0.5, +0.5, -0.5, -0.5]),
        "diag": np.array([+0.5, -0.5, +0.5, -0.5]),
    }
    ok = True
    for name, offs in cases.items():
        gamma = cmd[None, :] + offs[None, :] + 0.02 * rng.standard_normal((len(t), 4))
        theta = np.full((len(t), 4), 120.0)
        res = analyse(t, contact, gamma, theta, (2.0, 28.0), cmd,
                      f"selftest[{name}]", quiet=True)
        got = res["proj"][name]
        others = [abs(res["proj"][k]) for k in cases if k != name]
        if abs(got - 0.5) > 0.02 or max(others) > 0.02:
            print(f"SELFTEST FAIL [{name}]: got {got:+.3f}, "
                  f"cross-talk {max(others):.3f}")
            ok = False
        else:
            print(f"selftest [{name}] planted +0.500 -> {got:+.3f}, "
                  f"cross-talk {max(others):.4f}  OK")
    # Planted ripple on leg B: 0.8 deg cosine peaking 30% into the stride.
    amp_true, ph_frac = 0.8, 0.30
    gamma = np.tile(cmd, (len(t), 1)).astype(float)
    phi_all = (t / stride) % 1.0
    gamma[:, 1] += amp_true * np.cos(2 * np.pi * (phi_all - ph_frac))
    theta = np.full((len(t), 4), 120.0)
    res = analyse(t, contact, gamma, theta, (2.0, 28.0), cmd,
                  "selftest[ripple]", quiet=True)
    amp, ph_got = res["rip_amp"][1], res["rip_ph"][1]
    if abs(amp - amp_true) > 0.05 or min(abs(ph_got - ph_frac),
                                         1 - abs(ph_got - ph_frac)) > 0.03:
        print(f"SELFTEST FAIL [ripple]: amp {amp:.3f} (want {amp_true}), "
              f"phase {ph_got:.3f} (want {ph_frac})")
        ok = False
    else:
        print(f"selftest [ripple] planted {amp_true} @ {ph_frac} stride -> "
              f"{amp:.3f} @ {ph_got:.3f}  OK")
    # Fooling case: gamma stuck at zero under a command must be REFUSED.
    gamma0 = 0.02 * rng.standard_normal((len(t), 4))
    try:
        analyse(t, contact, gamma0, theta, (2.0, 28.0), cmd,
                "selftest[refuse]", quiet=True)
        print("SELFTEST FAIL [refuse]: zero-gamma-under-command was accepted")
        ok = False
    except SystemExit:
        print("selftest [refuse] zero gamma under command -> refused  OK")
    print("SELFTEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--torque-csv")
    ap.add_argument("--odom-csv", help="band anchor only (S88 comparability)")
    ap.add_argument("--campaign", help="condition-dirs root; discovers "
                                       "run1..9 like aggregate_menger")
    ap.add_argument("--gamma-in", type=float, default=0.0)
    ap.add_argument("--gamma-out", type=float, default=0.0)
    ap.add_argument("--gamma-dir", type=float, default=0.0)
    ap.add_argument("--start", type=float, default=12.0)
    ap.add_argument("--end", type=float, default=30.0)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        sys.exit(selftest())

    if args.campaign:
        base = os.path.expanduser(args.campaign)
        pooled = {}
        for cond in sorted(os.listdir(base)):
            cdir = os.path.join(base, cond)
            if not os.path.isdir(cdir) or cond not in COND_CMD:
                continue
            gin, gout, gdir = COND_CMD[cond]
            cmd = commanded(gin, gout, gdir)
            print(f"== {cond} (cmd " + " ".join(f"{c:+.2f}" for c in cmd) + ")")
            for n in range(1, 10):
                tq = os.path.join(cdir, f"run{n}.csv")
                od = os.path.join(cdir, f"odom_run{n}.csv")
                if not (os.path.exists(tq) and os.path.exists(od)):
                    continue
                try:
                    t, contact, gamma, theta, band, anchor = band_for(
                        tq, od, args.start, args.end)
                    res = analyse(t, contact, gamma, theta, band, cmd,
                                  f"run{n} [{anchor}]")
                    pooled.setdefault(cond, []).append(res)
                except SystemExit as e:
                    print(f"  run{n} SKIPPED: {e}")
        print()
        print("== campaign summary: median projections of mean error (deg)")
        print(f"   {'condition':14} {'common':>8} {'L/R':>8} {'F/R':>8} "
              f"{'diag':>8} {'ripL/R':>8} {'ripdiag':>8}  n")
        for cond, rs in pooled.items():
            med = {k: float(np.median([r["proj"][k] for r in rs]))
                   for k in ("common", "LR", "FR", "diag")}
            ripd = float(np.median([r["proj_ripamp"]["diag"] for r in rs]))
            ripl = float(np.median([r["proj_ripamp"]["LR"] for r in rs]))
            print(f"   {cond:14} {med['common']:+8.3f} {med['LR']:+8.3f} "
                  f"{med['FR']:+8.3f} {med['diag']:+8.3f} {ripl:+8.3f} "
                  f"{ripd:+8.3f}  {len(rs)}")
        return

    if not args.torque_csv:
        ap.error("need --torque-csv (with optional --odom-csv), --campaign, "
                 "or --selftest")
    cmd = commanded(args.gamma_in, args.gamma_out, args.gamma_dir)
    t, contact, gamma, theta, band, anchor = band_for(
        args.torque_csv, args.odom_csv, args.start, args.end)
    print(f"{os.path.basename(args.torque_csv)}: band {band[0]:.2f}..{band[1]:.2f} s [{anchor}]")
    analyse(t, contact, gamma, theta, band, cmd,
            os.path.basename(args.torque_csv))


if __name__ == "__main__":
    main()
