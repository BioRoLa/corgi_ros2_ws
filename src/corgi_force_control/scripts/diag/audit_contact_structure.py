#!/usr/bin/env python3
"""Contact structure vs camber -- audit 1.2 of the camber-authority
consistency handover (log S89, lean thread). Offline only.

Per run: per-leg duty (debounced in_contact fraction over the steady band),
the F/R and L/R duty splits, and touchdown desync -- desync_vs_speed.py's
circular spread 1 - |mean(exp(2j pi phase))| over all four legs' debounced
touchdown phases against the leg-A median stride -- plus each leg's mean
phase offset from leg A in milliseconds.

The question (S89 P89-3): does the nose-heavy F/R split (~0.45/0.34 at the
config of record, S57 demand-reduction thread) move with commanded lambda
and with direction? If it is load-bearing for the direction asymmetry it
must differ between _pos and _neg at matched lambda.

Loaders/debounce from check_menger.py. Band anchoring as in
audit_gamma_decomp.py (odom overlap when available).

Self-test: planted square-wave contacts with duties 0.45/0.45/0.34/0.34 and
a +30 ms delay on leg D must come back with the duties to 0.01, the D
offset to 2 ms, and the analytic circular spread for phases {0,0,0,delta}.

Refusals: no contact edges, fewer than 8 strides in the band, thin band.

Usage:
    python3 audit_contact_structure.py --torque-csv run1.csv --odom-csv odom_run1.csv
    python3 audit_contact_structure.py --campaign ~/corgi_runs/menger_acker
    python3 audit_contact_structure.py --selftest
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_menger import (DEBOUNCE, LEGS, debounce, load_odom_csv,  # noqa: E402
                          load_torque_csv, touchdowns)

MIN_STRIDES = 8


def circular_spread(phases):
    """-> 1 - |mean unit phasor| over phases in [0,1). 0 = one virtual leg."""
    z = np.exp(2j * np.pi * np.asarray(phases))
    return float(1.0 - np.abs(z.mean()))


def analyse(t, contact, band, label, quiet=False):
    a0, a1 = band
    m = (t >= a0) & (t <= a1)
    if int(m.sum()) < 1000:
        raise SystemExit(f"{label}: only {int(m.sum())} samples in band.")
    duty = np.array([
        float(debounce(contact[m][:, i], DEBOUNCE).mean()) for i in range(4)])
    td = {}
    for i, leg in enumerate(LEGS):
        e = touchdowns(t, contact[:, i])
        e = e[(e >= a0) & (e <= a1)]
        if len(e) < MIN_STRIDES:
            raise SystemExit(f"{label}: only {len(e)} leg-{leg} touchdowns "
                             f"in the band -- no stride structure to audit.")
        td[leg] = e
    stride = float(np.median(np.diff(td["A"])))
    phases, offs_ms = [], {}
    for leg in LEGS:
        ph = (td[leg] % stride) / stride
        phases.extend(ph.tolist())
        # Mean phase offset from leg A, circular, in ms.
        za = np.exp(2j * np.pi * (td["A"] % stride) / stride).mean()
        zl = np.exp(2j * np.pi * ph).mean()
        d = np.angle(zl / za) / (2 * np.pi)
        offs_ms[leg] = float(d * stride * 1000.0)
    res = {
        "duty": duty,
        "fr_split": float(0.5 * (duty[0] + duty[1]) - 0.5 * (duty[2] + duty[3])),
        "lr_split": float(0.5 * (duty[0] + duty[3]) - 0.5 * (duty[1] + duty[2])),
        "desync": circular_spread(phases),
        "offs_ms": offs_ms, "stride_ms": stride * 1000.0,
    }
    if not quiet:
        print(f"  {label}: duty "
              + " ".join(f"{leg} {duty[i]:.3f}" for i, leg in enumerate(LEGS))
              + f"   F/R {res['fr_split']:+.3f}  L/R {res['lr_split']:+.3f}"
              + f"   desync {res['desync']:.3f}   offsets(ms) "
              + " ".join(f"{leg} {offs_ms[leg]:+5.0f}" for leg in LEGS[1:])
              + f"   stride {res['stride_ms']:.0f} ms")
    return res


def band_for(torque_csv, odom_csv, start, end):
    t, contact, _, _ = load_torque_csv(torque_csv)
    if odom_csv and os.path.exists(odom_csv):
        ot, _, _ = load_odom_csv(odom_csv)
        t0, t1 = max(ot[0], t[0]), min(ot[-1], t[-1])
        if t1 - t0 < 5.0:
            raise SystemExit(f"{torque_csv}: odom/torque overlap only "
                             f"{t1-t0:.2f} s.")
    else:
        t0, t1 = t[0], t[-1]
    return t, contact, (t0 + start, min(t1, t0 + end))


def selftest():
    dt, T, stride = 1e-3, 30.0, 0.26
    t = np.arange(0.0, T, dt)
    duties = [0.45, 0.45, 0.34, 0.34]
    delay = [0.0, 0.0, 0.0, 0.030]
    contact = np.zeros((len(t), 4), dtype=bool)
    for i in range(4):
        ph = ((t - delay[i]) / stride) % 1.0
        contact[:, i] = ph < duties[i]
    res = analyse(t, contact, (2.0, 28.0), "selftest", quiet=True)
    ok = True
    if np.abs(res["duty"] - np.array(duties)).max() > 0.01:
        print(f"SELFTEST FAIL duty: {res['duty']}")
        ok = False
    else:
        print(f"selftest duty {np.round(res['duty'], 3)}  OK")
    if abs(res["offs_ms"]["D"] - 30.0) > 2.0:
        print(f"SELFTEST FAIL offset D: {res['offs_ms']['D']:.1f} ms")
        ok = False
    else:
        print(f"selftest offset D {res['offs_ms']['D']:+.1f} ms (want +30)  OK")
    frac = delay[3] / stride
    want = 1.0 - abs((3.0 + np.exp(2j * np.pi * frac)) / 4.0)
    if abs(res["desync"] - want) > 0.005:
        print(f"SELFTEST FAIL desync: {res['desync']:.4f} want {want:.4f}")
        ok = False
    else:
        print(f"selftest desync {res['desync']:.4f} (analytic {want:.4f})  OK")
    # Fooling case: constant contact must be refused (no strides).
    try:
        analyse(t, np.ones((len(t), 4), dtype=bool), (2.0, 28.0),
                "selftest[flat]", quiet=True)
        print("SELFTEST FAIL: constant contact accepted")
        ok = False
    except SystemExit:
        print("selftest constant contact -> refused  OK")
    print("SELFTEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--torque-csv")
    ap.add_argument("--odom-csv")
    ap.add_argument("--campaign")
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
            if not os.path.isdir(cdir) or cond.startswith("replay"):
                continue
            hits = False
            for n in range(1, 10):
                tq = os.path.join(cdir, f"run{n}.csv")
                od = os.path.join(cdir, f"odom_run{n}.csv")
                if not (os.path.exists(tq) and os.path.exists(od)):
                    continue
                if not hits:
                    print(f"== {cond}")
                    hits = True
                try:
                    t, contact, band = band_for(tq, od, args.start, args.end)
                    res = analyse(t, contact, band, f"run{n}")
                    pooled.setdefault(cond, []).append(res)
                except SystemExit as e:
                    print(f"  run{n} SKIPPED: {e}")
        print()
        print("== campaign summary (medians)")
        print(f"   {'condition':14} {'dutyA':>6} {'dutyB':>6} {'dutyC':>6} "
              f"{'dutyD':>6} {'F/R':>7} {'L/R':>7} {'desync':>7}  n")
        for cond, rs in pooled.items():
            d = np.median(np.stack([r["duty"] for r in rs]), axis=0)
            fr = float(np.median([r["fr_split"] for r in rs]))
            lr = float(np.median([r["lr_split"] for r in rs]))
            ds = float(np.median([r["desync"] for r in rs]))
            print(f"   {cond:14} {d[0]:6.3f} {d[1]:6.3f} {d[2]:6.3f} "
                  f"{d[3]:6.3f} {fr:+7.3f} {lr:+7.3f} {ds:7.3f}  {len(rs)}")
        return

    if not args.torque_csv:
        ap.error("need --torque-csv, --campaign, or --selftest")
    t, contact, band = band_for(args.torque_csv, args.odom_csv,
                                args.start, args.end)
    print(f"{os.path.basename(args.torque_csv)}: band "
          f"{band[0]:.2f}..{band[1]:.2f} s")
    analyse(t, contact, band, os.path.basename(args.torque_csv))


if __name__ == "__main__":
    main()
