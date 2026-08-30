#!/usr/bin/env python3
"""Flight fraction against ACHIEVED camber, across every banked camber run.

Log S174 (Stage 3 cambered thread). Offline only, no simulator time.

THE QUESTION. S172 measured a paired flight cost of 7.1 points at lambda = 10
(24.4% -> 17.2%) and the handover asks whether it scales with lambda. A full
lambda in {0, 5, 10, 15} grid is already banked in ~/corgi_runs/menger_acker/,
so the shape has a zero-sim-time first answer. This tool produces it.

WHAT IT DOES NOT RE-DERIVE. Every number here comes out of the shipped tools:

    audit_gamma_decomp.band_for()      the S88-matched band anchoring
    audit_gamma_decomp.projections()   the L/R, F/R, common, diag basis
    check_menger.load_torque_csv()     the long-format capture loader
    check_menger.debounce(), DEBOUNCE  the same debounce the detectors use
    playback_ratio._series(), dominant_period()   the fidelity check

S173's lesson, from the other direction: the frame error that made the Jacobian
look singular came from building a second implementation of shipped kinematics.
Port through the shipped constants.

THE GATE IS REPORTED, NOT RAISED, AND NOT LOWERED. audit_gamma_decomp's gait
gate (flight > 25%) refuses S167's own control cell on 3 of 3 runs and refuses
both cells of S172's pair. It is another session's tool and other sections
depend on its behaviour, so it is left ALONE (S171 S5, S172). This tool imports
band_for/projections and reports the gate verdict as a column.

RAW vs DEBOUNCED, ON THE SAME BAND. audit_gamma_decomp:124 computes air from
RAW contact; gait_mode:85 debounces first. Those two also use different windows
(overlap+12..+30 vs the last 20 s), so comparing them across tools confounds
debounce with band. Both are computed here on the SAME band so the difference is
the debounce alone. S104 is why this matters: "costs flight" has already been a
metric artifact in this project once.

USAGE

    python3 flight_vs_camber.py --selftest
    python3 flight_vs_camber.py \
        --campaign ~/corgi_runs/menger_acker      --label menger    --kflight 12000 \
        --campaign ~/corgi_runs/menger_acker_final --label mengerF  --kflight 12000 \
        --campaign ~/corgi_runs/camber_pattern    --label pattern   --kflight 7150 \
        --csv /tmp/flight_vs_camber.csv
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from audit_gamma_decomp import band_for, projections            # noqa: E402
from check_menger import DEBOUNCE, LEGS, debounce, touchdowns    # noqa: E402
from playback_ratio import V070_STRIDE_S                         # noqa: E402
from playback_ratio import _series, dominant_period              # noqa: E402
from playback_ratio import Unfit as PlaybackUnfit                # noqa: E402

# audit_gamma_decomp's own thresholds, reported not applied.
GATE_AIR, GATE_ALLD, GATE_TH_LO, GATE_TH_HI = 25.0, 60.0, 100.0, 165.0

# P-U-4's validity bars (S174 S5).
MIN_BAND_SAMPLES = 1000
MIN_TD_PER_LEG = 8
PLAYBACK_TOL = 0.05

# S172's published numbers, used as the known-answer selftest.
S172 = {"lam0": {"air": 24.4, "LR": +0.121, "FR": +0.017},
        "lr10": {"air": 17.2, "LR": +9.676, "FR": -0.033}}


class Unfit(Exception):
    """This run cannot be measured. Never silently swallowed -- counted."""


def air_and_alld(contact):
    """-> (air%, all-down%) on RAW contact, audit_gamma_decomp:124's formula."""
    return (100.0 * float((~contact.any(axis=1)).mean()),
            100.0 * float(contact.all(axis=1).mean()))


def debounced(contact):
    """-> per-leg debounced contact, gait_mode:85's treatment."""
    return np.column_stack(
        [debounce(contact[:, i], DEBOUNCE) for i in range(contact.shape[1])])


def playback_ratio_theta(torque_csv):
    """-> theta period ratio against the v070 stride, or None if unmeasurable.

    Same functions playback_ratio.run_report uses, without its printing.
    """
    try:
        _leg, dt, ch = _series(torque_csv)
        period, _peak = dominant_period(ch["theta"], dt)
    except (PlaybackUnfit, Exception):        # noqa: BLE001 -- reported, not raised
        return None
    return period / V070_STRIDE_S


def measure(torque_csv, odom_csv, start, end):
    """-> dict of everything one run contributes. Raises Unfit, never SystemExit."""
    try:
        t, contact, gamma, theta, (a0, a1), anchor = band_for(
            torque_csv, odom_csv, start, end)
    except SystemExit as e:
        # band_for signals a bad pairing / unreadable capture this way, and
        # SystemExit is NOT an Exception -- it has to be named.
        raise Unfit(str(e))

    m = (t >= a0) & (t <= a1)
    n = int(m.sum())
    if n < MIN_BAND_SAMPLES:
        raise Unfit(f"only {n} samples in the band")

    c_raw = contact[m]
    c_deb = debounced(c_raw)
    air_raw, alld_raw = air_and_alld(c_raw)
    air_deb, alld_deb = air_and_alld(c_deb)

    ach = gamma[m].mean(axis=0)
    proj = projections(ach)
    th_max = float(theta[m].max())

    td = [len(touchdowns(t[m], c_raw[:, i])) for i in range(4)]
    ratio = playback_ratio_theta(torque_csv)

    gate_ok = (air_raw > GATE_AIR and alld_raw < GATE_ALLD
               and GATE_TH_LO < th_max < GATE_TH_HI)
    valid = (n >= MIN_BAND_SAMPLES and min(td) >= MIN_TD_PER_LEG
             and ratio is not None and abs(ratio - 1.0) <= PLAYBACK_TOL)

    return {
        "air_raw": air_raw, "alld_raw": alld_raw,
        "air_deb": air_deb, "alld_deb": alld_deb,
        "ach": ach, "LR": proj["LR"], "FR": proj["FR"],
        "common": proj["common"], "diag": proj["diag"],
        "theta_max": th_max, "n": n, "td_min": min(td),
        "playback": ratio, "band": (a0, a1), "anchor": anchor,
        "gate_ok": gate_ok, "valid": valid,
    }


def split_half_flight(torque_csv, odom_csv, start, end):
    """-> (flight in the first half of the band, flight in the second).

    Same split-half design run_variance_budget.py uses for swept beta and
    min vLeg. Flight is not one of its two metrics, so rather than widen a
    shared tool the decomposition is applied here and its OWN decompose() /
    budget_table() are imported unchanged.
    """
    t, contact, _g, _th, (a0, a1), _anchor = band_for(
        torque_csv, odom_csv, start, end)
    mid = 0.5 * (a0 + a1)
    out = []
    for lo, hi in ((a0, mid), (mid, a1)):
        m = (t >= lo) & (t <= hi)
        if int(m.sum()) < MIN_BAND_SAMPLES // 2:
            raise Unfit(f"half [{lo:.1f},{hi:.1f}] has {int(m.sum())} samples")
        out.append(air_and_alld(contact[m])[0])
    return tuple(out)


def ballistic_report(dirs, start, end):
    """Is the measured flight drop the GAIT, or the contact detector moving?

    Timeline Stage 3 task 4: "touchdown detection under camber -- contact
    timing shifts with lean because the rim reaches ground at a different
    alpha." That is the null hypothesis for S172's flight cost, so it is the
    same measurement, and it needs a flight signal the contact detector cannot
    influence. Two of them:

      BALLISTIC   over each detected flight window the body is in free fall,
                  so d2z/dt2 = -9.81. If the detector is under-reporting
                  flight under camber, the intervals it DOES report stay
                  ballistic while the ones it misses hide inside "stance" --
                  so the tell is not the windows' own curvature but the
                  fraction of the stride that is ballistic while labelled down.

      APEX        peak-to-trough body z per stride needs no contact signal at
                  all. A genuinely shorter aerial phase lowers the apex; a
                  detector that moved does not.
    """
    from touchdown_velocity_angle import load_odom_xyzt   # noqa: E402
    print("ballistic / apex cross-check on the flight fraction.")
    print("g = -9.81 m/s^2 is the known answer; the body is unforced in "
          "flight whatever the contact flag says.\n")
    print(f"{'cell':26} {'flight%':>8} {'zdd_air':>9} {'zdd_down':>9} "
          f"{'air_ballistic%':>15} {'apex_mm':>8} {'n_win':>6}")
    for d in dirs:
        d = os.path.expanduser(d)
        for tq in sorted(glob.glob(os.path.join(d, "run[0-9].csv"))):
            run = os.path.basename(tq)[3:-4]
            od = os.path.join(d, f"odom_run{run}.csv")
            if not os.path.exists(od):
                continue
            label = "/".join(d.rstrip("/").split("/")[-1:]) + f"/run{run}"
            try:
                t, contact, _g, _th, (a0, a1), _a = band_for(tq, od, start, end)
                ot, _ox, _oy, oz = load_odom_xyzt(od)
            except (SystemExit, Exception) as e:      # noqa: BLE001
                print(f"{label:26} unmeasurable -- {e}")
                continue
            m = (t >= a0) & (t <= a1)
            tb, cb = t[m], debounced(contact[m])
            up = ~cb.any(axis=1)
            zi = np.interp(tb, ot, oz)
            dt = float(np.median(np.diff(tb)))
            # second difference of z, in m/s^2
            zdd = np.full(len(zi), np.nan)
            zdd[1:-1] = (zi[2:] - 2 * zi[1:-1] + zi[:-2]) / (dt * dt)
            good = np.isfinite(zdd)
            zdd_air = float(np.median(zdd[good & up]))
            zdd_down = float(np.median(zdd[good & ~up]))
            # what fraction of samples LABELLED down are nonetheless within
            # 25% of free fall -- flight the detector did not call
            near_g = good & ~up & (np.abs(zdd + 9.81) < 0.25 * 9.81)
            hidden = 100.0 * float(near_g.sum()) / max(1, int((~up).sum()))
            # apex excursion per stride, contact-free
            edges = np.flatnonzero(np.diff(up.astype(int)) == 1) + 1
            apex = []
            for lo, hi in zip(edges[:-1], edges[1:]):
                if hi - lo > 5:
                    apex.append(zi[lo:hi].max() - zi[lo:hi].min())
            print(f"{label:26} {100 * up.mean():8.2f} {zdd_air:9.2f} "
                  f"{zdd_down:9.2f} {hidden:15.1f} "
                  f"{1000 * st_median(apex):8.1f} {len(apex):6d}")
    return 0


def st_median(v):
    return float(np.median(v)) if len(v) else float("nan")


def variance_report(dirs, start, end):
    """Price MORE runs against LONGER runs, for the FLIGHT metric."""
    from run_variance_budget import (FIXED_WALL_S, RTF,  # noqa: F401
                                     budget_table, decompose)
    print("variance budget for FLIGHT FRACTION, split-half within each run.")
    print("S167 S4 answered this for min vLeg (within-run scatter dominates; "
          "2 runs of 72 s beat 5 of 24 s).")
    print("Flight was never decomposed. It is the metric that gates this "
          "campaign, so it is decomposed here.\n")
    for d in dirs:
        d = os.path.expanduser(d)
        vals, skipped = {}, []
        for tq in sorted(glob.glob(os.path.join(d, "run[0-9].csv"))):
            run = os.path.basename(tq)[3:-4]
            od = os.path.join(d, f"odom_run{run}.csv")
            try:
                vals[run] = split_half_flight(
                    tq, od if os.path.exists(od) else None, start, end)
            except (Unfit, SystemExit) as e:
                skipped.append((run, str(e)))
        label = "/".join(d.rstrip("/").split("/")[-2:])
        if len(vals) < 2:
            print(f"  {label:28} only {len(vals)} usable runs -- skipped")
            continue
        r = decompose(vals)
        sd_b, sd_w = np.sqrt(r["sig_b"]), np.sqrt(r["sig_w_full"])
        print(f"  {label:28} n={r['n']}  mean {r['grand']:5.2f}%")
        for run, (h1, h2) in sorted(vals.items()):
            print(f"      run{run}  halves {h1:5.2f} / {h2:5.2f}  "
                  f"gap {h2 - h1:+5.2f}")
        print(f"      sd_between {sd_b:5.3f}   sd_within(full window) "
              f"{sd_w:5.3f}   half-to-half drift {r['half_gap']:+5.2f}")
        if sd_b + sd_w > 1e-9:
            frac = sd_w ** 2 / (sd_b ** 2 + sd_w ** 2)
            print(f"      within-run share of the variance: {100 * frac:.0f}%")
        budget_table(r["sig_b"], r["sig_w_full"], end - start)
        for run, why in skipped:
            print(f"      run{run} skipped: {why}")
        print()
    return 0


def scan(root, label, kflight, start, end):
    """-> (rows, skipped) over every run[0-9].csv under root's cell dirs."""
    root = os.path.expanduser(root)
    rows, skipped = [], []
    for cell in sorted(os.listdir(root)):
        cdir = os.path.join(root, cell)
        if not os.path.isdir(cdir):
            continue
        for tq in sorted(glob.glob(os.path.join(cdir, "run[0-9].csv"))):
            run = os.path.basename(tq)[3:-4]
            od = os.path.join(cdir, f"odom_run{run}.csv")
            try:
                r = measure(tq, od if os.path.exists(od) else None, start, end)
            except Unfit as e:
                skipped.append((label, cell, run, str(e)))
                continue
            r.update(campaign=label, cell=cell, run=run, kflight=kflight,
                     path=tq)
            rows.append(r)
    return rows, skipped


# --------------------------------------------------------------------------
# selftest -- three of these are numbers somebody else derived (S173's rule)
# --------------------------------------------------------------------------

def _synthetic(duty, n=6000, stride=27, chatter=None):
    """A four-leg synchronized gait with a known all-up fraction.

    chatter="air"     one-sample false contact in the MIDDLE of each air phase
    chatter="stance"  one-sample dropout in the MIDDLE of each stance

    Both are planted mid-phase on purpose. A chattering sample placed within
    two samples of a real edge does NOT simply get erased by debounce(): it
    leaves a sub-DEBOUNCE fragment on one side, and debounce() flips that
    fragment to the other polarity too. So a single bad sample near a boundary
    costs three. That is real shipped behaviour, it is worth knowing before
    reading air_deb on a chattering capture, and it is not what this test is
    trying to measure.
    """
    down = int(round(duty * stride))
    c = np.zeros((n, 4), dtype=bool)
    for k in range(0, n - stride, stride):
        c[k:k + down, :] = True
        if chatter == "stance":
            c[k + down // 2, :] = False
        elif chatter == "air":
            c[k + down + (stride - down) // 2, :] = True
    return c


def selftest():
    ok = True

    def check(name, got, want, tol):
        nonlocal ok
        good = abs(got - want) <= tol
        ok = ok and good
        print(f"  {'ok ' if good else 'FAIL'} {name:52} "
              f"{got:8.3f}  (want {want:.3f} +-{tol})")

    def assert_true(name, good):
        nonlocal ok
        ok = ok and good
        print(f"  {'ok ' if good else 'FAIL'} {name}")

    print("1. flight fraction on a planted gait of known duty")
    for duty in (0.40, 0.60, 0.75):
        air, alld = air_and_alld(_synthetic(duty))
        # all four legs share one contact interval, so all-up = 1 - duty and
        # all-down = duty, to within the integer rounding of the stride.
        check(f"duty {duty:.2f} -> air", air, 100.0 * (1 - duty), 2.0)
        check(f"duty {duty:.2f} -> all-down", alld, 100.0 * duty, 2.0)

    print("\n2. chatter biases RAW air in both directions; debounce removes it")
    clean, _ = air_and_alld(_synthetic(0.60))
    print(f"     clean gait air = {clean:.2f}%")
    for mode, direction in (("air", "under"), ("stance", "over")):
        c = _synthetic(0.60, chatter=mode)
        raw, _ = air_and_alld(c)
        deb, _ = air_and_alld(debounced(c))
        print(f"     chatter in {mode:6} -> raw {raw:6.2f}%  "
              f"debounced {deb:6.2f}%")
        if direction == "under":
            assert_true(f"a false contact in air makes RAW {direction}-report",
                        raw < clean - 0.5)
        else:
            assert_true(f"a dropout in stance makes RAW {direction}-report",
                        raw > clean + 0.5)
        check(f"debounced recovers the clean gait ({mode})", deb, clean, 0.2)

    print("\n2b. THE BOUNDARY CASE, recorded because it bit this selftest")
    # A chattering sample two from an edge leaves a 2-sample fragment, and
    # debounce() flips that fragment as well -- so it costs three, not one.
    c = _synthetic(0.60)
    stride, down = 27, int(round(0.60 * 27))
    for k in range(0, len(c) - stride, stride):
        c[k + down + 2, :] = True
    raw_b, _ = air_and_alld(c)
    deb_b, _ = air_and_alld(debounced(c))
    print(f"     one sample 2 from the edge -> raw {raw_b:6.2f}%  "
          f"debounced {deb_b:6.2f}%  (clean {clean:.2f}%)")
    assert_true("debounce OVER-corrects near an edge (documented, not a bug)",
                deb_b < clean - 1.0)

    print("\n3. projections recover a planted L/R pattern")
    p = projections(np.array([+10.0, -10.0, -10.0, +10.0]))
    check("planted L/R 10 deg -> LR", p["LR"], 10.0, 1e-9)
    check("planted L/R 10 deg -> FR", p["FR"], 0.0, 1e-9)
    check("planted L/R 10 deg -> common", p["common"], 0.0, 1e-9)

    print("\n4. KNOWN ANSWER -- reproduce S172 from the banked captures")
    base = os.path.expanduser("~/corgi_runs/camber_pattern")
    if not os.path.isdir(base):
        print("  SKIP -- ~/corgi_runs/camber_pattern is not on this machine")
    else:
        for cell, want in S172.items():
            tq = os.path.join(base, cell, "run1.csv")
            od = os.path.join(base, cell, "odom_run1.csv")
            if not os.path.exists(tq):
                print(f"  SKIP {cell} -- no capture")
                continue
            try:
                r = measure(tq, od, 12.0, 30.0)
            except Unfit as e:
                print(f"  FAIL {cell} -- {e}")
                ok = False
                continue
            check(f"S172 {cell} flight %", r["air_raw"], want["air"], 0.15)
            check(f"S172 {cell} L/R proj", r["LR"], want["LR"], 0.02)
            check(f"S172 {cell} F/R proj", r["FR"], want["FR"], 0.02)

    print("\n5. an unreadable input is refused, not defaulted")
    try:
        measure("/nonexistent/run1.csv", None, 12.0, 30.0)
        print("  FAIL a missing capture was not refused")
        ok = False
    except (Unfit, OSError, SystemExit):
        print("  ok  a missing capture raises rather than returning zeros")

    return ok


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--campaign", action="append", default=[])
    ap.add_argument("--label", action="append", default=[])
    ap.add_argument("--kflight", action="append", default=[])
    ap.add_argument("--start", type=float, default=12.0)
    ap.add_argument("--end", type=float, default=30.0)
    ap.add_argument("--csv")
    ap.add_argument("--variance", action="append", default=[],
                    help="cell dir; price more runs against longer runs")
    ap.add_argument("--ballistic", action="append", default=[],
                    help="cell dir; free-fall and apex cross-check on flight")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()

    if a.selftest:
        print("flight_vs_camber.py selftest\n")
        good = selftest()
        print(f"\n  SELFTEST {'PASS' if good else 'FAIL'}")
        return 0 if good else 1

    if a.variance:
        return variance_report(a.variance, a.start, a.end)

    if a.ballistic:
        return ballistic_report(a.ballistic, a.start, a.end)

    if not a.campaign:
        ap.error("need --campaign (repeatable), or --selftest")

    rows, skipped = [], []
    for i, root in enumerate(a.campaign):
        label = a.label[i] if i < len(a.label) else os.path.basename(root)
        kf = a.kflight[i] if i < len(a.kflight) else "?"
        r, s = scan(root, label, kf, a.start, a.end)
        rows += r
        skipped += s

    if not rows:
        print("no measurable runs found.")
        return 2

    print("flight vs ACHIEVED camber. band = odom/torque overlap "
          f"+{a.start:.0f}..+{a.end:.0f} s (S88-matched).")
    print("air_raw is audit_gamma_decomp's formula; air_deb is the SAME band "
          "with gait_mode's debounce.")
    print(f"gate = audit_gamma_decomp's own (flight>{GATE_AIR:.0f}%, "
          f"all-down<{GATE_ALLD:.0f}%) -- REPORTED, never applied.\n")

    hdr = (f"{'campaign':10} {'cell':14} {'run':>3} {'kf':>5} "
           f"{'|L/R|':>7} {'F/R':>7} {'air_raw':>8} {'air_deb':>8} "
           f"{'alld':>6} {'thmax':>6} {'pb':>6} {'td':>3} {'gate':>5} {'P-U-4':>6}")
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        pb = f"{r['playback']:.3f}" if r["playback"] is not None else "  --  "
        print(f"{r['campaign']:10} {r['cell']:14} {r['run']:>3} "
              f"{r['kflight']:>5} {abs(r['LR']):7.3f} {r['FR']:+7.3f} "
              f"{r['air_raw']:8.2f} {r['air_deb']:8.2f} {r['alld_raw']:6.1f} "
              f"{r['theta_max']:6.1f} {pb:>6} {r['td_min']:3d} "
              f"{'pass' if r['gate_ok'] else 'FAIL':>5} "
              f"{'valid' if r['valid'] else 'EXCL':>6}")

    n_valid = sum(1 for r in rows if r["valid"])
    print(f"\n{len(rows)} runs measured, {n_valid} pass P-U-4, "
          f"{len(rows) - n_valid} excluded, {len(skipped)} unmeasurable.")
    for s in skipped:
        print(f"  unmeasurable: {'/'.join(s[:3])} -- {s[3]}")

    if a.csv:
        with open(a.csv, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["campaign", "cell", "run", "kflight", "LR", "FR",
                        "common", "diag", "air_raw", "air_deb", "alld_raw",
                        "alld_deb", "theta_max", "playback", "td_min", "n",
                        "gate_ok", "valid", "gA", "gB", "gC", "gD"])
            for r in rows:
                w.writerow([r["campaign"], r["cell"], r["run"], r["kflight"],
                            f"{r['LR']:.4f}", f"{r['FR']:.4f}",
                            f"{r['common']:.4f}", f"{r['diag']:.4f}",
                            f"{r['air_raw']:.3f}", f"{r['air_deb']:.3f}",
                            f"{r['alld_raw']:.3f}", f"{r['alld_deb']:.3f}",
                            f"{r['theta_max']:.2f}",
                            "" if r["playback"] is None else f"{r['playback']:.4f}",
                            r["td_min"], r["n"], int(r["gate_ok"]),
                            int(r["valid"])]
                           + [f"{v:.4f}" for v in r["ach"]])
        print(f"\nwrote {a.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
