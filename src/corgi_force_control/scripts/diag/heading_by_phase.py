#!/usr/bin/env python3
"""Does the heading drift while the robot is AIRBORNE? P-W2's analyser.

Offline only. No simulator time.

The hypothesis (log S123, S125, carried into the Wander handover S3.1) is:
"nothing corrects heading while airborne, so more flight means more uncorrected
drift per stride." That has been argued from a two-point comparison -- the
default flies more (36%) and wanders more -- which cannot separate the
mechanism from anything else that differs between the two operating points.

Within a SINGLE run the mechanism makes a sharper, testable claim: heading
should change faster per unit time while no foot is down than while one is.
That test has hundreds of strides per run instead of eight runs per point.

METHOD. Contact comes from the torque capture at 1 kHz; yaw comes from the odom
quaternion at ~100 Hz and is unwrapped and interpolated onto the contact clock,
so phase boundaries land where the contact says they do. A phase segment is a
maximal run of samples with the same airborne/grounded state. For each segment
take dt and the NET heading change dpsi = psi(end) - psi(start). Then

    rate_air    = sum |dpsi| over airborne segments  / total airborne time
    rate_stance = sum |dpsi| over grounded segments  / total grounded time
    R           = rate_air / rate_stance                 <- the registered bar

R >= 1.5 supports the mechanism; R ~ 1 falsifies it as stated; R < 1 falsifies
it in the opposite direction.

WHY NET-PER-SEGMENT AND NOT SUM-OF-|INCREMENTS|. Summing |dpsi| sample by
sample accumulates measurement noise linearly and would manufacture a large R
for whichever phase is sampled more finely. Netting within a segment first
makes the statistic insensitive to sample rate.

WHAT R CANNOT SEPARATE. Within-stride yaw OSCILLATION also lands in |dpsi|, and
the body yaws back and forth every stride regardless of drift. So the SIGNED
decomposition is reported alongside: oscillation cancels in a signed sum over
many strides, and net drift does not. If the two disagree, the signed one is
about drift and the unsigned one is about wobble. Neither is the registered
statistic on its own -- R is -- but a verdict that rests on R while the signed
split says something else is a verdict worth distrusting.

Usage:
    python3 heading_by_phase.py --cell <dir> [--cell <dir> ...] [--start 12]
    python3 heading_by_phase.py --selftest
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import re
import sys

import numpy as np

DIAG = os.path.dirname(os.path.abspath(__file__))
START_S = 12.0
TAIL_S = 20.0
MIN_STRIDES = 20


def _load_check_menger():
    spec = importlib.util.spec_from_file_location("_cm", DIAG + "/check_menger.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules["_cm"] = mod
    spec.loader.exec_module(mod)
    return mod


def yaw_from_quat(q: np.ndarray) -> np.ndarray:
    """Unwrapped yaw (rad) from (n,4) x,y,z,w quaternions."""
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.unwrap(np.arctan2(2.0 * (w * z + x * y),
                                1.0 - 2.0 * (y * y + z * z)))


def segments(mask: np.ndarray):
    """(start_idx, stop_idx_exclusive, value) for each maximal constant run."""
    if len(mask) == 0:
        return []
    edges = np.flatnonzero(np.diff(mask.astype(np.int8)) != 0) + 1
    bounds = np.concatenate(([0], edges, [len(mask)]))
    return [(int(bounds[i]), int(bounds[i + 1]), bool(mask[bounds[i]]))
            for i in range(len(bounds) - 1)]


def phase_split(t: np.ndarray, psi: np.ndarray, airborne: np.ndarray) -> dict:
    """Accumulate net heading change and time, per phase."""
    acc = {True: {"abs": 0.0, "signed": 0.0, "dt": 0.0, "n": 0},
           False: {"abs": 0.0, "signed": 0.0, "dt": 0.0, "n": 0}}
    for i0, i1, val in segments(airborne):
        if i1 - i0 < 2:
            continue
        dt = float(t[i1 - 1] - t[i0])
        if dt <= 0:
            continue
        dpsi = float(psi[i1 - 1] - psi[i0])
        a = acc[val]
        a["abs"] += abs(dpsi)
        a["signed"] += dpsi
        a["dt"] += dt
        a["n"] += 1

    air, gnd = acc[True], acc[False]
    rate_air = air["abs"] / air["dt"] if air["dt"] > 0 else float("nan")
    rate_gnd = gnd["abs"] / gnd["dt"] if gnd["dt"] > 0 else float("nan")
    return {
        "n_air_seg": air["n"], "n_gnd_seg": gnd["n"],
        "t_air": air["dt"], "t_gnd": gnd["dt"],
        "air_time_frac": air["dt"] / (air["dt"] + gnd["dt"]) if (air["dt"] + gnd["dt"]) else float("nan"),
        "abs_air": air["abs"], "abs_gnd": gnd["abs"],
        "signed_air": air["signed"], "signed_gnd": gnd["signed"],
        "rate_air_deg_s": np.rad2deg(rate_air),
        "rate_gnd_deg_s": np.rad2deg(rate_gnd),
        # rate_gnd == 0 means the heading did not move at all while grounded,
        # which is R = infinity, not "no answer". Only a missing phase is nan.
        "R": (float("nan") if not np.isfinite(rate_air) or not np.isfinite(rate_gnd)
              else rate_air / rate_gnd if rate_gnd > 0
              else (float("inf") if rate_air > 0 else float("nan"))),
        "abs_air_share": air["abs"] / (air["abs"] + gnd["abs"]) if (air["abs"] + gnd["abs"]) else float("nan"),
    }



# ---------------------------------------------------------------------------
# The stride-regression statistic. See S131 in the log for why R was retired.
# ---------------------------------------------------------------------------

def stride_table(t, psi, airborne):
    """Per stride: airborne time, grounded time, NET heading change.

    Strides run from one airborne-segment onset to the next, so each stride
    contains exactly one full cycle of any stride-periodic yaw wobble. That is
    the whole point: over a complete cycle the wobble contributes zero to the
    net, so what survives is drift. Splitting WITHIN a stride does not work --
    a wobble deposits the same fixed signed amount into each phase every
    stride, so it accumulates in the per-phase books instead of cancelling,
    which is what broke both `R` and the signed share.
    """
    segs = segments(airborne)
    onsets = [i0 for i0, i1, val in segs if val and i1 - i0 >= 2]
    rows = []
    for a, b in zip(onsets, onsets[1:]):
        dt = float(t[b] - t[a])
        if dt <= 0:
            continue
        t_air = float(np.sum(airborne[a:b]) * (t[b] - t[a]) / (b - a))
        rows.append({"t_air": t_air, "t_gnd": dt - t_air, "dt": dt,
                     "dpsi": float(psi[b] - psi[a])})
    return rows


def stride_regression(rows):
    """Least squares dpsi ~ a*t_air + b*t_gnd, no intercept.

    `a` and `b` are the heading drift rates (rad/s) attributable to airborne
    and grounded time. The hypothesis is a > b. Identified by the stride-to-
    stride variation in how long each phase lasts, so it needs that variation
    to exist -- `cond` reports whether it does.
    """
    if len(rows) < 10:
        return None
    A = np.array([[r["t_air"], r["t_gnd"]] for r in rows])
    y = np.array([r["dpsi"] for r in rows])
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    resid = y - A @ coef
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    return {
        "a_deg_s": float(np.rad2deg(coef[0])),
        "b_deg_s": float(np.rad2deg(coef[1])),
        "ratio": float(coef[0] / coef[1]) if coef[1] != 0 else float("nan"),
        "n": len(rows),
        "cond": float(np.linalg.cond(A)),
        "r2": float(1.0 - np.sum(resid ** 2) / ss_tot) if ss_tot > 0 else float("nan"),
        "t_air_cv": float(np.std(A[:, 0]) / np.mean(A[:, 0])),
    }


def bootstrap_ab(rows, n_boot=4000, seed=11):
    """Percentile CI on a, b and a-b, resampling strides.

    r2 near zero says phase duration barely predicts per-stride heading change,
    but "barely" is not "not at all" and a small effect over 1200 strides can
    still be resolvable. The interval on a-b is the number the hypothesis
    actually stands or falls on, so it is computed rather than eyeballed.
    """
    if len(rows) < 30:
        return None
    A = np.array([[r["t_air"], r["t_gnd"]] for r in rows])
    y = np.array([r["dpsi"] for r in rows])
    rng = np.random.default_rng(seed)
    out = np.empty((n_boot, 3))
    n = len(rows)
    for i in range(n_boot):
        idx = rng.integers(0, n, n)
        c, *_ = np.linalg.lstsq(A[idx], y[idx], rcond=None)
        out[i] = (c[0], c[1], c[0] - c[1])
    lo, hi = np.percentile(out, [2.5, 97.5], axis=0)
    c, *_ = np.linalg.lstsq(A, y, rcond=None)
    return {
        "a": np.rad2deg(c[0]), "a_lo": np.rad2deg(lo[0]), "a_hi": np.rad2deg(hi[0]),
        "b": np.rad2deg(c[1]), "b_lo": np.rad2deg(lo[1]), "b_hi": np.rad2deg(hi[1]),
        "d": np.rad2deg(c[0] - c[1]),
        "d_lo": np.rad2deg(lo[2]), "d_hi": np.rad2deg(hi[2]),
        "n": n,
    }


def analyse_run(cm, odom_path: str, tq_path: str, start: float, tail: float) -> dict:
    t_o, _, quat = cm.load_odom_csv(odom_path)
    t_c, contact = cm.load_torque_csv(tq_path)[:2]

    psi_o = yaw_from_quat(quat)
    t0 = t_o[0]
    t_o = t_o - t0
    t_c = t_c - t_c[0]

    lo, hi = start, start + tail
    sel = (t_c >= lo) & (t_c <= hi)
    if sel.sum() < 1000:
        raise ValueError("fewer than 1000 contact samples in the band")
    t_c, contact = t_c[sel], contact[sel]

    # Yaw onto the contact clock. Refuse to extrapolate.
    if t_c[0] < t_o[0] or t_c[-1] > t_o[-1]:
        span = (max(t_c[0], t_o[0]), min(t_c[-1], t_o[-1]))
        keep = (t_c >= span[0]) & (t_c <= span[1])
        t_c, contact = t_c[keep], contact[keep]
    psi = np.interp(t_c, t_o, psi_o)

    airborne = contact.sum(axis=1) == 0
    out = phase_split(t_c, psi, airborne)
    out["strides"] = out["n_air_seg"]
    out["stride_rows"] = stride_table(t_c, psi, airborne)
    return out


def selftest() -> int:
    """Known answers. A statistic that cannot recover a planted signal is not
    a measurement, and this one is being asked to overturn a hypothesis."""
    ok = True
    n = 20000
    t = np.arange(n) * 1e-3
    # 250 ms stride, 25% airborne -- roughly the plant.
    phase = (t % 0.25) < 0.0625
    airborne = phase

    def check(name, psi, want_lo, want_hi):
        nonlocal ok
        r = phase_split(t, psi, airborne)["R"]
        good = want_lo <= r <= want_hi
        ok = ok and good
        print(f"  {'PASS' if good else 'FAIL'}  {name:44s} R = {r:8.3f} "
              f"(want {want_lo}-{want_hi})")

    # 1. Drift ONLY while airborne -> R must be large.
    psi = np.cumsum(np.where(airborne, 1e-4, 0.0))
    check("drift only while airborne", psi, 20.0, np.inf)
    # 2. Uniform drift -> R must be ~1.
    psi = np.cumsum(np.full(n, 1e-4))
    check("uniform drift, both phases", psi, 0.9, 1.1)
    # 3. Drift ONLY while grounded -> R must be ~0.
    psi = np.cumsum(np.where(airborne, 0.0, 1e-4))
    check("drift only while grounded", psi, 0.0, 0.05)
    # 4. Pure within-stride oscillation, no drift -> the SIGNED sums must
    #    cancel even though |dpsi| does not. This is the confound the docstring
    #    warns about, so it is asserted rather than hoped for.
    psi = 0.02 * np.sin(2 * np.pi * t / 0.25)
    s = phase_split(t, psi, airborne)
    net = abs(s["signed_air"] + s["signed_gnd"])
    tot = s["abs_air"] + s["abs_gnd"]
    good = net < 0.02 * tot
    ok = ok and good
    print(f"  {'PASS' if good else 'FAIL'}  {'oscillation cancels in the signed sum':44s} "
          f"net {net:.4f} vs total |dpsi| {tot:.3f}")

    # ---- the statistic that replaced R -------------------------------------
    # R is retired because it is dominated by duty factor: a PURE wobble with
    # zero drift scores R = (1-f)/f, which is 3.46 at f=0.22 and 1.83 at
    # f=0.35, and adding the hypothesis's own signal barely moves it. The
    # stride regression must recover all three planted cases instead.
    print("\n  stride regression (the replacement for R):")
    rng = np.random.default_rng(7)
    for frac_lo, frac_hi in ((0.18, 0.30),):
        # Stride durations and air fractions must VARY or the regression is
        # unidentified -- that variation is what separates a from b.
        tt, air, marks = [], [], []
        clock = 0.0
        for _ in range(400):
            f = rng.uniform(frac_lo, frac_hi)
            dur = rng.uniform(0.23, 0.27)
            n_a = int(round(f * dur * 1000))
            n_g = int(round((1 - f) * dur * 1000))
            air += [True] * n_a + [False] * n_g
            clock += (n_a + n_g) * 1e-3
        air = np.array(air)
        tt = np.arange(len(air)) * 1e-3
        wobble = 0.02 * np.sin(2 * np.pi * np.cumsum(
            np.where(np.diff(np.concatenate(([False], air)).astype(int)) > 0, 1.0, 0.0)))

        cases = [
            ("uniform drift 0.5 deg/s -> a ~ b",
             np.deg2rad(0.5) * tt, 0.7, 1.4),
            ("airborne-only drift -> a >> b",
             np.cumsum(np.where(air, np.deg2rad(2.0) * 1e-3, 0.0)), 5.0, np.inf),
            ("stance-only drift -> a << b",
             np.cumsum(np.where(~air, np.deg2rad(2.0) * 1e-3, 0.0)), -np.inf, 0.25),
        ]
        for name, drift, lo, hi in cases:
            rows = stride_table(tt, drift + wobble, air)
            fit = stride_regression(rows)
            r = fit["ratio"]
            good = lo <= r <= hi
            ok = ok and good
            print(f"    {'PASS' if good else 'FAIL'}  {name:40s} a/b = {r:8.3f} "
                  f"(want {lo}..{hi}, n={fit['n']}, cond={fit['cond']:.1f})")

        rows = stride_table(tt, wobble, air)
        fit = stride_regression(rows)
        good = abs(fit["a_deg_s"]) < 0.2 and abs(fit["b_deg_s"]) < 0.2
        ok = ok and good
        print(f"    {'PASS' if good else 'FAIL'}  {'pure wobble, no drift -> a ~ b ~ 0':40s} "
              f"a = {fit['a_deg_s']:+.3f}, b = {fit['b_deg_s']:+.3f} deg/s")

    print("SELFTEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cell", action="append", default=[])
    ap.add_argument("--start", type=float, default=START_S)
    ap.add_argument("--tail", type=float, default=TAIL_S)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if not a.cell:
        ap.error("give at least one --cell")

    cm = _load_check_menger()
    print("%-28s%5s%9s%9s%10s%12s%12s%9s" % (
        "cell", "run", "strides", "air_t%", "R", "rate_air", "rate_gnd", "|d|air%"))
    rows = []
    for cell in a.cell:
        cell = os.path.expanduser(cell.rstrip("/"))
        label = os.path.basename(os.path.dirname(cell)) + "/" + os.path.basename(cell)
        runs = sorted(int(re.search(r"odom_run(\d+)\.csv", f).group(1))
                      for f in os.listdir(cell) if re.fullmatch(r"odom_run\d+\.csv", f))
        for n in runs:
            odom = "%s/odom_run%d.csv" % (cell, n)
            tq = "%s/run%d.csv" % (cell, n)
            if not os.path.isfile(tq):
                print("%-28s%5d  DROPPED: no torque csv" % (label, n))
                continue
            try:
                r = analyse_run(cm, odom, tq, a.start, a.tail)
            except Exception as e:
                print("%-28s%5d  DROPPED: %s" % (label, n, e))
                continue
            if r["strides"] < MIN_STRIDES:
                print("%-28s%5d  DROPPED: only %d strides (<%d)"
                      % (label, n, r["strides"], MIN_STRIDES))
                continue
            r["cell"], r["run"] = label, n
            rows.append(r)
            print("%-28s%5d%9d%9.1f%10.3f%12.2f%12.2f%9.1f" % (
                label, n, r["strides"], 100 * r["air_time_frac"], r["R"],
                r["rate_air_deg_s"], r["rate_gnd_deg_s"], 100 * r["abs_air_share"]))

    if not rows:
        print("\nno runs admitted")
        return 1

    Rs = sorted(r["R"] for r in rows)
    med = Rs[len(Rs) // 2] if len(Rs) % 2 else 0.5 * (Rs[len(Rs)//2 - 1] + Rs[len(Rs)//2])
    tot_strides = sum(r["strides"] for r in rows)
    print("\n%d runs, %d strides" % (len(rows), tot_strides))
    print("  median R = %.3f   range [%.3f, %.3f]" % (med, Rs[0], Rs[-1]))
    print("  P-W2 bar: median R >= 1.5 over >= 200 strides")
    print("  *** R IS A RETIRED STATISTIC -- duty-factor biased, see S131. ***")
    print("  *** Reported only because it was the registered bar.        ***")
    verdict = ("SUPPORTED" if med >= 1.5 else
               "FALSIFIED as stated (heading drifts no faster airborne)"
               if med >= 0.95 else
               "FALSIFIED in the OPPOSITE direction (heading drifts faster in STANCE)")
    print("  strides: %d %s" % (tot_strides, "OK" if tot_strides >= 200 else "TOO FEW"))
    print("  VERDICT: %s" % verdict)

    # ---- the stride regression, pooled and per operating point -------------
    print("\n" + "=" * 78)
    print("STRIDE REGRESSION  dpsi ~ a*t_air + b*t_gnd   (R is retired -- see S131)")
    print("=" * 78)
    print("%-28s%5s%7s%10s%10s%10s%8s%8s" % (
        "cell", "run", "n", "a deg/s", "b deg/s", "a/b", "r2", "cv"))
    fits = []
    for r in rows:
        f = stride_regression(r["stride_rows"])
        if f is None:
            continue
        f["cell"], f["run"] = r["cell"], r["run"]
        fits.append(f)
        print("%-28s%5d%7d%+10.3f%+10.3f%10.3f%8.2f%8.3f" % (
            r["cell"], r["run"], f["n"], f["a_deg_s"], f["b_deg_s"],
            f["ratio"], f["r2"], f["t_air_cv"]))
    if fits:
        ratios = sorted(f["ratio"] for f in fits)
        m = (ratios[len(ratios)//2] if len(ratios) % 2
             else 0.5*(ratios[len(ratios)//2-1] + ratios[len(ratios)//2]))
        allrows = [x for r in rows for x in r["stride_rows"]]
        pooled = stride_regression(allrows)
        print("\n  median per-run a/b = %.3f   over %d runs" % (m, len(fits)))
        print("  POOLED over %d strides: a = %+.3f deg/s, b = %+.3f deg/s, a/b = %.3f"
              % (pooled["n"], pooled["a_deg_s"], pooled["b_deg_s"], pooled["ratio"]))
        print("  a is the drift rate attributable to AIRBORNE time, b to GROUNDED time.")
        print("  The hypothesis predicts a > b. a ~ b means heading drifts at the same")
        print("  rate whether or not a foot is down.")
        print("  r2 per run: %.3f-%.3f -- phase duration explains almost none of the"
              % (min(f["r2"] for f in fits), max(f["r2"] for f in fits)))
        print("  per-stride heading change, so the point estimate needs an interval.")
        bs = bootstrap_ab(allrows)
        if bs:
            print()
            print("  BOOTSTRAP, %d strides, 4000 resamples, 95%% percentile CI:" % bs["n"])
            print("    a (airborne) = %+.3f  [%+.3f, %+.3f] deg/s"
                  % (bs["a"], bs["a_lo"], bs["a_hi"]))
            print("    b (grounded) = %+.3f  [%+.3f, %+.3f] deg/s"
                  % (bs["b"], bs["b_lo"], bs["b_hi"]))
            print("    a - b        = %+.3f  [%+.3f, %+.3f] deg/s   <-- THE TEST"
                  % (bs["d"], bs["d_lo"], bs["d_hi"]))
            excl = bs["d_lo"] > 0 or bs["d_hi"] < 0
            print("    CI %s zero -> %s"
                  % ("EXCLUDES" if excl else "INCLUDES",
                     "a difference is resolved" if excl else
                     "no resolvable difference between airborne and grounded drift"))

    sa = sum(r["signed_air"] for r in rows)
    sg = sum(r["signed_gnd"] for r in rows)
    print("\n  signed decomposition (drift, oscillation cancelled):")
    print("    net yaw from airborne segments %+.2f deg" % np.rad2deg(sa))
    print("    net yaw from grounded segments %+.2f deg" % np.rad2deg(sg))
    if abs(sa) + abs(sg) > 0:
        print("    airborne share of |net| = %.1f%%   (its share of TIME = %.1f%%)"
              % (100 * abs(sa) / (abs(sa) + abs(sg)),
                 100 * sum(r["t_air"] for r in rows)
                 / sum(r["t_air"] + r["t_gnd"] for r in rows)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
