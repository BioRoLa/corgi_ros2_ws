#!/usr/bin/env python3
"""How far does the CONTACT migrate laterally under camber? Log S188.

Stage 1's skipped validation item 2, and the arbiter S183 assumed existed.

THE QUANTITY. `d_out` in coronal_bip is "lateral offset, hip to contact,
positive outboard", and `_foot_offset_body` is explicit that it is the BODY
FRAME -- the model carries body roll (rho) as a SEPARATE state and rotates the
body-frame offset by it. So the scored quantity here is body-frame

    delta_by(lam) = by(lam) - by(0)

which also removes the hip exactly, because the hip sits at a constant
body-frame y (+-0.12, corgi_driver._MODULE_XY). No quaternion needed.

⚠ S188 originally registered a DE-ROLLED (gravity-levelled) number and that was
corrected before any data existed: levelling folds rho into d_out, which the
model keeps separate, and would be wrong by ~23 mm at lam 20 -- about the size
of the effect. The levelled value is still printed as CONTEXT, never as the bar.

THREE CANDIDATES (mm, migration from lam = 0, hip-to-contact):

    lam           0+       5      10      15      20      30
    r*sin          0   +12.29  +23.79  +34.41  +44.06  +60.22   d0*cos + r*sin
    r_corner*sin   0    +0.96   +1.21   +0.76   -0.40   -4.78   d0*cos + rc*sin
    C++ map    -20.0    -7.64   +4.09  +15.09  +25.27  +42.90   (d0-wt/2)*cos + r*sin

The C++ form is the r*sin form MINUS the wheel-edge offset (wt/2)*cos(lam), so
the hypothesis space is two binary questions: is the swing coefficient r or
r_corner, and is the wheel-edge offset real.

Usage:
    contact_lateral.py --selftest
    contact_lateral.py --dir ~/corgi_runs/contact_lateral
"""
import argparse
import csv
import glob
import math
import os
import re
import sys

import numpy as np

# Geometry, from corgi_driver._MODULE_XY and the model constants.
MODULE_Y = {"A": +0.12, "B": -0.12, "C": -0.12, "D": +0.12}
OUTBOARD = {"A": +1.0, "B": -1.0, "C": -1.0, "D": +1.0}
D0 = 0.091675          # wheel plane outboard of the hip (m)
R_ROLL = 0.145         # wheel-mode rolling radius, valid at theta = 17 deg
R_CORNER = 0.015       # shoulder fillet
WT = 0.04              # wheel thickness

TAIL_S = 15.0          # the static hold is the tail of the capture
MIN_SAMPLES = 200      # P-Y-1
STATIC_SD_MM = 3.0     # a static hold's by must not wander more than this


def candidates(lam_deg):
    """-> dict of predicted migration (m) from lam = 0, hip-to-contact."""
    L = math.radians(lam_deg)
    c, s = math.cos(L), math.sin(L)
    step = 0.0 if abs(s) < 1e-4 else WT / 2.0
    return {
        "r_sin":    D0 * (c - 1.0) + R_ROLL * s,
        "rc_sin":   D0 * (c - 1.0) + R_CORNER * s,
        "cpp_edge": (D0 - step) * c + R_ROLL * s - D0,
    }


def load_contact(path):
    """-> {module: (t, by, bx, wz)} arrays, tail only."""
    rows = {m: [] for m in MODULE_Y}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            m = r.get("module")
            if m not in rows:
                continue
            try:
                rows[m].append((float(r["t"]), float(r["by"]),
                                float(r["bx"]), float(r["wz"])))
            except (ValueError, KeyError, TypeError):
                continue
    out = {}
    t_max = max((v[-1][0] for v in rows.values() if v), default=None)
    if t_max is None:
        return out
    for m, v in rows.items():
        if not v:
            continue
        a = np.array(v, dtype=float)
        a = a[a[:, 0] >= t_max - TAIL_S]
        if len(a):
            out[m] = a
    return out


def measure(path):
    """-> {module: dict} for one lambda, over the static tail."""
    per = load_contact(path)
    res = {}
    for m, a in per.items():
        by = a[:, 1]
        res[m] = {
            "n": len(by),
            "by_med": float(np.median(by)),
            "by_sd": float(np.std(by)),
            "by_se": float(np.std(by) / max(1.0, np.sqrt(len(by)))),
            "wz_med": float(np.median(a[:, 3])),
        }
    return res


def selftest():
    """Known answers: the candidate arithmetic, and the aggregation."""
    ok = True

    def chk(name, got, want, tol):
        nonlocal ok
        good = abs(got - want) <= tol
        ok = ok and good
        print(f"  {'ok ' if good else 'FAIL'} {name:46} {1000*got:+8.2f} mm "
              f"(want {1000*want:+.2f})")

    print("1. candidate arithmetic against the S188 table (mm)")
    for lam, exp in ((5, (0.01229, 0.00096, -0.00764)),
                     (10, (0.02379, 0.00121, 0.00409)),
                     (20, (0.04406, -0.00040, 0.02527)),
                     (30, (0.06022, -0.00478, 0.04290))):
        c = candidates(lam)
        chk(f"lam {lam:2d} r*sin", c["r_sin"], exp[0], 2e-5)
        chk(f"lam {lam:2d} r_corner*sin", c["rc_sin"], exp[1], 2e-5)
        chk(f"lam {lam:2d} C++ edge", c["cpp_edge"], exp[2], 2e-5)

    print("\n2. the edge step, and WHERE it actually switches")
    # The C++ gate is |sin gamma| >= 1e-4 on the RADIAN sine, i.e. gamma >=
    # 0.0057 deg -- NOT at any nonzero lambda. The first version of this test
    # probed 0.001 deg (sin = 1.7e-5, BELOW the gate) and failed the code for
    # being right. S173 measured the same switch and put it at "gamma ~ 0.006
    # deg"; this is that number, independently rediscovered by a bad test.
    z = candidates(0.0)
    below = candidates(0.0057 * 0.5)      # under the gate
    above = candidates(0.01)              # over it
    chk("lam 0      no step", z["cpp_edge"], 0.0, 1e-12)
    # Tolerance 0.1 mm, not 0.001: below the gate the r*sin term still
    # contributes +0.007 mm, which is not zero and is not the point. The
    # claim under test is "no 20 mm STEP", so the bar must be far below 20
    # and comfortably above the smooth term. A tolerance tight enough to
    # fail on 0.007 mm is testing floating point, not geometry.
    chk("lam 0.003  BELOW the 1e-4 gate, no 20mm step",
        below["cpp_edge"], 0.0, 1e-4)
    chk("lam 0.01   ABOVE the gate, full step", above["cpp_edge"], -0.019975, 5e-5)

    print("\n3. the three candidates are separable at every tested lambda")
    for lam in (5, 10, 15, 20, 30):
        v = list(candidates(lam).values())
        spread = max(v) - min(v)
        good = spread > 0.015
        ok = ok and good
        print(f"  {'ok ' if good else 'FAIL'} lam {lam:2d} spread "
              f"{1000*spread:6.1f} mm (need > 15)")
    return ok


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()

    if a.selftest:
        print("contact_lateral.py selftest\n")
        good = selftest()
        print(f"\n  SELFTEST {'PASS' if good else 'FAIL'}")
        return 0 if good else 1

    if not a.dir:
        ap.error("need --dir, or --selftest")

    d = os.path.expanduser(a.dir)
    files = {}
    for p in sorted(glob.glob(os.path.join(d, "contact_lam*.csv"))):
        m = re.search(r"contact_lam([0-9.]+)\.csv$", os.path.basename(p))
        if m:
            files[float(m.group(1))] = p
    if not files:
        print(f"no contact_lam*.csv under {d}")
        return 2
    if 0.0 not in files:
        print("!! no lambda = 0 control in this directory -- P-Y-1 FAILS, and")
        print("!! every number here is a migration from an unknown baseline.")
        return 2

    print("contact lateral migration -- BODY FRAME (the frame d_out is defined")
    print("in; see S188's correction). Static tail of each capture.\n")

    meas = {lam: measure(p) for lam, p in sorted(files.items())}

    print("-- P-Y-1: validity, scored FIRST --")
    valid = {}
    for lam in sorted(meas):
        bad = []
        for m in "ABCD":
            r = meas[lam].get(m)
            if r is None:
                bad.append(f"{m}:absent")
            elif r["n"] < MIN_SAMPLES:
                bad.append(f"{m}:n={r['n']}")
            elif 1000 * r["by_sd"] > STATIC_SD_MM:
                bad.append(f"{m}:sd={1000*r['by_sd']:.1f}mm")
        ns = [meas[lam][m]["n"] for m in "ABCD" if m in meas[lam]]
        se = max((meas[lam][m]["by_se"] for m in "ABCD" if m in meas[lam]), default=float("nan"))
        print(f"  lam {lam:5.1f}: n={min(ns) if ns else 0:5d} SE(med)={1000*se:5.3f}mm  "
              f"{'VALID' if not bad else 'EXCLUDED -- ' + ','.join(bad)}")
        if not bad:
            valid[lam] = meas[lam]

    SCORED = 0.0 in valid
    if not SCORED:
        print("\n!! P-Y-1 FAILS: the lambda = 0 control is not valid.")
        print("!! Per S188's own consequence clause, NOTHING BELOW IS SCORED.")
        print("!! The numbers are printed as OBSERVATIONS so the re-registration")
        print("!! has priors -- the S171 precedent. Do not read them as verdicts.")
        print()
    base = meas[0.0]     # baseline regardless; the failure is its SCATTER, not
                         # its median, and a migration needs some baseline.
    print(f"\n{len(valid)}/{len(meas)} lambdas pass P-Y-1; "
          f"{len(meas) - len(valid)} excluded and counted.\n")

    print("-- migration from lam=0, per leg, OUTBOARD-signed (mm) --")
    print(f"  {'lam':>5} {'A':>8} {'B':>8} {'C':>8} {'D':>8} {'mean':>8} "
          f"{'|r*sin|':>9} {'|rc*sin|':>9} {'|C++|':>9}")
    obs = {}
    for lam in sorted(meas):
        if lam == 0.0:
            continue
        if not all(m in meas[lam] for m in "ABCD"):
            continue
        per = {}
        for m in "ABCD":
            per[m] = OUTBOARD[m] * (meas[lam][m]["by_med"] - base[m]["by_med"])
        mean = float(np.mean(list(per.values())))
        obs[lam] = (mean, per)
        c = candidates(lam)
        print(f"  {lam:5.1f} " + " ".join(f"{1000*per[m]:8.2f}" for m in "ABCD")
              + f" {1000*mean:8.2f} "
              + f"{1000*c['r_sin']:9.2f} {1000*c['rc_sin']:9.2f} "
              f"{1000*c['cpp_edge']:9.2f}")

    if not obs:
        print("\nno non-zero lambdas survived. Nothing to discriminate.")
        return 1

    print("\n-- P-Y-2: which candidate, by summed |residual| over valid lam --")
    resid = {k: sum(abs(obs[l][0] - candidates(l)[k]) for l in obs)
             for k in ("r_sin", "rc_sin", "cpp_edge")}
    order = sorted(resid, key=resid.get)
    for k in order:
        print(f"  {k:10} summed |residual| {1000*resid[k]:8.2f} mm")
    best, second = order[0], order[1]
    factor = (resid[second] / resid[best]) if resid[best] > 1e-9 else float("inf")
    print(f"  -> best '{best}' beats '{second}' by {factor:.2f}x "
          f"(P-Y-2 needs >= 2.0): "
          f"{('PASS' if factor >= 2.0 else 'FAIL -- no candidate wins cleanly') if SCORED else 'UNSCORED (P-Y-1 failed)'}")

    print("\n-- P-Y-3: the edge step, |d(5) - d(0)| --")
    if 5.0 in obs:
        step_mm = 1000 * abs(obs[5.0][0])
        verdict = ("edge offset REAL (>10mm)" if step_mm > 10 else
                   "edge offset ABSENT (<5mm)" if step_mm < 5 else
                   "INDETERMINATE (5-10mm)")
        print(f"  {step_mm:.2f} mm -> {verdict}" + ("" if SCORED else "   [UNSCORED]"))
    else:
        print("  lam = 5 not valid; P-Y-3 UNSCORED.")

    print("\n-- P-Y-4: monotonicity over the valid non-zero lambdas --")
    lams = sorted(obs)
    vals = [obs[l][0] for l in lams]
    mono = all(b > a for a, b in zip(vals, vals[1:]))
    print(f"  {' -> '.join(f'{1000*v:+.1f}' for v in vals)} : "
          f"{('MONOTONE (PASS)' if mono else 'NOT monotone -- supports r_corner') if SCORED else ('monotone' if mono else 'NOT monotone') + ' [UNSCORED]'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
