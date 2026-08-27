#!/usr/bin/env python3
"""Score P-C-1..P-C-3 (log S226): does the CONTACT POSITION step across the
kappa step? 1-deg static holds at the running leg angle.

Reuses contact_lateral.load_contact / measure (static tail, body-frame by per
module) -- no hand-derived bridge, no candidate models. The scored quantity is
body-frame delta_by(lam) = by(lam) - by(0), per module, exactly S188 S3's
corrected convention.

Usage:
    contact_lateral_fine.py --dir ~/corgi_runs/contact_lateral_theta100
    contact_lateral_fine.py --selftest
"""
import argparse
import glob
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import contact_lateral as cl  # noqa: E402

MIN_SAMPLES = 200        # P-C-1
CTRL_SE_MM = 0.2         # P-C-1, lambda = 0 control, SE of the median
STEP_RATIO = 3.0         # P-C-2
STEP_MM = 3.0            # P-C-2
LAM_LO, LAM_HI = 8.0, 15.0


def score(mig, lams):
    """mig: {module: {lam: |delta_by| mm}}. -> (present, detail lines)."""
    lines, present = [], False
    mono_ok = True
    for m in "ABCD":
        ys = np.array([mig[m][l] for l in lams])
        inc = np.diff(ys) / np.diff(lams)
        med = float(np.median(inc))
        hits = [(lams[i], lams[i + 1], inc[i]) for i in range(len(inc))
                if inc[i] > STEP_RATIO * max(med, 1e-9) and inc[i] > STEP_MM]
        mono = bool(np.all(np.diff(ys) >= -1e-6))
        mono_ok &= mono
        lines.append(f"  {m}: increments mm/deg "
                     + " ".join(f"{v:5.2f}" for v in inc)
                     + f"  median {med:.2f}  "
                     + ("STEP " + ",".join(f"{a:.0f}->{b:.0f}:{v:.1f}" for a, b, v in hits)
                        if hits else "no step")
                     + ("" if mono else "  NOT monotone"))
        present |= bool(hits)
    return present, mono_ok, lines


def selftest():
    lams = [8, 9, 10, 11, 12, 13, 14, 15]
    smooth = {m: {l: 4.0 * l for l in lams} for m in "ABCD"}
    p, mono, _ = score(smooth, lams)
    ok = (p is False) and mono
    stepped = {m: {l: 4.0 * l + (20.0 if l >= 12 else 0.0) for l in lams} for m in "ABCD"}
    p2, mono2, _ = score(stepped, lams)
    ok &= (p2 is True) and mono2
    small = {m: {l: 4.0 * l + (2.0 if l >= 12 else 0.0) for l in lams} for m in "ABCD"}
    p3, _, _ = score(small, lams)
    ok &= (p3 is False)          # 2 mm bump: below the 3 mm floor
    print("selftest", "PASS" if ok else "FAIL")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return 0 if selftest() else 1
    if not a.dir:
        ap.error("need --dir or --selftest")

    d = os.path.expanduser(a.dir)
    files = {}
    for p in sorted(glob.glob(os.path.join(d, "contact_lam*.csv"))):
        m = re.search(r"contact_lam([0-9.]+)\.csv$", os.path.basename(p))
        if m:
            files[float(m.group(1))] = p
    if 0.0 not in files:
        print("!! no lambda = 0 control. P-C-1 FAILS; nothing scored.")
        return 2

    meas = {lam: cl.measure(p) for lam, p in sorted(files.items())}

    print("-- P-C-1: validity, scored FIRST --")
    valid = {}
    for lam in sorted(meas):
        bad = []
        for m in "ABCD":
            r = meas[lam].get(m)
            if r is None:
                bad.append(f"{m}:absent")
            elif r["n"] < MIN_SAMPLES:
                bad.append(f"{m}:n={r['n']}")
            elif lam == 0.0 and 1000 * r["by_se"] > CTRL_SE_MM:
                bad.append(f"{m}:SE={1000*r['by_se']:.3f}mm")
        ns = [meas[lam][m]["n"] for m in "ABCD" if m in meas[lam]]
        se = max((meas[lam][m]["by_se"] for m in "ABCD" if m in meas[lam]), default=float("nan"))
        sd = max((meas[lam][m]["by_sd"] for m in "ABCD" if m in meas[lam]), default=float("nan"))
        print(f"  lam {lam:5.1f}: n={min(ns) if ns else 0:5d} SE(med)={1000*se:6.3f}mm "
              f"sd={1000*sd:5.2f}mm  {'VALID' if not bad else 'EXCLUDED -- ' + ','.join(bad)}")
        if not bad:
            valid[lam] = meas[lam]
    if 0.0 not in valid:
        print("\n!! P-C-1 FAILS on the control. NOTHING BELOW IS SCORED (observations only).")
    base = meas[0.0]

    print("\n-- migration from lambda = 0, BODY FRAME, per module (mm; raw signed, then |.|) --")
    lams = sorted(l for l in valid if LAM_LO <= l <= LAM_HI)
    mig = {m: {} for m in "ABCD"}
    print("  lam   " + "  ".join(f"{m:>8}" for m in "ABCD"))
    for lam in sorted(valid):
        row = []
        for m in "ABCD":
            if m in valid[lam] and m in base:
                d_mm = 1000 * (valid[lam][m]["by_med"] - base[m]["by_med"])
                mig[m][lam] = abs(d_mm)
                row.append(f"{d_mm:+8.2f}")
            else:
                row.append(f"{'--':>8}")
        print(f"  {lam:4.0f}  " + "  ".join(row))

    missing = [l for l in (8, 9, 10, 11, 12, 13, 14, 15) if float(l) not in lams]
    if missing:
        print(f"\n  excluded/missing in 8..15: {missing} -- increments computed over the valid points only")
    if len(lams) < 3:
        print("  too few valid points to score P-C-2/3.")
        return 1

    print("\n-- P-C-2 (step) and P-C-3 (monotone) over", lams, "--")
    present, mono, lines = score(mig, lams)
    print("\n".join(lines))
    print()
    scored = 0.0 in valid
    tag = "" if scored else "  [UNSCORED -- P-C-1 failed on the control]"
    print(f"  P-C-2: contact-position step {'PRESENT' if present else 'ABSENT'} at 1 deg on the foot arc{tag}")
    print(f"  P-C-3: monotone {'PASS' if mono else 'FAIL'}{tag}")
    print("  P-C-4: " + ("mesh-facet candidate PROMOTED for S218's step."
                         if present else
                         "static-contact candidate EXCLUDED at 1 deg; dynamic threshold is leading."))
    return 0


if __name__ == "__main__":
    sys.exit(main())
