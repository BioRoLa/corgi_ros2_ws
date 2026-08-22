#!/usr/bin/env python3
"""Did k_tangential actually reach the impedance law? Verified from the DATA.

WHY THIS EXISTS. `k_tangential` never announces itself: it is declared as a
parameter and lands at `cmd->kz = k_tangential_` (gslip_pronk.cpp:2424) with no
log line anywhere in between. repeat_gain_regime.sh's four existing engagement
asserts are all greps, and there is nothing to grep for. Its own comment says
so: other parameters "must be verified from the DATA instead (e.g. the
stance-mode kp shifts with k_tangential)". This is that check.

S138 and S146 each cost a whole campaign to an unverified engagement, so this is
not optional decoration.

WHAT IS AND IS NOT PROVED. The controller also prints its leg-frame gains at
startup (LEG-FRAME GAINS line, added 2026-08-22). That is PROOF OF INTENT -- it
proves the parameter was read, and catches typos and launch-plumbing failures.
It does NOT prove the gain reached the impedance law; S138's apex channel
printed its parameters faithfully for ten runs and never fired. This script is
the PROOF OF ACTION half, and a campaign wants both.

THE MEASUREMENT

force_control builds K = kx(e_r e_r^T) + ky(e_y e_y^T) + kz(e_t e_t^T) with
kz = k_tangential, then K_joint = J_fb^T K J_fb and logs kp = K_joint(0,0). The
controller's J_fb is a RIGID-contact Jacobian (S147/S149), so the tangential
column has lever |P| = L(theta), the hip-to-contact distance, and one motor sees
half of it (dbeta/dphi = 0.5). Hence

    dkp/dk_tangential = (0.5 * L(theta))^2

This is POSE-DEPENDENT and that matters. S147 measured 0.020735 in the HOLD at
theta ~ 96 deg. The running gait sits at theta ~ 85.4 deg, where the same
formula gives 0.017640 -- and the banked kt_sweep confirms it:

    arm        measured d(kp)   fixed-slope pred   POSE-CORRECTED pred
    150 vs 600     -7.554          -9.331 (0.81x)      -7.955 (0.95x)
    300 vs 600     -5.049          -6.221 (0.81x)      -5.293 (0.95x)
   1200 vs 600    +10.133         +12.441 (0.81x)     +10.624 (0.95x)

The fixed slope is wrong by a CONSTANT 0.81x and the pose-corrected one by a
constant 0.95x -- identically at every arm, over an 8x range of k_tangential.
The residual 5% is systematic (kp is non-linear in theta, so kp at the median
theta is not the median kp) and is absorbed by the tolerance, not fitted out.

WHY A REFERENCE ARM, NOT AN ABSOLUTE CHECK. k_tangential is constant within a
run, so it is perfectly collinear with arm identity and there is no within-run
contrast. An absolute check would need an intercept, and the intercept does not
transfer between pose regimes: the hold sweep gives kp 36.95 at k_t 600, the
running gait gives 34.95. The reference must be the SHIPPED DEFAULT arm, because
if ITS parameter had silently failed it would have run at the default anyway --
"ignored" and "applied" are the same gains there. It is the one arm this check
cannot certify, and the one that does not need certifying.

THREE OUTCOMES, NOT TWO. The previous engagement assert's fatal flaw was
reporting "the assertion could not find its evidence" as "the parameter did not
engage" -- it fired INVALID on every arm of the k_tangential sweep. Every
"cannot tell" path here returns UNCERTIFIABLE and says so.

Usage:
    kt_engagement.py --ref DIR --ref-kt 600 --arm DIR --arm-kt 2400
    kt_engagement.py --selftest
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np

LEG_MOTORS = ("L_Motor", "R_Motor")
TAIL_S = 20.0                 # the majority window convention
KP_STANCE_MAX = 100.0         # the stance-mode gate every diag tool uses
MIN_STANCE_SAMPLES = 500
THETA_SD_WARN = 15.0          # deg; above this the pose is not a settled gait

# Hip-to-contact distance L(theta), metres, from legwheel LegLengthMap plus the
# 0.145 m foot radius. Generated 2026-08-22, 60-120 deg in 2 deg steps.
# Hardcoded rather than imported: legwheel lives in a different repo on the
# Windows filesystem and this must run inside WSL with no cross-repo path.
HIP_TO_CONTACT_DEG0 = 60.0
HIP_TO_CONTACT_STEP = 2.0
HIP_TO_CONTACT = (
    0.219449, 0.223044, 0.226642, 0.230244, 0.233851, 0.237464,
    0.241084, 0.244713, 0.248352, 0.252003, 0.255666, 0.259344,
    0.263036, 0.266744, 0.270468, 0.274208, 0.277963, 0.281733,
    0.285516, 0.289310, 0.293115, 0.296926, 0.300741, 0.304557,
    0.308369, 0.312173, 0.315964, 0.319739, 0.323490, 0.327213,
    0.330901,
)
MOTOR_FACTOR = 0.5            # dbeta/dphi_L = dbeta/dphi_R (force_estimation.cpp)

# Tolerance on measured/predicted. Wide on purpose: this check's job is to
# separate "engaged" from "did not engage at all", not to validate the geometry.
# The pose-corrected prediction runs a systematic 0.95x on banked data.
RATIO_LO, RATIO_HI = 0.50, 1.50
# Below this predicted shift the arms are too close to tell apart at all.
MIN_PRED_DKP = 2.0


class Uncertifiable(Exception):
    pass


def leg_length(theta_deg):
    """L(theta) by linear interpolation in the embedded table."""
    x = (theta_deg - HIP_TO_CONTACT_DEG0) / HIP_TO_CONTACT_STEP
    if x < 0 or x > len(HIP_TO_CONTACT) - 1:
        raise Uncertifiable("theta %.1f deg is outside the tabulated 60-120"
                            % theta_deg)
    i = int(np.floor(x))
    if i >= len(HIP_TO_CONTACT) - 1:
        return HIP_TO_CONTACT[-1]
    f = x - i
    return HIP_TO_CONTACT[i] * (1.0 - f) + HIP_TO_CONTACT[i + 1] * f


def slope_at(theta_deg):
    """dkp/dk_tangential at this pose, N.m/rad per (N/m)."""
    return (MOTOR_FACTOR * leg_length(theta_deg)) ** 2


def stance_kp(path):
    """-> (median stance-mode kp, median theta deg, theta sd deg, n)."""
    kp, th, t = [], [], []
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            if r.get("motor") not in LEG_MOTORS:
                continue
            try:
                kp.append(float(r["kp"]))
                th.append(float(r["theta"]))
                t.append(float(r["t"]))
            except (ValueError, KeyError):
                continue
    if len(t) < MIN_STANCE_SAMPLES:
        raise Uncertifiable("only %d leg-motor rows" % len(t))
    kp, th, t = np.array(kp), np.array(th), np.array(t)
    m = t >= (t.max() - TAIL_S)
    kp, th = kp[m], th[m]
    g = kp < KP_STANCE_MAX
    if g.sum() < MIN_STANCE_SAMPLES:
        raise Uncertifiable("only %d stance-mode samples in the tail" % g.sum())
    return (float(np.median(kp[g])), float(np.degrees(np.median(th[g]))),
            float(np.degrees(np.std(th[g]))), int(g.sum()))


def arm_summary(d):
    """Median across the arm's runs, plus the per-run spread."""
    runs = sorted(glob.glob(os.path.join(os.path.expanduser(d), "run[0-9].csv")))
    if not runs:
        raise Uncertifiable("no run[0-9].csv in %s" % d)
    kps, ths, sds, ok = [], [], [], []
    for p in runs:
        try:
            k, tm, ts, _n = stance_kp(p)
        except Uncertifiable as e:
            print("      %s: skipped -- %s" % (os.path.basename(p), e))
            continue
        kps.append(k); ths.append(tm); sds.append(ts); ok.append(os.path.basename(p))
    if not kps:
        raise Uncertifiable("no usable runs in %s" % d)
    return {"kp": float(np.median(kps)), "kp_runs": kps,
            "theta": float(np.median(ths)), "theta_sd": float(np.median(sds)),
            "n_runs": len(kps), "runs": ok}


def check(ref_dir, ref_kt, arm_dir, arm_kt):
    print("  reference arm: k_tangential %.0f  <- %s" % (ref_kt, ref_dir))
    ref = arm_summary(ref_dir)
    print("      stance kp %.3f  (runs %s)  theta %.2f deg, sd %.2f, n=%d"
          % (ref["kp"], ", ".join("%.2f" % k for k in ref["kp_runs"]),
             ref["theta"], ref["theta_sd"], ref["n_runs"]))
    print("  test arm:      k_tangential %.0f  <- %s" % (arm_kt, arm_dir))
    arm = arm_summary(arm_dir)
    print("      stance kp %.3f  (runs %s)  theta %.2f deg, sd %.2f, n=%d"
          % (arm["kp"], ", ".join("%.2f" % k for k in arm["kp_runs"]),
             arm["theta"], arm["theta_sd"], arm["n_runs"]))

    for nm, a in (("reference", ref), ("test", arm)):
        if a["theta_sd"] > THETA_SD_WARN:
            print("  ?? %s arm theta sd %.1f deg > %.1f -- pose is not settled;"
                  % (nm, a["theta_sd"], THETA_SD_WARN))
            print("     the pose-corrected prediction is less trustworthy here.")

    theta_mid = 0.5 * (ref["theta"] + arm["theta"])
    s = slope_at(theta_mid)
    pred = s * (arm_kt - ref_kt)
    meas = arm["kp"] - ref["kp"]
    print("  pose-corrected slope at theta %.2f deg : %.6f N.m/rad per (N/m)"
          % (theta_mid, s))
    print("  predicted d(kp) %+.3f   measured d(kp) %+.3f" % (pred, meas))

    if abs(pred) < MIN_PRED_DKP:
        raise Uncertifiable("predicted shift %.2f is under the %.1f detection "
                            "floor -- these arms are too close to tell apart"
                            % (abs(pred), MIN_PRED_DKP))
    ratio = meas / pred
    print("  ratio %.3f   (accept %.2f-%.2f; banked arms sit at 0.95)"
          % (ratio, RATIO_LO, RATIO_HI))
    return RATIO_LO <= ratio <= RATIO_HI, ratio


def selftest():
    """Gate the check on the banked sweep, whose answer is already known."""
    print("SELF-TEST")
    fails = 0
    base = os.path.expanduser("~/corgi_runs/kt_sweep")

    # 1. the geometry table must reproduce its generator
    for th, want in ((85.4, 0.265630), (96.0, 0.285516), (100.0, 0.293115)):
        got = leg_length(th)
        ok = abs(got - want) < 5e-4
        print("  L(%.1f) = %.6f  (want %.6f)  %s"
              % (th, got, want, "OK" if ok else "FAIL"))
        if not ok:
            fails += 1

    # 2. S147's hold slope must fall out of the same formula
    print("  slope(96.0) = %.6f   (S147 measured 0.020735 in the hold)"
          % slope_at(96.0))

    # 3. the three banked arms must all CONFIRM against the shipped reference
    if not os.path.isdir(base):
        print("  ?? no banked kt_sweep at %s -- geometry checked, data not" % base)
        print("\nSELFTEST PASS (partial)" if not fails else "\nSELFTEST FAIL")
        return 1 if fails else 0
    for kt in (150.0, 300.0, 1200.0):
        d = os.path.join(base, "kt%s" % kt)
        print("  --- banked arm k_t = %.0f, expect CONFIRMED ---" % kt)
        try:
            ok, ratio = check(os.path.join(base, "kt600.0"), 600.0, d, kt)
        except Uncertifiable as e:
            print("  FAIL: banked arm returned UNCERTIFIABLE -- %s" % e)
            fails += 1
            continue
        print("  -> %s" % ("CONFIRMED" if ok else "INVALID"))
        if not ok:
            print("  FAIL: a known-good arm did not confirm"); fails += 1

    # 4. THE FOOLING CASE: an arm compared against ITSELF claims a gain change
    #    that did not happen, and must come back INVALID.
    print("  --- fooling case: kt600 vs itself, LABELLED 2400 ---")
    try:
        ok, ratio = check(os.path.join(base, "kt600.0"), 600.0,
                          os.path.join(base, "kt600.0"), 2400.0)
        print("  -> %s (ratio %.3f)" % ("CONFIRMED" if ok else "INVALID", ratio))
        if ok:
            print("  FAIL: a non-engaging arm was certified"); fails += 1
    except Uncertifiable as e:
        print("  FAIL: should be INVALID, not UNCERTIFIABLE -- %s" % e)
        fails += 1

    # 5. two arms too close together must be UNCERTIFIABLE, not INVALID
    print("  --- undetectable pair: kt600 vs kt600 labelled 620 ---")
    try:
        check(os.path.join(base, "kt600.0"), 600.0,
              os.path.join(base, "kt600.0"), 620.0)
        print("  FAIL: should have refused as UNCERTIFIABLE"); fails += 1
    except Uncertifiable as e:
        print("  -> UNCERTIFIABLE, correctly: %s" % e)

    print("")
    print("SELFTEST PASS" if not fails else "SELFTEST FAIL (%d)" % fails)
    return 1 if fails else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref")
    ap.add_argument("--ref-kt", type=float)
    ap.add_argument("--arm")
    ap.add_argument("--arm-kt", type=float)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if not (a.ref and a.arm and a.ref_kt is not None and a.arm_kt is not None):
        ap.error("need --ref/--ref-kt/--arm/--arm-kt, or --selftest")
    print("k_tangential ENGAGEMENT CHECK")
    try:
        ok, _ratio = check(a.ref, a.ref_kt, a.arm, a.arm_kt)
    except Uncertifiable as e:
        print("  ?? UNCERTIFIABLE: %s" % e)
        print("  ?? This is NOT a failure of the arm. It means this check could")
        print("     not find the evidence to judge it. Arm NOT invalid.")
        return 2
    if ok:
        print("  k_tangential ENGAGEMENT CONFIRMED")
        return 0
    print("  !! k_tangential ENGAGEMENT FAILED -- the stance-mode kp did not")
    print("  !! move as the commanded gain change predicts.")
    print("  !! TREAT THIS ARM AS INVALID.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
