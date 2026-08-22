#!/usr/bin/env python3
"""Did the clocked-torque feedforward actually reach the impedance law?

WHY THIS EXISTS. S138 and S146 each cost a whole campaign to an unverified
engagement, and S164 named an engagement assertion as the precondition for
P-N-1..P-N-3. The controller has two halves of that already:

  PROOF OF INTENT   gslip_pronk's `CLOCK FEEDFORWARD: scale=... rate_stance=...`
                    banner. Proves the parameter was read and the rate derived
                    from the template that was actually loaded. Does NOT prove
                    the term reached the law.
  PROOF OF ACTION   force_control's `CLOCK FF ACTIVE: ... tau_beta=...` line,
                    throttled to 5 s, printing the value the impedance law just
                    computed from the live message and the measured pose. That
                    IS proof the term is in the law.

This script is the third, weakest, and most confounded check: does the torque
that reached the MOTORS move by the predicted amount. Read the two log lines
first. This one exists to catch the case where the term is computed correctly
and then lost downstream, and it must be read with its confound in mind.

THE MEASUREMENT

force_control adds f_clock = bz * (dbeta_ref * |hip->contact|) * e_t to the
commanded contact force, so the leg-axis (beta) torque gains

    d_tau_beta = bz * dbeta_ref * L(theta)^2

and beta is the COMMON mode of the two hip motors (theta is the differential),
so tau_beta = tau_L + tau_R. The driver logs that as t_ff -- `t_ff = J^T(F_des
+ M(-acc)) + coupling, from force_control`, corgi_driver.py:1183 -- after
convert_torque, which under CORGI_DIRBETA_TRANSFORM=1 SWAPS AND NEGATES the
pair on the dir_beta = -1 legs. So the sum flips sign on B and C, and the
expected per-leg sign pattern is DIR_BETA = {A:+1, B:-1, C:-1, D:+1}, the same
partition touchdown_phase.py needs for pos_error.

THE CONFOUND, MEASURED, NOT ASSUMED

t_ff also carries the impedance law's off-diagonal coupling, which depends on
the pose. On banked config-of-record captures the stance median of
(t_ff_L + t_ff_R) is -4.11 / +3.70 / +4.11 / -4.15 N.m on `laggard/base`
(theta ~81 deg) and -0.25 / +0.27 / +0.34 / -0.38 on `laggard/kt` (theta ~95
deg). Neither cell ran any feedforward. That is a 3.9 N.m arm-to-arm swing from
pose alone, against the 6.4 N.m this term is meant to add.

Two things follow, and both are enforced below:
  1. The comparison is POSE-MATCHED -- both arms are restricted to a common
     theta band, and if the bands do not overlap the answer is UNCERTIFIABLE,
     not a number.
  2. The tolerance is wide and the verdict is directional. Run-to-run sd of the
     per-leg median is 0.11-0.27 N.m, so the SHIFT is easy to see; what is hard
     is attributing it. A confirmed verdict here means "the torque moved by
     about the right amount, in the right direction, per leg" -- not "the
     feedforward caused it".

THREE OUTCOMES, NOT TWO (S151's rule). Every "cannot tell" path returns
UNCERTIFIABLE and says in as many words that the arm is NOT invalid.
    exit 0 = CONFIRMED, 1 = FAILED or INVALID, 2 = UNCERTIFIABLE.

Usage:
    clock_ff_engagement.py --ref DIR --arm DIR --arm-dbeta-ref 3.0098 \
                           [--ref-dbeta-ref 0.0] [--b-tangential 30]
    clock_ff_engagement.py --selftest
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np

LEG_MOTORS = ("L_Motor", "R_Motor")
LEGS = "ABCD"
# convert_torque swaps AND negates the pair on dir_beta = -1 legs, so the
# L+R sum -- which is the beta-axis torque -- flips sign on B and C.
DIR_BETA = {"A": +1.0, "B": -1.0, "C": -1.0, "D": +1.0}

TAIL_S = 20.0                 # the majority-window convention every tool uses
KP_STANCE_MAX = 100.0         # the stance-mode gate every tool uses
MIN_STANCE_SAMPLES = 200      # per leg, after the pose match
THETA_BAND_MIN_DEG = 4.0      # narrower than this and there is nothing to compare
# The two arms' in-band MEDIANS must agree to this. This is the real match
# test; the band width is not. See matched() for the case that made it
# necessary -- 15 deg apart inside a 37 deg band, reported CONFIRMED.
THETA_MATCH_MAX_DEG = 2.0

# Hip-to-contact L(theta), metres. Copied verbatim from kt_engagement.py, whose
# selftest pins it against the legwheel generator. Hardcoded for the same
# reason: legwheel lives in another repo on the Windows filesystem.
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

# Wide on purpose. The job is separating "engaged" from "did not engage at
# all" under a confound worth 60% of the signal, not validating the geometry.
RATIO_LO, RATIO_HI = 0.40, 1.60
MIN_PRED_NM = 1.0             # below this the arms cannot be told apart
MIN_LEGS_CONFIRM = 3          # of 4


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


def load_tff(path, inject=None):
    """-> {leg: (theta_deg, tff_sum)} over stance-mode samples in the tail.

    `inject` is a {leg: delta_Nm} map added to the beta-axis sum, used by the
    selftest to build a capture with a KNOWN answer without writing files.
    """
    per = {}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            if r.get("motor") not in LEG_MOTORS:
                continue
            try:
                d = per.setdefault((r["leg"], float(r["t"])), {})
                d["f" + r["motor"][0]] = float(r["t_ff"])
                d["k" + r["motor"][0]] = float(r["kp"])
                d["th"] = float(r["theta"])
            except (ValueError, KeyError):
                continue
    rows = {}
    for (leg, t), d in per.items():
        if "fL" not in d or "fR" not in d:
            continue
        s = d["fL"] + d["fR"]
        if inject and leg in inject:
            s += inject[leg]
        rows.setdefault(leg, []).append((t, s, max(d["kL"], d["kR"]), d["th"]))
    out = {}
    for leg, v in rows.items():
        a = np.array(sorted(v))
        a = a[a[:, 0] >= a[:, 0].max() - TAIL_S]
        g = a[:, 2] < KP_STANCE_MAX          # stance mode
        if g.sum() < MIN_STANCE_SAMPLES:
            continue
        out[leg] = (np.degrees(a[g, 3]), a[g, 1])
    if not out:
        raise Uncertifiable("no leg has %d stance samples in %s"
                            % (MIN_STANCE_SAMPLES, os.path.basename(path)))
    return out


def arm_load(d, inject=None):
    """Pool an arm's runs, per leg."""
    runs = sorted(glob.glob(os.path.join(os.path.expanduser(d), "run[0-9].csv")))
    if not runs:
        raise Uncertifiable("no run[0-9].csv in %s" % d)
    pooled, used = {}, []
    for p in runs:
        try:
            r = load_tff(p, inject)
        except Uncertifiable as e:
            print("      %s: skipped -- %s" % (os.path.basename(p), e))
            continue
        used.append(os.path.basename(p))
        for leg, (th, s) in r.items():
            a, b = pooled.setdefault(leg, ([], []))
            a.append(th)
            b.append(s)
    if not pooled:
        raise Uncertifiable("no usable runs in %s" % d)
    return ({leg: (np.concatenate(a), np.concatenate(b))
             for leg, (a, b) in pooled.items()}, used)


def matched(ref_leg, arm_leg):
    """Restrict both arms to a common theta band, and REFUSE if it is not a match.

    -> (ref_med, arm_med, lo, hi, n, theta_mid)

    ⚠ THE FIRST VERSION OF THIS FUNCTION DID NOT MATCH ANYTHING, and it
    certified a pure pose artefact. It took the intersection of the two arms'
    MIN/MAX extremes and then medianed each arm inside it, with the only guard
    rejecting NARROW overlaps -- backwards, because a wide band is precisely a
    band that excludes nothing. Run against the two banked cells this module's
    own header cites as the confound, neither of which ran any feedforward at
    all, it returned "CONFIRMED: 4 of 4 legs" at ratios 0.55-0.61: the 3.9 N.m
    pose swing is 61% of the 6.4 N.m prediction, which lands dead centre of a
    0.40-1.60 accept band. The in-band medians were 81.2 deg against 96.4 deg,
    15 deg apart, inside a band 37 deg wide.

    Two changes, both of which that case now fails:

    1. The band is built from PERCENTILES, not extremes -- the same p25/p95
       construction kp_at_matched_pose.py:76-77 already uses in this directory.
       A single sample at theta 17 deg can no longer set an edge.
    2. The two in-band MEDIANS must agree to THETA_MATCH_MAX_DEG. That is the
       check that was missing entirely: a band is only a match if both arms
       actually sit in the same part of it.

    Failing either is UNCERTIFIABLE, not FAILED. "These two arms were at
    different poses" is not evidence that the feedforward did not engage.
    """
    rth, rs = ref_leg
    ath, asum = arm_leg
    lo = max(np.percentile(rth, 25), np.percentile(ath, 25))
    hi = min(np.percentile(rth, 95), np.percentile(ath, 95))
    if hi - lo < THETA_BAND_MIN_DEG:
        raise Uncertifiable("central theta bands overlap by only %.1f deg "
                            "(ref p25-p95 %.1f-%.1f, arm %.1f-%.1f)"
                            % (max(hi - lo, 0.0),
                               np.percentile(rth, 25), np.percentile(rth, 95),
                               np.percentile(ath, 25), np.percentile(ath, 95)))
    rm = (rth >= lo) & (rth <= hi)
    am = (ath >= lo) & (ath <= hi)
    if rm.sum() < MIN_STANCE_SAMPLES or am.sum() < MIN_STANCE_SAMPLES:
        raise Uncertifiable("only %d/%d samples inside the matched band"
                            % (rm.sum(), am.sum()))
    rth_med = float(np.median(rth[rm]))
    ath_med = float(np.median(ath[am]))
    if abs(rth_med - ath_med) > THETA_MATCH_MAX_DEG:
        raise Uncertifiable(
            "the arms are at DIFFERENT POSES inside the band -- ref median "
            "%.1f deg, arm median %.1f deg, %.1f apart (max %.1f). The pose "
            "confound this check exists to control is not controlled here, so "
            "any ratio would be meaningless. The arm is NOT invalid"
            % (rth_med, ath_med, abs(rth_med - ath_med), THETA_MATCH_MAX_DEG))
    return (float(np.median(rs[rm])), float(np.median(asum[am])),
            lo, hi, int(min(rm.sum(), am.sum())),
            0.5 * (rth_med + ath_med))


def check(ref_dir, arm_dir, arm_dbeta, ref_dbeta=0.0, b_tangential=30.0,
          ref_inject=None, arm_inject=None):
    print("  reference arm: dbeta_ref %+.4f rad/s  <- %s" % (ref_dbeta, ref_dir))
    ref, ref_runs = arm_load(ref_dir, ref_inject)
    print("      %d legs, runs %s" % (len(ref), ", ".join(ref_runs)))
    print("  test arm:      dbeta_ref %+.4f rad/s  <- %s" % (arm_dbeta, arm_dir))
    arm, arm_runs = arm_load(arm_dir, arm_inject)
    print("      %d legs, runs %s" % (len(arm), ", ".join(arm_runs)))
    print()

    d_dbeta = arm_dbeta - ref_dbeta
    if d_dbeta == 0.0:
        raise Uncertifiable("both arms command the same dbeta_ref -- there is "
                            "no contrast to measure, and the arm is NOT invalid")

    print("  %-4s %9s %9s %9s %10s %9s %8s  %s"
          % ("leg", "theta", "ref t_ff", "arm t_ff", "measured", "predicted",
             "ratio", "verdict"))
    ok_legs, bad_legs, unc_legs = [], [], []
    for leg in LEGS:
        if leg not in ref or leg not in arm:
            print("  %-4s %s" % (leg, "absent from one arm -- UNCERTIFIABLE"))
            unc_legs.append(leg)
            continue
        try:
            rmed, amed, lo, hi, n, thmid = matched(ref[leg], arm[leg])
        except Uncertifiable as e:
            print("  %-4s UNCERTIFIABLE -- %s" % (leg, e))
            unc_legs.append(leg)
            continue
        pred = DIR_BETA[leg] * b_tangential * d_dbeta * leg_length(thmid) ** 2
        meas = amed - rmed
        if abs(pred) < MIN_PRED_NM:
            print("  %-4s UNCERTIFIABLE -- predicted shift %.2f N.m is below "
                  "the %.1f N.m floor; the arm is NOT invalid"
                  % (leg, abs(pred), MIN_PRED_NM))
            unc_legs.append(leg)
            continue
        ratio = meas / pred
        good = RATIO_LO <= ratio <= RATIO_HI
        (ok_legs if good else bad_legs).append(leg)
        print("  %-4s %5.1f-%3.1f %+9.3f %+9.3f %+10.3f %+9.3f %8.3f  %s"
              % (leg, lo, hi, rmed, amed, meas, pred, ratio,
                 "ok" if good else ("SIGN INVERTED" if ratio < -RATIO_LO
                                    else "off")))

    print()
    if len(ok_legs) + len(bad_legs) == 0:
        raise Uncertifiable("no leg produced a comparison -- every one was "
                            "below the detection floor or unmatched in pose")
    if len(ok_legs) >= MIN_LEGS_CONFIRM:
        print("  CONFIRMED: %d of 4 legs shifted by the predicted amount "
              "(accept band %.2f-%.2f)" % (len(ok_legs), RATIO_LO, RATIO_HI))
        if bad_legs:
            print("      but %s did not -- report this, do not average it away"
                  % ", ".join(bad_legs))
        print("      ⚠ This is the WEAKEST of the three checks. It says the "
              "motor torque moved by about the right amount at matched pose;")
        print("        it does not exclude the pose confound. The decisive "
              "evidence is force_control's CLOCK FF ACTIVE line.")
        return 0
    if len(unc_legs) >= 2 and not bad_legs:
        raise Uncertifiable("%d legs could not be compared and none "
                            "contradicted the prediction" % len(unc_legs))
    print("  FAILED: only %d of 4 legs shifted as predicted (%s off)"
          % (len(ok_legs), ", ".join(bad_legs) or "none"))
    print("      A ratio near -1 means the SIGN is inverted -- the feedforward "
          "is adding to the brake instead of cancelling it.")
    print("      A ratio near 0 means the term never reached the motors.")
    return 1


def selftest():
    """Known answers, then the cases designed to fool it."""
    ok = True

    # Tolerance 1e-5, not 1e-6: this table is on a 2 deg grid, so L(85.4) is
    # interpolated and lands at 0.265632 against the 0.265630 S151 quotes from
    # the finer generator. 2e-6 is grid granularity, not an error.
    print("  1. geometry table (shared with kt_engagement.py):")
    for th, want in ((85.4, 0.265630), (96.0, 0.285516), (100.0, 0.293115)):
        got = leg_length(th)
        good = abs(got - want) < 1e-5
        ok = ok and good
        print("     L(%.1f) = %.6f  want %.6f  %s"
              % (th, got, want, "ok" if good else "FAIL"))

    print("  2. the predicted term at the config of record:")
    pred = 30.0 * 3.0098 * leg_length(85.4) ** 2
    print("     bz 30 * dbeta 3.0098 * L(85.4)^2 = %.3f N.m leg axis, "
          "%.3f N.m/motor  (want 6.37 / 3.19)" % (pred, pred / 2))
    if abs(pred - 6.371) > 0.02:
        ok = False
        print("     FAIL")

    base = os.path.expanduser("~/corgi_runs/laggard/base")
    if not os.path.isdir(base):
        print("  3-5. banked OFF capture absent -- SELFTEST PASS (partial)")
        return ok

    # 3. THE FOOLING CASE. The same OFF capture on both sides, but the test arm
    #    labelled as if it had commanded the clock. Nothing changed, so the
    #    measured shift is 0 against a 6.4 N.m prediction: this MUST fail.
    print("  3. fooling case -- OFF capture vs ITSELF, labelled dbeta_ref 3.0098:")
    try:
        rc = check(base, base, arm_dbeta=3.0098)
    except Uncertifiable as e:
        rc = 2
        print("     UNCERTIFIABLE -- %s" % e)
    print("     -> exit %d (want 1 = FAILED)" % rc)
    if rc != 1:
        ok = False
        print("     FAIL: a capture with no feedforward was not rejected")

    # 4. THE UNDETECTABLE PAIR. Same capture, a dbeta_ref too small to see.
    #    Must be UNCERTIFIABLE, and must say the arm is NOT invalid.
    print("  4. undetectable pair -- dbeta_ref 0.005 (predicted %.3f N.m):"
          % (30.0 * 0.005 * leg_length(85.4) ** 2))
    try:
        rc = check(base, base, arm_dbeta=0.005)
        print("     -> exit %d (want 2)" % rc)
        ok = False
        print("     FAIL: an indistinguishable pair returned a verdict")
    except Uncertifiable as e:
        print("     UNCERTIFIABLE -- %s" % e)

    # 5. THE POSITIVE CONTROL. Inject the predicted term, with the right
    #    per-leg sign, into the test arm. Must CONFIRM at ratio ~1.
    inj_nm = 30.0 * 3.0098 * leg_length(81.5) ** 2
    inject = {leg: DIR_BETA[leg] * inj_nm for leg in LEGS}
    print("  5. positive control -- inject %+.3f N.m with DIR_BETA signs:" % inj_nm)
    try:
        rc = check(base, base, arm_dbeta=3.0098, arm_inject=inject)
    except Uncertifiable as e:
        rc = 2
        print("     UNCERTIFIABLE -- %s" % e)
    print("     -> exit %d (want 0 = CONFIRMED)" % rc)
    if rc != 0:
        ok = False
        print("     FAIL: a correctly injected term was not confirmed")

    # 6. THE SIGN-INVERSION CASE. Same magnitude, wrong per-leg sign. This is
    #    the failure the whole feature is most likely to hit, and the one that
    #    would read as a clean falsification of a correct hypothesis.
    print("  6. sign inversion -- same magnitude, wrong sign:")
    inject_bad = {leg: -DIR_BETA[leg] * inj_nm for leg in LEGS}
    try:
        rc = check(base, base, arm_dbeta=3.0098, arm_inject=inject_bad)
    except Uncertifiable as e:
        rc = 2
        print("     UNCERTIFIABLE -- %s" % e)
    print("     -> exit %d (want 1 = FAILED)" % rc)
    if rc != 1:
        ok = False
        print("     FAIL: a sign-inverted feedforward was accepted")


    # 7. THE POSE-ARTEFACT CASE, and it is the one that matters most.
    #    Every case above compares `base` against ITSELF, so the two theta
    #    distributions are identical by construction and the pose match is
    #    never exercised. The first version of matched() therefore passed its
    #    whole selftest while certifying a pure artefact: run against two
    #    banked cells that BOTH ran zero feedforward but sat 15 deg apart in
    #    theta, it returned "CONFIRMED: 4 of 4 legs" at ratios 0.55-0.61.
    #    Neither capture predates the feature by accident -- they are the two
    #    cells this module's own header cites as the confound.
    kt = os.path.expanduser("~/corgi_runs/laggard/kt")
    if not os.path.isdir(kt):
        print("  7. pose-artefact case: laggard/kt absent -- cannot run")
        return ok
    print("  7. POSE ARTEFACT -- base (theta ~81) vs kt (theta ~96), both with")
    print("     ZERO feedforward, labelled as if the arm had commanded the clock:")
    try:
        rc = check(base, kt, arm_dbeta=3.0098)
        print("     -> exit %d (want 2 = UNCERTIFIABLE)" % rc)
        ok = False
        print("     FAIL: two arms at different poses returned a verdict.")
        print("     This is the exact defect that certified an artefact.")
    except Uncertifiable as e:
        print("     UNCERTIFIABLE -- %s" % e)
    return ok


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ref")
    ap.add_argument("--arm")
    ap.add_argument("--ref-dbeta-ref", type=float, default=0.0)
    ap.add_argument("--arm-dbeta-ref", type=float)
    ap.add_argument("--b-tangential", type=float, default=30.0)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()

    if a.selftest:
        print("clock_ff_engagement.py selftest")
        ok = selftest()
        print("\n  SELFTEST %s" % ("PASS" if ok else "FAIL"))
        return 0 if ok else 1

    if not (a.ref and a.arm and a.arm_dbeta_ref is not None):
        ap.error("need --ref, --arm and --arm-dbeta-ref")
    print("clock feedforward engagement -- PROOF OF ACTION (the weak third check)")
    try:
        return check(a.ref, a.arm, a.arm_dbeta_ref, a.ref_dbeta_ref,
                     a.b_tangential)
    except Uncertifiable as e:
        print("\n  UNCERTIFIABLE -- %s" % e)
        print("  The arm is NOT invalid. This check could not find its "
              "evidence; read the CLOCK FF ACTIVE line instead.")
        return 2


if __name__ == "__main__":
    sys.exit(main())
