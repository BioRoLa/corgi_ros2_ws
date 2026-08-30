#!/usr/bin/env python3
"""What happens in the first seconds of a run that goes on to collapse?
Log S219. The first capture that can SEE the onset.

S195 killed every collapse predictor for one structural reason: odom capture
opened ~7 s into the gait and both collapsed runs on record were already dead
in the first window. PRE_SETTLE_ODOM=1 (S196) starts the capture before the
settle, so this is the first dataset where the onset is on disk.

ANCHOR. With pre-settle capture the CSVs begin during standup, so "t0 + 12 s"
(every other analyser's band) lands in the settle. The gait onset is anchored
on the FIRST SAMPLE WITH ALL FOUR LEGS AIRBORNE -- the pronk's first flight --
read from the torque CSV's in_contact columns. Everything is reported in
seconds after that.

PER RUN, in 2 s windows from onset-4 to onset+30:
    v_fwd         heading-integrated forward speed (speed_from_odom's convention)
    yaw_rate      deg/s, signed (+ = CCW)
    fr_cmd        F/R camber command, deg (the yaw loop's output; 0 on camber-only cells)
    lr_cmd        L/R camber command, deg (the camber channel)
and two onset times per run:
    t_collapse    first window with v_fwd < 0.10 (S191 threshold), or none
    t_yawflip     first window whose yaw_rate sign differs from the plant's
                  healthy drift sign (negative, S195: 22 of 25 healthy runs)

PREDICTIONS (S219, written before the data):
    P-O-1  At least one run collapses in 10 (clamp n=5 at 2/10 banked, camber-
           only n=5 at 2/13 banked). P(none) ~ 14%; if none, the campaign is
           INCONCLUSIVE and says so -- not a result either way.
    P-O-2  In every collapsed run, t_yawflip <= t_collapse: the heading turns
           the wrong way BEFORE forward progress dies. Falsified if v_fwd dies
           first -- then the yaw flip is a consequence, not a precursor.
    P-O-3  In healthy runs the yaw rate never flips sign for two consecutive
           windows after onset+6. (A single-window flip is noise; two is the
           onset.) Falsified if healthy runs flip and recover -- then the flip
           does not discriminate.
    P-O-4  On the clamp cell, the F/R command reaches its clamp (|fr_cmd| >=
           0.95 deg) within the same window as t_yawflip or earlier, in every
           collapsed clamp run.
    P-O-5  (camber-only) the collapse, if any, occurs with |lr_cmd| at its
           commanded value -- i.e. it is not a camber-delivery failure.

Usage:
    onset_capture.py --selftest
    onset_capture.py --dir <cell> --label <name> [--dir ...]
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_menger import LEGS, load_odom_csv, load_torque_csv  # noqa: E402
from speed_from_odom import yaw_from_quat                      # noqa: E402
from yaw_saturation import FR_SIGN                             # noqa: E402

LR_SIGN = {"A": +1.0, "B": -1.0, "C": -1.0, "D": +1.0}
WIN = 2.0
PRE, POST = 4.0, 30.0
COLLAPSE = 0.10
HEALTHY_SIGN = -1.0      # S195: the plant drifts negative when nothing turns it


def gait_onset(t, contact):
    """first sample with all four legs airborne, after at least 2 s of capture."""
    air = ~contact.any(axis=1)
    air[t < t[0] + 2.0] = False
    idx = np.where(air)[0]
    return float(t[idx[0]]) if len(idx) else float("nan")


def windows(tq, od):
    t, contact, gamma, _th = load_torque_csv(tq)
    ot, xy, q = load_odom_csv(od)
    t_on = gait_onset(t, contact)
    if np.isnan(t_on):
        return None, []
    yaw = np.unwrap(yaw_from_quat(q))
    fr = sum(FR_SIGN[l] * gamma[:, i] for i, l in enumerate(LEGS)) / 4.0
    lr = sum(LR_SIGN[l] * gamma[:, i] for i, l in enumerate(LEGS)) / 4.0
    out = []
    k = -int(PRE / WIN)
    while k * WIN < POST:
        lo, hi = t_on + k * WIN, t_on + (k + 1) * WIN
        mo = (ot >= lo) & (ot < hi); mt = (t >= lo) & (t < hi)
        if mo.sum() > 20 and mt.sum() > 20:
            p, y = xy[mo], yaw[mo]; span = float(ot[mo][-1] - ot[mo][0])
            dx, dy = np.diff(p[:, 0]), np.diff(p[:, 1])
            v = float(np.sum(dx * np.cos(y[:-1]) + dy * np.sin(y[:-1]))) / span if span > 0.5 else float("nan")
            yr = float(np.degrees((y[-1] - y[0]) / span)) if span > 0.5 else float("nan")
            out.append({"t": k * WIN, "v": v, "yaw_rate": yr,
                        "fr": float(np.median(fr[mt])), "lr": float(np.median(lr[mt]))})
        k += 1
    return t_on, out


def onsets(w):
    """-> (t_collapse, t_yawflip) in s after onset, or None each."""
    post = [x for x in w if x["t"] >= 0]
    t_col = next((x["t"] for x in post if not np.isnan(x["v"]) and x["v"] < COLLAPSE), None)
    t_flip = None
    for a, b in zip(post, post[1:]):            # two consecutive windows, after onset+6
        if a["t"] >= 6.0 and np.sign(a["yaw_rate"]) == -HEALTHY_SIGN and np.sign(b["yaw_rate"]) == -HEALTHY_SIGN:
            t_flip = a["t"]; break
    return t_col, t_flip


def selftest():
    ok = True

    def chk(name, good):
        nonlocal ok
        ok = ok and good
        print("  %s %s" % ("ok " if good else "FAIL", name))
    t = np.arange(0, 20, 0.01)
    contact = np.ones((len(t), 4), bool); contact[t >= 5.0] = np.tile(((t[t >= 5.0] * 4) % 1 < 0.6)[:, None], (1, 4))
    chk("onset = first all-airborne sample after the standup (got %.2f, want 5.0x)" % gait_onset(t, contact),
        abs(gait_onset(t, contact) - 5.0) < 0.3)
    # onsets(): collapse at +8, flip at +4 (two windows) -> flip precedes
    w = [{"t": k * 2.0, "v": 0.3 if k * 2.0 < 8 else 0.05, "yaw_rate": -1.0 if k * 2.0 < 4 else +1.5, "fr": 0, "lr": 0} for k in range(-2, 15)]
    tc, tf = onsets(w)
    chk("collapse detected at +8 (got %s)" % tc, tc == 8.0)
    chk("two-window flip after onset+6 detected at +6 (got %s)" % tf, tf == 6.0)
    w2 = [{"t": k * 2.0, "v": 0.3, "yaw_rate": -1.0 if k != 5 else +1.0, "fr": 0, "lr": 0} for k in range(0, 15)]
    chk("a single-window flip is NOT an onset", onsets(w2) == (None, None))
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", action="append", default=[])
    ap.add_argument("--label", action="append", default=[])
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        print("onset_capture.py selftest\n"); g = selftest()
        print("\n  SELFTEST %s" % ("PASS" if g else "FAIL")); return 0 if g else 1
    if not a.dir:
        ap.error("need --dir, or --selftest")
    summary = []
    for i, d in enumerate(a.dir):
        d = os.path.expanduser(d); name = a.label[i] if i < len(a.label) else os.path.basename(d.rstrip("/"))
        for tq in sorted(glob.glob(os.path.join(d, "run[0-9].csv"))):
            n = os.path.basename(tq)[3:-4]; od = os.path.join(d, "odom_run%s.csv" % n)
            if not os.path.exists(od):
                continue
            t_on, w = windows(tq, od)
            if t_on is None:
                print("  %s/run%s: no gait onset found" % (name, n)); continue
            tc, tf = onsets(w)
            state = "COLLAPSED @%+.0fs" % tc if tc is not None else "healthy"
            print("\n== %s/run%s  onset at %.2f s  -> %s   yaw-flip: %s" % (name, n, t_on, state, ("@%+.0fs" % tf) if tf is not None else "none"))
            print("   %6s %7s %9s %7s %7s" % ("t", "v_fwd", "yaw d/s", "F/R", "L/R"))
            for x in w:
                flag = " <collapse" if (not np.isnan(x["v"]) and x["v"] < COLLAPSE and x["t"] >= 0) else ""
                print("   %+6.0f %7.3f %9.2f %7.2f %7.2f%s" % (x["t"], x["v"], x["yaw_rate"], x["fr"], x["lr"], flag))
            summary.append((name, n, tc, tf, w))
    print("\n=== summary ===")
    col = [s for s in summary if s[2] is not None]
    print("  runs %d, collapsed %d" % (len(summary), len(col)))
    if not col:
        print("  P-O-1 NOT MET: no collapse in this campaign -- INCONCLUSIVE for P-O-2..5 (say so; do not read the healthy runs as a negative)")
    else:
        for name, n, tc, tf, w in col:
            print("  %s/run%s: collapse @%+.0fs, yaw-flip %s -> P-O-2 %s" % (name, n, tc, ("@%+.0fs" % tf) if tf is not None else "none",
                  "PASS (flip first or same window)" if (tf is not None and tf <= tc) else "FAIL"))
    hf = [s for s in summary if s[2] is None and s[3] is not None]
    print("  P-O-3: healthy runs with a two-window flip: %d of %d -> %s" % (len(hf), len(summary) - len(col), "PASS" if not hf else "FAIL"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
