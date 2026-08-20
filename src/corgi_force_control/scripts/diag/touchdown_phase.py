"""At what point in the beta cycle does the foot actually land and leave?

S110 left one thing unexplained: the +65 ms shifted gait FLIES (36-41%
air) and still rolls backward in every stance bin, so its contact is
centred on the beta backswing -- but the label shift changes only gains,
never the trajectory. Something moves touchdown indirectly.

The template intends, per its own labels (v070, 265 rows, stance 0-107):
    touchdown at beta = -0.1614 rad  (stance onset row)
    liftoff   at beta = +0.1609 rad  (last stance row)
i.e. the foot lands at the BACK of the swing and rolls FORWARD through
stance. Measuring the beta actually held at each debounced rising and
falling edge says whether the real gait does that, and if not, where it
lands instead.

Reported per leg and pooled:
    beta_TD   beta at the debounced touchdown edge
    beta_LO   beta at the debounced liftoff edge
    sweep     beta_LO - beta_TD, signed: POSITIVE rolls the robot
              forward, negative rolls it backward
    on_fore   fraction of contact samples taken while beta is INCREASING
              (the forward sweep) -- 1.0 means the foot is only ever
              down during the forward stroke, 0.0 only during backswing

Read-only. Uses the same debounce as every touchdown detector here.
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_menger import debounce, DEBOUNCE  # noqa: E402

LEGS = "ABCD"
DIR_BETA = {"A": +1.0, "B": -1.0, "C": -1.0, "D": +1.0}
TAIL_S = 20.0
TPL_TD = -0.1614
TPL_LO = +0.1609


class Unfit(Exception):
    pass


def load(path):
    """-> {leg: (t, contact, beta_meas, beta_cmd)}.

    `beta` in the capture is leg.get_states(), i.e. MEASURED. The
    COMMANDED beta is recovered as beta_meas + 0.5*(e_L + e_R), where e
    is each hip motor's pos_error = phi_des - phi_fb: beta is the COMMON
    mode of the two hip motors (theta is the differential), so their
    mean error is the beta error. Same construction as
    report_kt_sweep.py's `dbeta = 0.5 * (eL + eR)`.

    This separates "the foot lands late in the beta cycle" (a timing
    fault, fixable by rotating beta) from "the leg has not reached its
    commanded beta yet" (a tracking fault, which rotating beta would
    make worse).

    ⚠ THE MIRROR IS NOT OPTIONAL. pos_error is in MOTOR coordinates and
    CORGI_DIRBETA_TRANSFORM mirrors beta on the right pair, so the raw
    common mode is sign-flipped on B and C. Measured 2026-08-20: the
    error at touchdown is {A,D} +0.13 and {B,C} -0.13 in every cell --
    pooling without the mirror averages them to ~0 and reports a
    confident "the leg tracks its command", which is false. dir_beta =
    {A:+1, B:-1, C:-1, D:+1}, the same partition as roll_sign/steer_sign.
    """
    per = {}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            if r["motor"] not in ("L_Motor", "R_Motor"):
                continue
            try:
                d = per.setdefault((r["leg"], float(r["t"])), {})
                d["c"] = int(r["in_contact"])
                d["b"] = float(r["beta"])
                d["e" + r["motor"][0]] = float(r["pos_error"])
            except (ValueError, KeyError):
                continue
    byleg = {}
    for (leg, t), d in per.items():
        if "eL" not in d or "eR" not in d:
            continue
        bcmd = d["b"] + DIR_BETA[leg] * 0.5 * (d["eL"] + d["eR"])
        byleg.setdefault(leg, []).append((t, d["c"], d["b"], bcmd))
    out = {}
    for leg, rows in byleg.items():
        rows.sort()
        a = np.array(rows)
        if len(a) < 500:
            continue
        out[leg] = (a[:, 0], a[:, 1].astype(bool), a[:, 2], a[:, 3])
    if not out:
        raise Unfit("no usable legs")
    return out


def stats(path, tail_s=TAIL_S):
    legs = load(path)
    td, lo, sweep, fore, tdc = [], [], [], [], []
    for leg, (t, c, b, bc) in legs.items():
        m = t >= (t.max() - tail_s)
        t, c, b, bc = t[m], c[m], b[m], bc[m]
        if len(t) < 200:
            continue
        d = debounce(c, DEBOUNCE)
        chg = np.diff(d.astype(int))
        rise = np.flatnonzero(chg > 0) + 1
        fall = np.flatnonzero(chg < 0) + 1
        for r0 in rise:
            nxt = fall[fall > r0]
            if not len(nxt):
                continue
            f0 = nxt[0]
            if f0 - r0 < 3:
                continue
            td.append(b[r0])
            tdc.append(bc[r0])
            lo.append(b[f0])
            sweep.append(b[f0] - b[r0])
        # fraction of contact time spent on the forward (rising) stroke
        db = np.diff(b)
        cm = d[:-1]
        if cm.sum() > 50:
            fore.append(float((db[cm] > 0).mean()))
    if len(td) < 20:
        raise Unfit(f"only {len(td)} stance episodes")
    return {"beta_TD": float(np.median(td)), "beta_LO": float(np.median(lo)),
            "beta_TD_cmd": float(np.median(tdc)),
            "track_err": float(np.median(tdc)) - float(np.median(td)),
            "sweep": float(np.median(sweep)),
            "on_fore": float(np.mean(fore)) if fore else np.nan,
            "n": len(td)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", action="append", required=True)
    ap.add_argument("--label", action="append", default=[])
    args = ap.parse_args()
    print(f"template intends: beta_TD {TPL_TD:+.4f} -> beta_LO {TPL_LO:+.4f}"
          f"  (sweep {TPL_LO-TPL_TD:+.4f}, forward)\n")
    print(f"{'cell':30} {'TD_meas':>9} {'TD_cmd':>9} {'track':>8} "
          f"{'beta_LO':>9} {'sweep':>9} {'n':>5}")
    for i, d in enumerate(args.dir):
        lab = args.label[i] if i < len(args.label) else os.path.basename(
            d.rstrip("/"))
        vals = []
        for p in sorted(glob.glob(os.path.join(os.path.expanduser(d),
                                               "run[0-9].csv"))):
            try:
                vals.append(stats(p))
            except Unfit as e:
                print(f"  {os.path.basename(p)}: REFUSED -- {e}")
        if not vals:
            continue
        f = lambda k: float(np.median([v[k] for v in vals]))
        print(f"{lab:30} {f('beta_TD'):+9.4f} {f('beta_TD_cmd'):+9.4f} "
              f"{f('track_err'):+8.4f} {f('beta_LO'):+9.4f} "
              f"{f('sweep'):+9.4f} {sum(v['n'] for v in vals):5d}")


if __name__ == "__main__":
    main()
