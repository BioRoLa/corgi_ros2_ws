"""How much of the flight phase runs on SOFT (stance) gains?

Alex, 2026-08-20: "why are the feet doing a midair wiggle."

Gain honesty (audit_degradation) asks P(stance gains | foot DOWN) -- is
the spring there when the leg is loaded. This asks the complement, which
nothing measured: P(stance gains | foot UP). A leg carrying leg-frame
stance gains while airborne is compliant with nothing to push against,
so it flops through the swing instead of tracking it -- the visible
"midair wiggle", and a candidate cause of the ~0.14 rad flight tracking
lag measured in S112.

The labelled stance window opens at template row 0 but the foot does not
land until row ~74 (S111), so ~74 ms of every 265 ms cycle is expected
to be exactly this. Measured here rather than inferred.

    floppy_frac   P(stance gains AND airborne)      -- share of the cycle
    floppy_given  P(stance gains | airborne)        -- share of flight

Regime is classified from the L_Motor `kd` histogram gap, the same
adaptive classifier audit_degradation uses, with its refusal path: below
k_flight ~ 4250 the two gain modes merge and the question is meaningless.

Read-only.
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from audit_degradation import (load_run, gait_band, kd_gap_threshold,  # noqa
                               Unfit)
from check_menger import debounce, DEBOUNCE  # noqa: E402


def stats(path):
    t, contact, kd, kp, tff = load_run(path)
    t0, t1 = gait_band(t, tff)
    m = (t >= t0) & (t <= t1)
    t, contact, kd = t[m], contact[m], kd[m]
    thr = kd_gap_threshold(kd)          # raises Unfit if unimodal
    deb = np.column_stack([debounce(contact[:, i], DEBOUNCE)
                           for i in range(4)])
    stance_gain = kd < thr              # soft = stance regime
    up = ~deb
    floppy = stance_gain & up
    return {
        "floppy_frac": float(floppy.mean()),
        "floppy_given_air": float(floppy.sum() / max(1, up.sum())),
        "air_frac": float(up.mean()),
        "stance_gain_frac": float(stance_gain.mean()),
        "honesty": float((stance_gain & deb).sum() / max(1, deb.sum())),
        "kd_thr": thr,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", action="append", required=True)
    ap.add_argument("--label", action="append", default=[])
    args = ap.parse_args()
    print("Per-leg samples over the gait band. 'floppy' = soft stance "
          "gains commanded while that leg is AIRBORNE.\n")
    print(f"{'cell':28} {'floppy%':>8} {'of flight':>10} {'legup%':>8} "
          f"{'softgain%':>10} {'honesty':>8}")
    for i, d in enumerate(args.dir):
        lab = args.label[i] if i < len(args.label) else os.path.basename(
            d.rstrip("/"))
        vals = []
        for p in sorted(glob.glob(os.path.join(os.path.expanduser(d),
                                               "run[0-9].csv"))):
            try:
                vals.append(stats(p))
            except (Unfit, SystemExit) as e:
                print(f"  {os.path.basename(p)}: REFUSED -- {e}")
        if not vals:
            continue
        f = lambda k: float(np.median([v[k] for v in vals]))
        print(f"{lab:28} {100*f('floppy_frac'):7.1f}% "
              f"{100*f('floppy_given_air'):9.1f}% {100*f('air_frac'):7.1f}% "
              f"{100*f('stance_gain_frac'):9.1f}% {f('honesty'):8.2f}")


if __name__ == "__main__":
    main()
