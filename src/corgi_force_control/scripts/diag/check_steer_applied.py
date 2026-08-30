"""Did the steering differential actually reach the legs, and how big was it?

A sweep that produces no trend has two very different explanations -- the
channel has no authority, or the channel was never applied -- and they are
indistinguishable from the yaw alone. check_ramp does not dump the commanded
stream, so this reads the MEASURED beta instead and asks whether the left pair
and the right pair were actually held apart during contact.

gslip_pronk applies `s * u` with s = {+1,-1,-1,+1} on A,B,C,D, gated on stance,
so the expected left-minus-right differential while the feet are down is 2*u --
16 deg of measured separation for an 8 deg command. Anything much smaller means
the legs did not track the command (torque limit, or the impedance being too
soft to hold the offset against ground contact), which is a completely different
finding from "the differential does not steer".

Usage:  python3 check_steer_applied.py <tag>=<offset_deg>:<dump.npz> ...
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_yaw_phase import contact_at  # noqa: E402

LEFT = (0, 3)
RIGHT = (1, 2)
START, END = 3.5, 10.0


def main():
    print(f"{'tag':>5} {'cmd u':>7} {'expect':>8} {'measured L-R':>13} "
          f"{'ratio':>7} {'L in stance':>12} {'R in stance':>12}")
    for arg in sys.argv[1:]:
        tag, rest = arg.split("=", 1)
        off_deg, path = rest.split(":", 1)
        u = float(off_deg)
        d = np.load(path, allow_pickle=True)
        mt, mv = d["motor_t"], d["motor_deg"]
        ct, cv = d["contact_t"], d["contact"]
        m = (mt >= START) & (mt <= END)
        mtt, beta = mt[m], mv[m][:, :, 1]
        down = contact_at(mtt, ct, cv)

        # Per leg, averaged only over the samples where THAT leg is down. The
        # offset is stance-gated, so averaging over flight too would dilute it
        # by roughly the duty factor and make a working channel look weak.
        per_leg = []
        for i in range(4):
            sel = down[:, i]
            per_leg.append(float(beta[sel, i].mean()) if sel.sum() > 50
                           else float("nan"))
        l = float(np.mean([per_leg[i] for i in LEFT]))
        r = float(np.mean([per_leg[i] for i in RIGHT]))
        print(f"{tag:>5} {u:+6.2f}d {2*u:+7.2f}d {l-r:+12.2f}d "
              f"{(l-r)/(2*u) if u else float('nan'):7.2f} "
              f"{l:+11.2f}d {r:+11.2f}d")


if __name__ == "__main__":
    main()
