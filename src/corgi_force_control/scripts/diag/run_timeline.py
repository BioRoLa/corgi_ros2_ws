"""Second-by-second health of one run: is it running, and when did it stop?

A whole-window average cannot tell "the gait was fine and then the robot fell
over at t=6" from "the gait was mediocre throughout", and those call for
completely different responses -- the first has a usable window in it, the
second does not. Roll and pitch are the discriminator: a running pronk holds
both inside a couple of degrees (measured < 1.5 deg), so anything past ~15 deg
is the body going over, and every number after that instant describes a robot
lying on the floor.

Usage:  python3 run_timeline.py <dump.npz> [more.npz ...]
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_yaw_phase import dedupe_time  # noqa: E402


def rpy(q):
    """-> roll, pitch, yaw in degrees. q is (n,4) as (x, y, z, w)."""
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)


def main():
    for path in sys.argv[1:]:
        d = np.load(path, allow_pickle=True)
        ot, ov = dedupe_time(d["odom_t"], d["odom"])
        ct, cv, mt, mv = d["contact_t"], d["contact"], d["motor_t"], d["motor_deg"]
        roll, pitch, yaw = rpy(ov[:, 6:10])

        print(f"=== {os.path.basename(path)}")
        print(f"  {'t':>7} {'flight':>7} {'alldn':>6} {'thmax':>7} "
              f"{'roll':>7} {'pitch':>7} {'dyaw':>8} {'v':>7}")
        lo, hi = int(np.floor(ot.min())), int(np.ceil(ot.max()))
        for a in range(lo, hi):
            cm = (ct >= a) & (ct < a + 1)
            om = (ot >= a) & (ot < a + 1)
            mm = (mt >= a) & (mt < a + 1)
            if int(cm.sum()) < 50 or int(om.sum()) < 5:
                continue
            s = cv[cm]
            o = ov[om]
            span = ot[om][-1] - ot[om][0]
            v = (np.hypot(np.diff(o[:, 0]), np.diff(o[:, 1])).sum() / span
                 if span > 0 else float("nan"))
            dy = yaw[om][-1] - yaw[om][0]
            print(f"  {a:5d}-{a+1:<2d} {100*(~s.any(axis=1)).mean():6.1f}% "
                  f"{100*s.all(axis=1).mean():5.1f}% "
                  f"{mv[mm][:, :, 0].max() if int(mm.sum()) else float('nan'):7.2f} "
                  f"{roll[om].mean():+7.2f} {pitch[om].mean():+7.2f} "
                  f"{dy:+8.2f} {v:7.3f}")
        # First instant the body is unambiguously over.
        bad = np.flatnonzero((np.abs(roll) > 15) | (np.abs(pitch) > 15))
        if len(bad):
            print(f"  ATTITUDE LOST at t = {ot[bad[0]]:.2f} s "
                  f"(roll {roll[bad[0]]:+.1f}, pitch {pitch[bad[0]]:+.1f}) "
                  f"-- nothing after this is a gait measurement")
        else:
            print(f"  attitude held all run (|roll| max {np.abs(roll).max():.1f}, "
                  f"|pitch| max {np.abs(pitch).max():.1f})")
        print()


if __name__ == "__main__":
    main()
