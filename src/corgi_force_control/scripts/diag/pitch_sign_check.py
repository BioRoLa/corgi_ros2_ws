#!/usr/bin/env python3
"""Does force_estimation use the right pitch sign on each leg? Checked against force plates.

force_estimation evaluates each leg at beta +/- body pitch (A, D: + ; B, C: -).
A wrong sign on a leg is a beta error of 2*pitch there, and a beta error d
rotates that leg's estimated sagittal force (fx, fz) by d. So: stand level,
then pitch the stance, and compare how the estimate CHANGES between the two
against how the plate changes. Using changes cancels the static offsets the
estimate already has. Then rotate the estimate by k*pitch and report the miss:
  right sign: smallest miss at k ~ 0 on every leg
  wrong sign: smallest miss at k ~ +/-2 on that leg

Run (sim): rock_cycle.sh pitch_fz rock_amp:=10.0 rock_hold_s:=5.0 stance_pitch_dz:=0.045
exp_h24 then holds level for half the hold (after the support box drops), ramps
the pitch over 1 s and rocks. Find the windows from the pitch column of
--timeline, then:
  pitch_sign_check.py <bag> --level 3.8 5.6 --pitched 7.5

Needs /force/state, /sensor/force_plate_1..4 and /imu in the bag. The pitch is
computed exactly as force_estimation does in sim; on hardware it negates it
(pitch *= -1), so pass --hardware for a real-plate run. Plates come from the bag
only (sim); see force_plate_track.py for the plate frame and leg mapping.

2026-10-12, sim, 5.6 deg pitch: misses at k = -2 / 0 / +2 were
B 12.6 / 2.9 / 6.8 N, C 12.1 / 1.2 / 13.5 N -> B/C's "beta - pitch" is right.
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import force_plate_track as fpt  # noqa: E402


def read_pitch(path, hardware):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from corgi_msgs.msg import ImuStamped
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    rows = []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic != "/imu":
            continue
        m = deserialize_message(data, ImuStamped)
        q, st = m.orientation, m.header.stamp
        n = np.sqrt(q.w ** 2 + q.x ** 2 + q.y ** 2 + q.z ** 2)
        w, x, y, z = q.w / n, q.x / n, q.y / n, q.z / n
        rows.append((st.sec + st.nanosec * 1e-9, np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))))
    if not rows:
        sys.exit("no /imu in the bag -- record it (rock_cycle.sh does)")
    a = np.asarray(rows)
    if hardware:
        a[:, 1] *= -1
    return a


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--level", nargs=2, type=float, metavar=("T0", "T1"), help="level reference window, bag time s")
    ap.add_argument("--pitched", type=float, metavar="T0", help="pitched stance from this bag time s to the end")
    ap.add_argument("--timeline", action="store_true", help="print pitch and total plate load every 0.5 s")
    ap.add_argument("--hardware", action="store_true", help="negate pitch as force_estimation does off-sim")
    ap.add_argument("--plate-map", default="a=1,b=4,c=3,d=2")
    args = ap.parse_args()

    b = fpt.read_bag(args.bag)
    if "plate" not in b:
        sys.exit("no /sensor/force_plate_1..4 in the bag")
    imu = read_pitch(args.bag, args.hardware)
    t0 = max(b["est"]["t"][0], b["plate"]["t"][0], imu[0, 0])
    t1 = min(b["est"]["t"][-1], b["plate"]["t"][-1], imu[-1, 0])
    g = np.arange(t0, t1, 0.002)
    pitch = np.interp(g, imu[:, 0], imu[:, 1])
    est = fpt.resample(b["est"], g)
    plate = fpt.plates_to_legs(fpt.resample(b["plate"], g), fpt.plate_map(None, 0.0, args.plate_map), 0.0)

    if args.timeline or args.level is None or args.pitched is None:
        tot = sum(plate[l + "fz"] for l in fpt.LEGS)
        for t in np.arange(g[0], g[-1], 0.5):
            i = np.searchsorted(g, t)
            print("%7.2f  pitch %+6.2f deg   plate sum fz %7.1f N" % (t, np.degrees(pitch[i]), tot[i]))
        if args.level is None or args.pitched is None:
            print("\npick --level (after the load settles, before the ramp) and --pitched from the above")
            return 0

    level = (g > args.level[0]) & (g < args.level[1])
    pitched = g > args.pitched
    if level.sum() < 100 or pitched.sum() < 100:
        sys.exit("window too short: level %d, pitched %d samples" % (level.sum(), pitched.sum()))
    dp = np.degrees(pitch[pitched].mean() - pitch[level].mean())
    print("pitch: level %+.2f deg, pitched %+.2f deg, change %+.2f deg" %
          (np.degrees(pitch[level].mean()), np.degrees(pitch[pitched].mean()), dp))
    if abs(dp) < 2.0:
        print("warning: under 2 deg of pitch change -- the test cannot tell k = 0 from k = +/-2")

    ks = np.arange(-4, 4.01, 0.25)
    print("\nleg  plate change fx/fz N   miss at k=-2    k=0     k=+2    best k")
    for l in fpt.LEGS:
        dpx = plate[l + "fx"][pitched].mean() - plate[l + "fx"][level].mean()
        dpz = plate[l + "fz"][pitched].mean() - plate[l + "fz"][level].mean()
        miss = []
        for k in ks:
            d = k * pitch
            fx = est[l + "fx"] * np.cos(d) + est[l + "fz"] * np.sin(d)
            fz = -est[l + "fx"] * np.sin(d) + est[l + "fz"] * np.cos(d)
            miss.append(np.hypot(fx[pitched].mean() - fx[level].mean() - dpx,
                                 fz[pitched].mean() - fz[level].mean() - dpz))
        miss = np.asarray(miss)
        at = lambda k: miss[np.argmin(np.abs(ks - k))]
        kb = ks[np.argmin(miss)]
        verdict = "" if abs(kb) < 1.0 else ("   <-- sign looks wrong" if abs(abs(kb) - 2) <= 0.75 else "   <-- off, not a sign flip")
        print(" %s   %+6.1f / %+6.1f        %6.1f  %6.1f  %6.1f    %+5.2f%s"
              % (l.upper(), dpx, dpz, at(-2), at(0), at(2), kb, verdict))
    return 0


if __name__ == "__main__":
    sys.exit(main())
