#!/usr/bin/env python3
"""The TOUCHDOWN ANGLE alpha -- the one G-SLIP state we have never measured.

WHY THIS EXISTS. Lu & Lin parameterise every fixed point by three touchdown
states (v, alpha, beta), and page 3 defines them precisely:

    alpha  touchdown angle -- between the HORIZONTAL and the direction of the
           touchdown VELOCITY VECTOR
    beta   landing angle -- between the horizontal and the segment from the
           point mass to the rim centre, i.e. the LEG angle

Our controller commands beta and measures beta. It has never measured alpha,
because alpha is not a knob: it is an outcome of the flight ballistics and the
apex height. So every comparison this project has made against the paper's
fixed points, and against Eita's (v, alpha, theta_0) table, has silently
matched on v and beta while leaving the third state unexamined.

That matters because alpha is where the regime gap is likely widest. The
model's grid runs alpha 5-25 deg (paper figures 8-10) and Eita's runs 10-18:
shallow, running INTO the feet. A plant at a third of design speed with the
same stride time falls nearly straight DOWN onto them. If that is what is
happening, the model's fixed points are not merely at a different speed --
they are at a qualitatively different touchdown condition, and a basin of
attraction computed around them says nothing about ours.

METHOD. Velocity is differentiated from the odometry POSITION rather than read
from the twist fields: the twist column offsets in a `ros2 topic echo --csv`
of nav_msgs/Odometry depend on the 36-element covariance blocks, and getting
that index wrong yields a plausible wrong number rather than an error.
Touchdown instants come from the torque capture's debounced contact edges --
the same edges touchdown_phase.py and leg_demand.py use -- and are matched to
the nearest odometry sample in SIM time.

    alpha = atan2(-vz, v_horizontal)     degrees, POSITIVE = descending

⚠ WHAT THIS IS NOT. The model's alpha belongs to a point mass. Ours is the
BODY's velocity from the supervisor, which carries pitch and heave the point
mass does not have, and the body is not the CoM of a massless-leg abstraction.
Read it as the regime indicator it is, not as a state to match to four
decimals.

Usage:
    touchdown_velocity_angle.py --dir DIR [--dir DIR ...]
    touchdown_velocity_angle.py --selftest
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_menger import debounce, DEBOUNCE          # noqa: E402
from touchdown_phase import load, Unfit, TAIL_S      # noqa: E402

SMOOTH_S = 0.03          # central-difference half-window, s
MIN_TD = 10


def load_odom_xyzt(path):
    """-> (t, x, y, z) in sim seconds and metres, by column position.

    Matches check_menger.load_odom_csv's convention: stamp sec/nsec at 0/1 and
    position at 4/5/6. Rows shorter than 11 fields are partial lines from a
    killed `topic echo` and are dropped, exactly as that loader does.
    """
    t, x, y, z = [], [], [], []
    with open(path, newline="") as fh:
        for row in csv.reader(fh):
            if len(row) < 11:
                continue
            try:
                t.append(float(row[0]) + 1e-9 * float(row[1]))
                x.append(float(row[4]))
                y.append(float(row[5]))
                z.append(float(row[6]))
            except ValueError:
                continue
    if len(t) < 200:
        raise Unfit("only %d odom rows in %s" % (len(t), os.path.basename(path)))
    a = np.array(sorted(zip(t, x, y, z)))
    return a[:, 0], a[:, 1], a[:, 2], a[:, 3]


def touchdown_times(torque_path, tail_s=TAIL_S):
    """Debounced rising contact edges, in sim seconds, pooled over legs."""
    legs = load(torque_path)
    out = []
    for _leg, (t, c, _b, _bc) in legs.items():
        m = t >= (t.max() - tail_s)
        t, c = t[m], c[m]
        if len(t) < 200:
            continue
        d = debounce(c, DEBOUNCE)
        chg = np.diff(d.astype(int))
        rise = np.flatnonzero(chg > 0) + 1
        fall = np.flatnonzero(chg < 0) + 1
        for r0 in rise:
            nxt = fall[fall > r0]
            if len(nxt) and nxt[0] - r0 >= 3:
                out.append(t[r0])
    return np.array(sorted(out))


def alphas(torque_path, odom_path, tail_s=TAIL_S):
    ot, ox, oy, oz = load_odom_xyzt(odom_path)
    dt = float(np.median(np.diff(ot)))
    if dt <= 0:
        raise Unfit("non-monotonic odom clock")
    k = max(1, int(round(SMOOTH_S / dt)))
    # Central difference over +-k samples. Endpoints are dropped rather than
    # one-sided: a one-sided derivative at an edge is biased, and touchdowns
    # there are no more informative than the ones in the middle.
    vx = (ox[2 * k:] - ox[:-2 * k]) / (ot[2 * k:] - ot[:-2 * k])
    vy = (oy[2 * k:] - oy[:-2 * k]) / (ot[2 * k:] - ot[:-2 * k])
    vz = (oz[2 * k:] - oz[:-2 * k]) / (ot[2 * k:] - ot[:-2 * k])
    vt = ot[k:-k]
    tds = touchdown_times(torque_path, tail_s)
    tds = tds[(tds >= vt.min()) & (tds <= vt.max())]
    if len(tds) < MIN_TD:
        raise Unfit("only %d touchdowns inside the odom window" % len(tds))
    idx = np.searchsorted(vt, tds)
    idx = np.clip(idx, 0, len(vt) - 1)
    vh = np.hypot(vx[idx], vy[idx])
    a = np.degrees(np.arctan2(-vz[idx], vh))
    return a, vh, vz[idx], len(tds)


def _pooled(d, prefix, tail_s):
    """-> (alpha, v_horiz, vz, n) pooled over a cell, from `prefix`_run<N>.csv.

    prefix is "odom" (body origin) or "com" (centre of mass). Both streams are
    nav_msgs/Odometry written by the same driver on the SAME tick with the SAME
    stamp, so load_odom_xyzt reads either unchanged and the two are
    sample-aligned -- no interpolation between clocks.
    """
    A, VH, VZ, n, skipped = [], [], [], 0, []
    for tp in sorted(glob.glob(os.path.join(d, "run[0-9].csv"))):
        pp = os.path.join(d, prefix + "_" + os.path.basename(tp))
        if not os.path.isfile(pp) or os.path.getsize(pp) < 10000:
            continue
        try:
            a, vh, vz, k = alphas(tp, pp, tail_s)
        except Unfit as e:
            skipped.append((os.path.basename(tp), str(e)))
            continue
        A.append(a); VH.append(vh); VZ.append(vz); n += k
    if not A:
        return None, skipped
    return (np.concatenate(A), np.concatenate(VH), np.concatenate(VZ),
            n), skipped


def report(dirs, tail_s, with_com=False, labels=None):
    print("  alpha = angle between horizontal and the touchdown velocity")
    print("  vector, POSITIVE = descending. Lu & Lin sweep 5-25 deg;")
    print("  Eita's table runs 10-18 deg.")
    if with_com:
        print()
        print("  BODY vs CoM. The model's alpha belongs to a POINT MASS. Ours")
        print("  has been the body ORIGIN, which carries S115's 17.9-33.8 deg")
        print("  of peak-to-peak pitch that a point mass does not have. The")
        print("  MEDIAN is robust to that (pitch is ~zero-mean over a stride);")
        print("  the SPREAD is not -- and the spread is the half that decides")
        print("  whether a basin computed around a fixed point means anything")
        print("  against a distribution. That is what the com rows are for.")
    print()
    print("  %-16s %-5s %8s %8s %8s %8s %9s %9s %6s"
          % ("cell", "src", "alpha", "p16", "p84", "p84-p16",
             "v_horiz", "vz", "n"))
    for i, d in enumerate(dirs):
        d = os.path.expanduser(d)
        name = (labels[i] if labels and i < len(labels)
                else os.path.basename(os.path.normpath(d)))
        sources = ["odom"] + (["com"] if with_com else [])
        got_any = False
        for src in sources:
            res, skipped = _pooled(d, src, tail_s)
            for f, why in skipped:
                print("      %s [%s]: skipped -- %s" % (f, src, why))
            if res is None:
                if src == "com":
                    print("  %-16s %-5s NO CoM CAPTURE -- the driver only "
                          "publishes sim/com_odom when CORGI_PUBLISH_COM=1"
                          % (name, src))
                continue
            got_any = True
            A, VH, VZ, n = res
            p16 = float(np.percentile(A, 16))
            p84 = float(np.percentile(A, 84))
            print("  %-16s %-5s %8.1f %8.1f %8.1f %8.1f %9.3f %9.3f %6d"
                  % (name, src, float(np.median(A)), p16, p84, p84 - p16,
                     float(np.median(VH)), float(np.median(VZ)), n))
        if not got_any:
            print("  %-16s UNFIT -- no usable run/odom pair" % name)


def selftest():
    """Known answers on synthetic kinematics."""
    ok = True
    # A body moving at 1 m/s forward and 1 m/s down must read 45 deg.
    vh, vz = 1.0, -1.0
    got = np.degrees(np.arctan2(-vz, vh))
    print("  1. vx=1, vz=-1 -> alpha %.2f deg (want 45.00)" % got)
    if abs(got - 45.0) > 1e-9:
        ok = False; print("     FAIL")
    # Lu & Lin's canonical fixed point: v = 2 m/s at alpha = 10 deg means
    # vz = -v*sin(10) = -0.347 and vh = v*cos(10) = 1.970.
    v, a = 2.0, 10.0
    vh2, vz2 = v * np.cos(np.radians(a)), -v * np.sin(np.radians(a))
    back = np.degrees(np.arctan2(-vz2, vh2))
    print("  2. round-trip the paper's (v 2.0, alpha 10) -> vh %.3f vz %.3f "
          "-> alpha %.2f" % (vh2, vz2, back))
    if abs(back - 10.0) > 1e-9:
        ok = False; print("     FAIL")
    # Ascending at touchdown is physically possible (the model's own text says
    # upward velocity at liftoff is not a necessary condition) and must come
    # back NEGATIVE rather than wrapping.
    got = np.degrees(np.arctan2(-0.5, 1.0))
    print("  3. ascending (vz=+0.5) -> alpha %.2f deg (must be negative)" % got)
    if got >= 0:
        ok = False; print("     FAIL")
    return ok


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", action="append", default=[])
    # ACCEPTED AND USED, added 2026-08-22. Its siblings (touchdown_phase,
    # body_attitude) all take --label, and sweep_torque_ceiling_cor.sh builds
    # ONE $ARGS string of "--dir X --label Y" pairs and passes it to both. This
    # script did not accept --label, so argparse exited 2 and that campaign's
    # entire alpha section has never produced a single line -- silently, since
    # the error went to the log and the campaign carried on. Found when the
    # campaign finally ran on 2026-08-22 (S181).
    ap.add_argument("--label", action="append", default=[])
    ap.add_argument("--tail", type=float, default=TAIL_S)
    ap.add_argument("--com", action="store_true",
                    help="also report alpha on the CoM (needs com_run<N>.csv, "
                         "i.e. a run with CORGI_PUBLISH_COM=1 RECORD_COM=1)")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        print("touchdown_velocity_angle.py selftest")
        ok = selftest()
        print("\n  SELFTEST %s" % ("PASS" if ok else "FAIL"))
        return 0 if ok else 1
    if not a.dir:
        ap.error("need at least one --dir")
    print("touchdown angle alpha -- the third G-SLIP state")
    report(a.dir, a.tail, with_com=a.com, labels=a.label)
    return 0


if __name__ == "__main__":
    sys.exit(main())
