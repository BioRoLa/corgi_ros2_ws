"""Does the stance kp scale with the commanded k_radial?

Reads the brackets written by sweep_k_radial_hold.sh and the torque-decomposition
CSV, and reports mean kp per arm.

THE MODEL BEING TESTED. force_control forms K in the leg frame and maps it to
joint space as K_joint = J^T K J, publishing the diagonal as kp. With the
Jacobian column resolved into leg-frame components (u_r, u_y, u_t):

    kp = k_radial * u_r^2 + k_lateral * u_y^2 + k_tangential * u_t^2

Held at a fixed pose, u is constant, so kp must be AFFINE in k_radial with
slope u_r^2 and a positive intercept from the other two terms. Two things
falsify it:

  slope ~ 0        the leg_frame branch is not applying k_radial at all
  slope < 0        a sign error somewhere in the mapping

and the fitted slope gives u_r^2 directly, which is checkable against geometry:
u_r should be about (dl/dtheta)/2 = 0.0516, so slope ~ 0.0027 N.m/rad per N/m.

WHY A FIT AND NOT A RATIO. Two arms would give a ratio contaminated by the
intercept; the other two stiffnesses contribute a constant that does not scale.
Fitting across four arms separates slope from intercept.
"""
import csv
import sys
from collections import defaultdict

import numpy as np

CSV = "/tmp/corgi_torque_terms.csv"
BRACKETS = "/tmp/k_radial_sweep_brackets.txt"
LEG_MOTORS = ("L_Motor", "R_Motor")

# Geometry expectation, for the slope sanity check.
DL_DTHETA = 0.10324
U_R_EXPECTED = DL_DTHETA / 2.0          # dtheta/dphi = 0.5, from the Jacobian


def main():
    brackets = []
    for line in open(sys.argv[2] if len(sys.argv) > 2 else BRACKETS):
        p = line.split()
        if len(p) == 3 and p[1] != "none":
            brackets.append((float(p[0]), float(p[1]), float(p[2])))
    if not brackets:
        print("no usable brackets")
        return

    rows = defaultdict(list)
    for r in csv.DictReader(open(sys.argv[1] if len(sys.argv) > 1 else CSV,
                                 newline="")):
        if r["motor"] not in LEG_MOTORS:
            continue
        rows[(r["leg"], r["motor"])].append(
            (float(r["t"]), int(r["in_contact"]), float(r["kp"]),
             float(r["t_stiff"]), float(r["pos_error"])))
    d = {k: np.array(v) for k, v in rows.items()}

    print()
    print("=" * 74)
    print("DOES STANCE kp SCALE WITH COMMANDED k_radial?")
    print("=" * 74)
    print(f"{'k_radial':>10} {'window (s)':>20} {'n':>7} {'mean kp':>9} "
          f"{'sd':>7} {'mean|err|':>10}")

    ks, kps = [], []
    for k, t0, t1 in brackets:
        # Trim the first 40% of each window: the pose is still settling after
        # the controller starts, and kp depends on pose through J.
        lo = t0 + 0.4 * (t1 - t0)
        acc_kp, acc_err, n = [], [], 0
        for v in d.values():
            sel = (v[:, 0] >= lo) & (v[:, 0] <= t1)
            if sel.any():
                acc_kp.append(v[sel, 2])
                acc_err.append(np.abs(v[sel, 4]))
                n += int(sel.sum())
        if not acc_kp:
            print(f"{k:10.0f} {t0:9.1f}-{t1:<9.1f} {'no samples':>7}")
            continue
        allkp = np.concatenate(acc_kp)
        allerr = np.concatenate(acc_err)
        ks.append(k)
        kps.append(allkp.mean())
        print(f"{k:10.0f} {t0:9.1f}-{t1:<9.1f} {n:7d} {allkp.mean():9.1f} "
              f"{allkp.std():7.1f} {allerr.mean():10.4f}")

    if len(ks) < 3:
        print("\nneed at least 3 arms to separate slope from intercept")
        return

    ks_a, kps_a = np.array(ks), np.array(kps)
    slope, intercept = np.polyfit(ks_a, kps_a, 1)
    pred = slope * ks_a + intercept
    ss_res = float(((kps_a - pred) ** 2).sum())
    ss_tot = float(((kps_a - kps_a.mean()) ** 2).sum())
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")

    print()
    print("=" * 74)
    print("FIT  kp = slope * k_radial + intercept")
    print("=" * 74)
    print(f"  slope     {slope:12.6f} N.m/rad per N/m")
    print(f"  intercept {intercept:12.2f} N.m/rad")
    print(f"  R^2       {r2:12.4f}")
    print()
    print(f"  implied u_r = sqrt(slope) = {np.sqrt(abs(slope)):.4f}"
          f"   (geometry expects ~{U_R_EXPECTED:.4f})")
    print()
    span = kps_a.max() - kps_a.min()
    frac = span / kps_a.mean() if kps_a.mean() else 0.0
    print(f"  kp spans {span:.1f} N.m/rad ({frac:.0%} of its mean) across a "
          f"{ks_a.max()/ks_a.min():.0f}x change in k_radial")
    print()
    # Refuse to name a verdict on a fit that explains nothing. The first real
    # run came back non-monotonic (174.6, 263.5, 269.5, 170.4) with R^2 = 0.088
    # and this function confidently reported "sign error" from the sign of a
    # meaningless slope. A line through scatter always has a sign.
    #
    # The pose check is the companion guard: kp = J^T K J depends on pose, so
    # the arms are only comparable if the pose repeated. Wildly different mean
    # tracking error across arms means it did not, and the comparison is
    # confounded regardless of how good the fit looks.
    if r2 < 0.7:
        print(f"  INCONCLUSIVE: R^2 = {r2:.3f}. The arms do not lie on a line,")
        print("  so no slope can be read from them -- and a slope's SIGN is")
        print("  meaningless when the fit explains nothing.")
        print()
        print("  Most likely the pose was not held constant across arms, which")
        print("  is the premise the whole comparison rests on: kp = J^T K J,")
        print("  and J is pose-dependent. Check mean|err| above -- if it varies")
        print("  by more than ~2x the legs were sitting differently in each arm")
        print("  and k_radial is confounded with geometry.")
        print()
        print("  To fix: log theta/beta/gamma alongside kp and compare arms at")
        print("  MATCHED pose, or hold the legs unloaded so the commanded pose")
        print("  is achieved exactly and repeats.")
        print("=" * 74)
        print()
        return
    if abs(slope) < 1e-4:
        print("  VERDICT: kp does NOT track k_radial.")
        print("  The leg_frame branch is not applying the commanded radial")
        print("  stiffness. That makes the measured stance gain independent of")
        print("  the design parameter, and explains a kp_stance/kp_flight ratio")
        print("  that no combination of the commanded gains can produce.")
    elif slope < 0:
        print("  VERDICT: kp moves the WRONG WAY with k_radial -- sign error.")
    else:
        ratio = np.sqrt(abs(slope)) / U_R_EXPECTED
        print(f"  VERDICT: kp does track k_radial, at {ratio:.2f}x the lever")
        print("  arm geometry implies.")
        if ratio > 1.5:
            print("  The mapping is live but over-scaled -- same shape as the")
            print("  k_lateral / kp_h error in S11.")
        else:
            print("  Scaling looks sane; the anomaly is elsewhere.")
    print("=" * 74)
    print()


if __name__ == "__main__":
    main()
