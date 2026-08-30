"""Does kp scale with k_radial when the POSE is held equal?

The two k_radial sweeps both came back with R^2 < 0.09 because the pose did not
repeat: kp = J^T K J and J depends on theta, and the robot does not sit still on
the support box -- theta swings up to 43 deg within a single arm. Comparing arm
means therefore compares geometry as much as gains.

But the arms OVERLAP. All four cover roughly theta 92-97 deg, so kp can be
compared inside a narrow common band where J is nearly identical. That turns a
confounded between-arm comparison into a controlled one, using data already on
disk.

WHAT THE ANSWER MEANS

  kp rises with k_radial, slope ~ u_r^2 ~ 0.0027
      the leg_frame mapping works; the anomalous kp_stance/kp_flight ratio has
      some other cause and the erosion is not a mis-applied radial gain
  kp flat in k_radial
      force_control is not applying the commanded radial stiffness. Since
      t_stiff = kp * error is the dominant torque term, that would put a
      mis-scaled gain directly under the 13-14x erosion

The band is chosen from the data rather than fixed, and the script refuses to
report if any arm contributes too few samples inside it -- a "matched pose"
built from 3 samples of one arm is not matched, it is a coincidence.
"""
import csv
import sys
from collections import defaultdict

import numpy as np

CSV = "/tmp/corgi_torque_terms.csv"
BRACKETS = "/tmp/k_radial_sweep_brackets.txt"
LEG_MOTORS = ("L_Motor", "R_Motor")
MIN_PER_ARM = 100          # samples required inside the band, per arm
DL_DTHETA = 0.10324
U_R_EXPECTED = DL_DTHETA / 2.0


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else CSV
    br_path = sys.argv[2] if len(sys.argv) > 2 else BRACKETS

    brackets = []
    for line in open(br_path):
        p = line.split()
        if len(p) == 3 and p[1] != "none":
            brackets.append((float(p[0]), float(p[1]), float(p[2])))

    rows = []
    for r in csv.DictReader(open(csv_path, newline="")):
        if r["motor"] not in LEG_MOTORS:
            continue
        if "theta" not in r or r["theta"] in (None, "", "nan"):
            print("CSV has no theta column -- rebuild the driver with pose "
                  "logging and re-run the sweep.")
            return
        rows.append((float(r["t"]), float(r["kp"]), float(r["theta"]),
                     float(r["beta"]), float(r["pos_error"])))
    a = np.array(rows)

    # Per-arm sample sets, with the settling head of each window trimmed.
    arms = []
    for k, t0, t1 in brackets:
        lo = t0 + 0.4 * (t1 - t0)
        s = a[(a[:, 0] >= lo) & (a[:, 0] <= t1)]
        if len(s):
            arms.append((k, s))
    if len(arms) < 3:
        print("need at least 3 arms")
        return

    # Common theta band: the intersection of the arms' central ranges, taken as
    # the widest interval every arm actually populates.
    lo_t = max(np.percentile(np.degrees(s[:, 2]), 25) for _, s in arms)
    hi_t = min(np.percentile(np.degrees(s[:, 2]), 95) for _, s in arms)
    print()
    print("=" * 74)
    print("kp vs k_radial AT MATCHED POSE")
    print("=" * 74)
    if hi_t <= lo_t:
        print(f"  no common theta band (lo {lo_t:.2f} >= hi {hi_t:.2f}).")
        print("  The arms never sat in the same pose; this data cannot answer")
        print("  the question however it is sliced.")
        return
    print(f"  common theta band: {lo_t:.2f} - {hi_t:.2f} deg")
    print()
    print(f"{'k_radial':>10} {'n in band':>10} {'theta mean':>11} "
          f"{'beta mean':>10} {'kp mean':>9} {'kp sd':>8} {'kp SE':>7}")

    ks, kps, ok = [], [], True
    for k, s in arms:
        th = np.degrees(s[:, 2])
        sel = (th >= lo_t) & (th <= hi_t)
        n = int(sel.sum())
        if n < MIN_PER_ARM:
            print(f"{k:10.0f} {n:10d}   TOO FEW SAMPLES IN BAND")
            ok = False
            continue
        kp = s[sel, 1]
        ks.append(k)
        kps.append(kp.mean())
        print(f"{k:10.0f} {n:10d} {th[sel].mean():11.2f} "
              f"{np.degrees(s[sel,3]).mean():10.2f} {kp.mean():9.1f} "
              f"{kp.std():8.1f} {kp.std()/np.sqrt(n):7.2f}")
    if not ok or len(ks) < 3:
        print("\n  refusing to fit -- not every arm is represented in the band")
        return

    ks_a, kps_a = np.array(ks), np.array(kps)
    slope, intercept = np.polyfit(ks_a, kps_a, 1)
    pred = slope * ks_a + intercept
    ss_res = float(((kps_a - pred) ** 2).sum())
    ss_tot = float(((kps_a - kps_a.mean()) ** 2).sum())
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")

    print()
    print(f"  fit: kp = {slope:.6f} * k_radial + {intercept:.1f}   R^2 = {r2:.3f}")
    print(f"  predicted slope if the mapping works: "
          f"{U_R_EXPECTED**2:.6f}  (u_r^2)")
    print(f"  kp span across an {ks_a.max()/ks_a.min():.0f}x k_radial change: "
          f"{kps_a.max()-kps_a.min():.1f} N.m/rad "
          f"({(kps_a.max()-kps_a.min())/kps_a.mean():.0%} of mean)")
    print()
    print("=" * 74)
    expected_span = U_R_EXPECTED**2 * (ks_a.max() - ks_a.min())
    print(f"  If the mapping worked, an 8x k_radial change should move kp by")
    print(f"  about {expected_span:.0f} N.m/rad. Observed: "
          f"{kps_a.max()-kps_a.min():.1f}.")
    print()
    if r2 < 0.7:
        print("  STILL INCONCLUSIVE at matched pose -- kp is not a function of")
        print("  k_radial in any simple way here. Do not read a slope.")
    elif slope > 0.5 * U_R_EXPECTED**2:
        print("  kp DOES track k_radial at roughly the expected rate. The")
        print("  leg_frame mapping is applying the commanded radial stiffness,")
        print("  and the kp_stance/kp_flight anomaly needs another explanation.")
    else:
        print("  kp does NOT track k_radial even at matched pose. force_control")
        print("  is not applying the commanded radial stiffness -- and since")
        print("  t_stiff = kp * error dominates the torque, that sits directly")
        print("  under the 13-14x erosion.")
    print("=" * 74)
    print()


if __name__ == "__main__":
    main()
