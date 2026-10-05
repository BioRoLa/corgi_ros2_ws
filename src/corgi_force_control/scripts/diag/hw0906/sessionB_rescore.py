#!/usr/bin/env python3
"""rescore.py -- Session B outcomes on the CORRECTED Vicon-to-bag assignment.

Assignment evidence (three channels, all agreeing):
  TIME    x2d capture time against the bag mtimes observed on the Orin.
  ORDER   the capture sequence reproduces the run order that was issued.
  PHYSICS the curvature sign matches the cell each bag's banner recorded.

The common-mode check is computed on YAW RATE, not curvature, per the memory
`mirror-contrast-cancels-a-failed-drift-gate`: -lambda ran 18 % faster than
+lambda on 09-06, so a curvature common mode mixes a speed difference into a
heading statistic.
"""
import math

# name -> (v_fwd m/s, yaw rate deg/s, kappa 1/m)
V = {
    "S3_L0A1":             (0.364, -0.16, -0.0074),
    "S3_L0A2":             (0.184, +9.34, +0.8857),
    "S3_L0A3":             (0.198, +7.06, +0.6227),
    "s3_diag_pitchoff_a1": (0.178, +2.34, +0.2290),
    "s3_ol10n_a1":         (0.292, -4.64, -0.2772),
    "s3_l10_a2":           (0.349, -1.32, -0.0658),
    "s3_ol10_a2":          (0.202, +3.52, +0.3034),
    "s3_ol10n_a2":         (0.298, -3.34, -0.1958),
    "s3_l0_a3":            (0.340, -2.52, -0.1292),
    "s3_ol15n_a1":         (0.248, -6.91, -0.4861),
    "s3_ol10n_a1b":        (0.092, -5.30, -1.0038),
    "s3_ol10n_a1c":        (0.299, -2.92, -0.1709),
    "s3_ol12n_a1":         (0.310, -7.22, -0.4065),
    "s3_ol12_a1":          (0.162, +7.85, +0.8463),
    "s3_l0_dp0_a1":        (0.290, -1.48, -0.0889),
}

# corrected assignment: vicon trial -> (bag, cell, disposition)
MAP = [
    ("S3_L0A1",             "s3_l0_a1",            "l0",      "baseline 1"),
    ("S3_L0A2",             "s3_ol10_a1",          "+10",     "VOID: power pulled mid-arc"),
    ("S3_L0A3",             "s3_ol10_a1v2",        "+10",     "valid"),
    ("s3_diag_pitchoff_a1", "s3_diag_pitchoff_a1", "l0*",     "exploratory: pitch channel off"),
    ("s3_ol10n_a1",         "s3_ol10n_a1",         "-10",     "valid (Vicon only; bag empty)"),
    ("s3_l10_a2",           "s3_l0_a2",            "l0",      "baseline 2 (trial name typo)"),
    ("s3_ol10_a2",          "s3_ol10_a2",          "+10",     "valid"),
    ("s3_ol10n_a2",         "s3_ol10n_a2",         "-10",     "valid"),
    ("s3_l0_a3",            "s3_l0_a3",            "l0",      "baseline 3"),
    ("s3_ol15n_a1",         "s3_ol15n_a1",         "-15",     "valid"),
    ("s3_ol10n_a1b",        "s3_ol10n_a1b",        "-10",     "VOID: 0.21 m spin"),
    ("s3_ol10n_a1c",        "s3_ol10n_a1c",        "-10",     "valid"),
    ("s3_ol12n_a1",         "s3_ol12n_a1",         "-12.5",   "valid"),
    ("s3_ol12_a1",          "s3_ol12_a1",          "+12.5",   "VOID: 0.44 m spin"),
    ("s3_l0_dp0_a1",        "s3_l0_dp0_a1",        "l0*",     "arc 14: d_pitch 0"),
]

NOVICON = ["s3_l0_a1b", "s3_l0_kp60_a1", "s3_l0_kp60d0_a1"]

print("=== corrected assignment ===")
print("  %-22s %-22s %-7s %-30s %6s %8s %9s"
      % ("vicon trial", "bag", "cell", "disposition", "v", "yaw", "kappa"))
for t, b, c, d in MAP:
    v, y, k = V[t]
    print("  %-22s %-22s %-7s %-30s %6.3f %+8.2f %+9.4f" % (t, b, c, d, v, y, k))
print("\n  bags with NO Vicon capture (23:12-23:41 gap, all exploratory): %s"
      % ", ".join(NOVICON))

l0 = [t for t, b, c, d in MAP if c == "l0"]
vbar = sum(V[t][0] for t in l0) / len(l0)
ybar = sum(V[t][1] for t in l0) / len(l0)
floor = 0.85 * vbar
print("\n=== the floor, from THIS session's baselines ===")
for t in l0:
    print("  %-22s v %.3f  yaw %+.2f" % (t, V[t][0], V[t][1]))
print("  lambda0 mean v = %.4f m/s  ->  0.85x floor = %.4f m/s" % (vbar, floor))
print("  lambda0 mean yaw = %+.2f deg/s   (spread %+.2f to %+.2f, range %.2f)"
      % (ybar, min(V[t][1] for t in l0), max(V[t][1] for t in l0),
         max(V[t][1] for t in l0) - min(V[t][1] for t in l0)))

print("\n=== every scored arc against the floor ===")
for t, b, c, d in MAP:
    if d.startswith("VOID"):
        continue
    v = V[t][0]
    print("  %-22s %-7s v %.3f = %.2fx floor  %s"
          % (b, c, v, v / floor, "PASS" if v >= floor else "FAIL"))

def cell(c):
    return [t for t, b, cc, d in MAP if cc == c and not d.startswith("VOID")]

print("\n=== mirror contrast at +/-10 ===")
p, n = cell("+10"), cell("-10")
kp = sum(V[t][2] for t in p) / len(p)
kn = sum(V[t][2] for t in n) / len(n)
yp = sum(V[t][1] for t in p) / len(p)
yn = sum(V[t][1] for t in n) / len(n)
print("  +10 arcs (n=%d): kappa %s  -> mean %+.4f" % (len(p), [round(V[t][2], 4) for t in p], kp))
print("  -10 arcs (n=%d): kappa %s  -> mean %+.4f" % (len(n), [round(V[t][2], 4) for t in n], kn))
print("  camber term  (kappa(+) - kappa(-))/2 = %+.4f 1/m   [09-06: +0.2420]" % ((kp - kn) / 2))
print("  common mode  (kappa(+) + kappa(-))/2 = %+.4f 1/m   vs lambda0 drift %+.4f"
      % ((kp + kn) / 2, sum(V[t][2] for t in l0) / len(l0)))
print("\n  on YAW RATE (the registered form of the check):")
print("  +10 yaw %s -> %+.2f ; -10 yaw %s -> %+.2f"
      % ([V[t][1] for t in p], yp, [V[t][1] for t in n], yn))
print("  camber term  %+.2f deg/s ;  common mode %+.2f deg/s  vs lambda0 %+.2f deg/s"
      % ((yp - yn) / 2, (yp + yn) / 2, ybar))
print("  |common mode - lambda0| = %.2f deg/s  against a lambda0 spread of %.2f deg/s"
      % (abs((yp + yn) / 2 - ybar), max(V[t][1] for t in l0) - min(V[t][1] for t in l0)))

print("\n=== repeatability inside the +10 cell ===")
print("  the two valid +10 arcs: yaw %+.2f and %+.2f deg/s (ratio %.1fx), kappa %+.4f and %+.4f"
      % (V[p[0]][1], V[p[1]][1], max(abs(V[p[0]][1]), abs(V[p[1]][1])) /
         min(abs(V[p[0]][1]), abs(V[p[1]][1])), V[p[0]][2], V[p[1]][2]))
print("  the three valid -10 arcs: yaw %s (range %.2f deg/s)"
      % ([V[t][1] for t in n], max(V[t][1] for t in n) - min(V[t][1] for t in n)))

print("\n=== other cells (n=1 each) ===")
for c in ("-12.5", "-15"):
    for t in cell(c):
        print("  %-7s %-22s v %.3f (%.2fx floor)  kappa %+.4f" % (c, t, V[t][0], V[t][0] / floor, V[t][2]))
print("  09-06 comparators: -15 kappa -0.4947 ; +10 +0.2320 ; -10 -0.2521")
