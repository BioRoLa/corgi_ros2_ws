#!/usr/bin/env python3
"""P-HW-rho scored at the desk (2026-09-11), from data already on disk.

Registered prediction (log 274 s6, before any hardware session): a held
open-loop lean lambda at the matrix conditions produces steady body roll
rho ~ -(0.2..0.45)*(dir*lambda) -- the sim partition mechanism, INVERTED
sign -- against the model statics' +0.016*lambda. Runbook R4 acceptance:
roll sign NEGATIVE, |rho| in [0.15, 0.65] deg/deg.

Sign anchor: imu_rock_desk.json P-SIGN-2 PASS (2026-09-08 bench bag
s3_imu_rock_desk): left-side-down tilt reads NEGATIVE IMU roll
(d_roll -21.24 deg, roll-vs-wx slope +0.994). The estimator frame is
therefore trusted for sign.

Estimator (declared before computing): per arc, rho_steady = median of
per-stride mean IMU roll (figs0906_bag_strides.json, stride clock as log
325.39) minus the same-session lambda0 baseline (mean of the two
k_roll-0.60 lambda0 arcs' medians); ratio = rho_steady / lambda_signed.
Windows 1-6 median reported beside the all-strides median as a
sensitivity check, all-strides primary. Level-comparable set mirrors
load_split.py: k_roll 0.60, launch-valid (s2_ol10_a1 k_roll 0.25 and
s2_ol10_a2 VOID shown, excluded from the verdict).
"""
import json, os, statistics
HERE = os.path.dirname(os.path.abspath(__file__))
D = json.load(open(os.path.join(HERE, "figs0906_bag_strides.json")))
CELL = {"s2_l0r_a1": 0, "s2_l0r_a2": 0, "s2_ol10_roll1": 10,
        "s2_ol10_a2": 10, "s2_ol10_a1": 10, "s2_ol15_a1": 15,
        "s2_ol10n_a2": -10, "s2_ol15n_a1": -15}
EXCL = {"s2_ol10_a2": "VOID (launch rule 325.43)",
        "s2_ol10_a1": "k_roll 0.25 (not level-comparable)"}
med = lambda v: statistics.median(v)
base = statistics.mean([med(D[r]["roll"]) for r in ("s2_l0r_a1", "s2_l0r_a2")])
out = {"baseline_lambda0_roll_deg": base, "arcs": {}, "verdict": None}
print("P-HW-rho score -- baseline lambda0 roll %.2f deg (l0r_a1 %.2f, l0r_a2 %.2f)"
      % (base, med(D["s2_l0r_a1"]["roll"]), med(D["s2_l0r_a2"]["roll"])))
print("%-16s %5s  %9s %8s %8s  %8s   %s" % ("arc", "lam", "roll_med", "w1-6", "d_roll", "ratio", "flag"))
ok, checked = True, 0
for r, lam in CELL.items():
    if lam == 0 or r not in D:
        continue
    roll = D[r]["roll"]
    m_all, m_w6 = med(roll), med(roll[:6])
    d = m_all - base
    ratio = d / lam
    flag = EXCL.get(r, "")
    print("%-16s %+5d  %8.2f %8.2f %+8.2f  %+8.3f   %s" % (r, lam, m_all, m_w6, d, ratio, flag))
    out["arcs"][r] = dict(lam=lam, roll_med=m_all, roll_med_w1_6=m_w6,
                          d_roll=d, ratio=ratio, excluded=bool(flag), reason=flag)
    if not flag:
        checked += 1
        if not (ratio < 0 and 0.15 <= abs(ratio) <= 0.65):
            ok = False
band = all(0.2 <= abs(a["ratio"]) <= 0.45 for a in out["arcs"].values() if not a["excluded"])
out["verdict"] = dict(n_scored=checked, sign_negative_and_in_runbook_band=ok,
                      inside_registered_sim_band_0p2_0p45=band)
print("\nVERDICT: %d level-comparable arcs; sign NEGATIVE + |ratio| in [0.15,0.65]: %s; "
      "all inside the registered sim-partition band [0.20,0.45]: %s"
      % (checked, "PASS" if ok else "FAIL", "yes" if band else "no"))
print("statics hypothesis (+0.016*lambda) predicts +0.16..0.24 deg at 10-15 deg: %s"
      % ("rejected by sign" if ok else "see table"))
json.dump(out, open(os.path.join(HERE, "phw_rho_score.json"), "w"), indent=1)
print("wrote phw_rho_score.json")
