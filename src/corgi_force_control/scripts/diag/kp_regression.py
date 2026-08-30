"""Separate the k_radial effect on kp from the pose effect, by regression.

The matched-pose slice is honest but wasteful: the robot will not sit still on
the support box (theta swings up to 43 deg inside one arm), so a narrow common
band keeps only 2 of 4 arms and a few hundred samples.

kp = J(pose)^T K J(pose) is smooth in pose and LINEAR in each stiffness, so the
whole dataset can be used at once:

    kp ~ c0 + c_k*k_radial + (pose terms)

with pose entered flexibly (theta, theta^2, beta, beta^2, gamma). The
coefficient on k_radial is then the effect at held pose, which is exactly the
quantity the sweep was trying to isolate.

  c_k ~ u_r^2 ~ 0.0027    the leg_frame branch applies the commanded stiffness
  c_k ~ 0                 it does not, and since t_stiff = kp * error dominates
                          the torque, a dead radial gain sits directly under the
                          measured 13-14x erosion

CAVEAT, STATED UP FRONT. k_radial is constant within each arm, so it is
partially confounded with whatever pose distribution that arm happened to
explore. The pose terms absorb the bulk of that, but this is an observational
fit, not a designed experiment: it is evidence, not proof. The clean version is
a rig that holds the pose fixed while the gain varies.
"""
import csv
import sys

import numpy as np

CSV = "/tmp/corgi_torque_terms.csv"
BRACKETS = "/tmp/k_radial_sweep_brackets.txt"
LEG_MOTORS = ("L_Motor", "R_Motor")
U_R_EXPECTED = 0.10324 / 2.0


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
        rows.append((float(r["t"]), float(r["kp"]), float(r["theta"]),
                     float(r["beta"]), float(r["gamma"])))
    a = np.array(rows)

    K, KP, TH, BE, GA = [], [], [], [], []
    for k, t0, t1 in brackets:
        lo = t0 + 0.4 * (t1 - t0)
        s = a[(a[:, 0] >= lo) & (a[:, 0] <= t1)]
        if not len(s):
            continue
        K.append(np.full(len(s), k))
        KP.append(s[:, 1]); TH.append(s[:, 2]); BE.append(s[:, 3])
        GA.append(s[:, 4])
    K = np.concatenate(K); KP = np.concatenate(KP)
    TH = np.concatenate(TH); BE = np.concatenate(BE); GA = np.concatenate(GA)

    print()
    print("=" * 74)
    print("kp ~ k_radial + POSE   (n = %d)" % len(KP))
    print("=" * 74)

    X = np.column_stack([np.ones_like(K), K, TH, TH**2, BE, BE**2, GA])
    names = ["const", "k_radial", "theta", "theta^2", "beta", "beta^2", "gamma"]
    coef, *_ = np.linalg.lstsq(X, KP, rcond=None)
    resid = KP - X @ coef
    dof = len(KP) - X.shape[1]
    s2 = float(resid @ resid) / dof
    cov = s2 * np.linalg.pinv(X.T @ X)
    se = np.sqrt(np.diag(cov))
    r2 = 1 - float(resid @ resid) / float(((KP - KP.mean()) ** 2).sum())

    print(f"{'term':>10} {'coef':>14} {'se':>12} {'t':>8}")
    for n_, c_, s_ in zip(names, coef, se):
        print(f"{n_:>10} {c_:14.6f} {s_:12.6f} {c_/s_ if s_ else 0:8.1f}")
    print(f"\n  model R^2 = {r2:.3f}")

    ck, sk = coef[1], se[1]
    print()
    print("=" * 74)
    print(f"  k_radial coefficient: {ck:.6f} +- {sk:.6f} N.m/rad per N/m")
    print(f"  expected if the mapping works (u_r^2): {U_R_EXPECTED**2:.6f}")
    print(f"  ratio to expected: {ck/U_R_EXPECTED**2:.2f}x")
    print()
    lo_ci, hi_ci = ck - 2 * sk, ck + 2 * sk
    print(f"  95% CI: [{lo_ci:.6f}, {hi_ci:.6f}]")

    # A NEGATIVE coefficient is physically impossible and therefore proves the
    # model is misspecified rather than proving anything about the robot.
    # kp = k_r*u_r^2 + k_lat*u_y^2 + k_t*u_t^2 with every u^2 >= 0, so
    # d(kp)/d(k_radial) >= 0 always. The first run of this script returned
    # -0.0034 +- 0.0002 (t = -17.2) and then declared the radial gain dead.
    #
    # The cause is structural and not fixable by adding pose terms: k_radial is
    # CONSTANT WITHIN EACH ARM, so it is perfectly collinear with arm identity,
    # and every between-arm difference the pose basis fails to absorb lands on
    # this coefficient. An observational fit cannot separate them.
    if ck < 0:
        print()
        print("  IMPOSSIBLE SIGN -- the model is misspecified, not the robot.")
        print("  kp is a sum of stiffnesses times squared lever arms, so its")
        print("  derivative w.r.t. k_radial cannot be negative. This fit is")
        print("  therefore uninterpretable, whatever its t-statistic.")
        print()
        print("  Root cause: k_radial is constant within each arm, so it is")
        print("  collinear with arm identity and absorbs any between-arm pose")
        print("  difference the pose terms miss. No amount of extra pose basis")
        print("  fixes a design where the treatment never varies within a unit.")
        print()
        print("  To answer this properly, either:")
        print("    - vary k_radial WITHIN a single hold, at one pose, or")
        print("    - compute the expected kp offline from the logged theta/")
        print("      beta/gamma and the known gains, and compare it to the kp")
        print("      force_control published, sample by sample.")
        print("  The second needs no robot at all and has no confound.")
        print("=" * 74)
        print()
        return
    if hi_ci < 0.5 * U_R_EXPECTED**2:
        print()
        print("  The commanded radial stiffness is NOT reaching kp at anything")
        print("  like the designed rate. Since t_stiff = kp * tracking error is")
        print("  the dominant torque term, a radial gain that does not respond")
        print("  to its own parameter sits directly under the erosion.")
    elif lo_ci > 0.5 * U_R_EXPECTED**2:
        print()
        print("  kp tracks k_radial at close to the designed rate; the")
        print("  leg_frame mapping is doing its job and the kp_stance/kp_flight")
        print("  anomaly must come from somewhere else.")
    else:
        print()
        print("  CI spans the decision boundary -- underpowered. More arms or a")
        print("  genuinely fixed pose are needed before calling this.")
    print("=" * 74)
    print()


if __name__ == "__main__":
    main()
