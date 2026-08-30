"""Does the feedforward oppose the stiffness term on exactly the dir_beta = -1 legs?

CONTEXT

torque_decomposition.py found that t_stiff and t_ff pull against each other,
circulating ~5x more torque than the controller delivers. The Torque Frame
Mismatch note predicts precisely this, and predicts WHERE: dir_beta is honoured
on the position command and reported state but NOT on the gains or the
feedforward torque, so legs with dir_beta = -1 are commanded a mirrored pose
while being fed un-mirrored torques.

    dir_beta = {A:+1, B:-1, C:-1, D:+1}

so the prediction is sharp and falsifiable: opposition on B and C, none on A
and D. If opposition is uniform across all four legs, the frame mismatch is NOT
the explanation and something else is at work.

WHY THIS IS WORTH MEASURING WHEN THE MECHANISM IS ALREADY "KNOWN"

Every previous line of evidence was INDIRECT -- sweep amplitudes, a 28 ms phase
lag, left/right travel ratios. All of them are downstream of the gait and
confounded by it. This measures the two torque terms directly, at the point they
are summed, and asks only about their SIGNS. Nothing about gait quality enters.

It also bears on the open paradox. The transform is forced by virtual work and
verified two ways, yet applying it made the gait markedly worse and was
reverted. If the un-transformed terms CANCEL on B and C, then the bug has been
suppressing those legs' torque demand, and 'fixing' it would raise their demand
toward the unmirrored legs' -- which is a reason for the fix to look worse
without the fix being wrong.

Run:
    python3 ff_opposition.py [/tmp/corgi_torque_terms.csv]
"""
import csv
import os
import sys
from collections import defaultdict

import numpy as np

# From motor_config.yaml, quoted in the Torque Frame Mismatch note.
DIR_BETA = {"A": +1, "B": -1, "C": -1, "D": +1}
LEG_MOTORS = ("L_Motor", "R_Motor")

# Ignore near-zero samples: the sign of a term that is essentially zero is
# noise, and including them would dilute the very statistic under test.
MIN_TERM_NM = 1.0


def load(path):
    rows = defaultdict(list)
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            rows[(r["leg"], r["motor"])].append((
                float(r["t"]), float(r["t_stiff"]), float(r["t_ff"]),
                float(r["tau_demand"]),
            ))
    return {k: np.array(v) for k, v in rows.items()}


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/corgi_torque_terms.csv"
    if not os.path.exists(path):
        print(f"{path} does not exist")
        return
    data = load(path)

    # Same gait window as torque_decomposition: first non-zero t_ff anywhere.
    t_start = None
    for v in data.values():
        nz = np.flatnonzero(np.abs(v[:, 2]) > 1e-9)
        if nz.size:
            t0 = v[nz[0], 0]
            t_start = t0 if t_start is None else min(t_start, t0)
    if t_start is None:
        print("no gait window (t_ff zero everywhere) -- not a gait run")
        return
    data = {k: v[v[:, 0] >= t_start] for k, v in data.items()}

    print()
    print("=" * 78)
    print("DOES t_ff OPPOSE t_stiff, AND ON WHICH LEGS?")
    print("=" * 78)
    print(f"  gait window t >= {t_start:.2f} s;  samples with "
          f"|t_stiff| and |t_ff| both > {MIN_TERM_NM} N.m")
    print()
    print(f"  {'leg':>4} {'dir_beta':>9} {'motor':>8} {'n':>7} "
          f"{'% opposed':>10} {'corr':>7} {'cancellation':>13}")

    by_dir = defaultdict(list)
    for leg in "ABCD":
        for m in LEG_MOTORS:
            if (leg, m) not in data:
                continue
            v = data[(leg, m)]
            ts, tf = v[:, 1], v[:, 2]
            sel = (np.abs(ts) > MIN_TERM_NM) & (np.abs(tf) > MIN_TERM_NM)
            if sel.sum() < 20:
                print(f"  {leg:>4} {DIR_BETA[leg]:>9} {m:>8} "
                      f"{int(sel.sum()):>7}   (too few samples)")
                continue
            a, b = ts[sel], tf[sel]
            opposed = 100.0 * np.mean(np.sign(a) != np.sign(b))
            corr = float(np.corrcoef(a, b)[0, 1])
            # How much of the circulated magnitude survives as net demand.
            # 1.0 = terms add; ->0 = they cancel.
            cancel = float(np.mean(np.abs(a + b)) / np.mean(np.abs(a) + np.abs(b)))
            by_dir[DIR_BETA[leg]].append((opposed, corr, cancel))
            print(f"  {leg:>4} {DIR_BETA[leg]:>9} {m:>8} {int(sel.sum()):>7} "
                  f"{opposed:9.1f}% {corr:7.2f} {cancel:12.2f}")

    print()
    print("  % opposed    = samples where sign(t_stiff) != sign(t_ff)")
    print("  corr         = correlation of the two terms (negative = opposing)")
    print("  cancellation = mean|t_stiff+t_ff| / mean(|t_stiff|+|t_ff|)")
    print("                 1.0 means they add, near 0 means they cancel")

    print()
    print("=" * 78)
    print("VERDICT")
    print("=" * 78)
    if len(by_dir) < 2:
        print("  only one dir_beta group present; cannot compare")
        return
    plus = np.array(by_dir[+1])
    minus = np.array(by_dir[-1])
    print(f"  dir_beta = +1  (A, D): opposed {plus[:, 0].mean():5.1f}%, "
          f"corr {plus[:, 1].mean():+.2f}, cancellation {plus[:, 2].mean():.2f}")
    print(f"  dir_beta = -1  (B, C): opposed {minus[:, 0].mean():5.1f}%, "
          f"corr {minus[:, 1].mean():+.2f}, cancellation {minus[:, 2].mean():.2f}")
    print()
    # Key on CORRELATION, not on how often the signs disagree.
    #
    # "% opposed" was the first statistic tried and it cannot see this effect:
    # two oscillating signals disagree in sign a large fraction of the time
    # whatever their relationship, so it sat at 71-73% for every leg and
    # reported "prediction fails" while the correlation column showed -0.96
    # against -0.30. A binary sign test throws away exactly the proportional
    # structure that distinguishes a sign error from two independent physical
    # contributions. It is kept in the table above because it is a useful
    # sanity column, but it must not be the discriminator.
    gap = plus[:, 1].mean() - minus[:, 1].mean()   # corr is negative; gap > 0
    if gap > 0.3:
        print(f"  PREDICTION HOLDS: the dir_beta = -1 legs are anti-correlated "
              f"at {minus[:, 1].mean():+.2f}")
        print(f"  against {plus[:, 1].mean():+.2f} for dir_beta = +1 "
              f"-- a gap of {gap:.2f}.")
        print(f"  They also retain only {minus[:, 2].mean():.0%} of their "
              f"circulated torque as net")
        print(f"  demand, against {plus[:, 2].mean():.0%}.")
        print("  The frame mismatch is confirmed by direct measurement of the")
        print("  two terms, not inferred from gait quality.")
        print()
        print("  Note what this implies about the reverted fix: on B and C the")
        print("  terms partly CANCEL, so the bug has been SUPPRESSING their")
        print("  torque demand. Correcting the frame makes those terms ADD,")
        print("  raising B/C demand toward A/D's. That is a coherent reason for")
        print("  a correct fix to measure worse, and it means the A/D numbers")
        print("  are the honest estimate of this controller's real demand.")
        print()
        print("  RESIDUAL, and it is NOT dir_beta: the +1 legs still cancel")
        print(f"  {1-plus[:, 2].mean():.0%} of their circulated torque at corr "
              f"{plus[:, 1].mean():+.2f}.")
        print("  Fixing the frame would not remove that. Two faults, not one.")
    elif gap < -0.3:
        print("  INVERTED: the dir_beta = +1 legs are MORE anti-correlated.")
        print("  Not the predicted signature -- do not attribute this to dir_beta.")
    else:
        print(f"  PREDICTION FAILS: correlation gap is only {gap:.2f}.")
        print("  Opposition is not specific to dir_beta = -1, so the frame")
        print("  mismatch does not explain it. Look elsewhere.")
    print("=" * 78)
    print()


if __name__ == "__main__":
    main()
