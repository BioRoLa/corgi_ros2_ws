"""Is the commanded gain regime out of phase with actual ground contact?

force_control switches gains on the TEMPLATE's row.in_stance; the driver reports
the SIMULATOR's measured contact. The regime is readable from kp, which is
sharply bimodal because the two branches command very different matrices:

    stance (leg_frame)  K = diag(k_radial, k_lateral, k_tangential) in leg frame
                        -> kp ~ 36 N.m/rad
    flight              K = k_flight * I  (isotropic, 12000)
                        -> kp ~ 289 N.m/rad

so a threshold between the modes classifies each sample's regime exactly, with
no model and no Jacobian.

The number that matters is P(stance gains | foot on the ground). Compare it to
the run's own overall stance-gain fraction, which is what independence would
give:

    P == baseline    schedule uncorrelated with contact
    P >> baseline    schedule tracks contact (what the design intends)
    P << baseline    ANTI-correlated: the leg is held by the stiff flight gain
                     exactly when it is loaded, so the virtual spring does not
                     exist for most of stance

Takes one or more CSVs and reports each plus the spread, because a single run
in this project has repeatedly been inside the noise.
"""
import csv
import sys

import numpy as np

THRESH = 100.0        # between the 36 and 289 modes
LEG_MOTORS = ("L_Motor", "R_Motor")


def load(path):
    t, c, kp, tf = [], [], [], []
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            if r["motor"] not in LEG_MOTORS:
                continue
            t.append(float(r["t"])); c.append(int(r["in_contact"]))
            kp.append(float(r["kp"])); tf.append(float(r["t_ff"]))
    return (np.array(t), np.array(c), np.array(kp), np.array(tf))


def main():
    paths = sys.argv[1:]
    if not paths:
        print("usage: gain_regime_crosstab.py run1.csv [run2.csv ...]")
        return

    print()
    print("=" * 78)
    print("GAIN REGIME vs ACTUAL CONTACT")
    print("=" * 78)
    print(f"{'run':>28} {'n':>9} {'stance-g %':>11} {'P(st|down)':>11} "
          f"{'baseline':>9} {'ratio':>7}")

    rows = []
    for p in paths:
        t, c, kp, tf = load(p)
        nz = np.flatnonzero(np.abs(tf) > 1e-9)
        if not nz.size:
            print(f"{p.split('/')[-1]:>28}   no gait window (t_ff all zero)")
            continue
        sel = t >= t[nz[0]]
        c, kp = c[sel], kp[sel]
        if len(kp) < 5000:
            print(f"{p.split('/')[-1]:>28} {len(kp):9d}   too few samples")
            continue
        stance_g = kp < THRESH
        baseline = stance_g.mean()
        down = c > 0.5
        if down.sum() < 1000:
            print(f"{p.split('/')[-1]:>28} {len(kp):9d}   too little contact")
            continue
        p_st_down = stance_g[down].mean()
        ratio = p_st_down / baseline if baseline else float("nan")
        rows.append((p_st_down, baseline, ratio))
        print(f"{p.split('/')[-1]:>28} {len(kp):9d} {100*baseline:10.1f}% "
              f"{100*p_st_down:10.1f}% {100*baseline:8.1f}% {ratio:7.2f}")

    if len(rows) < 2:
        print("\n  need at least 2 runs to speak about repeatability")
        return

    a = np.array(rows)
    print()
    print("=" * 78)
    print(f"  P(stance gains | foot down): mean {100*a[:,0].mean():.1f}%  "
          f"range {100*a[:,0].min():.1f}-{100*a[:,0].max():.1f}%  "
          f"(n = {len(a)} runs)")
    print(f"  chance baseline            : mean {100*a[:,1].mean():.1f}%")
    print(f"  ratio to baseline          : mean {a[:,2].mean():.2f}  "
          f"range {a[:,2].min():.2f}-{a[:,2].max():.2f}")
    print()
    if a[:, 2].max() < 0.9:
        print("  CONFIRMED, every run: the gain schedule is ANTI-correlated with")
        print("  contact. The leg runs the stiff flight gain while loaded, so the")
        print("  G-SLIP virtual spring is absent for most of real stance. That is")
        print("  a control-phase fault, not a stiffness-tuning problem, and it is")
        print("  a direct mechanism for the torque erosion.")
    elif a[:, 2].min() > 1.1:
        print("  The schedule TRACKS contact, as designed. The single-run")
        print("  anti-correlation was a phase accident -- retract it.")
    else:
        print("  MIXED across runs -- the effect is inside the run-to-run spread")
        print("  and nothing should be concluded from it yet.")
    print("=" * 78)
    print()


if __name__ == "__main__":
    main()
