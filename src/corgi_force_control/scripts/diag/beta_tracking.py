"""Beta and theta tracking error per arm, and the pitch-moment proxy k_t * |d_beta|.

NO EXTRA INSTRUMENTATION NEEDED. The commanded beta is not logged, but it does
not have to be: the two motor errors already in the CSV decompose exactly.
With the shipped coupling

    phi_L = beta + (theta - theta_0)        err_L = d_beta + d_theta
    phi_R = beta - (theta - theta_0)        err_R = d_beta - d_theta

    =>  d_beta  = (err_L + err_R) / 2
        d_theta = (err_L - err_R) / 2

WHY THE PRODUCT IS THE REAL METRIC. The pitch moment that splits the legs
front/rear comes from the TANGENTIAL leg force, which is k_tangential times the
fore-aft deflection -- so the moment goes as k_t * d_beta, not as either alone.
Lowering k_t reduces the coefficient but degrades tracking, which raises
d_beta. The earlier analysis predicted a minimum for exactly this reason, and a
sweep that watched only d_beta, or only k_t, would miss it.

STANCE ONLY. The moment is generated while the foot is loaded. Flight tracking
error is a different question (it sets whether the leg arrives at the right
touchdown pose) and is reported separately rather than mixed in.

Sign note: dir_beta negates the commanded beta on legs B and C, so d_beta is
module-frame there and ROS-frame on A and D. Magnitudes are unaffected, and only
magnitudes are used.

Run:
    python3 beta_tracking.py <k_t> run1.csv [run2.csv ...]
"""
import csv
import sys

import numpy as np

LEGS = "ABCD"
UNWRAP_GUARD = np.pi   # _find_closest_phi can add 2*pi; drop those samples


def load(path):
    """Per (leg, timestamp): err_L, err_R, in_contact, t_ff."""
    rec = {}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            m = r["motor"]
            if m not in ("L_Motor", "R_Motor"):
                continue
            key = (r["leg"], float(r["t"]))
            d = rec.setdefault(key, {})
            d["L" if m == "L_Motor" else "R"] = float(r["pos_error"])
            d["c"] = int(r["in_contact"])
            d["tf"] = max(abs(float(r["t_ff"])), d.get("tf", 0.0))
    rows = []
    for (leg, t), d in rec.items():
        if "L" in d and "R" in d:
            rows.append((t, LEGS.index(leg), d["L"], d["R"], d["c"], d["tf"]))
    a = np.array(sorted(rows))
    return a


def main():
    if len(sys.argv) < 3:
        print("usage: beta_tracking.py <k_tangential> run1.csv [run2.csv ...]")
        return
    kt = float(sys.argv[1])
    paths = sys.argv[2:]

    print()
    print(f"{'run':>10} {'n stance':>9} {'|d_beta| deg':>13} {'|d_theta| deg':>14} "
          f"{'kt*|d_beta| N':>14}")

    agg = []
    for p in paths:
        a = load(p)
        if not len(a):
            print(f"{p.split('/')[-1]:>10}   no paired samples")
            continue
        nz = np.flatnonzero(a[:, 5] > 1e-9)
        if not nz.size:
            print(f"{p.split('/')[-1]:>10}   no gait window")
            continue
        a = a[a[:, 0] >= a[nz[0], 0]]

        eL, eR = a[:, 2], a[:, 3]
        good = (np.abs(eL) < UNWRAP_GUARD) & (np.abs(eR) < UNWRAP_GUARD)
        dbeta = 0.5 * (eL + eR)
        dtheta = 0.5 * (eL - eR)
        stance = (a[:, 4] > 0.5) & good
        if stance.sum() < 200:
            print(f"{p.split('/')[-1]:>10} {int(stance.sum()):9d}   too few stance samples")
            continue

        db = float(np.sqrt(np.mean(dbeta[stance] ** 2)))
        dt = float(np.sqrt(np.mean(dtheta[stance] ** 2)))
        # Tangential force proxy: k_t * (lever * d_beta). Reported as k_t*d_beta
        # in consistent units so arms are comparable; the lever is common.
        proxy = kt * db
        agg.append((db, dt, proxy))
        print(f"{p.split('/')[-1]:>10} {int(stance.sum()):9d} "
              f"{np.degrees(db):13.3f} {np.degrees(dt):14.3f} {proxy:14.1f}")

    if agg:
        m = np.array(agg)
        print(f"{'MEAN':>10} {'':>9} {np.degrees(m[:,0].mean()):13.3f} "
              f"{np.degrees(m[:,1].mean()):14.3f} {m[:,2].mean():14.1f}")


if __name__ == "__main__":
    main()
