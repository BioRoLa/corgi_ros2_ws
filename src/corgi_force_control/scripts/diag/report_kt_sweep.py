"""One table per k_tangential arm: desync, gain regime, beta tracking, torque.

Reads the captures sweep_k_tangential.sh leaves in <base>/kt<value>/run*.csv and
puts every metric in the chain side by side, because the arms have to be judged
on the PRODUCT, not on any single column:

    k_t          the coefficient being swept
    |d_beta|     the deflection, which RISES as k_t falls (softer tracking)
    k_t*|d_beta| the tangential force, hence the pitch moment -- the real metric
    front/rear   the desync the moment is supposed to be causing
    regime ratio P(spring engaged | foot down) / chance, the downstream fault
    peak tau     the number that actually matters

A minimum in k_t*|d_beta| that shows up in front/rear desync and again in the
regime ratio is the result worth having. Movement in only one column is not.
"""
import csv
import glob
import os
import re
import sys

import numpy as np

LEGS = "ABCD"
FRONT, REAR = (0, 1), (2, 3)
KP_THRESH = 100.0
UNWRAP_GUARD = np.pi


def per_run(path):
    rec, kprow = {}, {}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            m = r["motor"]
            if m not in ("L_Motor", "R_Motor"):
                continue
            t, leg = float(r["t"]), r["leg"]
            d = rec.setdefault((leg, t), {})
            d["L" if m == "L_Motor" else "R"] = float(r["pos_error"])
            d["c"] = int(r["in_contact"])
            d["tf"] = max(abs(float(r["t_ff"])), d.get("tf", 0.0))
            d["tau"] = max(abs(float(r["tau_demand"])), d.get("tau", 0.0))
            k = kprow.setdefault(t, {})
            k[leg] = (float(r["kp"]), int(r["in_contact"]))

    rows = [(t, LEGS.index(leg), d["L"], d["R"], d["c"], d["tf"], d["tau"])
            for (leg, t), d in rec.items() if "L" in d and "R" in d]
    if not rows:
        return None
    a = np.array(sorted(rows))
    nz = np.flatnonzero(a[:, 5] > 1e-9)
    if not nz.size:
        return None
    a = a[a[:, 0] >= a[nz[0], 0]]

    eL, eR = a[:, 2], a[:, 3]
    good = (np.abs(eL) < UNWRAP_GUARD) & (np.abs(eR) < UNWRAP_GUARD)
    dbeta = 0.5 * (eL + eR)
    stance = (a[:, 4] > 0.5) & good
    if stance.sum() < 200:
        return None
    db = float(np.sqrt(np.mean(dbeta[stance] ** 2)))
    peak_tau = float(np.percentile(a[:, 6], 99.5))

    ts = sorted(t for t, v in kprow.items() if len(v) == 4)
    ts = [t for t in ts if t >= a[0, 0]]
    if len(ts) < 300:
        return None
    C = np.array([[kprow[t][l][1] for l in LEGS] for t in ts], dtype=bool)
    KP = np.array([kprow[t]["A"][0] for t in ts])
    f = C[:, list(FRONT)].mean(axis=1) > 0.5
    rr = C[:, list(REAR)].mean(axis=1) > 0.5
    fr = float((f != rr).mean())
    sg = KP < KP_THRESH
    down = C.mean(axis=1) > 0.5
    ratio = (float(sg[down].mean() / sg.mean())
             if down.sum() > 50 and sg.mean() > 0 else float("nan"))
    return db, fr, ratio, peak_tau


def main():
    base = sys.argv[1] if len(sys.argv) > 1 else "/home/alexc/corgi_runs/kt_sweep"
    arms = sorted(glob.glob(os.path.join(base, "kt*")),
                  key=lambda p: float(re.sub(r"[^0-9.]", "", os.path.basename(p))))
    if not arms:
        print(f"no arms under {base}")
        return

    print()
    print("=" * 86)
    print("k_tangential SWEEP")
    print("=" * 86)
    print(f"{'k_t':>7} {'n':>3} {'|d_beta| deg':>13} {'kt*|d_beta|':>12} "
          f"{'front/rear':>11} {'regime ratio':>13} {'peak tau':>10}")

    table = []
    for arm in arms:
        kt = float(re.sub(r"[^0-9.]", "", os.path.basename(arm)))
        res = [per_run(p) for p in sorted(glob.glob(os.path.join(arm, "run*.csv")))]
        res = [r for r in res if r]
        if not res:
            print(f"{kt:7.0f}  --  no usable runs")
            continue
        m = np.array(res)
        db, fr, ratio, tau = m[:, 0].mean(), m[:, 1].mean(), m[:, 2].mean(), m[:, 3].mean()
        table.append((kt, db, kt * db, fr, ratio, tau))
        print(f"{kt:7.0f} {len(res):3d} {np.degrees(db):13.3f} {kt*db:12.1f} "
              f"{100*fr:10.1f}% {ratio:13.2f} {tau:10.1f}")

    if len(table) < 3:
        print("\n  need at least 3 arms to speak about a minimum")
        return

    t = np.array(table)
    print()
    print("=" * 86)
    ship = 600.0
    base_row = t[np.argmin(np.abs(t[:, 0] - ship))]
    print(f"  shipped k_t = {ship:.0f}: moment proxy {base_row[2]:.1f}, "
          f"front/rear {100*base_row[3]:.1f}%, ratio {base_row[4]:.2f}, "
          f"peak tau {base_row[5]:.1f}")

    i = int(np.argmin(t[:, 2]))
    print(f"  minimum moment proxy at k_t = {t[i,0]:.0f}: {t[i,2]:.1f} "
          f"({100*(t[i,2]-base_row[2])/base_row[2]:+.0f}% vs shipped)")
    j = int(np.argmin(t[:, 3]))
    k = int(np.argmax(t[:, 4]))
    print(f"  minimum front/rear desync at k_t = {t[j,0]:.0f}: {100*t[j,3]:.1f}%")
    print(f"  best regime ratio        at k_t = {t[k,0]:.0f}: {t[k,4]:.2f}")
    print()
    if i == j == k and t[i, 0] != ship:
        print(f"  COHERENT: moment proxy, desync and regime ratio all favour")
        print(f"  k_t = {t[i,0]:.0f}. That is the chain moving together, which is")
        print("  the result worth having. Repeat at n = 3 before adopting.")
    elif i == j or j == k:
        print("  PARTIALLY coherent -- two of three metrics agree. Suggestive;")
        print("  the run-to-run spread at n = 2 can produce this by chance.")
    else:
        print("  NOT COHERENT: the three metrics point at different arms. That")
        print("  is what noise looks like at n = 2, or evidence the moment does")
        print("  not drive the desync at this operating point. Do not adopt")
        print("  anything from this table.")
    print("=" * 86)
    print()


if __name__ == "__main__":
    main()
