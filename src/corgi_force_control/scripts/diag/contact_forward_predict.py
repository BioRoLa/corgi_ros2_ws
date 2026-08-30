"""Forward-predict body-frame `by` from the SHIPPED contact_map_3d, using the
ACHIEVED gamma per leg. Log S190.

No hand-derived geometry: this is leg_model.cpp:291-323 as ported (and
validated against supervisor ground truth) in stage15_zleg_camber_analysis.py

    d_wheel(g) = D_PLANE + (0 if |sin g|<1e-4 else -W_HALF if sin g>0 else +W_HALF)
    y_c        = d_wheel*cos(g) + r_eff*sin(g)          # from the ABAD axis
    by         = SY[leg] * (Y_ABAD + y_c)               # body frame

Stage 1's discipline is ACHIEVED lambda only -- commanded mispredicts by ~3 deg
at kp 90 as a variable left/right split. This uses motor_deg, not the command.
"""
import csv
import glob
import math
import os
import re
import numpy as np

D_PLANE, W_HALF, Y_ABAD, R_EFF = 0.091675, 0.02, 0.12, 0.145
SY = {"A": +1.0, "B": -1.0, "C": -1.0, "D": +1.0}
LEG_IDX = {"A": 0, "B": 1, "C": 2, "D": 3}
TAIL = 15.0

CONT = "/home/alexc/corgi_runs/contact_lateral"
DUMP = "/home/alexc/camber_dumps"


def d_wheel(g):
    s = math.sin(g)
    if abs(s) < 1e-4:
        return D_PLANE
    return D_PLANE - W_HALF if s > 0 else D_PLANE + W_HALF


def by_pred(g, leg, r_eff=R_EFF):
    y_c = d_wheel(g) * math.cos(g) + r_eff * math.sin(g)
    return SY[leg] * (Y_ABAD + y_c)


def lw_pred(g, leg, coeff):
    """LegWheel's forms: NO edge offset."""
    y_c = D_PLANE * math.cos(g) + coeff * math.sin(g)
    return SY[leg] * (Y_ABAD + y_c)


def measured_by(path):
    rows = {m: [] for m in SY}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            if r.get("module") in rows:
                try:
                    rows[r["module"]].append((float(r["t"]), float(r["by"])))
                except (ValueError, TypeError):
                    pass
    out = {}
    tm = max((v[-1][0] for v in rows.values() if v), default=None)
    for m, v in rows.items():
        if not v:
            continue
        a = np.array(v)
        a = a[a[:, 0] >= tm - TAIL]
        if len(a):
            out[m] = float(np.median(a[:, 1]))
    return out


def achieved_gamma(lam):
    p = f"{DUMP}/camber_clat_lam{int(lam) if lam == int(lam) else lam}_kp90.npz"
    d = np.load(p, allow_pickle=True)
    t, md = d["motor_t"], d["motor_deg"]
    m = t >= t.max() - TAIL
    return {leg: math.radians(float(np.median(md[m][:, i, 2])))
            for leg, i in LEG_IDX.items()}


lams = sorted(float(re.search(r"lam([0-9.]+)", os.path.basename(p)).group(1))
              for p in glob.glob(f"{CONT}/contact_lam*.csv"))
meas = {l: measured_by(f"{CONT}/contact_lam{int(l)}.csv") for l in lams}
gam = {l: achieved_gamma(l) for l in lams}

print("ACHIEVED gamma per leg (deg) -- commanded is the lambda column")
print(f"{'lam':>5} {'A':>8} {'B':>8} {'C':>8} {'D':>8}   A/D-|B/C| split")
for l in lams:
    g = {k: math.degrees(v) for k, v in gam[l].items()}
    split = (abs(g["A"]) + abs(g["D"])) / 2 - (abs(g["B"]) + abs(g["C"])) / 2
    print(f"{l:5.0f} " + " ".join(f"{g[k]:8.3f}" for k in "ABCD")
          + f"   {split:+6.2f} deg")

print("\nForward prediction from the SHIPPED contact_map_3d, achieved gamma.")
print("delta by from lambda=0, body frame, mm.  (meas | cpp | r_sin | rc_sin)")
hdr = f"{'lam':>5} " + " ".join(f"{('leg '+k):>26}" for k in "AB")
print(f"{'lam':>5} {'leg':>4} {'meas':>8} {'cpp':>8} {'r_sin':>8} {'rc_sin':>8} {'cpp err':>9}")
tot = {"cpp": 0.0, "r_sin": 0.0, "rc_sin": 0.0}
n = 0
for l in lams:
    if l == 0:
        continue
    for leg in "ABCD":
        if leg not in meas[l] or leg not in meas[0.0]:
            continue
        dm = 1000 * (meas[l][leg] - meas[0.0][leg])
        g0, g1 = gam[0.0][leg], gam[l][leg]
        dc = 1000 * (by_pred(g1, leg) - by_pred(g0, leg))
        dr = 1000 * (lw_pred(g1, leg, R_EFF) - lw_pred(g0, leg, R_EFF))
        dq = 1000 * (lw_pred(g1, leg, 0.015) - lw_pred(g0, leg, 0.015))
        tot["cpp"] += abs(dc - dm); tot["r_sin"] += abs(dr - dm)
        tot["rc_sin"] += abs(dq - dm); n += 1
        print(f"{l:5.0f} {leg:>4} {dm:8.2f} {dc:8.2f} {dr:8.2f} {dq:8.2f} "
              f"{dc-dm:+9.2f}")

print(f"\nmean |residual| over {n} leg-points:")
for k, v in sorted(tot.items(), key=lambda kv: kv[1]):
    print(f"  {k:8} {v/n:7.2f} mm")
