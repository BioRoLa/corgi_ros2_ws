"""Scratch: recover the CLOCK phase exactly from the logged kp.

gslip_pronk.cpp sets the gain regime and dbeta_ref under the SAME predicate:

    const bool ff_stance = leg_in_stance(i, row);
    cmd->dbeta_ref = ... (ff_stance ? clock_rate_stance_ : ...)
    if (ff_stance) { kx = k_radial_; kz = k_tangential_; ... }
    else           { kx = ky = kz = k_flight_; ... }

contact_gated_gains is false in this campaign, so ff_stance == row.in_stance,
the TEMPLATE label. The driver's kp therefore switches exactly at the clock's
stance/flight boundary, and the logged kp column is a direct readout of the
clock phase -- no reconstruction, no anchor assumption.

Read-only scratch. 2026-08-22 adversarial review.
"""
import csv
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_menger import debounce, DEBOUNCE  # noqa: E402

DIR_BETA = {"A": +1.0, "B": -1.0, "C": -1.0, "D": +1.0}
TAIL_S = 20.0


def load(path):
    per = {}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            m = r["motor"]
            if m not in ("L_Motor", "R_Motor"):
                continue
            try:
                d = per.setdefault((r["leg"], float(r["t"])), {})
                d["c"] = int(r["in_contact"])
                d["b"] = float(r["beta"])
                s = m[0]
                d["kp" + s] = float(r["kp"])
                d["ff" + s] = float(r["t_ff"])
                d["e" + s] = float(r["pos_error"])
            except (ValueError, KeyError):
                continue
    byleg = {}
    for (leg, t), d in per.items():
        if "kpL" not in d or "kpR" not in d:
            continue
        g = DIR_BETA[leg]
        byleg.setdefault(leg, []).append(
            (t, d["c"], d["b"], 0.5 * (d["kpL"] + d["kpR"]),
             g * 0.5 * (d["ffL"] + d["ffR"]),
             g * 0.5 * (d["eL"] + d["eR"])))
    out = {}
    for leg, rows in byleg.items():
        rows.sort()
        a = np.array(rows)
        if len(a) < 500:
            continue
        out[leg] = dict(t=a[:, 0], c=a[:, 1].astype(bool), b=a[:, 2],
                        kp=a[:, 3], ff=a[:, 4], e=a[:, 5])
    return out


def main():
    base = os.path.expanduser("~/corgi_runs/clock_ff")
    cells = ["off", "on", "both"]
    print("=== N. kp LEVELS (is it a clean two-state clock readout?) ===")
    for c in cells:
        p = os.path.join(base, c, "run1.csv")
        legs = load(p)
        a = legs["A"]
        m = a["t"] >= a["t"].max() - TAIL_S
        kp = a["kp"][m]
        u, cnt = np.unique(np.round(kp, 3), return_counts=True)
        top = sorted(zip(cnt, u), reverse=True)[:6]
        print("  %-5s legA distinct kp: %s" % (
            c, ", ".join("%.2f(%.0f%%)" % (v, 100.0 * n / len(kp))
                         for n, v in top)))

    print("\n=== O. CLOCK PHASE FROM kp, vs REAL CONTACT ===")
    print("%-6s %11s %12s %22s %24s %14s" % (
        "cell", "clock_duty", "contact_duty", "clockstance_airborne",
        "contact_in_clockflight", "phi_TD"))
    summary = {}
    for c in cells:
        rows = []
        for p in sorted(glob.glob(os.path.join(base, c, "run[0-9].csv"))):
            legs = load(p)
            for leg, a in legs.items():
                m = a["t"] >= a["t"].max() - TAIL_S
                a = {k: v[m] for k, v in a.items()}
                kp = a["kp"]
                lo, hi = kp.min(), kp.max()
                if hi - lo < 1.0:
                    continue
                thr = 0.5 * (lo + hi)
                # which level is clock STANCE? the one whose k is the
                # virtual spring, i.e. NOT k_flight. Decide by duty: the
                # v070 template is 40.8% stance, so stance is the minority
                # state; confirm by printing both.
                hi_mask = kp > thr
                cs = hi_mask if hi_mask.mean() < 0.5 else ~hi_mask
                con = debounce(a["c"], DEBOUNCE)
                dt = float(np.median(np.diff(a["t"])))
                # clock stance onsets
                on_idx = np.flatnonzero(np.diff(cs.astype(int)) > 0) + 1
                per = np.diff(on_idx) * dt
                per = per[(per > 0.15) & (per < 0.45)]
                T = float(np.median(per)) if len(per) else np.nan
                chg = np.diff(con.astype(int))
                rise = np.flatnonzero(chg > 0) + 1
                fall = np.flatnonzero(chg < 0) + 1
                phis, phils = [], []
                for r0 in rise:
                    nx = fall[fall > r0]
                    if not len(nx):
                        continue
                    f0 = nx[0]
                    if f0 - r0 < 3:
                        continue
                    prev = on_idx[on_idx <= r0]
                    if len(prev) and np.isfinite(T):
                        phis.append(((r0 - prev[-1]) * dt) / T)
                        phils.append(((f0 - prev[-1]) * dt) / T)
                rows.append(dict(
                    clock_duty=float(cs.mean()),
                    contact_duty=float(con.mean()),
                    air_cs=float((cs & ~con).sum() / max(1, cs.sum())),
                    con_cf=float((con & ~cs).sum() / max(1, con.sum())),
                    phi_TD=float(np.mean(phis)) if phis else np.nan,
                    phi_LO=float(np.mean(phils)) if phils else np.nan,
                    kp_stance=float(np.median(kp[cs])),
                    kp_flight=float(np.median(kp[~cs])),
                    ff_air_cs=float(np.median(a["ff"][cs & ~con]))
                    if (cs & ~con).sum() else np.nan,
                    ff_con_cf=float(np.median(a["ff"][con & ~cs]))
                    if (con & ~cs).sum() else np.nan,
                    e_air_cs=float(np.median(a["e"][cs & ~con]))
                    if (cs & ~con).sum() else np.nan,
                ))
        s = {k: float(np.nanmedian([r[k] for r in rows])) for k in rows[0]}
        summary[c] = s
        print("%-6s %11.3f %12.3f %22.3f %24.3f %14.3f" % (
            c, s["clock_duty"], s["contact_duty"], s["air_cs"], s["con_cf"],
            s["phi_TD"]))

    print("\n  kp levels used: %s" % ", ".join(
        "%s stance %.1f / flight %.1f" % (c, summary[c]["kp_stance"],
                                          summary[c]["kp_flight"])
        for c in cells))
    print("  (template v070 clock stance duty = 0.408)")

    print("\n=== P. FF WHERE IT LANDS, clock phase from kp (N.m per motor,"
          " beta axis) ===")
    print("%-6s %18s %20s" % ("cell", "ff airborne&clockstance",
                              "ff contact&clockFLIGHT"))
    for c in cells:
        print("%-6s %18.3f %20.3f" % (
            c, summary[c]["ff_air_cs"], summary[c]["ff_con_cf"]))
    print("  deltas vs off:")
    for c in ("on", "both"):
        print("    %-5s airborne&clockstance %+7.3f    contact&clockFLIGHT %+7.3f"
              % (c, summary[c]["ff_air_cs"] - summary["off"]["ff_air_cs"],
                 summary[c]["ff_con_cf"] - summary["off"]["ff_con_cf"]))

    print("\n=== Q. EDGES IN CLOCK PHASE (kp anchored) ===")
    print("%-6s %10s %10s %14s" % ("cell", "phi_TD", "phi_LO", "shift vs off"))
    for c in cells:
        d = ("" if c == "off" else "TD %+.3f LO %+.3f" % (
            summary[c]["phi_TD"] - summary["off"]["phi_TD"],
            summary[c]["phi_LO"] - summary["off"]["phi_LO"]))
        print("%-6s %10.3f %10.3f %14s" % (
            c, summary[c]["phi_TD"], summary[c]["phi_LO"], d))


if __name__ == "__main__":
    main()
