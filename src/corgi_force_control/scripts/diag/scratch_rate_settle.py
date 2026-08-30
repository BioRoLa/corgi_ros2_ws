"""Scratch: settle rate-vs-duration with AGGREGATES, not ratios of medians.

The per-episode stance duration is quantised to the 10 ms capture grid, so a
median lands on 0.080/0.090/0.100 and a median-of-ratios is unusable. Total
swept beta divided by total contact time is immune to both.

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
            if r["motor"] != "L_Motor":
                continue
            try:
                per.setdefault(r["leg"], []).append(
                    (float(r["t"]), int(r["in_contact"]), float(r["beta"])))
            except (ValueError, KeyError):
                continue
    out = {}
    for leg, rows in per.items():
        rows.sort()
        a = np.array(rows)
        if len(a) < 500:
            continue
        out[leg] = (a[:, 0], a[:, 1].astype(bool), a[:, 2])
    return out


def main():
    base = os.path.expanduser("~/corgi_runs/clock_ff")
    cells = ["off", "on", "both"]
    print("AGGREGATE over all contact episodes, last 20 s, 3 runs x 4 legs\n")
    print("%-6s %9s %11s %11s %12s %12s %10s %9s" % (
        "cell", "n_ep", "sum_sweep", "sum_time", "rate=S/T", "mean_stance",
        "mean_sweep", "%bwd_ep"))
    res = {}
    for c in cells:
        S = T = 0.0
        n = 0
        nb = 0
        durs = []
        sws = []
        for p in sorted(glob.glob(os.path.join(base, c, "run[0-9].csv"))):
            for leg, (t, ic, b) in load(p).items():
                m = t >= t.max() - TAIL_S
                t, ic, b = t[m], ic[m], b[m]
                dt = float(np.median(np.diff(t)))
                d = debounce(ic, DEBOUNCE)
                chg = np.diff(d.astype(int))
                rise = np.flatnonzero(chg > 0) + 1
                fall = np.flatnonzero(chg < 0) + 1
                for r0 in rise:
                    nx = fall[fall > r0]
                    if not len(nx):
                        continue
                    f0 = nx[0]
                    if f0 - r0 < 3:
                        continue
                    sw = b[f0] - b[r0]
                    du = (f0 - r0) * dt
                    S += sw
                    T += du
                    durs.append(du)
                    sws.append(sw)
                    n += 1
                    if sw < 0:
                        nb += 1
        res[c] = dict(S=S, T=T, rate=S / T, n=n,
                      mdur=float(np.mean(durs)), msw=float(np.mean(sws)),
                      bwd=100.0 * nb / n)
        r = res[c]
        print("%-6s %9d %11.3f %11.3f %12.4f %12.4f %10.4f %8.1f%%" % (
            c, r["n"], r["S"], r["T"], r["rate"], r["mdur"], r["msw"],
            r["bwd"]))

    print("\nattribution of the change in sweep-per-episode vs off:")
    print("  sweep_per_ep = rate x mean_stance")
    for c in ("on", "both"):
        a, o = res[c], res["off"]
        tot = np.log(a["msw"] / o["msw"]) if a["msw"] > 0 and o["msw"] > 0 else np.nan
        dr = np.log(a["rate"] / o["rate"]) if a["rate"] > 0 else np.nan
        dd = np.log(a["mdur"] / o["mdur"])
        print("    %-5s sweep/ep %+.1f%%   =  rate %+.1f%%  +  duration %+.1f%%"
              % (c, 100 * tot, 100 * dr, 100 * dd))
        print("           (rate %.4f -> %.4f rad/s, stance %.4f -> %.4f s)"
              % (o["rate"], a["rate"], o["mdur"], a["mdur"]))

    print("\nkinematic check -- a planted foot means v_body ~ rate * lever.")
    print("  lever L from the campaign's own torque prediction:")
    print("    6.37 = b_tangential(30) * 3.0098 * L^2  ->  L = %.4f m" %
          np.sqrt(6.37 / (30.0 * 3.0098)))
    L = np.sqrt(6.37 / (30.0 * 3.0098))
    vf = {"off": 0.242, "on": 0.213, "both": 0.065}
    print("  %-6s %12s %12s %10s" % ("cell", "rate*L", "v_fwd(odom)", "slip"))
    for c in cells:
        pred = res[c]["rate"] * L
        print("  %-6s %12.4f %12.4f %9.1f%%" % (
            c, pred, vf[c], 100.0 * (1.0 - vf[c] / pred) if pred > 0 else 0.0))


if __name__ == "__main__":
    main()
