#!/usr/bin/env python3
"""Where the lean's torque lands: per-leg ABAD load split from the banked per-stride peaks (log 325.46 §4; OI #53).

Reads figs0906_bag_strides.json (tauH = per-stride PEAK |torque_h| per leg on the
/motor/state scale, stride clock as log 325.39) and, per run, takes the median
over complete windows 1-6 per leg, subtracts the same statistic of the
same-config lambda-0 pair (s2_l0r_a1, s2_l0r_a2), and reports the within-pair
split: D vs A under +gamma (left pair), B vs C under -gamma (right pair).

Caveats it prints: D is CENSORED at the ~41 N.m ceiling (windows >= 39 N.m are
counted), s2_ol10_a1 ran k_roll 0.25 (not level-comparable) and s2_ol10_a2 is
VOID under the launch rule of 325.43 -- both are shown but excluded from the
summary. C's delta under -gamma is compared with C's own lambda-0 spread.

Usage: load_split.py [--json figs0906_bag_strides.json] [--out load_split.json]
Self-test: the lambda-0 mean must read 18.05 / 17.91 / 29.26 / 21.18 and
D's +gamma delta 17.0-18.3 (the values verified on 2026-09-08).
"""
import os, json, argparse
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
LEGS = ["A", "B", "C", "D"]
NM = {"A": "A FL", "B": "B FR", "C": "C RR", "D": "D RL"}
CELL = {"s2_l0r_a1": ("l0", 0, 0.60, ""), "s2_l0r_a2": ("l0", 0, 0.60, ""),
        "s2_ol10_roll1": ("+10", 10, 0.60, ""), "s2_ol10_a2": ("+10", 10, 0.60, "VOID (launch rule 325.43)"),
        "s2_ol10_a1": ("+10", 10, 0.25, "k_roll 0.25 (pre-freeze, not level-comparable)"),
        "s2_ol15_a1": ("+15", 15, 0.60, ""), "s2_ol10n_a2": ("-10", -10, 0.60, ""), "s2_ol15n_a1": ("-15", -15, 0.60, "")}
NWIN, SAT = 6, 39.0


def leg_key(run, leg):
    tauH = run["tauH"]
    for k in (leg, leg.lower()):
        if k in tauH:
            return k
    raise KeyError(leg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", default=os.path.join(HERE, "figs0906_bag_strides.json"))
    ap.add_argument("--out", default=os.path.join(HERE, "load_split.json"))
    a = ap.parse_args()
    data = json.load(open(a.json))
    med, sat, out = {}, {}, {"runs": {}, "baseline": {}, "delta": {}, "summary": {}}
    for run, d in data.items():
        med[run] = {}; sat[run] = {}
        for L in LEGS:
            v = np.abs(np.array(d["tauH"][leg_key(d, L)][:NWIN], dtype=float))
            med[run][L] = float(np.median(v)); sat[run][L] = int(np.sum(v >= SAT))
        out["runs"][run] = dict(cell=CELL.get(run, ("?", 0, None, ""))[0], k_roll=CELL.get(run, ("?", 0, None, ""))[2],
                                flag=CELL.get(run, ("?", 0, None, ""))[3], median_peak_tauH_w1_6=med[run],
                                windows_ge_39=sat[run], n_sat=d.get("n_sat"), n_sat_leg=d.get("n_sat_leg"))
    base = {L: float(np.mean([med["s2_l0r_a1"][L], med["s2_l0r_a2"][L]])) for L in LEGS}
    spread = {L: abs(med["s2_l0r_a1"][L] - med["s2_l0r_a2"][L]) for L in LEGS}
    out["baseline"] = dict(mean=base, l0_spread=spread, arcs=["s2_l0r_a1", "s2_l0r_a2"])
    print("per-leg median of per-stride PEAK |tau_h| over complete windows 1-%d (N.m); [n windows >= %.0f]" % (NWIN, SAT))
    print("%-16s %-5s %5s   %6s %6s %6s %6s" % ("run", "cell", "kroll", "A", "B", "C", "D"))
    for run in ["s2_l0r_a1", "s2_l0r_a2", "s2_ol10_roll1", "s2_ol10_a2", "s2_ol10_a1", "s2_ol15_a1", "s2_ol10n_a2", "s2_ol15n_a1"]:
        if run not in med:
            continue
        c = CELL[run]
        print("%-16s %-5s %5.2f   %s   %s" % (run, c[0], c[2],
              "  ".join("%5.1f[%d]" % (med[run][L], sat[run][L]) for L in LEGS), c[3]))
    print("%-16s %-5s %5s   %s" % ("lambda0 mean", "l0", "", "  ".join("%5.1f   " % base[L] for L in LEGS)))
    print("%-16s %-5s %5s   %s" % ("lambda0 |a1-a2|", "", "", "  ".join("%5.1f   " % spread[L] for L in LEGS)))
    print("\ndelta vs the lambda0 mean (N.m); D under +gamma is a censored LOWER bound where windows sit >= 39")
    for run in ["s2_ol10_roll1", "s2_ol15_a1", "s2_ol10n_a2", "s2_ol15n_a1", "s2_ol10_a1", "s2_ol10_a2"]:
        if run not in med:
            continue
        c = CELL[run]; dl = {L: med[run][L] - base[L] for L in LEGS}
        out["delta"][run] = dict(cell=c[0], delta=dl, excluded=bool(c[3]), reason=c[3])
        if c[1] > 0:
            pair = "left pair  D/A = %.2f" % (dl["D"] / dl["A"]) if dl["A"] > 0 else "left pair  A delta <= 0"
        else:
            pair = "right pair B/C = %.2f" % (dl["B"] / dl["C"]) if dl["C"] > 0 else "right pair C delta <= 0"
        print("   %-16s %-5s  dA %+5.1f  dB %+5.1f  dC %+5.1f  dD %+5.1f   %s   %s"
              % (run, c[0], dl["A"], dl["B"], dl["C"], dl["D"], pair, ("EXCLUDED: " + c[3]) if c[3] else ""))
    inc_pos = [r for r in ("s2_ol10_roll1", "s2_ol15_a1") if r in med]
    inc_neg = [r for r in ("s2_ol10n_a2", "s2_ol15n_a1") if r in med]
    dD = [med[r]["D"] - base["D"] for r in inc_pos]; dA = [med[r]["A"] - base["A"] for r in inc_pos]
    dB = [med[r]["B"] - base["B"] for r in inc_neg]; dC = [med[r]["C"] - base["C"] for r in inc_neg]
    out["summary"] = dict(pos_arcs=inc_pos, neg_arcs=inc_neg,
                          D_plus=[min(dD), max(dD)], A_plus=[min(dA), max(dA)], B_minus=[min(dB), max(dB)], C_minus=[min(dC), max(dC)],
                          C_l0_spread=spread["C"],
                          reading="+gamma: added ABAD load lands on D (censored lower bound) against A; -gamma: on B; C's delta is inside C's lambda0 spread")
    print("\nSUMMARY (level-comparable, launch-valid arcs only): +gamma D %+.1f..%+.1f vs A %+.1f..%+.1f | -gamma B %+.1f..%+.1f vs C %+.1f..%+.1f (C lambda0 spread %.1f)"
          % (min(dD), max(dD), min(dA), max(dA), min(dB), max(dB), min(dC), max(dC), spread["C"]))
    # self-test against the values verified 2026-09-08
    want = (18.05, 17.91, 29.26, 21.18)
    ok = all(abs(base[L] - w) < 0.05 for L, w in zip(LEGS, want)) and 16.9 <= min(dD) and max(dD) <= 18.4
    print("SELFTEST", "PASS" if ok else "FAIL", "(lambda0 mean %s; D +gamma delta %.1f-%.1f)" % (
        "/".join("%.2f" % base[L] for L in LEGS), min(dD), max(dD)))
    with open(a.out, "w") as fh:
        json.dump(out, fh, indent=1)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
