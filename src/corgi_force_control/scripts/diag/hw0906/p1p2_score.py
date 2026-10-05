#!/usr/bin/env python3
"""P1/P2 scored on Session B (registered log 325.46, scored 2026-09-11).

Registration, verbatim basis: Delta|tau_h| per leg = median over complete
windows 1-6 of the arc minus the same over windows 1-6 of that session's
lambda0 arcs (stride clock as 325.39).
  P1: in every clean +gamma arc (10, 12.5 deg) Delta(D) > Delta(A) AND D
      reaches 39 N.m within the first FIVE windows.
  P2: in every clean -gamma arc (10, 12.5, 15 deg) no window of D reaches
      39 N.m (windows 1-6 basis; whole-arc max reported beside).
  B-vs-C split reported beside, NOT registered. A miss on P1 or P2 in any
  clean arc => the within-pair split is not robust; the paper keeps
  "inferred" wording.
Clean = scored + launch-valid per data/hw/manifest.json: +10 a1v2, +10 a2,
+12.5 ol12_a1 (near-spin: launch-valid, floor-failing -- scored, and the
verdict is ALSO reported without it in case "clean" is read as
floor-passing); -10 a1c, -10 a2, -12.5, -15. Baseline: s3_l0_a1/a2/a3.

BONUS (unregistered, labelled): P-HW-rho repeat check on the same arcs,
same estimator as log 328 (median per-stride roll minus lambda0 baseline,
over signed lean).
"""
import json, os, statistics
HERE = os.path.dirname(os.path.abspath(__file__))
D = json.load(open(os.path.join(HERE, "figs0908_bag_strides.json")))
LEGS = "ABCD"
L0 = ["s3_l0_a1", "s3_l0_a2", "s3_l0_a3"]
POS = [("s3_ol10_a1v2", 10, ""), ("s3_ol10_a2", 10, ""),
       ("s3_ol12_a1", 12.5, "near-spin, 0.54x floor")]
NEG = [("s3_ol10n_a1c", -10, ""), ("s3_ol10n_a2", -10, ""),
       ("s3_ol12n_a1", -12.5, ""), ("s3_ol15n_a1", -15, "")]
def key(run, leg):
    t = D[run]["tauH"]
    return t[leg] if leg in t else t[leg.lower()]
def med16(run, leg):
    v = [abs(x) for x in key(run, leg)[:6]]
    return statistics.median(v)
base = {L: statistics.mean([med16(r, L) for r in L0]) for L in LEGS}
print("lambda0 baseline (median w1-6, mean of %s): %s" % (",".join(L0),
      "  ".join("%s %.1f" % (L, base[L]) for L in LEGS)))
print("\n%-14s %6s  dA     dB     dC     dD    D w1-5 max  D arc max  clauses" % ("arc", "lam"))
p1_ok, p2_ok, p1_ok_strict = True, True, True
out = {"baseline": base, "arcs": {}}
for run, lam, flag in POS + NEG:
    d = {L: med16(run, L) - base[L] for L in LEGS}
    Dw = [abs(x) for x in key(run, "D")]
    d15 = max(Dw[:5]); dall = max(Dw)
    if lam > 0:
        c1 = d["D"] > d["A"]; c2 = d15 >= 39.0
        verdict = "P1 %s (dD>dA %s, D>=39 in w1-5 %s)" % ("PASS" if c1 and c2 else "MISS", c1, c2)
        if not (c1 and c2):
            p1_ok = False
            if "spin" not in flag: p1_ok_strict = False
    else:
        c = max(Dw[:6]) < 39.0
        verdict = "P2 %s (no D window>=39 in w1-6: %s; arc max %.1f)" % ("PASS" if c else "MISS", c, dall)
        if not c: p2_ok = False
    print("%-14s %6.1f  %+5.1f  %+5.1f  %+5.1f  %+5.1f   %6.1f     %6.1f    %s  %s"
          % (run, lam, d["A"], d["B"], d["C"], d["D"], d15, dall, verdict, flag))
    out["arcs"][run] = dict(lam=lam, delta=d, D_w15_max=d15, D_arc_max=dall, flag=flag)
print("\nB-vs-C beside (not registered): -gamma dB vs dC shown above.")
print("\nVERDICT: P1 %s (all clean +gamma arcs)%s; P2 %s (all clean -gamma arcs)"
      % ("PASS" if p1_ok else "MISS",
         "" if p1_ok == p1_ok_strict else " [excluding the near-spin: %s]" % ("PASS" if p1_ok_strict else "MISS"),
         "PASS" if p2_ok else "MISS"))
print("Registration clause: a miss on P1 or P2 in any clean arc => within-pair split not robust, paper keeps 'inferred'.")
# ---- unregistered P-HW-rho repeat check ----
rbase = statistics.mean([statistics.median(D[r]["roll"]) for r in L0])
print("\nP-HW-rho REPEAT CHECK (unregistered, informational; lambda0 roll baseline %+.2f deg):" % rbase)
for run, lam, flag in POS + NEG:
    m = statistics.median(D[run]["roll"])
    print("  %-14s %6.1f  roll_med %+6.2f  d %+6.2f  ratio %+.3f  %s" % (run, lam, m, m - rbase, (m - rbase) / lam, flag))
out["p1"] = p1_ok; out["p2"] = p2_ok
json.dump(out, open(os.path.join(HERE, "p1p2_score.json"), "w"), indent=1)
print("\nwrote p1p2_score.json")
