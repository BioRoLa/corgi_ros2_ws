#!/usr/bin/env python3
"""Turn the two analyser JSONs into Python-source data blocks that the assembler splices into the
figure generators, so each generator carries its numbers inline (hardcoded from analyser output)."""
import json
S = "/mnt/c/Users/alexc/AppData/Local/Temp/claude/C--Users-alexc-Documents-Obsidian-Vault/fefde951-4709-4df4-a73b-a67aab98cd92/scratchpad"
H = json.load(open(S + "/figs0906_vicon_hops.json"))
B = json.load(open(S + "/figs0906_bag_strides.json"))
ORDER_V = ["L0_RA1", "L0RA2", "OL10_R1", "OL10A2", "OL10_NA2", "OL15_A1", "OL15_NA1", "OL10_A1"]
ORDER_B = ["s2_l0r_a1", "s2_l0r_a2", "s2_ol10_roll1", "s2_ol10_a2", "s2_ol10n_a2", "s2_ol15_a1", "s2_ol15n_a1", "s2_ol10_a1"]


def fl(xs, nd):
    return "[" + ", ".join("%.*f" % (nd, x) for x in xs) + "]"


def bl(xs):
    return "[" + ", ".join("True" if x else "False" for x in xs) + "]"


L = ["# ---- numbers HARDCODED from figs0906_vicon_hops.py output (scratchpad, 2026-09-06); never re-derived here ----",
     "# v = per-hop net horizontal displacement / hop duration (m/s), hop index from the Vicon window start;",
     "# full = hop period within 0.22-0.32 s (template 0.2642 s); launch/stop segments are False.",
     "# n_floor = registered first full hop < 0.257; n_floor3 = first of >=3 consecutive full hops < 0.257 (exploratory).",
     "HOPS = {"]
for n in ORDER_V:
    d = H[n]
    L.append(' "%s": dict(v_arc=%.3f, median=%.3f, n_full=%d, n_floor=%s, n_floor3=%s,' %
             (n, d["v_arc"], d["median_v_full"], d["n_full"], d["n_floor"], d["n_floor3"]))
    L.append('   v=%s,' % fl(d["v"], 3))
    L.append('   full=%s),' % bl(d["full"]))
L.append("}")
open(S + "/figs0906_data_vicon.txt", "w", encoding="utf-8", newline="\n").write("\n".join(L) + "\n")

L = ["# ---- numbers HARDCODED from figs0906_bag_strides.py output (scratchpad, 2026-09-06); never re-derived here ----",
     "# per-stride peak |tau_h| (N.m) per leg and per-stride mean IMU roll (deg); stride detector as l_trend.py,",
     "# stride 1 = l_trend.py i = 0. n_sat = registered first stride with any leg >= 39 N.m (and which leg).",
     "STRIDES = {"]
for n in ORDER_B:
    d = B[n]
    L.append(' "%s": dict(n=%d, gait=%.1f, n_sat=%s, n_sat_leg=%r,' % (n, d["n"], d["gait"], d["n_sat"], d["n_sat_leg"]))
    for leg in "ABCD":
        L.append('   %s=%s,' % (leg, fl(d["tauH"][leg], 1)))
    L.append('   roll=%s),' % fl(d["roll"], 2))
L.append("}")
open(S + "/figs0906_data_bag.txt", "w", encoding="utf-8", newline="\n").write("\n".join(L) + "\n")
print("emitted data blocks")
