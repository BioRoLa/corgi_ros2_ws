#!/usr/bin/env python3
"""Splice the emitted data blocks into the generator templates and write the self-contained
generators beside their PNGs in Research/SLIP/Figures. Each marker must occur exactly once."""
S = "/mnt/c/Users/alexc/AppData/Local/Temp/claude/C--Users-alexc-Documents-Obsidian-Vault/fefde951-4709-4df4-a73b-a67aab98cd92/scratchpad"
F = "/mnt/c/Users/alexc/Documents/Obsidian Vault/Research/SLIP/Figures"
JOBS = [("tpl_per_stride_speed.py", "make_figures_2026-09-06_per_stride_speed.py", {"# @@DATA_VICON@@": "figs0906_data_vicon.txt"}),
        ("tpl_abad_torque_per_stride.py", "make_figures_2026-09-06_abad_torque_per_stride.py", {"# @@DATA_BAG@@": "figs0906_data_bag.txt"}),
        ("tpl_roll_per_stride.py", "make_figures_2026-09-06_roll_per_stride.py", {"# @@DATA_BAG@@": "figs0906_data_bag.txt"})]
for tpl, out, subs in JOBS:
    src = open(S + "/" + tpl, encoding="utf-8").read()
    for marker, frag in subs.items():
        n = src.count(marker)
        if n != 1:
            raise SystemExit("ABORT: marker %r occurs %d times in %s" % (marker, n, tpl))
        block = open(S + "/" + frag, encoding="utf-8").read().rstrip("\n")
        src = src.replace(marker, block)
    open(F + "/" + out, "w", encoding="utf-8", newline="\n").write(src)
    print("assembled", out)
