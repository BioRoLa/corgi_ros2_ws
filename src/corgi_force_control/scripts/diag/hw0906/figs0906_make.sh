#!/usr/bin/env bash
set -e
S=/mnt/c/Users/alexc/AppData/Local/Temp/claude/C--Users-alexc-Documents-Obsidian-Vault/fefde951-4709-4df4-a73b-a67aab98cd92/scratchpad
F="/mnt/c/Users/alexc/Documents/Obsidian Vault/Research/SLIP/Figures"
python3 "$S/figs0906_emit_literals.py"
python3 "$S/figs0906_assemble.py"
cd "$F"
python3 make_figures_2026-09-06_per_stride_speed.py
python3 make_figures_2026-09-06_abad_torque_per_stride.py
python3 make_figures_2026-09-06_roll_per_stride.py
python3 make_figures_2026-09-06_speed_by_kroll.py
