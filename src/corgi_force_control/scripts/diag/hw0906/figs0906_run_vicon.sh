#!/usr/bin/env bash
set -e
D=/home/alexc/corgi_runs/hw_2026-09-06
S=/mnt/c/Users/alexc/AppData/Local/Temp/claude/C--Users-alexc-Documents-Obsidian-Vault/fefde951-4709-4df4-a73b-a67aab98cd92/scratchpad
python3 "$S/figs0906_vicon_hops.py" "$S/figs0906_vicon_hops.json" \
  L0_RA1="$D/vicon/L0_RA1.c3d" \
  L0RA2="$D/vicon/L0RA2.c3d" \
  OL10_R1="$D/vicon/OL10_R1.c3d" \
  OL10A2="$D/vicon/OL10A2.c3d" \
  OL10_NA2="$D/vicon/OL10_NA2.c3d" \
  OL15_A1="$D/vicon/OL15_A1.c3d" \
  OL15_NA1="$D/vicon/OL15_NA1.c3d" \
  OL10_A1="$D/vicon/OL10_A1.c3d" \
  OL10_NA1="$D/vicon/OL10_NA1.c3d" | grep -E '^(==|   hops)'
