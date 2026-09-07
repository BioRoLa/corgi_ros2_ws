#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash 2>/dev/null || true
if [ -f /home/alexc/gslip_merge_ws/install/setup.bash ]; then
  source /home/alexc/gslip_merge_ws/install/setup.bash
fi
D=/home/alexc/corgi_runs/hw_2026-09-06/bags
S=/mnt/c/Users/alexc/AppData/Local/Temp/claude/C--Users-alexc-Documents-Obsidian-Vault/fefde951-4709-4df4-a73b-a67aab98cd92/scratchpad
python3 "$S/l0_cell.py" \
  "$D/s2_l0_a1/s2_l0_a1_0.db3" \
  "$D/s2_l0_a2/s2_l0_a2_0.db3" \
  "$D/s2_l0_a3/s2_l0_a3_0.db3" \
  "$D/s2_l0_a4/s2_l0_a4_0.db3" \
  "$D/s2_l0_a5/s2_l0_a5_0.db3"
