#!/usr/bin/env bash
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f /home/alexc/gslip_merge_ws/install/setup.bash ] && source /home/alexc/gslip_merge_ws/install/setup.bash
S=/mnt/c/Users/alexc/AppData/Local/Temp/claude/C--Users-alexc-Documents-Obsidian-Vault/fefde951-4709-4df4-a73b-a67aab98cd92/scratchpad
D=/home/alexc/corgi_runs/hw_2026-09-06/bags
python3 "$S/compliance.py" \
  "$D/s2_l0r_a1/s2_l0r_a1_0.db3" \
  "$D/s2_ol10_roll1/s2_ol10_roll1_0.db3" \
  "$D/s2_ol10n_a2/s2_ol10n_a2_0.db3" \
  "$D/s2_ol15_a1/s2_ol15_a1_0.db3" \
  "$D/s2_ol15n_a1/s2_ol15n_a1_0.db3" \
  "$D/s2_l0_a4/s2_l0_a4_0.db3" \
  "$D/s2_l0_a1/s2_l0_a1_0.db3"
