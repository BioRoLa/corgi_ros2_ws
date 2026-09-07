#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash 2>/dev/null || true
if [ -f /home/alexc/gslip_merge_ws/install/setup.bash ]; then
  source /home/alexc/gslip_merge_ws/install/setup.bash
fi
S=/mnt/c/Users/alexc/AppData/Local/Temp/claude/C--Users-alexc-Documents-Obsidian-Vault/fefde951-4709-4df4-a73b-a67aab98cd92/scratchpad
python3 "$S/exploratory_slope.py" 2>&1 | grep -vE "RuntimeWarning|var = nanvar|cen\[:, a\]|nanmean|Mean of empty" | tee "$S/exploratory_slope_out.txt"
