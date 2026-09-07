#!/usr/bin/env bash
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f /home/alexc/gslip_merge_ws/install/setup.bash ] && source /home/alexc/gslip_merge_ws/install/setup.bash
S=/mnt/c/Users/alexc/AppData/Local/Temp/claude/C--Users-alexc-Documents-Obsidian-Vault/fefde951-4709-4df4-a73b-a67aab98cd92/scratchpad
python3 "$S/imu_vs_vicon2.py" "$S/imu_vs_vicon2.json" 2>&1 | grep -v RuntimeWarning | grep -v "nanvar(a" | grep -v "cen\[:, a\]" | tee "$S/imu_vs_vicon2_out.txt"
