#!/usr/bin/env bash
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f /home/alexc/gslip_merge_ws/install/setup.bash ] && source /home/alexc/gslip_merge_ws/install/setup.bash
S=/mnt/c/Users/alexc/AppData/Local/Temp/claude/C--Users-alexc-Documents-Obsidian-Vault/fefde951-4709-4df4-a73b-a67aab98cd92/scratchpad
cd "$S"
rm -f "$S/imu_v3.done"
python3 "$S/imu_vs_vicon2.py" "$S/imu_vs_vicon3.json" 2>&1 | grep -v RuntimeWarning | grep -v "nanvar(a" | grep -v "cen\[:, a\]"
echo "##### EXTRACT"
python3 "$S/imu_extract.py" "$S/imu_vs_vicon3.json"
echo "##### EXCESS TIME"
python3 "$S/imu_excess_time.py" s2_l0_a3 L0_ramp_3 s2_l0r_a1 L0_RA1 s2_ol10_roll1 OL10_R1 s2_ol10n_a2 OL10_NA2 2>&1 | grep -v Warning | grep -v "nanvar(a" | grep -v "cen\[:, a\]"
echo "##### DONE"
touch "$S/imu_v3.done"
