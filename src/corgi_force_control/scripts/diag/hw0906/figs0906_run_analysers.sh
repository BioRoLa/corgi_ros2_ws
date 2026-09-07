#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash 2>/dev/null || true
if [ -f /home/alexc/gslip_merge_ws/install/setup.bash ]; then
  source /home/alexc/gslip_merge_ws/install/setup.bash
fi
D=/home/alexc/corgi_runs/hw_2026-09-06
S=/mnt/c/Users/alexc/AppData/Local/Temp/claude/C--Users-alexc-Documents-Obsidian-Vault/fefde951-4709-4df4-a73b-a67aab98cd92/scratchpad
echo "################ VICON per-hop speed"
python3 "$S/figs0906_vicon_hops.py" "$S/figs0906_vicon_hops.json" \
  L0_RA1="$D/vicon/L0_RA1.c3d" \
  L0RA2="$D/vicon/L0RA2.c3d" \
  OL10_R1="$D/vicon/OL10_R1.c3d" \
  OL10A2="$D/vicon/OL10A2.c3d" \
  OL10_NA2="$D/vicon/OL10_NA2.c3d" \
  OL15_A1="$D/vicon/OL15_A1.c3d" \
  OL15_NA1="$D/vicon/OL15_NA1.c3d" \
  OL10_A1="$D/vicon/OL10_A1.c3d" \
  OL10_NA1="$D/vicon/OL10_NA1.c3d"
echo "################ BAG per-stride ABAD torque and roll"
python3 "$S/figs0906_bag_strides.py" "$S/figs0906_bag_strides.json" \
  s2_l0r_a1="$D/bags/s2_l0r_a1/s2_l0r_a1_0.db3" \
  s2_l0r_a2="$D/bags/s2_l0r_a2/s2_l0r_a2_0.db3" \
  s2_ol10_roll1="$D/bags/s2_ol10_roll1/s2_ol10_roll1_0.db3" \
  s2_ol10_a2="$D/bags/s2_ol10_a2/s2_ol10_a2_0.db3" \
  s2_ol10n_a2="$D/bags/s2_ol10n_a2/s2_ol10n_a2_0.db3" \
  s2_ol15_a1="$D/bags/s2_ol15_a1/s2_ol15_a1_0.db3" \
  s2_ol15n_a1="$D/bags/s2_ol15n_a1/s2_ol15n_a1_0.db3" \
  s2_ol10_a1="$D/bags/s2_ol10_a1/s2_ol10_a1_0.db3"
