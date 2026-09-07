#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source /home/alexc/gslip_merge_ws/install/setup.bash
S=/mnt/c/Users/alexc/AppData/Local/Temp/claude/C--Users-alexc-Documents-Obsidian-Vault/fefde951-4709-4df4-a73b-a67aab98cd92/scratchpad
R=/home/alexc/corgi_runs/hw_2026-09-06/bags
ARGS=""
for run in s2_l0r_a1 s2_l0r_a2 s2_ol10_roll1 s2_ol10_a2 s2_ol10n_a1 s2_ol10n_a2 s2_ol15_a1 s2_ol15n_a1 s2_ol10_a1; do
  [ -f $R/$run/${run}_0.db3 ] && ARGS="$ARGS $R/$run/${run}_0.db3"
done
python3 $S/ff_scan.py $ARGS
