#!/usr/bin/env bash
# Self-test launch_check.py on the known answer: roll1 (good launch) vs a2 (flight fraction 0.99).
source /opt/ros/humble/setup.bash
source /home/alexc/gslip_merge_ws/install/setup.bash
S=$(cd "$(dirname "$0")" && pwd)   # the repo copy, not a scratchpad (fixed 2026-09-08, log 325.46)
R=/home/alexc/corgi_runs/hw_2026-09-06/bags
for run in s2_ol10_roll1 s2_ol10_a2 s2_ol10n_a2 s2_l0r_a1; do
  echo "================ $run"
  python3 $S/launch_check.py $R/$run/${run}_0.db3 2>&1 | tail -n 8
done
