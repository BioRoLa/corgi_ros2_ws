#!/bin/bash
# Reproduce the campaign's leg_demand and speed_from_odom tables verbatim.
cd /home/alexc/corgi_ws/corgi_ros2_ws/src/corgi_force_control/scripts/diag || exit 1
echo "===================== leg_demand.py (with odom) ====================="
python3 leg_demand.py \
  --dir ~/corgi_runs/clock_ff/off  --label off \
  --dir ~/corgi_runs/clock_ff/on   --label on \
  --dir ~/corgi_runs/clock_ff/both --label both \
  --odom
echo
echo "===================== speed_from_odom.py ============================"
for cell in off on both; do
  for r in 1 2 3; do
    echo "--- $cell run$r ---"
    python3 speed_from_odom.py ~/corgi_runs/clock_ff/$cell/odom_run$r.csv 2>&1 | tail -20
  done
done
