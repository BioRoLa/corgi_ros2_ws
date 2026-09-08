#!/usr/bin/env bash
# Read-only: score one arc on the Orin with the registered launch rule, and
# print the whole-arc per-leg peaks beside it. Usage: check_arc.sh s3_l0_a1
set -o pipefail
H=biorola@192.168.30.244
RUN="${1:?give a run id}"
ssh -o BatchMode=yes -o ConnectTimeout=6 "$H" "bash -s" <<REMOTE
source /opt/ros/humble/setup.bash
source ~/corgi_ws/corgi_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=67
W=~/corgi_ws/corgi_ros2_ws
D=\$W/src/corgi_force_control/scripts/diag/hw0906
B=\$W/bag/$RUN/${RUN}_0.db3
echo "== bag =="
ls -l \$W/bag/$RUN 2>&1 | tail -3
[ -f "\$B" ] || { echo "NO BAG for $RUN"; exit 1; }
echo "== banner lines from the controller log =="
grep -E "ff=|range=|turn_rate=|k_yaw=|k_roll=|k_radial=|k_tangential=|k_lateral=|b_lateral=|k_flight=|coupling=|k_pitch=|PITCH CHANNEL|K_TANGENTIAL OVERRIDDEN|ACKER|HEADING HOLD" \
  \$W/output_data/ctl_$RUN.log 2>/dev/null | head -14
echo "== launch_check (registered rule 325.43) =="
python3 \$D/launch_check.py "\$B" 2>&1 | tail -20
echo "== whole-arc per-leg peaks =="
python3 \$D/l_trend.py "\$B" 2>&1 | tail -12
REMOTE
