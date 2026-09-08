#!/usr/bin/env bash
# Read-only: run the existing front-leg diagnostic on a cambered arc against a
# straight reference. Usage: diag_front.sh <cambered_run> <reference_run>
set -o pipefail
H=biorola@192.168.30.244
C="${1:?cambered run id}"
R="${2:?reference run id}"
ssh -o BatchMode=yes -o ConnectTimeout=6 "$H" "bash -s" <<REMOTE
source /opt/ros/humble/setup.bash
source ~/corgi_ws/corgi_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=67
W=~/corgi_ws/corgi_ros2_ws
D=\$W/src/corgi_force_control/scripts/diag/hw0906
python3 \$D/l15_diag.py \$W/bag/$C/${C}_0.db3 \$W/bag/$R/${R}_0.db3 2>&1 | tail -70
REMOTE
