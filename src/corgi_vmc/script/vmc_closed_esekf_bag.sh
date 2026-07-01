#!/bin/bash
# Record VMC (esekf mode) experiment bag.
# Topics: VMC I/O + ESEKF/fusion odometry + raw IMU + motor + force

SCRIPT_DIR=$(dirname "$(realpath "$0")")

# Write bags into source space when run from install space
if [[ -d "$SCRIPT_DIR/../../../../../src/corgi_vmc" ]]; then
  BASE_DIR=$(realpath "$SCRIPT_DIR/../../../../../src/corgi_vmc")
else
  BASE_DIR=$(dirname "$SCRIPT_DIR")
fi

OUTPUT=${1:-"$BASE_DIR/bag/vmc_esekf_$(date +%Y%m%d_%H%M%S)"}
mkdir -p "$(dirname "$OUTPUT")"

ros2 bag record \
  /trigger \
  /imu_raw \
  /motor/state \
  /motor/command \
  /force/state \
  /ekf \
  /gmo/contact_state \
  /odom_mapping \
  /fusion/bv \
  /lidar_odom \
  /impedance/command \
  /walk/swing_phase \
  /power/command \
  /power/state \
  -o "$OUTPUT"
