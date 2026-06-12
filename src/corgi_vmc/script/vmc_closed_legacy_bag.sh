#!/bin/bash
# Record VMC (odom_legacy mode) experiment bag.
# Topics: VMC I/O + legacy odometry + IMU + motor + force

SCRIPT_DIR=$(dirname "$(realpath "$0")")

# Write bags into source space when run from install space
if [[ -d "$SCRIPT_DIR/../../../../../src/corgi_vmc" ]]; then
  BASE_DIR=$(realpath "$SCRIPT_DIR/../../../../../src/corgi_vmc")
else
  BASE_DIR=$(dirname "$SCRIPT_DIR")
fi

OUTPUT=${1:-"$BASE_DIR/bag/vmc_legacy_$(date +%Y%m%d_%H%M%S)"}
mkdir -p "$(dirname "$OUTPUT")"

ros2 bag record \
  /trigger \
  /imu \
  /motor/state \
  /motor/command \
  /force/state \
  /odometry/legacy/position \
  /odometry/legacy/velocity \
  /odometry/legacy/contact \
  /odometry/legacy/z_position_hip \
  /impedance/command \
  /walk/swing_phase \
  -o "$OUTPUT"
