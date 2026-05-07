#!/bin/bash

SCRIPT_DIR=$(dirname "$(realpath "$0")")
PARENT_DIR=$(dirname "$SCRIPT_DIR")

# When this script is run from install space, write bags into source space if available.
if [[ -d "$SCRIPT_DIR/../../../../../src/corgi_mpc" ]]; then
  BASE_DIR=$(realpath "$SCRIPT_DIR/../../../../../src/corgi_mpc")
else
  BASE_DIR="$PARENT_DIR"
fi

OUTPUT=${1:-"$BASE_DIR/bag/mpc_open_$(date +%Y%m%d_%H%M%S)"}

mkdir -p "$(dirname "$OUTPUT")"

ros2 bag record \
  /trigger \
  /imu \
  /motor/command \
  /motor/state \
  /force/state \
  /odometry/legacy/position \
  /odometry/legacy/velocity \
  /odometry/legacy/contact \
  /odometry/legacy/z_position_hip \
  /walk/swing_phase \
  -o "$OUTPUT"