#!/bin/bash

SCRIPT_DIR=$(dirname "$(realpath "$0")")
PARENT_DIR=$(dirname "$SCRIPT_DIR")

OUTPUT=${1:-"$PARENT_DIR/bag/leg_odom$(date +%Y%m%d_%H%M%S)"}

mkdir -p "$(dirname "$OUTPUT")"

ros2 bag record \
  /imu_raw \
  /motor/state \
  /trigger \
  /ekf \
  /ekf/orientation \
  /ekf/ba \
  /ekf/bw \
  /gmo/contact_state \
  -o "$OUTPUT"