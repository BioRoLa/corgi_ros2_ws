#!/bin/bash

SCRIPT_DIR=$(dirname "$(realpath "$0")")
PARENT_DIR=$(dirname "$SCRIPT_DIR")

# When this script is run from install space, write bags into source space if available.
if [[ -d "$SCRIPT_DIR/../../../../../src/corgi_odometry" ]]; then
  BASE_DIR=$(realpath "$SCRIPT_DIR/../../../../../src/corgi_odometry")
else
  BASE_DIR="$PARENT_DIR"
fi

OUTPUT=${1:-"$BASE_DIR/bag/odom_fusion$(date +%Y%m%d_%H%M%S)"}

mkdir -p "$(dirname "$OUTPUT")"

ros2 bag record \
  /imu_raw \
  /motor/state \
  /trigger \
  /gmo/contact_state \
  /ekf \
  /ekf/orientation \
  /ekf/ba \
  /ekf/bw \
  /lidar_odom \
  /odom_mapping \
  /odometry \
  /fusion/bv \
  -o "$OUTPUT"