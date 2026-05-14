#!/bin/bash

SCRIPT_DIR=$(dirname "$(realpath "$0")")
PARENT_DIR=$(dirname "$SCRIPT_DIR")

# When this script is run from install space, write bags into source space if available.
if [[ -d "$SCRIPT_DIR/../../../../../src/corgi_odometry_legacy" ]]; then
  BASE_DIR=$(realpath "$SCRIPT_DIR/../../../../../src/corgi_odometry_legacy")
else
  BASE_DIR="$PARENT_DIR"
fi

OUTPUT=${1:-"$BASE_DIR/bag/legacy_odom$(date +%Y%m%d_%H%M%S)"}

mkdir -p "$(dirname "$OUTPUT")"

ros2 bag record \
  /trigger \
  /motor/state \
  /imu \
  /odometry/legacy/velocity \
  /odometry/legacy/position \
  /odometry/legacy/contact \
  -o "$OUTPUT"
