#!/bin/bash
# Record event-walk + attitude-control + odometry outputs on the real robot.

SCRIPT_DIR=$(dirname "$(realpath "$0")")

# When run from install space, write bags into source space if available.
if [[ -d "$SCRIPT_DIR/../../../../../src/corgi_event_walk" ]]; then
  BASE_DIR=$(realpath "$SCRIPT_DIR/../../../../../src/corgi_event_walk")
else
  BASE_DIR=$(dirname "$SCRIPT_DIR")
fi

OUTPUT=${1:-"$BASE_DIR/bag/event_walk_esekf_$(date +%Y%m%d_%H%M%S)"}
mkdir -p "$(dirname "$OUTPUT")"

ros2 bag record \
  /imu_raw \
  /livox/lidar \
  /livox/imu \
  /Odometry \
  /ekf \
  /lidar_odom \
  /odom_mapping \
  /fusion/bv \
  /tf \
  /tf_static \
  /walk/command \
  /walk/phase \
  /walk/swing_mask \
  /walk/swing_phase \
  /attitude/stable \
  /motor/command \
  /power/command \
  /power/state \
  -o "$OUTPUT"
