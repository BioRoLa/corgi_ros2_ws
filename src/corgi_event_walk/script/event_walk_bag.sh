#!/bin/bash
# Record event-walk, attitude, real odometry, and corgi_ros_bridge ROS topics.

set -euo pipefail

SCRIPT_DIR=$(dirname "$(realpath "$0")")
EXPERIMENT_NAME=${1:-event_walk}

# When run from install space, write bags into source space if available.
if [[ -d "$SCRIPT_DIR/../../../../../src/corgi_event_walk" ]]; then
  BASE_DIR=$(realpath "$SCRIPT_DIR/../../../../../src/corgi_event_walk")
else
  BASE_DIR=$(dirname "$SCRIPT_DIR")
fi

OUTPUT=${2:-"$BASE_DIR/bag/${EXPERIMENT_NAME}_$(date +%Y%m%d_%H%M%S)"}
mkdir -p "$(dirname "$OUTPUT")"

ros2 bag record \
  /imu_raw \
  /livox/lidar \
  /livox/imu \
  /Odometry \
  /ekf \
  /ekf/orientation \
  /ekf/ba \
  /ekf/bw \
  /gmo/contact_state \
  /lidar_odom \
  /odom_mapping \
  /fusion/bv \
  /tf \
  /tf_static \
  /trigger \
  /walk/command \
  /walk/phase \
  /walk/swing_mask \
  /walk/swing_phase \
  /attitude/stable \
  /motor/command \
  /steer/command \
  /robot/command \
  /config/command \
  /motor/state \
  /power/state \
  /steer/state \
  /robot/state \
  /config/state \
  /log \
  -o "$OUTPUT"
