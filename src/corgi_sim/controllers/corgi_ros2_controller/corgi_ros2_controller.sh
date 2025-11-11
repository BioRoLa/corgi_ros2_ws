#!/bin/bash
# Wrapper script to source ROS2 environment before running the Python controller

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Source the corgi_sim workspace
source ~/corgi_ws/corgi_ros_ws/install/setup.bash

# Run the actual Python controller
exec python3 "$(dirname "$0")/corgi_ros2_controller.py" "$@"
