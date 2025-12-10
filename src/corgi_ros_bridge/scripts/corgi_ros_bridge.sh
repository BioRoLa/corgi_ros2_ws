#!/bin/bash
ros2 run corgi_ros_bridge corgi_ros_bridge &
BRIDGE_PID=$!

trap "kill $BRIDGE_PID; exit;" INT
wait $BRIDGE_PID