#!/usr/bin/env bash
# Read back the gains actually in effect on a running gslip_pronk_node.
#
# The node only logs k_radial at startup, so a launch-line b_radial that failed
# to apply is otherwise invisible -- and b_radial is the difference between a
# 57% hop and a 69% one. Always confirm rather than assume the arg landed.

source /opt/ros/humble/setup.bash
source ~/corgi_ws/corgi_ros2_ws/install/setup.bash

if ! ros2 node list 2>/dev/null | grep -q gslip_pronk_node; then
    echo "gslip_pronk_node is not running"
    exit 1
fi

for p in k_radial b_radial k_lateral k_tangential k_flight k_roll k_yaw \
         spring_rest_reference template_path; do
    printf '  %-22s %s\n' "$p" "$(ros2 param get /gslip_pronk_node "$p" 2>/dev/null | sed 's/^.*value is: //')"
done
