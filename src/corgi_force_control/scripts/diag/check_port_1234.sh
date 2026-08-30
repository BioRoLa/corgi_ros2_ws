#!/usr/bin/env bash
# Start the simulator and test whether WSL can actually reach Webots on 1234.
#
# The driver reports "Cannot connect to Webots instance" while Webots itself
# logs that it IS listening on 1234. This tells us which side is wrong, before
# changing any configuration.

export TMPDIR=/mnt/c/Users/alexc/AppData/Local/Temp/webots_ws
export CORGI_EXPERIMENT_MODE=1
mkdir -p "$TMPDIR"

source /opt/ros/humble/setup.bash
source ~/corgi_ws/corgi_ros2_ws/install/setup.bash
cd ~/corgi_ws/corgi_ros2_ws || exit 1

setsid nohup ros2 launch corgi_sim Corgi_launch.py > /tmp/nc_sim.log 2>&1 < /dev/null &

# Wait for Webots to announce it is listening, or for the driver to give up.
for _ in $(seq 1 45); do
    grep -qE 'Waiting for local or remote connection|successfully connected' /tmp/nc_sim.log 2>/dev/null && break
    sleep 2
done

GW=$(ip route show default | awk '{print $3}')
echo "default gateway (windows host): $GW"
echo

for target in 127.0.0.1 "$GW"; do
    if nc -z -w2 "$target" 1234 2>/dev/null; then
        echo "  $target:1234  OPEN"
    else
        echo "  $target:1234  closed"
    fi
done

echo
echo "--- what the sim log says ---"
grep -E 'successfully connected|Waiting for local or remote|Cannot connect' /tmp/nc_sim.log | tail -4
