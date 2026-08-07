#!/usr/bin/env bash
# Does the Webots driver process actually SEE CORGI_CONTACT_INTERVAL?
#
# corgi_driver.py:479 reads it from os.environ (default '10'), the same way it
# reads CORGI_EXPERIMENT_MODE at :489. But webots-controller re-execs with a
# sanitised environment, which is why CORGI_EXPERIMENT_MODE cannot be set from a
# launch file. Reading /proc/<pid>/environ is the only way to know for sure
# which variables survive to the process that actually reads them.

source /opt/ros/humble/setup.bash
source ~/corgi_ws/corgi_ros2_ws/install/setup.bash
cd ~/corgi_ws/corgi_ros2_ws

export TMPDIR=/mnt/c/Users/alexc/AppData/Local/Temp/webots_ws
export CORGI_EXPERIMENT_MODE=1
export CORGI_CONTACT_INTERVAL=100
mkdir -p "$TMPDIR"

for p in "webots_ros2_drive[r]" "webots-controlle[r]" "Corgi_launc[h]" "webots\.ex[e]"; do
    pkill -9 -f "$p" 2>/dev/null
done
powershell.exe -Command \
    "Get-Process webots,webots-bin -ErrorAction SilentlyContinue | Stop-Process -Force" \
    >/dev/null 2>&1
sleep 6

setsid nohup ros2 launch corgi_sim Corgi_launch.py > /tmp/env_sim.log 2>&1 < /dev/null &
for _ in $(seq 1 90); do
    grep -q "successfully connected" /tmp/env_sim.log 2>/dev/null && break
    sleep 2
done
grep -q "successfully connected" /tmp/env_sim.log 2>/dev/null || { echo "SIM FAILED"; exit 1; }

echo "=== processes running corgi_driver ==="
ps -eo pid,args | grep "[c]orgi_driver\|[w]ebots_ros2_driver" | head -5

echo
echo "=== CORGI_* seen by each candidate process ==="
for pid in $(pgrep -f "corgi_driver|webots_ros2_driver"); do
    echo "--- pid $pid : $(tr '\0' ' ' < /proc/$pid/cmdline | cut -c1-70)"
    tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null \
        | grep -E "^CORGI_" || echo "    (no CORGI_* variables)"
done

echo
echo "shell has: CORGI_CONTACT_INTERVAL=$CORGI_CONTACT_INTERVAL "\
"CORGI_EXPERIMENT_MODE=$CORGI_EXPERIMENT_MODE"
