#!/usr/bin/env bash
# One complete measurement run from a FRESH simulator.
#
# Between-run variance is driven by accumulated robot state: runs immediately
# after a restart reach full leg extension and 66-74% flight, later runs stop
# 2-4 deg short and manage ~19%. A Webots *Reset* is not enough, and *Reload*
# drops the extern controller without recovering. So every run gets its own
# restart.
#
# Usage: sim_cycle.sh <template.csv> [extra launch args...]
#
# No `set -u`: the ROS setup scripts reference unbound variables.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TPL="$1"; shift
EXTRA="$@"

source /opt/ros/humble/setup.bash
source ~/corgi_ws/corgi_ros2_ws/install/setup.bash
cd ~/corgi_ws/corgi_ros2_ws

# Webots must write the temp world somewhere the WINDOWS binary can read;
# \\wsl.localhost paths fail silently and the robot loads invisible.
export TMPDIR=/mnt/c/Users/alexc/AppData/Local/Temp/webots_ws
# corgi_driver calls simulationSetMode(PAUSE) unless this is set, and it CANNOT
# be set from a launch file -- webots-controller re-execs with a sanitised env.
export CORGI_EXPERIMENT_MODE=1
mkdir -p "$TMPDIR"

# --- tear everything down --------------------------------------------------
# Bracket trick throughout: a bare `pkill -f name` matches this script's own
# command line and kills the caller.
for p in "gslip_pron[k]_node" "force_contro[l]_node" "force_estimatio[n]_node" \
         "topi[c] pub" "corgi_control_pane[l]" "corgi_data_recorde[r]" \
         "webots_ros2_drive[r]" "webots-controlle[r]" "Corgi_launc[h]" \
         "webots\.ex[e]"; do
    pkill -9 -f "$p" 2>/dev/null
done
# The WSL-side pkill does not reap the WINDOWS Webots process, which keeps
# holding port 1234; the next launch then fails with "not in the list of robots
# with <extern> controllers".
powershell.exe -Command \
    "Get-Process webots,webots-bin -ErrorAction SilentlyContinue | Stop-Process -Force" \
    >/dev/null 2>&1
sleep 6
ros2 daemon stop >/dev/null 2>&1
sleep 2

# --- bring the simulator up ------------------------------------------------
setsid nohup ros2 launch corgi_sim Corgi_launch.py > /tmp/cyc_sim.log 2>&1 < /dev/null &
for _ in $(seq 1 90); do
    grep -q "successfully connected" /tmp/cyc_sim.log 2>/dev/null && break
    sleep 2
done
if ! grep -q "successfully connected" /tmp/cyc_sim.log 2>/dev/null; then
    echo "SIM FAILED TO START"; exit 1
fi
sleep 4

# --- controller ------------------------------------------------------------
setsid nohup ros2 launch corgi_force_control gslip_pronk.launch.py \
    template_path:="$TPL" $EXTRA > /tmp/cyc_ctl.log 2>&1 < /dev/null &
for _ in $(seq 1 90); do
    grep -q "Holding pose" /tmp/cyc_ctl.log 2>/dev/null && break
    sleep 2
done
if ! grep -q "Holding pose" /tmp/cyc_ctl.log 2>/dev/null; then
    echo "CONTROLLER FAILED TO REACH HOLD"; exit 1
fi

# --- trigger and measure ---------------------------------------------------
setsid nohup ros2 topic pub -r 50 /trigger corgi_msgs/msg/TriggerStamped \
    "{enable: true}" > /tmp/cyc_trig.log 2>&1 < /dev/null &
sleep 20

# Order matters: the gait stops roughly a minute after the trigger, so anything
# run late reads a static robot (symptom: sample rates collapsing and zero
# stride boundaries).
python3 "$HERE/check_pronk.py" 2>/dev/null | grep -E "airborne|^   [ABCD] |A vs"
echo "  --- attitude ---"
python3 "$HERE/check_rpy.py" 2>/dev/null | grep -E "mean|contact %"

pkill -9 -f "topi[c] pub" 2>/dev/null
