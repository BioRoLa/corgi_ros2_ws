#!/usr/bin/env bash
# One speed-ramp measurement from a FRESH simulator, measured PER SEGMENT.
#
# Differs from sim_cycle.sh in three ways that matter for the ramp:
#
#   1. The recorder starts BEFORE the trigger, so it captures the commanded-beta
#      fiducial that locates template t=0. sim_cycle.sh starts measuring 20 s
#      after the trigger and cannot recover the phase.
#   2. The trigger is held for the whole pass. gslip_pronk exits its run loop the
#      instant `trigger_` goes false, so killing the publisher early truncates
#      the gait -- that is what "Stopped after 8 strides" means in the logs.
#   3. It waits on sim time, not wall time. Webots runs ~14x slower than real
#      time here and the factor varies, so a fixed wall delay lands at a
#      different gait age every run.
#
# Usage: ramp_cycle.sh <template.csv> [extra launch args...]
#
# No `set -u`: the ROS setup scripts reference unbound variables.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TPL="$1"; shift
EXTRA="$@"

source /opt/ros/humble/setup.bash
source ~/corgi_ws/corgi_ros2_ws/install/setup.bash
cd ~/corgi_ws/corgi_ros2_ws

export TMPDIR=/mnt/c/Users/alexc/AppData/Local/Temp/webots_ws
export CORGI_EXPERIMENT_MODE=1
mkdir -p "$TMPDIR"

# --- tear everything down --------------------------------------------------
for p in "gslip_pron[k]_node" "force_contro[l]_node" "force_estimatio[n]_node" \
         "topi[c] pub" "corgi_control_pane[l]" "corgi_data_recorde[r]" \
         "check_ram[p]" "webots_ros2_drive[r]" "webots-controlle[r]" \
         "Corgi_launc[h]" "webots\.ex[e]"; do
    pkill -9 -f "$p" 2>/dev/null
done
powershell.exe -Command \
    "Get-Process webots,webots-bin -ErrorAction SilentlyContinue | Stop-Process -Force" \
    >/dev/null 2>&1
sleep 6
ros2 daemon stop >/dev/null 2>&1
sleep 2

# --- bring the simulator up ------------------------------------------------
setsid nohup ros2 launch corgi_sim Corgi_launch.py > /tmp/ramp_sim.log 2>&1 < /dev/null &
for _ in $(seq 1 90); do
    grep -q "successfully connected" /tmp/ramp_sim.log 2>/dev/null && break
    sleep 2
done
if ! grep -q "successfully connected" /tmp/ramp_sim.log 2>/dev/null; then
    echo "SIM FAILED TO START -- see /tmp/ramp_sim.log"
    grep -iE "Cannot connect|not in the list" /tmp/ramp_sim.log | tail -3
    exit 1
fi
sleep 4

# --- controller ------------------------------------------------------------
setsid nohup ros2 launch corgi_force_control gslip_pronk.launch.py \
    template_path:="$TPL" $EXTRA > /tmp/ramp_ctl.log 2>&1 < /dev/null &
for _ in $(seq 1 120); do
    grep -q "Holding pose" /tmp/ramp_ctl.log 2>/dev/null && break
    sleep 2
done
if ! grep -q "Holding pose" /tmp/ramp_ctl.log 2>/dev/null; then
    echo "CONTROLLER FAILED TO REACH HOLD -- see /tmp/ramp_ctl.log"; exit 1
fi

# --- recorder first, THEN trigger ------------------------------------------
# Deliberately NOT setsid here: setsid forks, so $! would be the wrapper rather
# than the recorder and `wait` would return immediately.
python3 "$HERE/check_ramp.py" "$TPL" --dump /tmp/ramp_raw.npz \
    > /tmp/ramp_meas.log 2>&1 < /dev/null &
REC=$!
sleep 3

setsid nohup ros2 topic pub -r 50 /trigger corgi_msgs/msg/TriggerStamped \
    "{enable: true}" > /tmp/ramp_trig.log 2>&1 < /dev/null &

# Hold the trigger until the recorder decides it has a full pass (or times out).
wait $REC 2>/dev/null
pkill -9 -f "topi[c] pub" 2>/dev/null

cat /tmp/ramp_meas.log
echo
echo "--- attitude over the run ---"
python3 "$HERE/check_rpy.py" 2>/dev/null | grep -E "mean|contact %"
echo "--- controller ---"
grep -E "Loaded|Settling|Running the|strides" /tmp/ramp_ctl.log | tail -4
