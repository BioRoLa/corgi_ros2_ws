#!/usr/bin/env bash
# One force-rock run on the simulated force plates, scored.
#
# Fresh sim (Corgi_ABAD_force_plate.wbt), force estimation + force control,
# exp_h24 in a rock mode, a bag of command / estimate / plates, then
# force_plate_track.py on it. The sim plates share the bag's clock, so the
# score includes lag.
#
# Usage: rock_cycle.sh <rock_mode> [extra exp_h24 params, e.g. rock_amp:=20 rock_hz:=0.5]
#   rock_mode: diag_fz | pitch_fz | roll_fz | fore_fx | lateral_fy
# Output: ~/corgi_runs/rock/<stamp>_<mode>/{bag,score.txt,score.json}
#
# No `set -u`: the ROS setup scripts reference unbound variables.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:?usage: rock_cycle.sh <rock_mode> [name:=value ...]}"; shift
PARAMS=()
for kv in "$@"; do PARAMS+=(-p "$kv"); done

source /opt/ros/humble/setup.bash
source ~/corgi_ws/corgi_ros2_ws/install/setup.bash
cd ~/corgi_ws/corgi_ros2_ws

OUT=~/corgi_runs/rock/$(date +%Y%m%d_%H%M%S)_$MODE
mkdir -p "$OUT"

# Webots must write the temp world somewhere the WINDOWS binary can read
# (see sim_cycle.sh).
export TMPDIR=/mnt/c/Users/alexc/AppData/Local/Temp/webots_ws
export CORGI_EXPERIMENT_MODE=1
# The plant of record is two env vars (see repeat_gain_regime.sh): B/C
# torques converted at the ROS<->module boundary (corgi_driver convert_torque;
# without it B and C are fed un-mirrored feedforward and report module-frame
# torques), and the theta linkage stop. Override with =0 on purpose only.
export CORGI_DIRBETA_TRANSFORM=${CORGI_DIRBETA_TRANSFORM:-1}
export CORGI_THETA_STOP=${CORGI_THETA_STOP:-1}
mkdir -p "$TMPDIR"

RUN_START=$(date +%s)
fresh_grep() {
    [ -f "$1" ] || return 1
    [ "$(stat -c %Y "$1")" -ge "$RUN_START" ] || return 1
    grep -q "$2" "$1" 2>/dev/null
}

KILL_PATTERNS=("exp_h2[4]_node" "force_contro[l]_node" "force_estimatio[n]_node"
    "topi[c] pub" "ros2 ba[g] record" "webots_ros2_drive[r]" "webots-controlle[r]"
    "Corgi_experiment_launc[h]" "force_control.launc[h]" "webots\.ex[e]")

teardown_all() {
    for p in "${KILL_PATTERNS[@]}"; do
        pkill -9 -f "$p" 2>/dev/null
    done
    powershell.exe -Command \
        "Get-Process webots,webots-bin -ErrorAction SilentlyContinue | Stop-Process -Force" \
        >/dev/null 2>&1
    sleep 6
    timeout 15 ros2 daemon stop >/dev/null 2>&1
    sleep 2
}

finish() {
    pkill -INT -f "ros2 ba[g] record" 2>/dev/null
    sleep 3
    teardown_all
}

teardown_all

# --- simulator + plates ------------------------------------------------------
setsid nohup ros2 launch corgi_sim Corgi_experiment_launch.py \
    world:=Corgi_ABAD_force_plate.wbt > "$OUT/sim.log" 2>&1 < /dev/null &
for _ in $(seq 1 90); do
    fresh_grep "$OUT/sim.log" "publishing 4 force plates" && \
        fresh_grep "$OUT/sim.log" "Driver Initialized" && break
    sleep 2
done
if ! fresh_grep "$OUT/sim.log" "publishing 4 force plates"; then
    echo "SIM OR PLATES FAILED TO START -- see $OUT/sim.log"; finish; exit 1
fi
echo "CORGI_DIRBETA_TRANSFORM=$CORGI_DIRBETA_TRANSFORM CORGI_THETA_STOP=$CORGI_THETA_STOP CORGI_G_JOINT_MODE=${CORGI_G_JOINT_MODE:-gear (default)}" | tee "$OUT/plant.txt"
if [ "$CORGI_DIRBETA_TRANSFORM" = "1" ] && ! fresh_grep "$OUT/sim.log" "DIR_BETA TRANSFORM ON"; then
    sleep 3   # the banner is logged just after the plates come up
    if ! fresh_grep "$OUT/sim.log" "DIR_BETA TRANSFORM ON"; then
        echo "TRANSFORM REQUESTED BUT THE DRIVER DID NOT ENABLE IT -- see $OUT/sim.log"; finish; exit 1
    fi
fi
sleep 3

# --- estimation + control + rock publisher -------------------------------------
setsid nohup ros2 launch corgi_force_control force_control.launch.py \
    > "$OUT/ctl.log" 2>&1 < /dev/null &
sleep 3
setsid nohup ros2 run corgi_force_control exp_h24_node --ros-args \
    -p use_sim_time:=true -p rock_mode:="$MODE" -p rock_hold_s:=4.0 "${PARAMS[@]}" \
    > "$OUT/exp.log" 2>&1 < /dev/null &
for _ in $(seq 1 150); do
    fresh_grep "$OUT/exp.log" "Initialization phase completed" && break
    sleep 2
done
if ! fresh_grep "$OUT/exp.log" "Initialization phase completed"; then
    echo "exp_h24 NEVER STOOD UP -- see $OUT/exp.log"; finish; exit 1
fi
if grep -q "using legacy" "$OUT/exp.log"; then
    echo "exp_h24 rejected the rock params -- see $OUT/exp.log"; finish; exit 1
fi

# --- record, trigger, wait for the rock to finish -----------------------------
setsid nohup ros2 bag record -o "$OUT/bag" /impedance/command /force/state /motor/state /motor/command /imu \
    /sensor/force_plate_1 /sensor/force_plate_2 /sensor/force_plate_3 /sensor/force_plate_4 \
    > "$OUT/bag.log" 2>&1 < /dev/null &
sleep 3
setsid nohup ros2 topic pub -r 50 /trigger corgi_msgs/msg/TriggerStamped \
    "{enable: true}" > /dev/null 2>&1 < /dev/null &
# Webots on Windows runs well below real time from WSL; allow 20 min wall
for _ in $(seq 1 600); do
    fresh_grep "$OUT/exp.log" "Rock phase finished" && break
    sleep 2
done
if ! fresh_grep "$OUT/exp.log" "Rock phase finished"; then
    echo "ROCK DID NOT FINISH -- see $OUT/exp.log"; finish; exit 1
fi

finish
python3 "$HERE/force_plate_track.py" "$OUT/bag" --json "$OUT/score.json" | tee "$OUT/score.txt"
echo "run in $OUT"
