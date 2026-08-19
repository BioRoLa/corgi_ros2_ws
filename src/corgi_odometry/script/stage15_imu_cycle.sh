#!/usr/bin/env bash
# One camber-roll measurement WITH IMU, from a FRESH simulator.
#
# This is camber_cycle.sh (scripts/diag, commit ecc917d) with exactly one
# functional change: the recorder is corgi_odometry/script/record_camber_imu.py
# (adds imu_t/imu_quat/imu_gyro/imu_accel + odom_ang to the dump). It lives in
# corgi_odometry/script because scripts/diag belongs to the demand-reduction
# thread; camber_roll.py and check_camber_turn.py are INVOKED from there,
# never modified. All structure -- fresh sim per run, recorder before
# trigger, fresh_grep evidence rule, bounded verified teardown -- is theirs;
# see camber_cycle.sh for the reasons, each bought with a failed run.
#
# Usage:  stage15_imu_cycle.sh <lam_deg> <pattern> [tag]
# Env:    ROLL_TIME (default 20), FOLD_SETTLE (default 0), CAMBER_EXTRA
#
# No `set -u`: the ROS setup scripts reference unbound variables.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIAG="$HOME/corgi_ws/corgi_ros2_ws/src/corgi_force_control/scripts/diag"
LAM="${1:?usage: stage15_imu_cycle.sh <lam_deg> <pattern> [tag]}"
PAT="${2:?usage: stage15_imu_cycle.sh <lam_deg> <pattern> [tag]}"
TAG="${3:-raw}"
DUMPDIR="$HOME/camber_dumps"
DUMP="$DUMPDIR/camber_$TAG.npz"
ROLL="${ROLL_TIME:-20}"
mkdir -p "$DUMPDIR"

FS="${FOLD_SETTLE:-0}"
UNTIL=$(python3 -c "print(7.5 + $FS + $ROLL + 2.0)")
CAMBER_EXTRA="--fold-settle $FS $CAMBER_EXTRA"

source /opt/ros/humble/setup.bash
source ~/corgi_ws/corgi_ros2_ws/install/setup.bash
cd ~/corgi_ws/corgi_ros2_ws

export TMPDIR=/mnt/c/Users/alexc/AppData/Local/Temp/webots_ws
export CORGI_EXPERIMENT_MODE=1
mkdir -p "$TMPDIR"

RUN_START=$(date +%s)
fresh_grep() {   # fresh_grep <file> <pattern>
    [ -f "$1" ] || return 1
    [ "$(stat -c %Y "$1")" -ge "$RUN_START" ] || return 1
    grep -q "$2" "$1" 2>/dev/null
}

KILL_PATTERNS=("camber_rol[l]" "record_cambe[r]" "gslip_pron[k]_node"
    "force_contro[l]_node" "force_estimatio[n]_node" "topi[c] pub"
    "corgi_control_pane[l]" "corgi_data_recorde[r]"
    "webots_ros2_drive[r]" "webots-controlle[r]" "Corgi_launc[h]"
    "webots\.ex[e]")

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

verify_dead() {
    local alive=""
    for p in "${KILL_PATTERNS[@]}"; do
        alive="$alive$(pgrep -af "$p" 2>/dev/null)"
    done
    [ -z "$alive" ] && return 0
    echo "SIM NOT CLEAR after teardown:"
    for p in "${KILL_PATTERNS[@]}"; do
        pgrep -af "$p" 2>/dev/null
    done
    return 1
}

teardown_all

setsid nohup ros2 launch corgi_sim Corgi_launch.py \
    > /tmp/camber_sim.log 2>&1 < /dev/null &
for _ in $(seq 1 90); do
    fresh_grep /tmp/camber_sim.log "successfully connected" && break
    sleep 2
done
if ! fresh_grep /tmp/camber_sim.log "successfully connected"; then
    echo "SIM FAILED TO START -- see /tmp/camber_sim.log"
    grep -iE "Cannot connect|not in the list" /tmp/camber_sim.log | tail -3
    exit 1
fi
sleep 4

setsid nohup python3 "$DIAG/camber_roll.py" \
    --lam-deg "$LAM" --pattern "$PAT" --roll-time "$ROLL" $CAMBER_EXTRA \
    > /tmp/camber_ctl.log 2>&1 < /dev/null &
for _ in $(seq 1 120); do
    fresh_grep /tmp/camber_ctl.log "holding for the trigger" && break
    sleep 2
done
if ! fresh_grep /tmp/camber_ctl.log "holding for the trigger"; then
    echo "COMMANDER NEVER STOOD UP -- see /tmp/camber_ctl.log"
    tail -5 /tmp/camber_ctl.log
    exit 1
fi

# recorder first, THEN trigger (NOT setsid: `wait` needs the real PID)
python3 "$HERE/record_camber_imu.py" --dump "$DUMP" --until "$UNTIL" \
    > /tmp/camber_meas.log 2>&1 < /dev/null &
REC=$!
sleep 3

setsid nohup ros2 topic pub -r 50 /trigger corgi_msgs/msg/TriggerStamped \
    "{enable: true}" > /tmp/camber_trig.log 2>&1 < /dev/null &

wait $REC 2>/dev/null
pkill -9 -f "topi[c] pub" 2>/dev/null
pkill -9 -f "camber_rol[l]" 2>/dev/null

cp /tmp/camber_ctl.log "$DUMPDIR/camber_$TAG.ctl.log" 2>/dev/null

cat /tmp/camber_meas.log
echo
echo "--- commander ---"
grep -E "Trigger at|--- |Roll complete|enter |final" /tmp/camber_ctl.log | tail -12
echo
echo "--- circle fit ---"
python3 "$DIAG/check_camber_turn.py" "$DUMP" --lam-deg "$LAM" --pattern "$PAT"

teardown_all
if ! verify_dead; then
    teardown_all
    if ! verify_dead; then
        echo "TEARDOWN FAILED TWICE -- kill the survivors above by PID."
        exit 3
    fi
fi
echo "sim clear."
