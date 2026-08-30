#!/usr/bin/env bash
# Stand the Corgi up, walk it clear of the support block, then hand control to
# the G-SLIP pronk controller.
#
# Why the handover is needed: the G-SLIP nominal stance stands the hip at
# 0.293 m, while the support block needs 0.331 m to clear. The robot therefore
# cannot lift itself off the block into the running pose -- its legs extend and
# touch down but the body never rises. The trot CSV stands at theta = 120 deg
# (hip 0.331 m), which does clear it, so the sequence is:
#
#   1. csv_control plays its first 5000 rows  -> robot stands up on the block
#   2. /trigger goes true                     -> csv_control plays the gait and
#                                                the robot trots off the block
#   3. csv_control is stopped once clear
#   4. gslip_pronk ramps DOWN from the measured tall pose to theta = 100 deg
#   5. /trigger goes true again               -> pronk
#
# Both csv_control and gslip_pronk listen on /trigger, so the trigger publisher
# is stopped between stages -- otherwise stage 5 would also resume the CSV, and
# gslip_pronk would skip its ramp and start running immediately.
#
# Timing uses SIMULATION time, read from /clock. Webots on this machine steps
# well below real time, so wall-clock sleeps would cut the trot short.
#
# Stage 2 waits on DISTANCE TRAVELLED, read from the odom->base_link TF, not on
# a duration -- how far the robot gets in a given time depends on the CSV and on
# how fast Webots happens to be stepping. The time limit is only a fallback so
# the script cannot hang.
#
# Usage:
#   ./gslip_handover.sh [clear_metres] [max_trot_seconds] [csv_name]
# Defaults: walk 0.9 m clear, giving up after 40 s of sim time, using
# demo_trot_straight_sim.

set -u

# 0.5 m clears the 0.4 m block from a centred start. The trot only manages
# about 0.45 m in 40 s of sim time, so asking for more just burns the fallback.
CLEAR_METRES="${1:-0.5}"
TROT_SECONDS="${2:-40}"
CSV_NAME="${3:-demo_trot_straight_sim}"

log() { echo "[handover] $*"; }

sim_time() {
    timeout 5 ros2 topic echo --once /clock 2>/dev/null \
        | awk '/sec:/ {s=$2} /nanosec:/ {n=$2} END {if (s=="") print ""; else printf "%.3f", s + n/1e9}'
}

# Forward position of the robot, from the odom->base_link transform.
body_x() {
    timeout 6 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null \
        | grep -m1 -A1 "Translation" \
        | grep -oE "\[.*\]" \
        | tr -d '[]' | cut -d, -f1
}

# Wait until the robot has travelled `want` metres, giving up after
# `limit` seconds of sim time so a stalled gait cannot hang the script.
wait_for_clearance() {
    local want="$1" limit="$2" t0 x0 now x moved
    t0=$(sim_time)
    x0=$(body_x)
    if [ -z "$t0" ]; then
        log "ERROR: no /clock -- is the simulation running and unpaused?"
        return 1
    fi
    if [ -z "$x0" ]; then
        log "WARNING: no odom->base_link TF; falling back to a ${limit}s sim-time wait"
        while :; do
            sleep 2
            now=$(sim_time)
            [ -z "$now" ] && continue
            awk -v a="$now" -v b="$t0" -v w="$limit" 'BEGIN{exit !(a-b >= w)}' && return 0
        done
    fi
    log "walking clear: need ${want} m from x0=${x0} (limit ${limit}s sim)"
    while :; do
        sleep 2
        x=$(body_x); now=$(sim_time)
        [ -z "$x" ] && continue
        moved=$(awk -v a="$x" -v b="$x0" 'BEGIN{d=a-b; if(d<0)d=-d; printf "%.3f", d}')
        if awk -v d="$moved" -v w="$want" 'BEGIN{exit !(d >= w)}'; then
            log "travelled ${moved} m -- clear of the block"
            return 0
        fi
        if [ -n "$now" ] && awk -v a="$now" -v b="$t0" -v l="$limit" 'BEGIN{exit !(a-b >= l)}'; then
            log "WARNING: only travelled ${moved} m before the ${limit}s limit; continuing anyway"
            return 0
        fi
    done
}

start_trigger() {
    ros2 topic pub -r 50 /trigger corgi_msgs/msg/TriggerStamped '{enable: true}' \
        > /tmp/gslip_trigger.log 2>&1 &
    TRIGGER_PID=$!
    log "trigger publishing (pid $TRIGGER_PID)"
}

stop_trigger() {
    if [ -n "${TRIGGER_PID:-}" ]; then
        kill "$TRIGGER_PID" 2>/dev/null
        wait "$TRIGGER_PID" 2>/dev/null
        log "trigger stopped"
        TRIGGER_PID=""
    fi
}

# `ros2 run` forks a python wrapper AND the real binary. Killing the wrapper
# PID leaves the binary alive and still publishing /motor/command -- which is
# how a stopped CSV can come back to life and fight the pronk for control of
# the legs. Always match on the executable name.
kill_by_name() {
    local pattern="$1" i
    pkill -f "$pattern" 2>/dev/null
    for i in 1 2 3 4 5; do
        pgrep -f "$pattern" >/dev/null 2>&1 || return 0
        sleep 1
    done
    pkill -9 -f "$pattern" 2>/dev/null
    sleep 1
    if pgrep -f "$pattern" >/dev/null 2>&1; then
        log "WARNING: '$pattern' survived SIGKILL"
        return 1
    fi
    return 0
}

cleanup() {
    stop_trigger
    kill_by_name corgi_csv_control
    kill_by_name gslip_pronk
}
trap cleanup EXIT INT TERM

if [ -z "$(sim_time)" ]; then
    log "ERROR: /clock is not publishing. Start the simulator and press Play."
    exit 1
fi

# --- Stage 1: stand up -------------------------------------------------------
log "stage 1: standing up with $CSV_NAME"
ros2 run corgi_csv_control corgi_csv_control "$CSV_NAME" \
    --ros-args -p use_sim_time:=true > /tmp/gslip_csv.log 2>&1 &
CSV_PID=$!

for _ in $(seq 1 120); do
    grep -q "Leg Transform Finished" /tmp/gslip_csv.log 2>/dev/null && break
    sleep 2
done
if ! grep -q "Leg Transform Finished" /tmp/gslip_csv.log 2>/dev/null; then
    log "ERROR: stand-up did not finish; see /tmp/gslip_csv.log"
    exit 1
fi
log "stood up"

# --- Stage 2: trot clear of the block ---------------------------------------
log "stage 2: trotting off the block"
start_trigger
wait_for_clearance "$CLEAR_METRES" "$TROT_SECONDS" || exit 1

# --- Stage 3: stop the CSV ---------------------------------------------------
stop_trigger
log "stage 3: stopping csv_control"
if ! kill_by_name corgi_csv_control; then
    log "ERROR: csv_control still alive; it would fight the pronk for /motor/command"
    exit 1
fi
CSV_PID=""
log "csv_control stopped; /motor/command is free"
sleep 2

# --- Stage 4: hand over ------------------------------------------------------
log "stage 4: starting gslip_pronk (ramps down from the measured pose)"
ros2 launch corgi_force_control gslip_pronk.launch.py \
    > /tmp/gslip_pronk.log 2>&1 &
GSLIP_PID=$!

for _ in $(seq 1 120); do
    grep -q "Holding pose" /tmp/gslip_pronk.log 2>/dev/null && break
    sleep 2
done
if ! grep -q "Holding pose" /tmp/gslip_pronk.log 2>/dev/null; then
    log "ERROR: gslip_pronk did not reach its hold; see /tmp/gslip_pronk.log"
    exit 1
fi
grep -E "Ramping|Loaded" /tmp/gslip_pronk.log | sed 's/^/[handover] /'
log "holding the G-SLIP stance"

# --- Stage 5: pronk ----------------------------------------------------------
log "stage 5: triggering the pronk -- Ctrl-C to stop"
start_trigger
wait "$GSLIP_PID"
