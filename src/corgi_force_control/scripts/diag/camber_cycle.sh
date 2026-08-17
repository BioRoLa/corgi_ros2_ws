#!/usr/bin/env bash
# One camber-roll measurement from a FRESH simulator.
#
# Structure is ramp_cycle.sh's, and for the same reasons -- between-run
# variance is driven by accumulated robot state, a Webots *Reset* is not
# enough, and the recorder has to be up BEFORE the trigger or the run has no
# anchor. What differs is the vehicle: this drives camber_roll.py, which
# publishes position commands straight to motor/command, instead of the
# gslip_pronk controller and its stride template.
#
# Usage:
#   camber_cycle.sh <lam_deg> <pattern> [tag]   pattern is lr | fr | none
#
#   The tag NAMES the dump; it is not a path, and that is deliberate. Invoking
#   this through Git Bash on the Windows side (wsl.exe -e bash -lc '...')
#   silently rewrites anything that looks like a POSIX path -- an absolute
#   dump path arrives as C:/Users/... and the recorder dies at savez AFTER the
#   whole run has been paid for. Tags cannot be mangled.
#
#   ROLL_TIME     seconds of rolling (default 20). The circle fit wants at
#                 least a quarter turn; 20 s at the default rate is >1 lap at
#                 both 10 and 20 deg.
#   CAMBER_EXTRA  extra flags forwarded verbatim to camber_roll.py, e.g.
#                 "--lift rear --acker".
#
# No `set -u`: the ROS setup scripts reference unbound variables.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAM="${1:?usage: camber_cycle.sh <lam_deg> <pattern> [tag]}"
PAT="${2:?usage: camber_cycle.sh <lam_deg> <pattern> [tag]}"
TAG="${3:-raw}"
DUMPDIR="$HOME/camber_dumps"
DUMP="$DUMPDIR/camber_$TAG.npz"
ROLL="${ROLL_TIME:-20}"
mkdir -p "$DUMPDIR"

# camber_roll.py's schedule is settle 1 + fold 3 + lean 2 + settle 1.5 = 7.5 s
# before the roll starts. Record a little past the end so the last samples are
# not clipped mid-stride.
UNTIL=$(python3 -c "print(7.5 + $ROLL + 2.0)")

source /opt/ros/humble/setup.bash
source ~/corgi_ws/corgi_ros2_ws/install/setup.bash
cd ~/corgi_ws/corgi_ros2_ws

# Webots must write the temp world somewhere the WINDOWS binary can read;
# \\wsl.localhost paths fail silently and the robot loads invisible.
export TMPDIR=/mnt/c/Users/alexc/AppData/Local/Temp/webots_ws
# corgi_driver calls simulationSetMode(PAUSE) unless this is set, and it CANNOT
# be set from a launch file -- webots-controller re-execs with a sanitised env.
# It is also what makes /trigger remove SUPPORT_BOX.
export CORGI_EXPERIMENT_MODE=1
mkdir -p "$TMPDIR"

# --- tear everything down --------------------------------------------------
# Bracket trick throughout: a bare `pkill -f name` matches this script's own
# command line and kills the caller.
for p in "camber_rol[l]" "record_cambe[r]" "gslip_pron[k]_node" \
         "force_contro[l]_node" "force_estimatio[n]_node" "topi[c] pub" \
         "corgi_control_pane[l]" "corgi_data_recorde[r]" \
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
setsid nohup ros2 launch corgi_sim Corgi_launch.py \
    > /tmp/camber_sim.log 2>&1 < /dev/null &
for _ in $(seq 1 90); do
    grep -q "successfully connected" /tmp/camber_sim.log 2>/dev/null && break
    sleep 2
done
if ! grep -q "successfully connected" /tmp/camber_sim.log 2>/dev/null; then
    echo "SIM FAILED TO START -- see /tmp/camber_sim.log"
    grep -iE "Cannot connect|not in the list" /tmp/camber_sim.log | tail -3
    exit 1
fi
sleep 4

# --- commander -------------------------------------------------------------
setsid nohup python3 "$HERE/camber_roll.py" \
    --lam-deg "$LAM" --pattern "$PAT" --roll-time "$ROLL" $CAMBER_EXTRA \
    > /tmp/camber_ctl.log 2>&1 < /dev/null &
for _ in $(seq 1 120); do
    grep -q "holding for the trigger" /tmp/camber_ctl.log 2>/dev/null && break
    sleep 2
done
if ! grep -q "holding for the trigger" /tmp/camber_ctl.log 2>/dev/null; then
    echo "COMMANDER NEVER STOOD UP -- see /tmp/camber_ctl.log"
    tail -5 /tmp/camber_ctl.log
    exit 1
fi

# --- recorder first, THEN trigger ------------------------------------------
# Deliberately NOT setsid: setsid forks, so $! would be the wrapper rather than
# the recorder and `wait` would return immediately.
python3 "$HERE/record_camber.py" --dump "$DUMP" --until "$UNTIL" \
    > /tmp/camber_meas.log 2>&1 < /dev/null &
REC=$!
sleep 3

setsid nohup ros2 topic pub -r 50 /trigger corgi_msgs/msg/TriggerStamped \
    "{enable: true}" > /tmp/camber_trig.log 2>&1 < /dev/null &

# Hold the trigger until the recorder has the whole schedule (or times out).
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
python3 "$HERE/check_camber_turn.py" "$DUMP" --lam-deg "$LAM" --pattern "$PAT"
