#!/usr/bin/env bash
# Camber-in-pronk campaign: the Stage 3 mechanism demonstration (log s87).
#
# Runs the v070 pronk (config of record: CORGI_DIRBETA_TRANSFORM=1 +
# CORGI_THETA_STOP=1) with the open-loop Ackermann camber pair held, n runs
# per condition from a FRESH simulator each, capturing the odom CSV + torque
# CSV pair that check_menger.py scores.
#
# This is repeat_gain_regime.sh's run loop wearing stage15_imu_cycle.sh's
# armour (fresh_grep evidence rule, bounded verified teardown) -- a NEW file
# because repeat_gain_regime.sh belongs to the demand-reduction thread and is
# not modified, the same reasoning stage15_imu_cycle.sh recorded when it
# copied camber_cycle.sh.
#
# Usage:   menger_acker_campaign.sh <condition> [condition...]
# Conditions: lam0_default lam0 lam5_pos lam5_neg lam10_pos lam10_neg
#             lam15_pos lam15_neg
# Env:     RUNS (default 3), GAIT_WALL (default 200), SETTLE_WALL (default 45),
#          RECORD_IMU=1 -> run 1 of lam0 and lam5_pos also records the
#          Stage 1.5 IMU npz (record_camber_imu.py, trigger-anchored).
#          EXTRA_ARGS -> appended verbatim to every launch (word-split), e.g.
#          EXTRA_ARGS='stance_label_shift_s:=0.066' for the S89 C1 phase A/B.
#          Runs with EXTRA_ARGS are NOT comparable to unmodified campaigns --
#          use a fresh BASE_OUT. If it contains stance_label_shift_s, the
#          shift's own WARN is asserted like the ACKER announcement.
#
# lam0_default runs the FEEDBACK DEFAULTS (k_yaw 0.15): it is the
# bit-identity check against s57's banked numbers (chord 0.216-0.253 m/s,
# yaw -1.3..-2.8 deg/s). Every other condition zeroes k_yaw/d_yaw so the
# saturated yaw-hold (measured pinned at its 5 deg clamp in all three s57
# baselines) cannot contaminate the gamma traces.
#
# Ackermann magnitudes are the LEGGED-pose pair (h = 0.2736 m at the v070
# mean stance theta 89.56 deg; s87's registered table), NOT camber_roll.py's
# wheel-mode numbers.
#
# No `set -u`: the ROS setup scripts reference unbound variables.

WS=~/corgi_ws/corgi_ros2_ws
DIAG="$WS/src/corgi_force_control/scripts/diag"
ODOM_SCRIPT="$WS/src/corgi_odometry/script"
TPL="$WS/src/corgi_force_control/config/gslip_pronk_template_v070.csv"
BASE_OUT=${BASE_OUT:-~/corgi_runs/menger_acker}
RUNS=${RUNS:-3}
GAIT_WALL=${GAIT_WALL:-200}
SETTLE_WALL=${SETTLE_WALL:-45}

# rad: (in, out) per magnitude, legged-pose apex condition (log s87 table).
declare -A ACK_IN=(  [lam5]=0.08727 [lam10]=0.17453 [lam15]=0.26180 )
declare -A ACK_OUT=( [lam5]=0.07690 [lam10]=0.13765 [lam15]=0.18719 )

cond_args() {  # -> launch args for a condition, on stdout
  local c="$1"
  local common="template_path:=$TPL"
  local nofb="k_yaw:=0.0 d_yaw:=0.0"
  case "$c" in
    lam0_default) echo "$common" ;;
    lam0)         echo "$common $nofb" ;;
    lam*_pos|lam*_neg)
      local mag="${c%_*}" dir
      [ "${c##*_}" = pos ] && dir=1.0 || dir=-1.0
      echo "$common $nofb gamma_acker_in:=${ACK_IN[$mag]}" \
           "gamma_acker_out:=${ACK_OUT[$mag]} gamma_acker_dir:=$dir" ;;
    *) return 1 ;;
  esac
}

[ $# -ge 1 ] || { echo "usage: menger_acker_campaign.sh <condition>..."; exit 2; }
for c in "$@"; do
  cond_args "$c" >/dev/null || { echo "unknown condition: $c"; exit 2; }
done

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export TMPDIR=/mnt/c/Users/alexc/AppData/Local/Temp/webots_ws
export CORGI_EXPERIMENT_MODE=1
export CORGI_TORQUE_DEBUG=1
export CORGI_DIRBETA_TRANSFORM=1
export CORGI_THETA_STOP=1
mkdir -p "$TMPDIR"

KILL_PATTERNS=("gslip_pron[k]_node" "force_contro[l]_node"
    "force_estimatio[n]_node" "topi[c] pub" "topi[c] echo"
    "record_camber_im[u]" "corgi_control_pane[l]"
    "webots_ros2_drive[r]" "webots-controlle[r]" "Corgi_launc[h]"
    "webots\.ex[e]"
    # Corgi_launch.py spawns a corgi_data_recorder that outlives the run;
    # 48 of them had accumulated by 2026-08-20 and their DDS discovery
    # load was wedging `topic echo` (dead 1-row odom captures).
    "corgi_data_recorde[r]")

teardown_all() {
  for p in "${KILL_PATTERNS[@]}"; do pkill -9 -f "$p" 2>/dev/null; done
  # Windows-side kill NEEDS interop, which a setsid-detached campaign does
  # not have (its session's interop socket is gone) -- run this script from
  # an ATTACHED session (e.g. inside the Monitor's wsl.exe). A webots-bin
  # orphaned on the Windows side holds port 1234 and kills every later
  # launch with "process has died, exit code 1" + EACCES on the temp world
  # (2026-08-19, cost the first main-campaign run).
  timeout 20 /mnt/c/Windows/System32/taskkill.exe /F /IM webots.exe \
      /IM webots-bin.exe >/dev/null 2>&1
  rm -f /dev/shm/fastrtps_* 2>/dev/null
  sleep 6
  # The CLI daemon wedges silently and a wedged daemon starves `topic echo`
  # (two 1-row odom captures on 2026-08-20 before this line existed).
  pkill -9 -f 'ros2cli.daemon' 2>/dev/null
  timeout 15 ros2 daemon stop >/dev/null 2>&1
  sleep 2
}

verify_dead() {
  local alive=""
  for p in "${KILL_PATTERNS[@]}"; do
    alive="$alive$(pgrep -af "$p" 2>/dev/null)"
  done
  # Windows side too: pgrep cannot see an orphaned webots-bin.exe.
  local win
  win=$(timeout 20 /mnt/c/Windows/System32/tasklist.exe 2>/dev/null \
        | grep -i webots)
  if [ -z "$alive" ] && [ -z "$win" ]; then return 0; fi
  echo "SIM NOT CLEAR after teardown:"
  for p in "${KILL_PATTERNS[@]}"; do pgrep -af "$p" 2>/dev/null; done
  [ -n "$win" ] && echo "windows-side: $win"
  return 1
}

for COND in "$@"; do
  ARGS="$(cond_args "$COND")${EXTRA_ARGS:+ $EXTRA_ARGS}"
  OUT="$BASE_OUT/$COND"
  mkdir -p "$OUT"
  echo "################ CONDITION $COND ($ARGS) ################"

  for RUN in $(seq 1 "$RUNS"); do
    echo "======== $COND run $RUN / $RUNS ========"
    RUN_START=$(date +%s)
    fresh_grep() {
      [ -f "$1" ] || return 1
      [ "$(stat -c %Y "$1")" -ge "$RUN_START" ] || return 1
      grep -q "$2" "$1" 2>/dev/null
    }

    teardown_all
    if ! verify_dead; then
      teardown_all
      verify_dead || { echo "TEARDOWN FAILED TWICE -- aborting."; exit 3; }
    fi
    rm -f /tmp/corgi_torque_terms.csv /tmp/corgi_sched_events.csv

    setsid ros2 launch corgi_sim Corgi_launch.py \
        > "/tmp/ma_sim_${COND}_$RUN.log" 2>&1 < /dev/null &
    ok=0
    for _ in $(seq 1 60); do
      [ -f /tmp/corgi_torque_terms.csv ] && { ok=1; break; }
      sleep 3
    done
    [ "$ok" = 1 ] || { echo "  !! driver never connected, skipping run"; continue; }
    echo "  driver up"
    # Pre-warm CLI discovery against the fresh sim before anything relies
    # on it: the daemon restarted by the first CLI call can cache a stale
    # graph and starve `topic echo` (the 1-row odom failure mode).
    timeout 15 ros2 topic list > /dev/null 2>&1
    sleep 2

    # shellcheck disable=SC2086  -- ARGS is intentionally word-split
    setsid ros2 launch corgi_force_control gslip_pronk.launch.py $ARGS \
        > "/tmp/ma_ctl_${COND}_$RUN.log" 2>&1 < /dev/null &
    ok=0
    for _ in $(seq 1 40); do
      fresh_grep "/tmp/ma_ctl_${COND}_$RUN.log" 'waiting for trigger' && { ok=1; break; }
      sleep 3
    done
    [ "$ok" = 1 ] || { echo "  !! controller never reached hold, skipping run"; continue; }
    echo "  controller holding"

    # Stage 1.5 free-ride: recorder BEFORE the trigger, trigger-anchored.
    IMU_PID=""
    if [ -n "${RECORD_IMU:-}" ] && [ "$RUN" = 1 ] \
        && { [ "$COND" = lam0 ] || [ "$COND" = lam5_pos ]; }; then
      # --until is TRIGGER-ANCHORED sim time and the dump is written only
      # when it elapses. The runs reach ~26-33 s post-trigger, so 35 never
      # arrived and the first campaign's recorders died dumpless when the
      # run teardown killed them. 22 lands the dump mid-gait.
      setsid python3 "$ODOM_SCRIPT/record_camber_imu.py" \
          --dump "$OUT/imu_run$RUN.npz" --until 22 \
          > "/tmp/ma_imu_${COND}_$RUN.log" 2>&1 < /dev/null &
      IMU_PID=$!
      sleep 3
      echo "  IMU recorder up (until sim t=22, trigger-anchored)"
    fi

    setsid ros2 topic pub -r 50 /trigger corgi_msgs/msg/TriggerStamped \
        '{enable: true}' > /dev/null 2>&1 < /dev/null &

    sleep "$SETTLE_WALL"

    setsid ros2 topic echo /sim/base_odom --csv \
        > "$OUT/odom_run$RUN.csv" 2>/dev/null < /dev/null &
    ODOM_PID=$!
    # Liveness check: a wedged CLI produces a silent echo that looks
    # identical to a healthy one until the run is over. One restart.
    sleep 8
    if [ "$(wc -l < "$OUT/odom_run$RUN.csv" 2>/dev/null)" -lt 10 ]; then
      echo "  !! odom echo silent after 8 s -- restarting it once"
      kill "$ODOM_PID" 2>/dev/null
      pkill -9 -f 'ros2cli.daemon' 2>/dev/null
      setsid ros2 topic echo /sim/base_odom --csv \
          > "$OUT/odom_run$RUN.csv" 2>/dev/null < /dev/null &
      ODOM_PID=$!
    fi

    echo "  gait running; capturing for ${GAIT_WALL}s wall"
    sleep "$GAIT_WALL"

    kill "$ODOM_PID" 2>/dev/null
    [ -n "$IMU_PID" ] && kill "$IMU_PID" 2>/dev/null
    cp /tmp/corgi_torque_terms.csv "$OUT/run$RUN.csv"
    cp "/tmp/ma_ctl_${COND}_$RUN.log" "$OUT/ctl_run$RUN.log" 2>/dev/null
    # Scheduler event CSV (present only on event_sched runs with
    # CORGI_SCHED_DEBUG=1; the controller flushes as it writes, so a
    # teardown kill loses at most the last block).
    [ -f /tmp/corgi_sched_events.csv ] \
        && cp /tmp/corgi_sched_events.csv "$OUT/sched_run$RUN.csv"
    SIMT=$(tail -1 "$OUT/run$RUN.csv" | cut -d, -f1)
    echo "  saved: torque $(wc -l < "$OUT/run$RUN.csv") rows (sim t=$SIMT), " \
         "odom $(wc -l < "$OUT/odom_run$RUN.csv") rows"

    # Engagement asserts. A camber command that silently failed to engage
    # looks exactly like "camber does not curve the path".
    case "$ARGS" in
      *gamma_acker_dir:=*)
        if fresh_grep "/tmp/ma_ctl_${COND}_$RUN.log" 'ACKER CAMBER set'; then
          echo "  ACKER CONFIRMED:" \
               "$(grep -o 'ACKER CAMBER set: [^"]*' "/tmp/ma_ctl_${COND}_$RUN.log" | head -1)"
        else
          echo "  !! ACKER requested but never announced itself -- run INVALID"
        fi ;;
      *)
        if fresh_grep "/tmp/ma_ctl_${COND}_$RUN.log" 'ACKER CAMBER set'; then
          echo "  !! baseline run logged ACKER engagement -- run INVALID"
        fi ;;
    esac
    case "$ARGS" in
      *stance_label_shift_s:=*)
        if fresh_grep "/tmp/ma_ctl_${COND}_$RUN.log" 'STANCE LABELS SHIFTED'; then
          echo "  SHIFT CONFIRMED:" \
               "$(grep -o 'STANCE LABELS SHIFTED by [^"]*' "/tmp/ma_ctl_${COND}_$RUN.log" | head -1)"
        else
          echo "  !! label shift requested but never announced -- run INVALID"
        fi ;;
    esac
    case "$ARGS" in
      *event_sched:=true*|*event_sched:=True*)
        if fresh_grep "/tmp/ma_ctl_${COND}_$RUN.log" 'EVENT SCHED ENGAGED'; then
          echo "  SCHED CONFIRMED:" \
               "$(grep -o 'EVENT SCHED ENGAGED: [^"]*' "/tmp/ma_ctl_${COND}_$RUN.log" | head -1)"
        else
          echo "  !! event_sched requested but never ENGAGED -- run INVALID"
          grep -o 'EVENT SCHED REFUSED: [^"]*' "/tmp/ma_ctl_${COND}_$RUN.log" | head -2
        fi ;;
      *)
        if fresh_grep "/tmp/ma_ctl_${COND}_$RUN.log" 'EVENT SCHED ENGAGED'; then
          echo "  !! non-scheduler run logged EVENT SCHED engagement -- run INVALID"
        fi ;;
    esac
    case "$ARGS" in
      # NOTE: an explicit couple_gain:=0.0 would false-negative here; pass
      # no couple_gain at all for coupling-off runs. (The first pattern
      # attempt, couple_gain:=0.0*, glob-matched 0.002 and silently
      # SKIPPED the assert -- caught 2026-08-20.)
      *couple_gain:=*)
        if fresh_grep "/tmp/ma_ctl_${COND}_$RUN.log" 'PHASE COUPLING ENGAGED'; then
          echo "  COUPLING CONFIRMED:" \
               "$(grep -o 'PHASE COUPLING ENGAGED: [^"]*' "/tmp/ma_ctl_${COND}_$RUN.log" | head -1)"
        else
          echo "  !! couple_gain requested but never ENGAGED -- run INVALID"
        fi ;;
    esac
    case "$ARGS" in
      *k_pitch:=*|*d_pitch:=*)
        if fresh_grep "/tmp/ma_ctl_${COND}_$RUN.log" 'PITCH CHANNEL set'; then
          echo "  PITCH CHANNEL CONFIRMED:" \
               "$(grep -o 'PITCH CHANNEL set: [^\"]*' "/tmp/ma_ctl_${COND}_$RUN.log" | head -1)"
        else
          echo "  !! pitch channel requested but never announced -- run INVALID"
        fi ;;
    esac
    case "$ARGS" in
      *k_flight:=*|*b_flight:=*)
        if fresh_grep "/tmp/ma_ctl_${COND}_$RUN.log" 'FLIGHT GAINS set'; then
          echo "  FLIGHT GAINS CONFIRMED:" \
               "$(grep -o 'FLIGHT GAINS set: [^\"]*' "/tmp/ma_ctl_${COND}_$RUN.log" | head -1)"
        else
          echo "  !! k_flight/b_flight requested but never announced -- run INVALID"
        fi ;;
    esac
    case "$ARGS" in
      *push_sync:=true*|*push_sync:=True*)
        if fresh_grep "/tmp/ma_ctl_${COND}_$RUN.log" 'PUSH SYNC ENGAGED'; then
          echo "  PUSHSYNC CONFIRMED:" \
               "$(grep -o 'PUSH SYNC ENGAGED: [^"]*' "/tmp/ma_ctl_${COND}_$RUN.log" | head -1)"
          echo "  PUSHSYNC RELEASES:" \
               "$(grep -o 'PUSH SYNC RELEASES: [^"]*' "/tmp/ma_ctl_${COND}_$RUN.log" | head -1)"
        else
          echo "  !! push_sync requested but never ENGAGED -- run INVALID"
          grep -o 'PUSH SYNC REFUSED: [^"]*' "/tmp/ma_ctl_${COND}_$RUN.log" | head -2
        fi ;;
    esac
    case "$ARGS" in
      *slave_stance:=true*|*slave_stance:=True*)
        if fresh_grep "/tmp/ma_ctl_${COND}_$RUN.log" 'SLAVE STANCE ENGAGED'; then
          echo "  SLAVE CONFIRMED:" \
               "$(grep -o 'SLAVE STANCE ENGAGED: [^"]*' "/tmp/ma_ctl_${COND}_$RUN.log" | head -1)"
        else
          echo "  !! slave_stance requested but never ENGAGED -- run INVALID"
          grep -o 'SLAVE STANCE REFUSED: [^"]*' "/tmp/ma_ctl_${COND}_$RUN.log" | head -2
        fi ;;
    esac
    case "$ARGS" in
      *turn_rate:=*|*steer_offset:=*|*k_steer_yaw:=*)
        if fresh_grep "/tmp/ma_ctl_${COND}_$RUN.log" 'Turn: turn_rate'; then
          echo "  TURN CONFIRMED:"                "$(grep -o 'Turn: turn_rate[^"]*' "/tmp/ma_ctl_${COND}_$RUN.log" | head -1)"
          echo "  STEER CONFIRMED:"                "$(grep -o 'Steering: k_steer[^"]*' "/tmp/ma_ctl_${COND}_$RUN.log" | head -1)"
        else
          echo "  !! turn/steer requested but never announced -- run INVALID"
        fi ;;
    esac
    case "$ARGS" in
      *gamma_yaw_limit:=*)
        if fresh_grep "/tmp/ma_ctl_${COND}_$RUN.log" 'GENTLE YAW CLAMP set'; then
          echo "  CLAMP CONFIRMED:" \
               "$(grep -o 'GENTLE YAW CLAMP set: [^"]*' "/tmp/ma_ctl_${COND}_$RUN.log" | head -1)"
        else
          echo "  !! yaw clamp requested but never announced -- run INVALID"
        fi ;;
    esac
  done
done

teardown_all
verify_dead || true
echo
echo "campaign captures under $BASE_OUT:"
ls -la "$BASE_OUT"/*/ 2>/dev/null
