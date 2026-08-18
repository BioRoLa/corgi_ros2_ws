#!/bin/bash
# Repeat the pronk n times and keep one torque CSV per run, so the gain-regime
# cross-tab can be checked against the documented noise floor.
#
# WHAT IS BEING CONFIRMED. One capture showed that while a foot is actually on
# the ground the controller is running FLIGHT gains 71.2% of the time -- the
# schedule is anti-correlated with contact (28.8% stance gains against a 43.8%
# chance baseline). If that repeats, the virtual spring does not exist for most
# of stance and that is the erosion mechanism. If it does not repeat, it was one
# run's phase accident.
#
# FRESH SIM PER RUN, which is the established practice here (S27) -- run-to-run
# variance in this project has swallowed larger effects than this one, and a
# shared simulator would correlate the arms.
#
# TIMING. The simulator runs at ~10% of real time (measured: a 25 s wall dwell
# advanced sim time 2.6 s), and the controller needs ~18 s wall for clock sync
# plus standup before it holds. GAIT_WALL is therefore wall-clock and generous;
# 200 s buys roughly 20 s of sim time, about 90 strides.
#
# NO `set -u` HERE. The ROS setup scripts reference unbound variables and abort
# under it -- a hazard already recorded in the implementation log, and one this
# script tripped over on its first run regardless.

WS=~/corgi_ws/corgi_ros2_ws
OUTDIR=${OUTDIR:-~/corgi_runs/gain_regime}
N=${N:-3}
GAIT_WALL=${GAIT_WALL:-200}
SETTLE_WALL=${SETTLE_WALL:-45}
# Extra launch args for the controller, e.g. CTL_ARGS="contact_gated_gains:=true".
# Kept as one string so the A/B pair differs in exactly one place and the
# difference is visible in the captured log.
CTL_ARGS=${CTL_ARGS:-}

mkdir -p "$OUTDIR"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

teardown() {
  pkill -f '[C]orgi_launch.py' 2>/dev/null || true
  pkill -f '[g]slip_pronk_node' 2>/dev/null || true
  pkill -f '[f]orce_control_node' 2>/dev/null || true
  pkill -f '[f]orce_estimation_node' 2>/dev/null || true
  pkill -f '[w]ebots_ros2_driver' 2>/dev/null || true
  pkill -f '[w]ebots-controller' 2>/dev/null || true
  pkill -f '[c]orgi_control_panel' 2>/dev/null || true
  pkill -f 'topic pub .*[t]rigger' 2>/dev/null || true
  pkill -f 'topic echo .*[b]ase_odom' 2>/dev/null || true
  /mnt/c/Windows/System32/taskkill.exe /F /IM webots.exe /IM webots-bin.exe >/dev/null 2>&1 || true
  rm -f /dev/shm/fastrtps_* 2>/dev/null || true
  sleep 3
}

for RUN in $(seq 1 "$N"); do
  echo "################ RUN $RUN / $N ################"
  teardown
  rm -f /tmp/corgi_torque_terms.csv

  export TMPDIR=/mnt/c/Users/alexc/AppData/Local/Temp/webots_ws
  export CORGI_EXPERIMENT_MODE=1
  export CORGI_TORQUE_DEBUG=1
  setsid ros2 launch corgi_sim Corgi_launch.py > "/tmp/sim_run$RUN.log" 2>&1 < /dev/null &

  ok=0
  for i in $(seq 1 60); do
    [ -f /tmp/corgi_torque_terms.csv ] && { ok=1; break; }
    sleep 3
  done
  [ "$ok" = 1 ] || { echo "  run $RUN: driver never connected, skipping"; continue; }
  echo "  driver up"

  # shellcheck disable=SC2086  -- CTL_ARGS is intentionally word-split
  setsid ros2 launch corgi_force_control gslip_pronk.launch.py $CTL_ARGS \
      > "/tmp/ctl_run$RUN.log" 2>&1 < /dev/null &

  # Wait for the hold phase rather than guessing: the log line is the signal.
  ok=0
  for i in $(seq 1 40); do
    grep -q 'waiting for trigger' "/tmp/ctl_run$RUN.log" 2>/dev/null && { ok=1; break; }
    sleep 3
  done
  [ "$ok" = 1 ] || { echo "  run $RUN: controller never reached hold, skipping"; continue; }
  echo "  controller holding; triggering"

  setsid ros2 topic pub -r 50 /trigger corgi_msgs/msg/TriggerStamped \
      '{enable: true}' > /dev/null 2>&1 < /dev/null &

  sleep "$SETTLE_WALL"

  # Optional: record body odometry alongside the torque capture, so speed is
  # measured in the SAME run the demand is (RECORD_ODOM=1). Exists because the
  # label-shift conditions change contact duty, and a demand reduction bought
  # with forward speed is not a reduction. Full --csv keeps the Odometry field
  # order fixed by the message spec (stamp at cols 0-1, position at 4-6).
  if [ -n "${RECORD_ODOM:-}" ]; then
    setsid ros2 topic echo /sim/base_odom --csv \
        > "$OUTDIR/odom_run${RUN}.csv" 2>/dev/null < /dev/null &
    ODOM_PID=$!
  fi

  echo "  gait running; capturing for ${GAIT_WALL}s wall"
  sleep "$GAIT_WALL"

  if [ -n "${RECORD_ODOM:-}" ]; then
    kill "$ODOM_PID" 2>/dev/null
    echo "  odom saved: $(wc -l < "$OUTDIR/odom_run${RUN}.csv") rows"
  fi

  cp /tmp/corgi_torque_terms.csv "$OUTDIR/run${RUN}.csv"
  SIMT=$(tail -1 /tmp/corgi_torque_terms.csv | cut -d, -f1)
  echo "  run $RUN saved: $(wc -l < "$OUTDIR/run${RUN}.csv") rows, sim t = $SIMT"
  # Assert the arm actually ran in the condition it claims. A run whose special
  # condition silently failed to engage looks identical to "the change does
  # nothing", which is the exact shape of wrong answer this session kept
  # producing.
  #
  # The check is SPECIFIC to the contact gate, not to CTL_ARGS being set: the
  # first version fired "INVALID" on every arm of a k_tangential sweep, because
  # it assumed any CTL_ARGS meant the gate. Other parameters must be verified
  # from the DATA instead (e.g. the stance-mode kp shifts with k_tangential).
  # Optional sim-side engagement assertion: SIM_ASSERT is a pattern that must
  # appear in the SIM launch log (driver messages land there, not in ctl logs).
  # Exists because a sim-side env flag that silently failed to engage looks
  # identical to "the change does nothing".
  if [ -n "${SIM_ASSERT:-}" ]; then
    if grep -q "$SIM_ASSERT" "/tmp/sim_run$RUN.log"; then
      echo "  sim assert OK: $SIM_ASSERT"
    else
      echo "  !! run $RUN: sim assert FAILED ($SIM_ASSERT) -- arm INVALID"
    fi
  fi
  case "$CTL_ARGS" in
    *stance_label_shift_s:=*)
      if grep -q 'STANCE LABELS SHIFTED' "/tmp/ctl_run$RUN.log"; then
        echo "  label shift CONFIRMED in run $RUN:" \
             "$(grep -o 'SHIFTED by [+-][0-9]* ms' "/tmp/ctl_run$RUN.log" | head -1)"
      else
        echo "  !! run $RUN: shift requested but never announced itself"
        echo "  !! treat this arm as INVALID"
      fi
      ;;
  esac
  case "$CTL_ARGS" in
    *stance_label_duty:=*)
      if grep -q 'STANCE LABEL DUTY set' "/tmp/ctl_run$RUN.log"; then
        echo "  label duty CONFIRMED in run $RUN:" \
             "$(grep -o 'DUTY set to [0-9.]*' "/tmp/ctl_run$RUN.log" | head -1)"
      elif grep -q 'stance_label_duty=.*REFUSED' "/tmp/ctl_run$RUN.log"; then
        echo "  !! run $RUN: duty requested but REFUSED by the controller"
        echo "  !! treat this arm as INVALID"
      else
        echo "  !! run $RUN: duty requested but never announced itself"
        echo "  !! treat this arm as INVALID"
      fi
      ;;
  esac
  case "$CTL_ARGS" in
    *stance_sweep_scale:=*)
      if grep -q 'STANCE SWEEP SCALED' "/tmp/ctl_run$RUN.log"; then
        echo "  sweep scale CONFIRMED in run $RUN:" \
             "$(grep -o 'SCALED by [0-9.]*' "/tmp/ctl_run$RUN.log" | head -1)"
      elif grep -q 'stance_sweep_scale=.*REFUSED' "/tmp/ctl_run$RUN.log"; then
        echo "  !! run $RUN: sweep scale requested but REFUSED by the controller"
        echo "  !! treat this arm as INVALID"
      else
        echo "  !! run $RUN: sweep scale requested but never announced itself"
        echo "  !! treat this arm as INVALID"
      fi
      ;;
  esac
  case "$CTL_ARGS" in
    *contact_gated_gains:=true*)
      if grep -q 'CONTACT-GATED GAINS ON' "/tmp/ctl_run$RUN.log"; then
        echo "  gate CONFIRMED ON in run $RUN"
      else
        echo "  !! run $RUN: gate requested but never announced itself"
        echo "  !! treat this arm as INVALID"
      fi
      if grep -q 'contact gate STALE' "/tmp/ctl_run$RUN.log"; then
        echo "  !! run $RUN: gate went STALE at some point -- partially ungated"
      fi
      ;;
  esac
done

teardown
echo
echo "captures in $OUTDIR:"
ls -la "$OUTDIR"
