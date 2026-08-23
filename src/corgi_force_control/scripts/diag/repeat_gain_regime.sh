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
# First run number to write. Lets a sweep call this once per
# (cell, repetition) and still produce run1..runN per cell, which is
# what an INTERLEAVED campaign needs. Interleaving costs nothing here
# because every run already tears down and relaunches the whole stack.
RUN_START=${RUN_START:-1}
GAIT_WALL=${GAIT_WALL:-200}   # HARD TIMEOUT only when GAIT_SIM is set
# Seconds of SIM time to capture per run. 0 = old wall-clock behaviour,
# bit-identical, so no existing harness changes. New campaigns should set it:
# wall-clock windows make the capture a property of the machine's load rather
# than of the experiment, which cost S161 its cross-cell comparability.
GAIT_SIM=${GAIT_SIM:-0}
SPAN_REF=""
SETTLE_WALL=${SETTLE_WALL:-45}

start_captures() {
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

  # CENTRE-OF-MASS ground truth, for alpha (RECORD_COM=1, log S180). Same
  # column layout as base_odom -- it is the same message type -- so
  # load_odom_xyzt reads it unchanged. The driver only publishes this when
  # CORGI_PUBLISH_COM=1, so the caller must set BOTH; if it sets only this one
  # the echo produces an empty file rather than a wrong one, and the
  # `COM PUBLISH: ON` line in the driver log is the proof of intent.
  if [ -n "${RECORD_COM:-}" ]; then
    setsid ros2 topic echo /sim/com_odom --csv \
        > "$OUTDIR/com_run${RUN}.csv" 2>/dev/null < /dev/null &
    COM_PID=$!
  fi
}

# Extra launch args for the controller, e.g. CTL_ARGS="contact_gated_gains:=true".
# Kept as one string so the A/B pair differs in exactly one place and the
# difference is visible in the captured log.
CTL_ARGS=${CTL_ARGS:-}
# k_tangential engagement check (see the block after the RUN loop). KT_REF_DIR
# is a REFERENCE ARM's OUTDIR -- it must be an arm run at the shipped default,
# because that is the one arm whose own engagement cannot silently fail
# ("ignored" and "applied" are the same gains there). Set by the sweep script.
KT_REF_DIR=${KT_REF_DIR:-}
KT_REF=${KT_REF:-600.0}

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

for RUN in $(seq "$RUN_START" "$((RUN_START + N - 1))"); do
  echo "################ RUN $RUN / $N ################"
  teardown
  rm -f /tmp/corgi_torque_terms.csv

  export TMPDIR=/mnt/c/Users/alexc/AppData/Local/Temp/webots_ws
  export CORGI_EXPERIMENT_MODE=1
  export CORGI_TORQUE_DEBUG=1
  # THE CONFIG OF RECORD IS TWO ENV VARS, NOT JUST THE LAUNCH GAINS.
  #
  # Added 2026-08-22 (S157) after a whole factorial ran without them. Of the 27
  # harnesses in this directory, ONLY menger_acker_campaign.sh set these -- so
  # every campaign run through THIS script, including the entire banked
  # kt_sweep, ran a DIFFERENT PLANT from the one the config of record names.
  #
  # CORGI_DIRBETA_TRANSFORM mirrors beta on the right leg pair
  # (corgi_driver.py:685). Without it two legs sweep the wrong way, and the
  # measured consequence is severe: the v070 baseline went BACKWARD at
  # -0.248 m/s here against +0.284 forward on the same template and gains
  # under menger_acker_campaign.sh.
  #
  # Override with CORGI_DIRBETA_TRANSFORM=0 to deliberately run the old plant,
  # but say so in the campaign's own header if you do.
  export CORGI_DIRBETA_TRANSFORM=${CORGI_DIRBETA_TRANSFORM:-1}
  export CORGI_THETA_STOP=${CORGI_THETA_STOP:-1}
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

  # PRE_SETTLE_ODOM=1 starts the odom capture BEFORE the settle sleep, so the
  # recording covers the ramp and the first seconds of gait instead of opening
  # ~7 s of sim time in (log S195/S196). That blind spot is why the yaw
  # collapse has no mechanism: BOTH collapsed runs on record are already at
  # v_fwd ~ 0 in the first window of every capture, so nothing observable
  # precedes the failure and no metric measured over the band can predict it.
  # Costs ~7 s of extra CSV per run (~0.4 MB) and nothing else.
  #
  # Default OFF: existing harnesses stay bit-identical, the same contract
  # GAIT_SIM=0 keeps above.
  if [ -n "${PRE_SETTLE_ODOM:-}" ]; then
    echo "  PRE-SETTLE CAPTURE: odom starts before settle (S196)"
    start_captures
  fi

  sleep "$SETTLE_WALL"

  if [ -z "${PRE_SETTLE_ODOM:-}" ]; then
    start_captures
  fi


  # ---- CAPTURE WINDOW: sim time, not wall time -----------------------------
  #
  # WHY THIS EXISTS (2026-08-22). GAIT_WALL is WALL clock, and it buys however
  # much sim time the machine happens to deliver. Webots here runs ~14x slower
  # than real time and that factor is not constant: it collapses under any
  # concurrent load. In S161 it did, and the campaign captured 27-31 s of sim
  # in its FIRST cell and only 10-13 s in the other three.
  #
  # That is not a small blemish. Every shipped analyser uses TAIL_S = 20 s, so
  # cell 1 was scored on the settled tail of a 30 s run and cells 2-4 on their
  # ENTIRE capture including the entry transient -- and the shortfall lands on
  # the LATER cells, which is exactly where a sweep puts its treatment arms.
  #
  # GAIT_SIM (seconds of SIM time) makes the window a property of the
  # experiment instead of a property of the machine. GAIT_WALL stays as the
  # hard timeout: a run that cannot reach the target says so loudly rather than
  # being silently truncated.
  #
  # Default GAIT_SIM=0 keeps the old wall-clock behaviour bit-identical, so no
  # existing harness changes under it. New campaigns should set it.
  TQ=${CORGI_TORQUE_DEBUG_PATH:-/tmp/corgi_torque_terms.csv}
  simt() { tail -1 "$TQ" 2>/dev/null | cut -d, -f1; }
  W0=$(date +%s)
  S0=$(simt)
  case "$S0" in ''|*[!0-9.]*) S0=0 ;; esac

  if [ "${GAIT_SIM:-0}" != "0" ]; then
    echo "  gait running; capturing ${GAIT_SIM}s of SIM time (wall timeout ${GAIT_WALL}s)"
    SHORT=""
    while :; do
      SN=$(simt); case "$SN" in ''|*[!0-9.]*) SN=$S0 ;; esac
      WN=$(( $(date +%s) - W0 ))
      if awk -v a="$SN" -v b="$S0" -v g="$GAIT_SIM" 'BEGIN{exit !(a-b >= g)}'; then
        break
      fi
      if [ "$WN" -ge "$GAIT_WALL" ]; then
        SHORT=1
        break
      fi
      sleep 2
    done
  else
    echo "  gait running; capturing for ${GAIT_WALL}s wall (GAIT_SIM unset)"
    sleep "$GAIT_WALL"
    SHORT=""
  fi

  S1=$(simt); case "$S1" in ''|*[!0-9.]*) S1=$S0 ;; esac
  W1=$(( $(date +%s) - W0 ))
  SPAN=$(awk -v a="$S1" -v b="$S0" 'BEGIN{printf "%.2f", a-b}')
  RTF=$(awk -v s="$SPAN" -v w="$W1" 'BEGIN{printf "%.3f", (w>0)? s/w : 0}')
  echo "  captured ${SPAN}s sim in ${W1}s wall (real-time factor ${RTF})"
  if [ -n "$SHORT" ]; then
    echo "  !! run $RUN hit the WALL TIMEOUT before ${GAIT_SIM}s of sim."
    echo "  !! Only ${SPAN}s captured. Something is loading the machine."
  fi

  # Cross-run comparability. A sweep is only a sweep if every cell got the same
  # window; S161 lost that and nobody noticed, because `sim t = X` was printed
  # every run and read by no one.
  # Persisted in FILES, not a shell variable: an interleaved campaign calls
  # this script once per run, so an in-process SPAN_REF resets every time and
  # the check silently never fires. Two references are kept -- this cell's own
  # first run, and the CAMPAIGN's first run, which is the cross-cell drift
  # S161 actually suffered and which a per-cell reference cannot see.
  CELL_REF_F="$OUTDIR/.span_ref"
  CAMP_REF_F="$(dirname "$OUTDIR")/.span_ref"
  [ -f "$CELL_REF_F" ] && SPAN_REF=$(cat "$CELL_REF_F") || SPAN_REF=""
  [ -f "$CAMP_REF_F" ] && CAMP_REF=$(cat "$CAMP_REF_F") || CAMP_REF=""
  if [ -n "$CAMP_REF" ] && awk -v a="$SPAN" -v b="$CAMP_REF" \
       'BEGIN{d=(a-b)/b; if(d<0)d=-d; exit !(d>0.20)}'; then
    echo "  !! run $RUN captured ${SPAN}s against the CAMPAIGN's first run"
    echo "  !! (${CAMP_REF}s) -- more than 20% apart. Cells run at different"
    echo "  !! throughput are NOT a matched comparison (S166). Find the load."
  fi
  [ -z "$CAMP_REF" ] && printf '%s' "$SPAN" > "$CAMP_REF_F"
  if [ -z "$SPAN_REF" ]; then
    printf '%s' "$SPAN" > "$CELL_REF_F"
    echo "  (reference span for this cell: ${SPAN}s)"
  elif awk -v a="$SPAN" -v b="$SPAN_REF" 'BEGIN{d=(a-b)/b; if(d<0)d=-d; exit !(d>0.20)}'; then
    echo "  !! run $RUN captured ${SPAN}s against run 1's ${SPAN_REF}s -- more"
    echo "  !! than 20% apart. These runs are NOT a matched comparison. Find"
    echo "  !! the load before trusting anything that pools them."
  fi

  if [ -n "${RECORD_ODOM:-}" ]; then
    kill "$ODOM_PID" 2>/dev/null
    echo "  odom saved: $(wc -l < "$OUTDIR/odom_run${RUN}.csv") rows"
  fi

  if [ -n "${RECORD_COM:-}" ]; then
    kill "$COM_PID" 2>/dev/null
    COM_ROWS=$(wc -l < "$OUTDIR/com_run${RUN}.csv" 2>/dev/null || echo 0)
    echo "  com  saved: $COM_ROWS rows"
    # An EMPTY com capture is the tell that CORGI_PUBLISH_COM was not set on
    # the sim side -- the echo succeeds, the topic just never publishes. Say so
    # loudly here rather than letting an analyser discover it later.
    if [ "$COM_ROWS" -lt 100 ]; then
      echo "  !! com_run${RUN}.csv has only $COM_ROWS rows. The driver only"
      echo "  !! publishes sim/com_odom when CORGI_PUBLISH_COM=1 -- check the"
      echo "  !! 'COM PUBLISH:' line in the sim log. RECORD_COM alone is not"
      echo "  !! enough; alpha-on-CoM cannot be computed from this run."
    fi
  fi

  # WHICH TEMPLATE DID IT ACTUALLY LOAD?
  #
  # S159: omitting template_path does NOT give v070. The controller falls back
  # to <share>/config/gslip_pronk_template.csv -- the v~1.20 template, 225 rows,
  # 0.2249 s stride, beta sweep +0.6357 -- a gait for 1.92 m/s on a plant that
  # does ~0.28. A whole factorial was invalidated by this before anyone noticed,
  # because nothing announced it. The controller does print the stride at
  # startup; this surfaces it, and shouts when no template was named.
  TPL_LINE=$(grep -o 'template rows ([0-9.]* s stride)' "/tmp/ctl_run$RUN.log"              2>/dev/null | head -1)
  case "$CTL_ARGS" in
    *template_path:=*)
      echo "  template: ${TPL_LINE:-<not announced>} (named explicitly)" ;;
    *)
      echo "  ?? run $RUN: NO template_path was passed, so the controller used"
      echo "  ?? its DEFAULT -- ${TPL_LINE:-<not announced>}. That default is the"
      echo "  ?? v~1.20 template (0.2249 s stride), NOT v070 (0.2662 s)."
      echo "  ?? If this campaign meant v070, its runs are INVALID." ;;
  esac
  # QUARANTINE A SHORT RUN, do not just complain about it.
  #
  # A run that hit the wall timeout before reaching GAIT_SIM used to be copied
  # to run$RUN.csv exactly like a healthy one. Every analyser globs
  # run[0-9].csv and masks `t >= t.max() - 20.0`, so a 12 s capture is scored
  # on its ENTIRE file including standup and the pre-trigger hold, and none of
  # their Unfit guards catch it -- which is precisely how §161 produced four
  # cells that were not a comparison. Printing "!!" and saving it anyway means
  # the warning scrolls past and the number still lands in the median.
  #
  # Renaming is the established quarantine here (the suspend-invalid pattern):
  # the run[0-9].csv glob drops it automatically, and the file is still on disk
  # for anyone who wants to look.
  SIMT=$(tail -1 /tmp/corgi_torque_terms.csv | cut -d, -f1)
  if [ -n "$SHORT" ]; then
    DEST="$OUTDIR/run${RUN}_short_invalid.csv"
    cp /tmp/corgi_torque_terms.csv "$DEST"
    [ -n "${RECORD_ODOM:-}" ] && mv "$OUTDIR/odom_run${RUN}.csv" \
        "$OUTDIR/odom_run${RUN}_short_invalid.csv" 2>/dev/null
    [ -n "${RECORD_COM:-}" ] && mv "$OUTDIR/com_run${RUN}.csv" \
        "$OUTDIR/com_run${RUN}_short_invalid.csv" 2>/dev/null
    echo "  !! run $RUN QUARANTINED as ${DEST##*/}: it captured ${SPAN}s of"
    echo "  !! sim against the ${GAIT_SIM}s target, so its 20 s tail would be"
    echo "  !! the whole run including standup. The analysers' run[0-9].csv"
    echo "  !! glob will skip it. Re-run this cell on an idle machine."
  else
    cp /tmp/corgi_torque_terms.csv "$OUTDIR/run${RUN}.csv"
    echo "  run $RUN saved: $(wc -l < "$OUTDIR/run${RUN}.csv") rows, sim t = $SIMT"
  fi
  # Bank the logs beside the capture. Every engagement banner lives in them
  # and /tmp/ctl_run$RUN.log is overwritten by the next cell, so evidence for
  # cell 1 is gone by the time cell 4 is analysed.
  cp "/tmp/ctl_run$RUN.log" "$OUTDIR/ctl_run${RUN}.log" 2>/dev/null || true
  cp "/tmp/sim_run$RUN.log" "$OUTDIR/sim_run${RUN}.log" 2>/dev/null || true
  # Did the plant run at the TEMPLATE's rate? See playback_ratio.py's header.
  # Reported, not gated: a bad ratio can be controller starvation OR the leg
  # refusing to follow (S153 / Open Issue #21), and this cannot tell them
  # apart. Runs between runs, so it competes with nothing.
  if [ -z "$SHORT" ] && [ -f "$OUTDIR/run${RUN}.csv" ]; then
    python3 "$(dirname "$0")/playback_ratio.py" "$OUTDIR/run${RUN}.csv" \
        2>/dev/null | sed -n 's/^      -> /  playback: /p'
  fi
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
    *k_tangential:=*)
      # PROOF OF INTENT only. The controller announces its leg-frame gains at
      # startup (added 2026-08-22) so a launch-plumbing failure shows up in
      # run 1 instead of after ten. It proves the parameter was READ; the
      # engagement CHECK after the loop proves it reached the impedance law.
      # Compare the node's REPORTED gain against the REQUESTED one, rather
      # than keying on the OVERRIDDEN line. That line only prints when the
      # value differs from the launch default, so an arm that legitimately
      # requests the default (600) produces no line -- and the first version
      # of this check read that absence as failure and fired INVALID on the
      # reference arm. Fixed 2026-08-22 after it did exactly that.
      WANT=$(printf '%s' "$CTL_ARGS" | sed -n 's/.*k_tangential:=\([0-9.]*\).*/\1/p')
      GOT=$(grep -o 'k_tangential=[0-9.]*' "/tmp/ctl_run$RUN.log" | head -1             | cut -d= -f2)
      if [ -n "$GOT" ] && [ -n "$WANT" ] &&          awk -v a="$WANT" -v b="$GOT" 'BEGIN{exit !(a-b<0.01 && b-a<0.01)}'; then
        echo "  k_tangential read OK in run $RUN: requested $WANT, node reports $GOT"
      elif [ -n "$GOT" ]; then
        echo "  !! run $RUN: requested k_tangential $WANT but the node reports"
        echo "  !! $GOT -- the launch argument did not reach it. INVALID."
      else
        echo "  ?? run $RUN: no LEG-FRAME GAINS line at all -- controller"
        echo "  ?? binary predates 2026-08-22. Cannot certify from the log;"
        echo "  ?? the data check after the loop still applies."
      fi
      ;;
  esac
  case "$CTL_ARGS" in
    *clock_ff_scale:=*)
      # The clocked-torque feedforward (Lu & Lin eq 11's D term, log S164).
      # TWO asserts, because they catch different failures and the second is
      # the one S151 says actually counts.
      #
      # 1. PROOF OF INTENT -- gslip_pronk's banner. Compare the node's
      #    REPORTED scale against the REQUESTED one rather than keying on the
      #    banner's presence, the same fix the k_tangential assert above needed
      #    after it fired INVALID on its own reference arm.
      # 2. PROOF OF ACTION -- force_control's CLOCK FF ACTIVE line, which
      #    prints the tau_beta the impedance law JUST COMPUTED from the live
      #    message and the measured pose. A non-zero value there is the
      #    evidence; the banner is not. S138's apex channel printed its
      #    parameters faithfully for ten runs and never fired.
      WANT=$(printf '%s' "$CTL_ARGS" | sed -n 's/.*clock_ff_scale:=\([0-9.]*\).*/\1/p')
      GOT=$(grep -o 'CLOCK FEEDFORWARD: scale=[0-9.]*' "/tmp/ctl_run$RUN.log" \
            | head -1 | cut -d= -f2)
      if [ -n "$GOT" ] && [ -n "$WANT" ] && \
         awk -v a="$WANT" -v b="$GOT" 'BEGIN{exit !(a-b<0.001 && b-a<0.001)}'; then
        echo "  clock_ff read OK in run $RUN: requested $WANT, node reports $GOT"
        echo "    $(grep -o 'CLOCK FEEDFORWARD: .*' "/tmp/ctl_run$RUN.log" | head -1)"
      elif [ -n "$GOT" ]; then
        echo "  !! run $RUN: requested clock_ff_scale $WANT but the node reports"
        echo "  !! $GOT -- the launch argument did not reach it. INVALID."
      else
        echo "  ?? run $RUN: no CLOCK FEEDFORWARD line -- controller binary"
        echo "  ?? predates 2026-08-22. Cannot certify from the log."
      fi
      # PROOF OF ACTION. force_control prints this line ONLY when the term is
      # live (dbeta_ref != 0), so the line's PRESENCE is the evidence and its
      # ABSENCE on a scale>0 arm is the failure. Take the largest |tau_beta|
      # seen, not `head -1`:
      #   - the line is throttled to 5 s of SIM time and lands at an arbitrary
      #     gait phase, so one sample is a lottery;
      #   - with clock_ff_phase=stance, 59.4% of the v070 cycle is flight,
      #     where the term is legitimately absent.
      # An earlier version read head -1 and would have marked a correctly
      # engaged arm INVALID about three times in five.
      TAU=$(grep -o 'tau_beta=[+-][0-9.]*' "/tmp/ctl_run$RUN.log" \
            | cut -d= -f2 | awk 'function abs(x){return x<0?-x:x}
                                 {if (abs($1) > m) {m = abs($1); v = $1}}
                                 END{if (NR) print v}')
      NTAU=$(grep -c 'CLOCK FF ACTIVE' "/tmp/ctl_run$RUN.log" 2>/dev/null)
      if awk -v s="$WANT" 'BEGIN{exit !(s <= 0.001)}'; then
        # The off arm. Silence is the CORRECT result here, and a line would
        # mean the feedforward fired on an arm that asked for none.
        if [ "${NTAU:-0}" = "0" ]; then
          echo "  clock_ff correctly SILENT in run $RUN (scale $WANT, off arm)"
        else
          echo "  !! run $RUN: scale $WANT but force_control printed CLOCK FF"
          echo "  !! ACTIVE $NTAU times -- the off arm is not off. INVALID."
        fi
      elif [ -z "$TAU" ]; then
        echo "  !! run $RUN: scale $WANT but force_control never printed"
        echo "  !! CLOCK FF ACTIVE. The feedforward did not reach the"
        echo "  !! impedance law. INVALID."
      elif awk -v t="$TAU" 'BEGIN{exit !(t > 0.001 || t < -0.001)}'; then
        echo "  clock_ff ACTION CONFIRMED in run $RUN: peak |tau_beta| = $TAU N.m" \
             "over $NTAU throttled samples, at scale $WANT"
      else
        echo "  !! run $RUN: scale $WANT but every CLOCK FF ACTIVE sample read"
        echo "  !! tau_beta=$TAU. Intent and action disagree -- arm INVALID."
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

# ---------------------------------------------------------------------------
# k_tangential engagement, PROOF OF ACTION. Unlike the four asserts inside the
# loop this one is per-ARM: k_tangential is constant within a run, so it is
# perfectly collinear with arm identity and there is no within-run contrast to
# fit. The check is a DIFFERENCE in stance-mode kp against a reference arm,
# predicted from the pose-corrected geometry (0.5*L(theta))^2, and it can only
# run once all N runs are on disk. Validated on the banked kt_sweep: three
# arms confirm at ratio 1.02-1.05, an arm compared against itself comes back
# INVALID at 0.000, and an undetectable pair comes back UNCERTIFIABLE.
# ---------------------------------------------------------------------------
case "$CTL_ARGS" in
  *k_tangential:=*)
    KT_VAL=$(printf '%s' "$CTL_ARGS" | sed -n 's/.*k_tangential:=\([0-9.]*\).*/\1/p')
    if [ -z "$KT_VAL" ]; then
      echo "  ?? k_tangential requested but its value could not be parsed from"
      echo "  ?? CTL_ARGS -- engagement UNCERTIFIABLE, arm NOT invalid."
    elif [ -z "$KT_REF_DIR" ]; then
      echo "  ?? no KT_REF_DIR set -- engagement UNCERTIFIABLE, arm NOT invalid."
      echo "  ?? Set KT_REF_DIR to a reference arm run at the shipped default."
    elif [ "$KT_REF_DIR" = "$OUTDIR" ]; then
      echo "  this IS the reference arm (k_tangential $KT_VAL) -- nothing to"
      echo "  check it against, and nothing that needs checking."
    else
      echo
      echo "--- k_tangential engagement check ---"
      python3 "$WS/src/corgi_force_control/scripts/diag/kt_engagement.py"           --ref "$KT_REF_DIR" --ref-kt "$KT_REF"           --arm "$OUTDIR" --arm-kt "$KT_VAL"
      case $? in
        0) echo "  ENGAGEMENT CONFIRMED -- arm is valid." ;;
        1) echo "  !! ENGAGEMENT FAILED -- TREAT THIS ARM AS INVALID." ;;
        *) echo "  ?? ENGAGEMENT UNCERTIFIABLE -- arm NOT invalid, but this" ;
           echo "  ?? campaign cannot claim the gain was applied." ;;
      esac
    fi
    ;;
esac

teardown
echo
echo "captures in $OUTDIR:"
ls -la "$OUTDIR"
