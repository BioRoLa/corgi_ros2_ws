#!/usr/bin/env bash
# STAGE 3 CAMBERED GATE -- sustained arcs matching the template's sagittal
# state. Gate text: Thesis Timeline, Stage 3, "The gate, defined" (2026-08-23).
#
# ############################## DRAFT ######################################
# This harness is written BEFORE the campaign that uses it is registered. It
# REFUSES to run unless REGISTERED_SECTION is set to the log S-number of the
# registration entry, so an unregistered campaign cannot be started by
# accident. Two parameters are deliberately left without defaults and must
# come from the measured lambda -> kappa map (S214) and the registration that
# chose them (S215): CAM_LAM_DEG and CAM_DIR. NOTE, corrected 2026-08-23: this
# comment originally cited "S202's matched-turn result (S203) -- the lambda
# and direction that hit kappa ~0.33". That was a forward reference written
# while S202 was still running; S202 then FAILED its matching bar (the result
# is S208, and it recommends the OPPOSITE direction), and the map (S214) showed
# kappa(lambda) is a step, so no fixed lambda hits 0.33 at all (S218). The gate
# ran at lambda 14, dir +1, from S214.
# ###########################################################################
#
# WHAT IT ASKS. >= 5 VALID arcs of <= 8 attempts at ONE cell. An arc is a run
# whose steady band [t0+12, end] sustains >= 180 deg of heading change with a
# coherent path, with v_fwd >= 0.10 in EVERY 4 s window (no absorbing
# collapse), v_fwd >= 0.235 (0.85 x the lambda=0 baseline), the S152 screen
# (beta_TD, forward fraction, touchdowns), and the theta period within +-5%
# of the template's 0.2642 s (also the playback check). One conjunctive
# validity rule, one consequence. score_stage3_gate.py owns every clause.
#
# WHY 50 s OF SIM. At kappa 0.33 and the record speed, 180 deg is ~9.5 m and
# ~32 s of turning AFTER the 12 s settle -> a 44 s band minimum; 50 gives
# margin. That is 1.8x the longest sustained turn ever demonstrated (S129,
# 18 s), which is the point: the gate tests SUSTAINED cambered running.
#
# NOT a camber-vs-anything comparison. Single cell, no differential arm: the
# comparison is S202's business. This asks only "does the cambered pronk hold
# a 180 deg arc while still being the v070 pronk?"
#
# COST. 8 x (~50 s sim at ~1/14 real time + launch + settle) ~= 8 x 17 min
# ~= 2.3 h exclusive simulator. Announce before starting.
#
# No `set -u` -- ROS setup scripts reference unbound variables.

WS=~/corgi_ws/corgi_ros2_ws
DIAG="$WS/src/corgi_force_control/scripts/diag"
CFG="$WS/src/corgi_force_control/config"

# ---- the registration guard ------------------------------------------------
case "${REGISTERED_SECTION:-}" in
  ''|*[!0-9]*)
    echo "!! REFUSING: this campaign is not registered. Write the registration"
    echo "!! entry in the implementation log FIRST (predictions, cost, decision"
    echo "!! rule -- S126), then run with REGISTERED_SECTION=<its S-number>."
    exit 1 ;;
esac
[ -n "${SCREEN_BETA_TD:-}" ] && [ -n "${SCREEN_BETA_TOL:-}" ] && [ -n "${SCREEN_FWD_LO:-}" ] && [ -n "${SCREEN_FWD_HI:-}" ] || {
  echo "!! REFUSING: the S152-screen bands (SCREEN_BETA_TD/_TOL, SCREEN_FWD_LO/_HI) have"
  echo "!! NO defaults. A dry run on banked lambda=15 showed the lambda=10 band"
  echo "!! (-0.084 +- 0.006) fails 2 of 5 cambered runs FOR BEING CAMBERED (#22's"
  echo "!! trap). Derive them from the +1 cells bracketing this lambda (S215 used"
  echo "!! in the registration entry, then pass them."; exit 1; }
[ -n "${CAM_LAM_DEG:-}" ] && [ -n "${CAM_DIR:-}" ] || {
  echo "!! REFUSING: CAM_LAM_DEG and CAM_DIR have NO defaults. They come from"
  echo "!! the S214 map and the S215 registration -- the lambda and direction on"
  echo "!! this plant. Set both explicitly."; exit 1; }

. "$WS/src/corgi_force_control/scripts/diag/preflight_plant.sh"
preflight_plant || exit 1
# IS THE SIMULATOR FREE, AND QUIET? S203. This script was written after the
# sweep that wired the other 25, so it inherited the plant guard from its
# template but not this one -- the same copy-inheritance gap S203 closed.
. "$WS/src/corgi_force_control/scripts/diag/preflight_sim.sh"
preflight_sim || exit 1
# Does the launch file forward every parameter the node declares? The first
# dip campaign passed gamma_acker_dip:=0.15 and the launch file dropped it
# silently (S128's trap, again); two cells were quarantined before the
# certify grep caught it. Reconciled up front now, for every harness that
# passes node parameters.
. "$WS/src/corgi_force_control/scripts/diag/preflight_launch_args.sh"
preflight_launch_args || exit 1
NPER=${NPER:-8}
GAIT_SIM=${GAIT_SIM:-50}
GAIT_WALL=${GAIT_WALL:-800}
BASE=${BASE:-/home/alexc/corgi_runs/stage3_gate}

TPL_ARG="template_path:=$CFG/gslip_pronk_template_v070.csv"
FLIGHT_ARGS="k_flight:=7150.0 b_flight:=115.8"
ATT_ARGS="k_yaw:=0.0 d_yaw:=0.0"
CAM_LAM_RAD=$(awk -v d="$CAM_LAM_DEG" 'BEGIN{printf "%.5f", d*3.14159265358979/180.0}')
CAM_LAM_FMT=$(printf '%.2f' "$CAM_LAM_DEG")
CAM_ARGS="gamma_acker_in:=$CAM_LAM_RAD gamma_acker_out:=$CAM_LAM_RAD gamma_acker_dir:=$CAM_DIR"

echo "==========================================================="
echo " STAGE 3 CAMBERED GATE -- sustained arcs (registered: S$REGISTERED_SECTION)"
echo "==========================================================="
echo " cell     : $CAM_ARGS   (lambda $CAM_LAM_DEG deg, dir $CAM_DIR)"
echo " both     : $ATT_ARGS $FLIGHT_ARGS"
echo " template : $TPL_ARG"
echo " attempts : $NPER (gate: >= 5 valid arcs of <= 8)"
echo " window   : ${GAIT_SIM}s of SIM time per run (NOT wall), ${GAIT_WALL}s timeout"
echo " base     : $BASE"
echo
echo " AN ARC IS VALID when ALL of (score_stage3_gate.py):"
echo "   arc      >= 180 deg heading change over [t0+12, end]; Kasa fit arc >= 90"
echo "   collapse v_fwd >= 0.10 m/s in EVERY 4 s window of the band"
echo "   speed    band v_fwd >= 0.235 m/s"
echo "   screen   beta_TD and fwd fraction within the bands SET AT REGISTRATION"
echo "            from the +1 cells bracketing this lambda (SCREEN_BETA_TD, SCREEN_BETA_TOL,"
echo "            SCREEN_FWD_LO, SCREEN_FWD_HI -> score_stage3_gate.py); >= 8 TD/leg"
echo "   stride   theta period within +-5% of 0.2642 s (= playback check)"
[ -n "${R_LO:-}" ] && echo "   radius   R_fit within [$R_LO, $R_HI] m  (S217: the Stage 4 radius is a CLAUSE here)"
echo " CONSEQUENCES (Timeline, pre-committed): 5-8 -> Stage 3 cambered CLOSES;"
echo " 3-4 -> not closed, failing clause named, no re-registration; <= 2 -> the"
echo " off-ramp question is asked NOW."
echo

# ---- PREFLIGHT -------------------------------------------------------------
STALE=$(pgrep -f 'Corgi_launch.py|gslip_pronk_node|webots_ros2_driver' 2>/dev/null | wc -l)
[ "$STALE" = 0 ] || { echo "!! stale sim processes -- REFUSING (the simulator is SHARED):";
  pgrep -fa 'Corgi_launch.py|gslip_pronk_node|webots_ros2_driver' | head; exit 1; }
FOREIGN=$(pgrep -f 'usr/local/webots' 2>/dev/null | wc -l)
[ "$FOREIGN" = 0 ] || { echo "!! a Linux-side Webots that is NOT the Corgi sim is running. REFUSING."; exit 1; }
LOAD=$(cut -d' ' -f1 /proc/loadavg)
awk -v l="$LOAD" 'BEGIN{exit !(l > 4.0)}' && { echo "!! load average $LOAD before start. REFUSING."; exit 1; }
echo "stale-launch clean; no foreign Webots; load $LOAD."
if command -v powershell.exe > /dev/null 2>&1; then
  WINWB=$(powershell.exe -NoProfile -Command \
      "@(Get-Process webots* -ErrorAction SilentlyContinue).Count" 2>/dev/null | tr -d '\r\n ')
  case "$WINWB" in
    ''|*[!0-9]*) echo "windows-side Webots check: inconclusive ('$WINWB') -- continuing." ;;
    0) echo "windows-side Webots check: none running." ;;
    *) echo "!! $WINWB WINDOWS-side webots.exe still running (holds port 1234). REFUSING."; exit 1 ;;
  esac
fi
BIN="$WS/install/corgi_force_control/lib/corgi_force_control"
for NEED in 'LEG-FRAME GAINS' 'ATTITUDE GAINS' 'gamma_acker'; do
  [ "$(strings "$BIN/gslip_pronk_node" 2>/dev/null | grep -c "$NEED")" != 0 ] || {
    echo "!! INSTALLED gslip_pronk_node lacks '$NEED'. Rebuild. REFUSING."; exit 1; }
done
[ -f "$CFG/gslip_pronk_template_v070.csv" ] || { echo "!! v070 missing"; exit 1; }
NZ=$(awk -F, 'NR>1 && $4+0 != 0 {n++} END {print n+0}' "$CFG/gslip_pronk_template_v070.csv")
[ "$NZ" = "0" ] || { echo "!! v070 gamma column not identically zero. REFUSING."; exit 1; }
for T in score_stage3_gate.py cross_track.py yaw_excursion.py touchdown_phase.py playback_ratio.py; do
  python3 "$DIAG/$T" --selftest > "/tmp/s3g_$T.out" 2>&1 || {
    echo "!! $T selftest FAILED -- the gate cannot be scored:"; tail -12 "/tmp/s3g_$T.out"; exit 1; }
done
echo "installed controller carries every banner; v070 clean; all five scorer selftests pass."
if [ -d "$BASE" ] && [ -n "$(ls -A "$BASE" 2>/dev/null)" ]; then
  echo "!! $BASE already has content -- a fresh campaign needs a fresh base. REFUSING."; exit 1; fi
mkdir -p "$BASE/cam"
{
  echo "campaign  stage3_gate  registered S$REGISTERED_SECTION  $(date -Iseconds)"
  echo "cam   $CAM_ARGS"
  echo "both  $ATT_ARGS $FLIGHT_ARGS $TPL_ARG"
  echo "attempts $NPER  gait_sim $GAIT_SIM"
} > "$BASE/DESIGN.txt"

certify() {  # certify <ctl_log>
  local LOG=$1 ok=1
  grep -q 'k_flight=7150.0 b_flight=115.8' "$LOG" || { echo "  !! flight gains not 7150/115.8"; ok=0; }
  grep -q 'ATTITUDE GAINS: k_yaw=0.0000' "$LOG"    || { echo "  !! k_yaw not certified 0"; ok=0; }
  grep -q 'Loaded 265 template rows' "$LOG"        || { echo "  !! template not 265 rows"; ok=0; }
  grep -q "ACKER CAMBER set: in=${CAM_LAM_FMT} deg" "$LOG" \
    && echo "  ACKER CONFIRMED: $(grep -o 'ACKER CAMBER set: [^"]*' "$LOG" | head -1)" \
    || { echo "  !! ACKER CAMBER not announced at $CAM_LAM_FMT deg"; ok=0; }
  grep -q 'turn_rate=0.0000' "$LOG" || { echo "  !! a turn_rate is set -- this cell is open-loop camber only"; ok=0; }
  [ "$ok" = 1 ]
}

# ---- ATTEMPTS --------------------------------------------------------------
OUT="$BASE/cam"
for REP in $(seq 1 "$NPER"); do
  echo
  echo "################################################################"
  echo "###  attempt $REP/$NPER -- cambered arc, lambda $CAM_LAM_DEG deg, dir $CAM_DIR"
  echo "###  ON THE RENDER: four legs cambered LEFT/RIGHT, no steer, the path"
  echo "###  curling steadily ONE way for the whole run. A pirouette or a stall"
  echo "###  is a RESULT (an invalid arc), not a harness fault -- not retried."
  echo "################################################################"
  for ATTEMPT in 1 2; do
    N=1 RUN_START=$REP OUTDIR="$OUT" RECORD_ODOM=1 PRE_SETTLE_ODOM=1 \
      GAIT_SIM=$GAIT_SIM GAIT_WALL=$GAIT_WALL \
      CTL_ARGS="$CAM_ARGS $ATT_ARGS $FLIGHT_ARGS $TPL_ARG" \
      bash "$DIAG/repeat_gain_regime.sh"
    if [ -f "$OUT/run$REP.csv" ]; then
      [ "$ATTEMPT" = 2 ] && echo "  (attempt $REP succeeded on RETRY -- cold start, S171 S6)"
      if certify "$OUT/ctl_run$REP.log"; then echo "  attempt $REP CERTIFIED"
      else
        echo "  !! attempt $REP is INVALID -- config not certified. Quarantining (it still counts as an attempt)."
        mv "$OUT/run$REP.csv" "$OUT/run${REP}_uncertified.csv"
        mv "$OUT/odom_run$REP.csv" "$OUT/odom_run${REP}_uncertified.csv" 2>/dev/null
      fi
      break
    fi
    [ "$ATTEMPT" = 1 ] && echo "  !! attempt $REP produced NO CAPTURE. Cold-start mode -- retrying ONCE." \
                        || echo "  !! attempt $REP produced NO CAPTURE twice. Counted as an attempt, not an arc."
  done
done

# ---- SCORE -----------------------------------------------------------------
echo
echo "==========================================================="
echo " SCORE -- score_stage3_gate.py, as registered in S$REGISTERED_SECTION"
echo "==========================================================="
python3 "$DIAG/score_stage3_gate.py" --base "$BASE" \
  --beta-td "${SCREEN_BETA_TD:?set from the registration}" --beta-tol "${SCREEN_BETA_TOL:?}" \
  --fwd-lo "${SCREEN_FWD_LO:?}" --fwd-hi "${SCREEN_FWD_HI:?}" \
  ${R_LO:+--r-lo "$R_LO"} ${R_HI:+--r-hi "$R_HI"}
echo
echo "-- reported, not gated: odom-derived ballistic fraction (#22) -------------"
python3 "$DIAG/flight_vs_camber.py" --ballistic "$OUT" 2>&1 | tail -12
echo
echo "Done. Captures in $BASE. Record the verdict in the log as it stands."
