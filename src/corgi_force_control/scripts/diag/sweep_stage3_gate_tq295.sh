#!/usr/bin/env bash
# STAGE 3 CAMBERED GATE at the REAL ABAD CEILING -- TQ295 VARIANT of
# sweep_stage3_gate.sh (2026-09-05). Provenance: Alex corrected the gearing
# record on 2026-09-05 -- the ABADs run the HT-04's STOCK 6:1 (29.5 N.m
# stall); the 9:1 was a planned upgrade, never installed. Every prior
# campaign simulated MAX_TORQUE_ABAD=44.25, a gearbox the robot does not
# have. This variant re-runs S215's gate with the clamp the hardware can
# actually deliver. Registered: S326 (which see for predictions and the
# pre-committed consequences).
#
# DIFFERENCES from sweep_stage3_gate.sh, exhaustively:
#   1. REFUSES to run unless CORGI_MAX_TORQUE_ABAD is set (that is the
#      variant's whole point); exports it and records it in DESIGN.txt.
#   2. PINS k_tangential:=600.0 -- S300 moved the launch default to 1200 on
#      2026-08-30; S216 ran at 600 (its ctl banner says so). Unpinned, this
#      campaign would confound the clamp change with a gain change.
#   3. certify() additionally requires the k_tangential=600.0 banner (ctl
#      log) and the driver's torque-ceiling announcement matching the env
#      (sim log) -- S28's lesson: trust the announcement, never the env.
#   4. BASE defaults to ~/corgi_runs/stage3_gate_tq295.
# Everything else -- clauses, screens, scorer, attempts, windows -- is S215
# verbatim. See the original header for the gate's rationale.
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
  echo "!! NO defaults. Derive them from the +1 cells bracketing this lambda"
  echo "!! (S215's registration), then pass them."; exit 1; }
[ -n "${CAM_LAM_DEG:-}" ] && [ -n "${CAM_DIR:-}" ] || {
  echo "!! REFUSING: CAM_LAM_DEG and CAM_DIR have NO defaults. They come from"
  echo "!! the S214 map and the S215 registration. Set both explicitly."; exit 1; }
# TQ295 VARIANT: the clamp is the experiment. No default, no fallback.
case "${CORGI_MAX_TORQUE_ABAD:-}" in
  ''|*[!0-9.]*)
    echo "!! REFUSING: CORGI_MAX_TORQUE_ABAD is not set (or not numeric). This"
    echo "!! variant exists to run the gate at the real 6:1 ceiling -- set it"
    echo "!! explicitly (29.5 for the stock gearbox)."; exit 1 ;;
esac
export CORGI_MAX_TORQUE_ABAD
ABAD_FMT=$(printf '%.2f' "$CORGI_MAX_TORQUE_ABAD")

. "$WS/src/corgi_force_control/scripts/diag/preflight_plant.sh"
preflight_plant || exit 1
. "$WS/src/corgi_force_control/scripts/diag/preflight_sim.sh"
preflight_sim || exit 1
. "$WS/src/corgi_force_control/scripts/diag/preflight_launch_args.sh"
preflight_launch_args || exit 1
NPER=${NPER:-8}
GAIT_SIM=${GAIT_SIM:-50}
GAIT_WALL=${GAIT_WALL:-800}
BASE=${BASE:-/home/alexc/corgi_runs/stage3_gate_tq295}

TPL_ARG="template_path:=$CFG/gslip_pronk_template_v070.csv"
FLIGHT_ARGS="k_flight:=7150.0 b_flight:=115.8"
ATT_ARGS="k_yaw:=0.0 d_yaw:=0.0"
PIN_ARGS="k_tangential:=600.0"   # TQ295 VARIANT: S216's value; launch default moved to 1200 (S300)
CAM_LAM_RAD=$(awk -v d="$CAM_LAM_DEG" 'BEGIN{printf "%.5f", d*3.14159265358979/180.0}')
CAM_LAM_FMT=$(printf '%.2f' "$CAM_LAM_DEG")
CAM_ARGS="gamma_acker_in:=$CAM_LAM_RAD gamma_acker_out:=$CAM_LAM_RAD gamma_acker_dir:=$CAM_DIR"

echo "==========================================================="
echo " STAGE 3 CAMBERED GATE at ABAD CEILING $ABAD_FMT N.m (registered: S$REGISTERED_SECTION)"
echo "==========================================================="
echo " cell     : $CAM_ARGS   (lambda $CAM_LAM_DEG deg, dir $CAM_DIR)"
echo " both     : $ATT_ARGS $FLIGHT_ARGS $PIN_ARGS"
echo " clamp    : CORGI_MAX_TORQUE_ABAD=$CORGI_MAX_TORQUE_ABAD  (stock 6:1 stall 29.5; prior campaigns ran 44.25)"
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
echo "            (SCREEN_BETA_TD, SCREEN_BETA_TOL, SCREEN_FWD_LO, SCREEN_FWD_HI"
echo "            -> score_stage3_gate.py); >= 8 TD/leg"
echo "   stride   theta period within +-5% of 0.2642 s (= playback check)"
[ -n "${R_LO:-}" ] && echo "   radius   R_fit within [$R_LO, $R_HI] m"
echo " CONSEQUENCES: pre-committed in S$REGISTERED_SECTION -- >= 5 valid -> 6:1 suffices for"
echo " the Vicon session; 3-4 -> failing clause named, install if clip-linked;"
echo " <= 2 -> install the 9:1 and re-examine S216's closure."
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
  echo "campaign  stage3_gate_tq295  registered S$REGISTERED_SECTION  $(date -Iseconds)"
  echo "cam   $CAM_ARGS"
  echo "both  $ATT_ARGS $FLIGHT_ARGS $PIN_ARGS $TPL_ARG"
  echo "clamp CORGI_MAX_TORQUE_ABAD=$CORGI_MAX_TORQUE_ABAD"
  echo "attempts $NPER  gait_sim $GAIT_SIM"
} > "$BASE/DESIGN.txt"

certify() {  # certify <ctl_log> <sim_log>
  local LOG=$1 SLOG=$2 ok=1
  grep -q 'k_flight=7150.0 b_flight=115.8' "$LOG" || { echo "  !! flight gains not 7150/115.8"; ok=0; }
  grep -q 'ATTITUDE GAINS: k_yaw=0.0000' "$LOG"    || { echo "  !! k_yaw not certified 0"; ok=0; }
  grep -q 'Loaded 265 template rows' "$LOG"        || { echo "  !! template not 265 rows"; ok=0; }
  # TQ295 VARIANT: the pinned gain must be announced (S300 moved the default)
  grep -q 'k_tangential=600.0 ' "$LOG" || { echo "  !! k_tangential not certified 600.0"; ok=0; }
  grep -q "ACKER CAMBER set: in=${CAM_LAM_FMT} deg" "$LOG" \
    && echo "  ACKER CONFIRMED: $(grep -o 'ACKER CAMBER set: [^"]*' "$LOG" | head -1)" \
    || { echo "  !! ACKER CAMBER not announced at $CAM_LAM_FMT deg"; ok=0; }
  grep -q 'turn_rate=0.0000' "$LOG" || { echo "  !! a turn_rate is set -- this cell is open-loop camber only"; ok=0; }
  # TQ295 VARIANT: the driver must ANNOUNCE the clamp (S28: never trust the env)
  if grep -q "ABAD ${ABAD_FMT} N.m" "$SLOG"; then
    echo "  CLAMP CONFIRMED: $(grep -o 'Torque ceilings: [^"]*' "$SLOG" | head -1)"
  else
    echo "  !! driver did not announce ABAD ${ABAD_FMT} N.m -- the override did NOT engage:"
    grep -o 'Torque ceilings: [^"]*' "$SLOG" | head -1
    ok=0
  fi
  [ "$ok" = 1 ]
}

# ---- ATTEMPTS --------------------------------------------------------------
OUT="$BASE/cam"
for REP in $(seq 1 "$NPER"); do
  echo
  echo "################################################################"
  echo "###  attempt $REP/$NPER -- cambered arc, lambda $CAM_LAM_DEG deg, dir $CAM_DIR,"
  echo "###  ABAD clamp $ABAD_FMT N.m"
  echo "###  ON THE RENDER: four legs cambered LEFT/RIGHT, no steer, the path"
  echo "###  curling steadily ONE way for the whole run. A pirouette or a stall"
  echo "###  is a RESULT (an invalid arc), not a harness fault -- not retried."
  echo "################################################################"
  for ATTEMPT in 1 2; do
    N=1 RUN_START=$REP OUTDIR="$OUT" RECORD_ODOM=1 PRE_SETTLE_ODOM=1 \
      GAIT_SIM=$GAIT_SIM GAIT_WALL=$GAIT_WALL \
      CTL_ARGS="$CAM_ARGS $ATT_ARGS $FLIGHT_ARGS $PIN_ARGS $TPL_ARG" \
      bash "$DIAG/repeat_gain_regime.sh"
    if [ -f "$OUT/run$REP.csv" ]; then
      [ "$ATTEMPT" = 2 ] && echo "  (attempt $REP succeeded on RETRY -- cold start, S171 S6)"
      if certify "$OUT/ctl_run$REP.log" "$OUT/sim_run$REP.log"; then echo "  attempt $REP CERTIFIED"
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
