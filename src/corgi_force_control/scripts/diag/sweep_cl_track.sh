#!/usr/bin/env bash
# CLOSED-LOOP CAMBER: tracking campaign (log S239) and the Stage 4 matrix
# mirror (log S240). One harness, typed cells:
#   cl:NAME:RATE:FF_DEG:HI_DEG   closed-loop camber at turn_rate RATE
#   drift:NAME                   everything off (the lambda = 0 matrix cell)
#   diff:NAME:RATE               differential arm, S128's steer parameters
#   combo:NAME:RATE:FF_DEG:HI_DEG  CL camber AND differential together (S253):
#                                over-actuated, the loop resolves the split
#
# WHY (S239). The channel built in S237 has never closed a loop. Its authority
# at v ~ 0.35 m/s is R ~ 2.3-3.0 m = yaw rates 0.117-0.152 rad/s, so the
# tracking cells command INSIDE that envelope (-0.12, -0.15); S238's costed
# -0.09 (R 3.9 m) is outside it and was corrected at registration.
# WHY (S240). Every hardware matrix number should land on a sim prediction.
#
# COST. ~17 min/run at GAIT_SIM 50. S239: 6 runs ~1.7 h. S240: 10 runs ~2.8 h.
#
# No `set -u` -- ROS setup scripts reference unbound variables.

WS=~/corgi_ws/corgi_ros2_ws
DIAG="$WS/src/corgi_force_control/scripts/diag"
CFG="$WS/src/corgi_force_control/config"
. "$DIAG/preflight_plant.sh";       preflight_plant || exit 1
. "$DIAG/preflight_launch_args.sh"; preflight_launch_args || exit 1
. "$DIAG/preflight_sim.sh";         preflight_sim || exit 1

NPER=${NPER:-3}
GAIT_SIM=${GAIT_SIM:-50}
GAIT_WALL=${GAIT_WALL:-800}
BASE=${BASE:-/home/alexc/corgi_runs/cl_track}

TPL_ARG="template_path:=$CFG/gslip_pronk_template_v070.csv"
FLIGHT_ARGS="k_flight:=7150.0 b_flight:=115.8"
ATT_ARGS="k_yaw:=0.0 d_yaw:=0.0"
K_ACKER=${K_ACKER:-0.5}
D_ACKER=${D_ACKER:-0.15}
STEER_ARGS="steer_offset:=0.04363 k_steer_yaw:=-0.8 steer_limit:=0.2094"

rad() { awk -v d="$1" 'BEGIN{printf "%.7f", d*3.14159265358979/180.0}'; }

CELLS=${CELLS:-"cl:cl12:-0.12:10:15 cl:cl15:-0.15:10:15"}

echo "==========================================================="
echo " CLOSED-LOOP CAMBER -- S239 tracking / S240 matrix mirror"
echo "==========================================================="
echo " cells    : $CELLS"
echo " loop     : k_acker_yaw=$K_ACKER d_acker_yaw=$D_ACKER (ff/hi per cell)"
echo " both     : $ATT_ARGS $FLIGHT_ARGS"
echo " template : $TPL_ARG"
echo " window   : ${GAIT_SIM}s of SIM time per run (NOT wall), ${GAIT_WALL}s timeout"
echo " base     : $BASE"
echo
echo " BARS (S239, registered BEFORE the run, binding S126):"
echo "   P-CL-1 TRACKING  every VALID cl run: R_yaw within +-15% of v_fwd/|rate|"
echo "   P-CL-2 THE LOOP MOVES  median |gamma| differs >= 2 deg between cl cells"
echo "                    (feedback, not just the shared feedforward)"
echo "   P-CL-3 STALLS    descriptive count (yield ~0.7 expected, S235)"
echo " S240 is a deliverable (the mirror map), no bars beyond per-run validity."
echo

BIN="$WS/install/corgi_force_control/lib/corgi_force_control"
for NEED in 'LEG-FRAME GAINS' 'ATTITUDE GAINS' 'ACKER CL set' 'Turn: turn_rate'; do
  [ "$(strings "$BIN/gslip_pronk_node" 2>/dev/null | grep -c "$NEED")" != 0 ] || {
    echo "!! INSTALLED gslip_pronk_node lacks '$NEED'. Rebuild. REFUSING."; exit 1; }
done
echo "installed controller carries every banner this campaign certifies against."

[ -f "$CFG/gslip_pronk_template_v070.csv" ] || { echo "!! v070 missing"; exit 1; }
for T in score_three_radii.py; do
  python3 "$DIAG/$T" --selftest > "/tmp/cl_$T.out" 2>&1 || {
    echo "!! $T selftest FAILED:"; tail -15 "/tmp/cl_$T.out"; exit 1; }
done
echo "scorer selftests pass."

if [ -d "$BASE" ] && [ -n "$(ls -A "$BASE" 2>/dev/null)" ]; then
  echo "!! $BASE already has content. REFUSING."; exit 1
fi
mkdir -p "$BASE"
{ echo "campaign  cl_track  $(date -Iseconds)"
  echo "cells $CELLS"
  echo "loop k_acker_yaw=$K_ACKER d_acker_yaw=$D_ACKER"
  echo "both  $ATT_ARGS $FLIGHT_ARGS $TPL_ARG"
  echo "n $NPER  gait_sim $GAIT_SIM"; } > "$BASE/DESIGN.txt"

certify() {  # certify <type> <ctl_log>  (uses CELL_RATE CELL_FF CELL_HI)
  local TYPE=$1 LOG=$2 ok=1
  grep -q 'k_flight=7150.0 b_flight=115.8' "$LOG" || { echo "  !! flight gains"; ok=0; }
  grep -q 'ATTITUDE GAINS: k_yaw=0.0000' "$LOG"    || { echo "  !! k_yaw not 0"; ok=0; }
  grep -q 'Loaded 265 template rows' "$LOG"        || { echo "  !! template rows"; ok=0; }
  case "$TYPE" in
    cl)
      local FFF KF DF RF
      FFF=$(printf '%.2f' "$CELL_FF"); RF=$(printf '%.4f' "$CELL_RATE")
      KF=$(printf '%.3f' "$K_ACKER"); DF=$(printf '%.3f' "$D_ACKER")
      grep -q "ACKER CL set: ff=${FFF} deg k_yaw=${KF} d_yaw=${DF}" "$LOG" \
        && echo "  CL CONFIRMED: $(grep -o 'ACKER CL set: [^"]*' "$LOG" | head -1)" \
        || { echo "  !! ACKER CL not announced at ff=${FFF}"; ok=0; }
      grep -q "Turn: turn_rate=${RF} rad/s" "$LOG" || { echo "  !! turn_rate ${RF}"; ok=0; }
      grep -q 'k_steer_yaw=0.000' "$LOG" || { echo "  !! differential steer engaged"; ok=0; }
      grep -q 'ACKER CAMBER set' "$LOG" && { echo "  !! open-loop CAMBER in a cl cell"; ok=0; } ;;
    drift)
      grep -q 'turn_rate=0.0000' "$LOG" || { echo "  !! drift cell has turn_rate"; ok=0; }
      grep -qE 'ACKER (CL|CAMBER) set' "$LOG" && { echo "  !! drift cell cambered"; ok=0; } ;;
    diff)
      local RF; RF=$(printf '%.4f' "$CELL_RATE")
      grep -q "Turn: turn_rate=${RF} rad/s" "$LOG" || { echo "  !! turn_rate ${RF}"; ok=0; }
      grep -q 'k_steer_yaw=-0.800' "$LOG" || { echo "  !! k_steer_yaw -0.8"; ok=0; }
      grep -qE 'ACKER (CL|CAMBER) set' "$LOG" && { echo "  !! diff cell cambered"; ok=0; } ;;
    combo)
      local FFF KF DF RF
      FFF=$(printf '%.2f' "$CELL_FF"); RF=$(printf '%.4f' "$CELL_RATE")
      KF=$(printf '%.3f' "$K_ACKER"); DF=$(printf '%.3f' "$D_ACKER")
      grep -q "ACKER CL set: ff=${FFF} deg k_yaw=${KF} d_yaw=${DF}" "$LOG" \
        && echo "  CL CONFIRMED: $(grep -o 'ACKER CL set: [^"]*' "$LOG" | head -1)" \
        || { echo "  !! ACKER CL not announced at ff=${FFF}"; ok=0; }
      grep -q "Turn: turn_rate=${RF} rad/s" "$LOG" || { echo "  !! turn_rate ${RF}"; ok=0; }
      grep -q 'k_steer_yaw=-0.800' "$LOG" || { echo "  !! combo steer arm NOT engaged"; ok=0; }
      grep -q 'ACKER CAMBER set' "$LOG" && { echo "  !! open-loop CAMBER in a combo cell"; ok=0; } ;;
  esac
  [ "$ok" = 1 ]
}

run_cell() {  # run_cell <spec> <rep>
  local SPEC=$1 REP=$2 TYPE NAME ARGS OUT REST
  TYPE="${SPEC%%:*}"; REST="${SPEC#*:}"; NAME="${REST%%:*}"
  CELL_RATE=""; CELL_FF=""; CELL_HI=""
  case "$TYPE" in
    cl)
      CELL_RATE=$(echo "$REST" | cut -d: -f2)
      CELL_FF=$(echo "$REST" | cut -d: -f3)
      CELL_HI=$(echo "$REST" | cut -d: -f4)
      ARGS="gamma_acker_ff:=$(rad "$CELL_FF") k_acker_yaw:=$K_ACKER d_acker_yaw:=$D_ACKER gamma_acker_hi:=$(rad "$CELL_HI") turn_rate:=$CELL_RATE k_steer_yaw:=0.0 steer_offset:=0.0" ;;
    drift) ARGS="" ;;
    diff)
      CELL_RATE=$(echo "$REST" | cut -d: -f2)
      ARGS="turn_rate:=$CELL_RATE $STEER_ARGS" ;;
    combo)
      CELL_RATE=$(echo "$REST" | cut -d: -f2)
      CELL_FF=$(echo "$REST" | cut -d: -f3)
      CELL_HI=$(echo "$REST" | cut -d: -f4)
      ARGS="gamma_acker_ff:=$(rad "$CELL_FF") k_acker_yaw:=$K_ACKER d_acker_yaw:=$D_ACKER gamma_acker_hi:=$(rad "$CELL_HI") turn_rate:=$CELL_RATE $STEER_ARGS" ;;
    *) echo "!! unknown cell type $TYPE"; exit 1 ;;
  esac
  OUT="$BASE/$NAME"; mkdir -p "$OUT"
  echo
  echo "################################################################"
  echo "###  rep $REP/$NPER, CELL $NAME ($TYPE)  rate=$CELL_RATE ff=$CELL_FF hi=$CELL_HI"
  case "$TYPE" in
    cl)    echo "###  ON THE RENDER: cambered L/R, magnitude BREATHING with the"
           echo "###  heading error (this is the new thing). Stall = counted." ;;
    drift) echo "###  ON THE RENDER: straight pronk drifting with kappa -0.067." ;;
    diff)  echo "###  ON THE RENDER: legs vertical, differential turn." ;;
    combo) echo "###  ON THE RENDER: cambered L/R AND differential steer together;"
           echo "###  the loop decides the split. Watch for the arms fighting." ;;
  esac
  echo "################################################################"
  for ATTEMPT in 1 2; do
    N=1 RUN_START=$REP OUTDIR="$OUT" RECORD_ODOM=1 \
      GAIT_SIM=$GAIT_SIM GAIT_WALL=$GAIT_WALL \
      CTL_ARGS="$ARGS $ATT_ARGS $FLIGHT_ARGS $TPL_ARG" \
      bash "$DIAG/repeat_gain_regime.sh"
    if [ -f "$OUT/run$REP.csv" ]; then
      [ "$ATTEMPT" = 2 ] && echo "  (succeeded on RETRY -- cold start, S171 S6)"
      if certify "$TYPE" "$OUT/ctl_run$REP.log"; then
        echo "  run $REP CERTIFIED ($NAME)"
      else
        echo "  !! run $REP of $NAME INVALID -- quarantining."
        mv "$OUT/run$REP.csv" "$OUT/run${REP}_uncertified.csv"
        mv "$OUT/odom_run$REP.csv" "$OUT/odom_run${REP}_uncertified.csv" 2>/dev/null
      fi
      return
    fi
    [ "$ATTEMPT" = 1 ] && echo "  !! NO CAPTURE, retrying ONCE (cold start)" \
                       || echo "  !! NO CAPTURE on either attempt -- rep LOST"
  done
}

for REP in $(seq 1 "$NPER"); do
  for CELL in $CELLS; do run_cell "$CELL" "$REP"; done
done

echo
echo "==========================================================="
echo " SCORE -- score_three_radii.py, band [12, $((GAIT_SIM-2))] s"
echo "==========================================================="
SCORE_ARGS=""
for CELL in $CELLS; do R="${CELL#*:}"; SCORE_ARGS="$SCORE_ARGS --cell $BASE/${R%%:*}"; done
python3 "$DIAG/score_three_radii.py" $SCORE_ARGS --end $((GAIT_SIM-2))
echo
echo "Done. Captures in $BASE. Record verdicts in the log as they stand."
