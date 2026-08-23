#!/usr/bin/env bash
# THE turn_rate -> radius MAP for the differential arm, at the re-aimed Stage 4
# radius. Log S230 registers it; this produces the captures and self-certifies
# every run.
#
# WHY. Stage 4 was re-aimed to R ~ 2.0-2.5 m (the camber channel's own radius,
# S214/S221). The differential arm is banked at ONE rate only, +0.09 rad/s
# (R ~ 3.1 m, S129 -- and drift-OPPOSED: the plant's yaw drift is kappa -0.067
# and the matched comparison pairs the differential arm with camber +1, which
# turns WITH the drift, so the paired arm is turn_rate NEGATIVE, S226/S228).
# Decision 6 (2026-08-24): calibrate, do not interpolate -- twice now a rate
# interpolated from one banked point missed (S202, S208).
#
# DESIGN. Three rates, same sign as camber +1, interleaved, n per cell from
# NPER. Default cells (name:rate):
#   d09  -0.09   the banked magnitude, now drift-aided (is it the same R?)
#   d13  -0.13   the model's R ~ 2.2 m at v ~ 0.28 m/s  (R = v / rate)
#   d17  -0.17   R ~ 1.65 m, the tight edge of the matrix
# steer_offset / k_steer_yaw / steer_limit verbatim from S128's differential
# configuration. GAIT_SIM 50 so every fit arc clears 90 deg (S228's lesson).
# Scored by score_three_radii.py: R_menger / R_fit / R_yaw per run.
#
# COST. 3 cells x NPER x ~17 min. NPER=2 -> ~1.7 h exclusive simulator.
# A pirouette or a backwards run is a RESULT (S129: this arm fails to turn
# 2-3 times in 5 when drift-opposed), not a harness fault; not retried.
#
# No `set -u` -- ROS setup scripts reference unbound variables.

WS=~/corgi_ws/corgi_ros2_ws
DIAG="$WS/src/corgi_force_control/scripts/diag"
CFG="$WS/src/corgi_force_control/config"
# IS THE SIMULATED ROBOT THE RECORDED ROBOT? S198 / #26. Prints the plant
# identity into this campaign's own log and refuses a dirty or unbuilt plant.
. "$WS/src/corgi_force_control/scripts/diag/preflight_plant.sh"
preflight_plant || exit 1
# Does the launch file forward every parameter the node declares? The first
# dip campaign passed gamma_acker_dip:=0.15 and the launch file dropped it
# silently (S128's trap, again); two cells were quarantined before the
# certify grep caught it. Reconciled up front now, for every harness that
# passes node parameters.
. "$WS/src/corgi_force_control/scripts/diag/preflight_launch_args.sh"
preflight_launch_args || exit 1
# IS THE SIMULATOR FREE, AND QUIET? S203. Same one-file treatment as the plant
# guard above, for the same reason: these checks spread by copy-paste and only
# reached 7 of 25 campaigns (the WINDOWS-side webots.exe one) to 11 of 25 (the
# stale-launch one), and the variants disagreed about what to grep. This is the
# union. Self-tested: preflight_sim_selftest.sh, 9 faked-probe cases.
. "$WS/src/corgi_force_control/scripts/diag/preflight_sim.sh"
preflight_sim || exit 1
NPER=${NPER:-2}
GAIT_SIM=${GAIT_SIM:-50}
GAIT_WALL=${GAIT_WALL:-800}
# FRESH BASE: the retry loop tests bare file existence, so a stale run$REP.csv
# from an earlier campaign would read as "succeeded on attempt 1" and be scored.
BASE=${BASE:-/home/alexc/corgi_runs/turn_rate_map}

TPL_ARG="template_path:=$CFG/gslip_pronk_template_v070.csv"
FLIGHT_ARGS="k_flight:=7150.0 b_flight:=115.8"
ATT_ARGS="k_yaw:=0.0 d_yaw:=0.0"

STEER_ARGS="steer_offset:=0.04363 k_steer_yaw:=-0.8 steer_limit:=0.2094"
diff_args() {  # diff_args <rate>
  echo "turn_rate:=$1 $STEER_ARGS"
}
# name:rate
CELLS=${CELLS:-"d09:-0.09 d13:-0.13 d17:-0.17"}

echo "==========================================================="
echo " turn_rate -> RADIUS MAP, differential arm -- P-TR-1..P-TR-3 (log S230)"
echo "==========================================================="
echo " cells    : $CELLS   (name:rate rad/s), n = $NPER each, interleaved"
echo " steer    : $STEER_ARGS"
echo " both     : $ATT_ARGS $FLIGHT_ARGS"
echo " template : $TPL_ARG"
echo " window   : ${GAIT_SIM}s of SIM time per run (NOT wall), ${GAIT_WALL}s timeout"
echo " base     : $BASE"
echo " banked   : +0.09 only (turn_rep/k7150/lam0_default, n=10, drift-opposed)"
echo
echo " BARS -- registered in S230 BEFORE this ran, and binding (S126):"
echo "   P-TR-1  TRACKING   on every VALID run, R_yaw within +-25% of v_fwd/|rate|"
echo "                      (the heading loop holds the commanded rate)"
echo "   P-TR-2  MONOTONE   cell-median |kappa| increases d09 < d13 < d17"
echo "   P-TR-3  RELIABILITY descriptive, not powered: valid-run count per cell"
echo "                      (S129 saw 2-3 failures in 5 drift-opposed; drift-aided may differ)"
echo " The map itself is the deliverable: the rate that gives R ~ 2.2 m is read off it."
echo

# ---- PREFLIGHT. Everything here runs before any sim time is spent. ---------


BIN="$WS/install/corgi_force_control/lib/corgi_force_control"
for NEED in 'LEG-FRAME GAINS' 'ATTITUDE GAINS' 'Turn: turn_rate' 'Steering: k_steer'; do
  [ "$(strings "$BIN/gslip_pronk_node" 2>/dev/null | grep -c "$NEED")" != 0 ] || {
    echo "!! INSTALLED gslip_pronk_node lacks '$NEED' -- a cell could not be"
    echo "!! certified from its own log. Rebuild. REFUSING."; exit 1; }
done
echo "installed controller carries every banner this campaign certifies against."

[ -f "$CFG/gslip_pronk_template_v070.csv" ] || { echo "!! v070 missing"; exit 1; }
NZ=$(awk -F, 'NR>1 && $4+0 != 0 {n++} END {print n+0}' "$CFG/gslip_pronk_template_v070.csv")
[ "$NZ" = "0" ] || { echo "!! v070 gamma column is NOT identically zero. REFUSING."; exit 1; }
echo "v070 present, gamma column identically zero."

# The scorers must pass their own selftests before they are trusted with this.
for T in score_three_radii.py; do
  python3 "$DIAG/$T" --selftest > "/tmp/mt_$T.out" 2>&1 || {
    echo "!! $T selftest FAILED -- the bars cannot be scored:"; tail -15 "/tmp/mt_$T.out"; exit 1; }
done
echo "scorer selftests pass."

if [ -d "$BASE" ] && [ -n "$(ls -A "$BASE" 2>/dev/null)" ]; then
  echo "!! $BASE already has content. A fresh campaign needs a fresh base --"
  echo "!! set BASE=... or move the old one. REFUSING."; exit 1
fi
mkdir -p "$BASE"
{
  echo "campaign  turn_rate_map  $(date -Iseconds)"
  echo "cells $CELLS"
  echo "steer $STEER_ARGS"
  echo "both  $ATT_ARGS $FLIGHT_ARGS $TPL_ARG"
  echo "n $NPER  gait_sim $GAIT_SIM"
} > "$BASE/DESIGN.txt"

# ---- per-run self-certification -------------------------------------------
certify() {  # certify <ctl_log> -> 0 ok, 1 INVALID  (uses CELL_RATE)
  local LOG=$1 ok=1
  grep -q 'k_flight=7150.0 b_flight=115.8' "$LOG" || { echo "  !! flight gains not 7150/115.8"; ok=0; }
  grep -q 'ATTITUDE GAINS: k_yaw=0.0000' "$LOG"    || { echo "  !! k_yaw not certified 0"; ok=0; }
  grep -q 'Loaded 265 template rows' "$LOG"        || { echo "  !! template not 265 rows"; ok=0; }
  local RATEF
  RATEF=$(printf '%.4f' "$CELL_RATE")
  grep -q "Turn: turn_rate=${RATEF} rad/s" "$LOG" \
    && echo "  TURN CONFIRMED: $(grep -o 'Turn: turn_rate[^"]*' "$LOG" | head -1)" \
    || { echo "  !! turn_rate ${RATEF} not announced"; ok=0; }
  grep -q 'k_steer_yaw=-0.800' "$LOG" \
    && echo "  STEER CONFIRMED: $(grep -o 'Steering: k_steer[^"]*' "$LOG" | head -1)" \
    || { echo "  !! k_steer_yaw -0.8 not announced"; ok=0; }
  grep -q 'ACKER CAMBER set' "$LOG" && { echo "  !! differential cell has CAMBER"; ok=0; }
  grep -q 'ACKER DIP set' "$LOG" && { echo "  !! a DIP is set"; ok=0; }
  [ "$ok" = 1 ]
}

run_cell() {  # run_cell <name:lam:dir> <rep>
  local SPEC=$1 REP=$2 NAME ARGS OUT
  NAME="${SPEC%%:*}"; CELL_RATE="${SPEC#*:}"
  OUT="$BASE/$NAME"
  ARGS=$(diff_args "$CELL_RATE")
  mkdir -p "$OUT"
  echo
  echo "################################################################"
  echo "###  rep $REP/$NPER, CELL $NAME : turn_rate $CELL_RATE rad/s"
  echo "###  ON THE RENDER: legs VERTICAL, a commanded turn via differential"
  echo "###  beta, curling the SAME way as camber +1 (with the drift)."
  echo "###  Pirouette/backwards = a RESULT (excluded from the map, counted)."
  echo "################################################################"
  for ATTEMPT in 1 2; do
    N=1 RUN_START=$REP OUTDIR="$OUT" RECORD_ODOM=1 \
      GAIT_SIM=$GAIT_SIM GAIT_WALL=$GAIT_WALL \
      CTL_ARGS="$ARGS $ATT_ARGS $FLIGHT_ARGS $TPL_ARG" \
      bash "$DIAG/repeat_gain_regime.sh"
    if [ -f "$OUT/run$REP.csv" ]; then
      [ "$ATTEMPT" = 2 ] && echo "  (cell $NAME succeeded on RETRY -- cold start, S171 S6)"
      if certify "$OUT/ctl_run$REP.log"; then
        echo "  run $REP CERTIFIED ($NAME)"
      else
        echo "  !! run $REP of $NAME is INVALID -- config not certified. Quarantining."
        mv "$OUT/run$REP.csv" "$OUT/run${REP}_uncertified.csv"
        mv "$OUT/odom_run$REP.csv" "$OUT/odom_run${REP}_uncertified.csv" 2>/dev/null
      fi
      return
    fi
    if [ "$ATTEMPT" = 1 ]; then
      echo "  !! cell $NAME rep $REP produced NO CAPTURE. Cold-start failure mode,"
      echo "  !! not a result. Retrying ONCE."
    else
      echo "  !! cell $NAME rep $REP produced NO CAPTURE on either attempt."
      echo "  !! That rep is LOST for pairing -- note it."
    fi
  done
}

# ---- ATTEMPTS, interleaved by repetition ------------------------------------
for REP in $(seq 1 "$NPER"); do
  for CELL in $CELLS; do run_cell "$CELL" "$REP"; done
done

# ---- ANALYSIS --------------------------------------------------------------
echo
echo "==========================================================="
echo " SCORE -- score_three_radii.py, band [12, $((GAIT_SIM-2))] s"
echo "==========================================================="
SCORE_ARGS=""
for CELL in $CELLS; do SCORE_ARGS="$SCORE_ARGS --cell $BASE/${CELL%%:*}"; done
python3 "$DIAG/score_three_radii.py" $SCORE_ARGS --end $((GAIT_SIM-2))
echo
echo "Done. Captures in $BASE. Record the map and the verdicts in the log as they stand."
