#!/usr/bin/env bash
# THE lambda -> kappa MAP, per direction, drift-subtracted. Log S212 registers
# it; this produces the captures and self-certifies every run.
#
# WHY. S211: kappa(lambda) is SUPERLINEAR on the +1 side (0.012/deg at 10,
# 0.025/deg at 15) and the plant's own drift (kappa -0.067) aids +1 turns and
# opposes -1 turns. S202's calibration interpolated linearly from one
# direction without removing drift and missed twice. Nothing downstream --
# the matched comparison's re-registration, the Stage 3 gate's lambda --
# can be aimed without a measured map.
#
# DESIGN. Fill the GAPS in the banked map, do not repeat it. Banked at the
# config of record: +1 at 10 deg (n=9), +1 at 15 (n=5), -1 at 13 (n=1), -1 at
# 17.7 (n=8), drift n=15. New cells, n = 2 each, INTERLEAVED:
#   p20   +1 at 20 deg   (the ACKER clamp; does the superlinearity continue?)
#   m10   -1 at 10 deg   (the direct comparison with +1 at 10)
#   m15   -1 at 15 deg   (the direct comparison with +1 at 15)
#   m20   -1 at 20 deg
# Result: 3 lambdas x 2 directions with n >= 2 everywhere, plus drift.
# GAIT_SIM 24 to match camber_lambda (the banked +1 points).
#
# COST. 8 runs x ~14 min ~= 1.9 h exclusive simulator. Announce first.
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
GAIT_SIM=${GAIT_SIM:-24}
GAIT_WALL=${GAIT_WALL:-420}
# FRESH BASE: the retry loop tests bare file existence, so a stale run$REP.csv
# from an earlier campaign would read as "succeeded on attempt 1" and be scored.
BASE=${BASE:-/home/alexc/corgi_runs/lambda_map}

TPL_ARG="template_path:=$CFG/gslip_pronk_template_v070.csv"
FLIGHT_ARGS="k_flight:=7150.0 b_flight:=115.8"
ATT_ARGS="k_yaw:=0.0 d_yaw:=0.0"

rad() { awk -v d="$1" 'BEGIN{printf "%.5f", d*3.14159265358979/180.0}'; }
cam_args() {  # cam_args <lam_deg> <dir>
  echo "gamma_acker_in:=$(rad "$1") gamma_acker_out:=$(rad "$1") gamma_acker_dir:=$2"
}
# name:lambda_deg:dir
CELLS=${CELLS:-"p20:20:1.0 m10:10:-1.0 m15:15:-1.0 m20:20:-1.0"}

echo "==========================================================="
echo " lambda -> kappa MAP, per direction -- P-L-1..P-L-3 (log S212)"
echo "==========================================================="
echo " cells    : $CELLS   (name:lambda_deg:dir), n = $NPER each, interleaved"
echo " both     : $ATT_ARGS $FLIGHT_ARGS"
echo " template : $TPL_ARG"
echo " window   : ${GAIT_SIM}s of SIM time per run (NOT wall), ${GAIT_WALL}s timeout"
echo " base     : $BASE"
echo " banked   : +1@10 (camber_lambda/lam10, inner_outer/k0), +1@15 (camber_lambda/lam15),"
echo "            -1@13 (matched_turn/cam), -1@17.7 (matched_turn_cal2/cam), drift (3 cells)"
echo
echo " BARS -- registered in S212 BEFORE this ran, and binding (S126):"
echo "   P-L-1  SHAPE   per-degree authority at 20 deg >= that at 15 deg, +1 side"
echo "                  (superlinear continues) -- falsified if it falls back to the 10-deg value"
echo "   P-L-2  SYMMETRY drift-subtracted authority ratio |+1|/|-1| within [0.8, 1.25] at EACH of 10/15/20"
echo "   P-L-3  NO CLAMP BITE 20 deg is not rate-limited: ACKER banner shows clamp 20.0 and achieved"
echo "                  |gamma| >= 18 deg on the commanded legs"
echo " The map itself is the deliverable; the bars test the two claims S211 made about it."
echo

# ---- PREFLIGHT. Everything here runs before any sim time is spent. ---------


BIN="$WS/install/corgi_force_control/lib/corgi_force_control"
for NEED in 'LEG-FRAME GAINS' 'ATTITUDE GAINS' 'gamma_acker'; do
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
for T in score_lambda_map.py; do
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
  echo "campaign  lambda_map  $(date -Iseconds)"
  echo "cells $CELLS"
  echo "both  $ATT_ARGS $FLIGHT_ARGS $TPL_ARG"
  echo "n $NPER  gait_sim $GAIT_SIM"
} > "$BASE/DESIGN.txt"

# ---- per-run self-certification -------------------------------------------
certify() {  # certify <ctl_log> -> 0 ok, 1 INVALID  (uses CELL_LAM / CELL_DIR)
  local LOG=$1 ok=1
  grep -q 'k_flight=7150.0 b_flight=115.8' "$LOG" || { echo "  !! flight gains not 7150/115.8"; ok=0; }
  grep -q 'ATTITUDE GAINS: k_yaw=0.0000' "$LOG"    || { echo "  !! k_yaw not certified 0"; ok=0; }
  grep -q 'Loaded 265 template rows' "$LOG"        || { echo "  !! template not 265 rows"; ok=0; }
  local LAMF DIRF
  LAMF=$(printf '%.2f' "$CELL_LAM"); DIRF=$(printf '%+.0f' "$CELL_DIR")
  grep -q "ACKER CAMBER set: in=${LAMF} deg out=${LAMF} deg dir=${DIRF}" "$LOG" \
    && echo "  ACKER CONFIRMED: $(grep -o 'ACKER CAMBER set: [^"]*' "$LOG" | head -1)" \
    || { echo "  !! ACKER CAMBER not announced at ${LAMF} deg dir ${DIRF}"; ok=0; }
  grep -q 'turn_rate=0.0000' "$LOG" || { echo "  !! a turn_rate is set"; ok=0; }
  grep -q 'ACKER DIP set' "$LOG" && { echo "  !! a DIP is set -- the map is undipped"; ok=0; }
  [ "$ok" = 1 ]
}

run_cell() {  # run_cell <name:lam:dir> <rep>
  local SPEC=$1 REP=$2 NAME ARGS OUT
  NAME="${SPEC%%:*}"; CELL_LAM="${SPEC#*:}"; CELL_DIR="${CELL_LAM#*:}"; CELL_LAM="${CELL_LAM%%:*}"
  OUT="$BASE/$NAME"
  ARGS=$(cam_args "$CELL_LAM" "$CELL_DIR")
  mkdir -p "$OUT"
  echo
  echo "################################################################"
  echo "###  rep $REP/$NPER, CELL $NAME : lambda $CELL_LAM deg, dir $CELL_DIR"
  echo "###  ON THE RENDER: four legs cambered LEFT/RIGHT at $CELL_LAM deg; the"
  echo "###  path curls one way for dir +1 and the OTHER way for dir -1."
  echo "###  Pirouette/stall = a RESULT (excluded from the map, counted)."
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
echo " SCORE -- score_lambda_map.py, as registered in S212"
echo "==========================================================="
python3 "$DIAG/score_lambda_map.py" --base "$BASE"
echo
echo "Done. Captures in $BASE. Record the map and the verdicts in the log as they stand."
