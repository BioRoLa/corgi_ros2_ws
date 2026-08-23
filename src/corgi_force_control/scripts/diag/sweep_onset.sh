#!/usr/bin/env bash
# ONSET CAPTURE: record the first seconds of runs that may collapse. Log S219.
#
# WHY. S195 killed every collapse predictor for one structural reason: the
# odom capture opened ~7 s into the gait and both collapsed runs on record
# were already dead in the first window. PRE_SETTLE_ODOM=1 (S196) starts the
# capture before the settle. This is the first campaign that can SEE an onset.
# The collapse census is 4: two on `clamp` (k_yaw 0.15 + 1 deg clamp), two on
# camber-only cells with NO heading loop -- so the mechanism is not (only) the
# loop, and both kinds are run here.
#
# CELLS, n = 5 each, INTERLEAVED, config of record, GAIT_SIM 24 (the banked
# census's window), PRE_SETTLE_ODOM 1:
#   clamp   k_yaw 0.15 d_yaw 0.02 gamma_yaw_limit 0.0175 -- yaw_gentle/clamp verbatim
#   cam10   Ackermann camber 10 deg dir +1, k_yaw 0     -- camber_lambda/lam10 verbatim
#
# A collapse is a RESULT, not a harness fault: never retried, never
# quarantined. P(no collapse in 10) ~ 14%; then the campaign is inconclusive
# and onset_capture.py says so.
#
# COST. 10 runs x ~14 min ~= 2.3 h exclusive simulator. Announce first.
#
# No `set -u` -- ROS setup scripts reference unbound variables.

WS=~/corgi_ws/corgi_ros2_ws
DIAG="$WS/src/corgi_force_control/scripts/diag"
CFG="$WS/src/corgi_force_control/config"
# IS THE SIMULATED ROBOT THE RECORDED ROBOT? S198 / #26. Prints the plant
# identity into this campaign's own log and refuses a dirty or unbuilt plant.
. "$WS/src/corgi_force_control/scripts/diag/preflight_plant.sh"
preflight_plant || exit 1
# Does the launch file forward every parameter the node declares? This
# campaign's first launch passed gamma_acker_dip:=0.15 and the launch file
# dropped it silently (S128's trap); two cells were quarantined before the
# certify grep caught it. Reconciled up front now (S210 callout).
. "$WS/src/corgi_force_control/scripts/diag/preflight_launch_args.sh"
preflight_launch_args || exit 1
# IS THE SIMULATOR FREE, AND QUIET? S203. Same one-file treatment as the plant
# guard above, for the same reason: these checks spread by copy-paste and only
# reached 7 of 25 campaigns (the WINDOWS-side webots.exe one) to 11 of 25 (the
# stale-launch one), and the variants disagreed about what to grep. This is the
# union. Self-tested: preflight_sim_selftest.sh, 9 faked-probe cases.
. "$WS/src/corgi_force_control/scripts/diag/preflight_sim.sh"
preflight_sim || exit 1
NPER=${NPER:-5}
GAIT_SIM=${GAIT_SIM:-24}
GAIT_WALL=${GAIT_WALL:-420}
# FRESH BASE: the retry loop tests bare file existence, so a stale run$REP.csv
# from an earlier campaign would read as "succeeded on attempt 1" and be scored.
BASE=${BASE:-/home/alexc/corgi_runs/onset}

TPL_ARG="template_path:=$CFG/gslip_pronk_template_v070.csv"
FLIGHT_ARGS="k_flight:=7150.0 b_flight:=115.8"
ATT_ARGS=""

CLAMP_ARGS="k_yaw:=0.15 d_yaw:=0.02 gamma_yaw_limit:=0.0175"
CAM10_ARGS="gamma_acker_in:=0.17453 gamma_acker_out:=0.17453 gamma_acker_dir:=1.0 k_yaw:=0.0 d_yaw:=0.0"

CELLS=${CELLS:-"clamp cam10"}

echo "==========================================================="
echo " ONSET CAPTURE -- the first seconds of runs that may collapse (log S219)"
echo "==========================================================="
echo " clamp    : $CLAMP_ARGS"
echo " cam10    : $CAM10_ARGS"
echo " both     : $FLIGHT_ARGS, PRE_SETTLE_ODOM=1 (capture covers the settle and the onset)"
echo " template : $TPL_ARG"
echo " n        : $NPER per cell, interleaved by repetition"
echo " window   : ${GAIT_SIM}s of SIM time per run (NOT wall), ${GAIT_WALL}s timeout"
echo " base     : $BASE"
echo
echo " PREDICTIONS -- registered in S219 BEFORE this ran (onset_capture.py docstring):"
echo "   P-O-1  >= 1 collapse in 10; if none, INCONCLUSIVE (P(none) ~ 14%)"
echo "   P-O-2  in every collapsed run the yaw rate flips sign BEFORE v_fwd < 0.10"
echo "   P-O-3  healthy runs never flip for two consecutive windows after onset+6"
echo "   P-O-4  clamp cell: F/R command at its clamp in or before the flip window"
echo "   P-O-5  cam10: collapse occurs with the camber delivered, not lost"
echo " A collapse is a RESULT. Never retried, never quarantined."
echo

# ---- PREFLIGHT. Everything here runs before any sim time is spent. ---------


BIN="$WS/install/corgi_force_control/lib/corgi_force_control"
for NEED in 'LEG-FRAME GAINS' 'ATTITUDE GAINS' 'gamma_acker' 'GENTLE YAW CLAMP'; do
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
for T in onset_capture.py yaw_excursion.py; do
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
  echo "campaign  onset  $(date -Iseconds)"
  echo "clamp $CLAMP_ARGS"
  echo "cam10 $CAM10_ARGS"
  echo "pre_settle_odom 1"
  echo "both  $ATT_ARGS $FLIGHT_ARGS $TPL_ARG"
  echo "n $NPER  gait_sim $GAIT_SIM"
} > "$BASE/DESIGN.txt"

# ---- per-run self-certification -------------------------------------------
certify() {  # certify <cell> <ctl_log> -> 0 ok, 1 INVALID
  local CELL=$1 LOG=$2 ok=1
  grep -q 'k_flight=7150.0 b_flight=115.8' "$LOG" || { echo "  !! flight gains not 7150/115.8"; ok=0; }
  grep -q 'Loaded 265 template rows' "$LOG"        || { echo "  !! template not 265 rows"; ok=0; }
  grep -q 'turn_rate=0.0000' "$LOG" || { echo "  !! a turn_rate is set"; ok=0; }
  grep -q 'ACKER DIP set' "$LOG" && { echo "  !! a DIP is set"; ok=0; }
  case "$CELL" in
    clamp)
      grep -q 'ATTITUDE GAINS: k_yaw=0.1500' "$LOG" || { echo "  !! k_yaw not 0.15"; ok=0; }
      grep -q 'GENTLE YAW CLAMP set: +-1.00 deg' "$LOG" \
        && echo "  CLAMP CONFIRMED: $(grep -o 'GENTLE YAW CLAMP set: [^"]*' "$LOG" | head -1)" \
        || { echo "  !! 1 deg yaw clamp not announced"; ok=0; }
      grep -q 'ACKER CAMBER set' "$LOG" && { echo "  !! clamp cell has camber"; ok=0; } ;;
    cam10)
      grep -q 'ATTITUDE GAINS: k_yaw=0.0000' "$LOG" || { echo "  !! k_yaw not 0"; ok=0; }
      grep -q 'ACKER CAMBER set: in=10.00 deg out=10.00 deg dir=+1' "$LOG" \
        && echo "  ACKER CONFIRMED: $(grep -o 'ACKER CAMBER set: [^"]*' "$LOG" | head -1)" \
        || { echo "  !! ACKER CAMBER 10 deg +1 not announced"; ok=0; } ;;
  esac
  [ "$ok" = 1 ]
}

run_cell() {  # run_cell <cell> <rep>
  local NAME=$1 REP=$2 OUT="$BASE/$1" ARGS
  case "$NAME" in clamp) ARGS="$CLAMP_ARGS";; cam10) ARGS="$CAM10_ARGS";; esac
  mkdir -p "$OUT"
  echo
  echo "################################################################"
  echo "###  rep $REP/$NPER, CELL $NAME"
  if [ "$NAME" = clamp ]; then
    echo "###  ON THE RENDER: legs vertical, heading held by a 1 deg yaw clamp. A"
    echo "###  run that starts to pirouette is THE RESULT -- do not touch it."
  else
    echo "###  ON THE RENDER: four legs cambered LEFT/RIGHT at 10 deg, no heading"
    echo "###  loop. A pirouette is THE RESULT -- do not touch it."
  fi
  echo "################################################################"
  for ATTEMPT in 1 2; do
    N=1 RUN_START=$REP OUTDIR="$OUT" RECORD_ODOM=1 PRE_SETTLE_ODOM=1 \
      GAIT_SIM=$GAIT_SIM GAIT_WALL=$GAIT_WALL \
      CTL_ARGS="$ARGS $ATT_ARGS $FLIGHT_ARGS $TPL_ARG" \
      bash "$DIAG/repeat_gain_regime.sh"
    if [ -f "$OUT/run$REP.csv" ]; then
      [ "$ATTEMPT" = 2 ] && echo "  (cell $NAME succeeded on RETRY -- cold start, S171 S6)"
      if certify "$NAME" "$OUT/ctl_run$REP.log"; then
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
echo " ONSET -- onset_capture.py, as registered in S219"
echo "==========================================================="
python3 "$DIAG/onset_capture.py" --dir "$BASE/clamp" --label clamp --dir "$BASE/cam10" --label cam10
echo
echo "Done. Captures in $BASE. Record the verdicts in the log as they stand."
