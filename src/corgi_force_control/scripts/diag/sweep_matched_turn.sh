#!/usr/bin/env bash
# CAMBER vs DIFFERENTIAL STEERING AT MATCHED CURVATURE -- the thesis's headline
# comparison, run paired and interleaved in simulation. Log S202 registers the
# bars; this script only produces the captures and self-certifies every run.
#
# WHY THIS EXISTS. Open Issue #25: the thesis claims camber beats differential
# steering on cross-track, energy and repeatability, and as of 2026-08-23 NONE
# of the three had a registered measurement. S199 then read all three from
# banked runs (cross-campaign, unregistered) and every one came back favourable
# or neutral for camber. This is the registered version: SAME session,
# interleaved, matched commanded curvature, statistic chosen before the data.
#
# DESIGN.
#   cam   Ackermann camber pair, OPEN-LOOP, lambda = 13 deg, dir = -1 so the
#         turn has the SAME SIGN as the differential arm (dir=+1 gave kappa < 0
#         in camber_lambda; turn_rate +0.09 gave kappa > 0 in turn_rep). The
#         plant's own yaw drift is signed (-0.5..-2 deg/s, S195), so both arms
#         must see it the same way round.
#         13 deg is interpolated from the banked lambda->kappa map at the config
#         of record: 10 deg -> 0.21/0.26, 15 deg -> 0.445. Target kappa ~0.33
#         (R ~3.0 m), which is #24's reconciled Stage 4 radius AND what the
#         differential arm delivered at turn_rate 0.09 (S129: 0.28-0.35).
#   diff  S128's differential configuration VERBATIM, read back from
#         turn_rep/k7150/lam0_default/ctl_run1.log: turn_rate 0.09 rad/s,
#         steer_offset 0.04363 rad (2.50 deg), k_steer_yaw -0.8, steer_limit
#         0.2094 (12.0 deg). No camber.
#   both  config of record (v070, k_flight 7150, b_flight 115.8, DIRBETA=1,
#         THETA_STOP=1), k_yaw:=0.0 passed EXPLICITLY and certified from the
#         ATTITUDE GAINS banner -- turn_rep's logs predate that banner, so the
#         differential arm's k_yaw was never certified before. n = 8 per cell,
#         interleaved by repetition.
#
# THE ASYMMETRY THAT IS INTRINSIC, NOT A DEFECT. The camber arm is open-loop;
# the differential arm closes a yaw loop through k_steer_yaw. That is what the
# two steering methods ARE on this robot, and the comparison is between them
# as they exist. It means the differential arm's circle residual is closer to
# true tracking error and the camber arm's is path regularity -- S199's caveat,
# carried into the registration.
#
# GAIT_SIM = 30, not 24. #24's arc protocol scores Menger kappa over the steady
# band, but P-4 (cross-track) fits a circle, and check_turn's Kasa fit is
# flagged under 90 deg of arc. At kappa 0.33 and v 0.3 m/s a 12 s band covers
# ~68 deg; the 18 s band from GAIT_SIM 30 covers ~100 deg. Costs ~1.5 min/run.
#
# PRE-REGISTERED CALIBRATION RULE (S202 S3). After rep 1 of BOTH cells, their
# achieved |kappa| are compared. If the camber arm is outside +-20% of the
# differential arm, the campaign STOPS, lambda is rescaled by the ratio
# (kappa is near-linear in lambda over 10-15 deg), and the campaign restarts
# in a FRESH base. Written before any run, so it is a calibration step and not
# a rescue. The rule may fire at most once.
#
# COST. 16 runs x ~15 min = ~4.1 h exclusive simulator (+~30 min if the
# calibration rule fires). Announce before starting -- the simulator is shared.
#
# No `set -u` -- ROS setup scripts reference unbound variables.

WS=~/corgi_ws/corgi_ros2_ws
DIAG="$WS/src/corgi_force_control/scripts/diag"
CFG="$WS/src/corgi_force_control/config"
# IS THE SIMULATED ROBOT THE RECORDED ROBOT? S198 / #26. Prints the plant
# identity into this campaign's own log and refuses a dirty or unbuilt plant.
. "$WS/src/corgi_force_control/scripts/diag/preflight_plant.sh"
preflight_plant || exit 1
# IS THE SIMULATOR FREE, AND QUIET? S202. Same one-file treatment as the plant
# guard above, for the same reason: these checks spread by copy-paste and only
# reached 7 of 25 campaigns (the WINDOWS-side webots.exe one) to 11 of 25 (the
# stale-launch one), and the variants disagreed about what to grep. This is the
# union. Self-tested: preflight_sim_selftest.sh, 9 faked-probe cases.
. "$WS/src/corgi_force_control/scripts/diag/preflight_sim.sh"
preflight_sim || exit 1
NPER=${NPER:-8}
GAIT_SIM=${GAIT_SIM:-30}
GAIT_WALL=${GAIT_WALL:-520}
# FRESH BASE: the retry loop tests bare file existence, so a stale run$REP.csv
# from an earlier campaign would read as "succeeded on attempt 1" and be scored.
BASE=${BASE:-/home/alexc/corgi_runs/matched_turn}

TPL_ARG="template_path:=$CFG/gslip_pronk_template_v070.csv"
FLIGHT_ARGS="k_flight:=7150.0 b_flight:=115.8"
ATT_ARGS="k_yaw:=0.0 d_yaw:=0.0"

CAM_LAM_DEG=${CAM_LAM_DEG:-13}
CAM_LAM_RAD=$(awk -v d="$CAM_LAM_DEG" 'BEGIN{printf "%.5f", d*3.14159265358979/180.0}')
CAM_DIR=${CAM_DIR:--1.0}
# The ACKER banner prints lambda as %.2f; a rescaled lambda like 17.7 must be
# matched as 17.70, not 17.7.00 (the first draft would have quarantined every
# cam run of a rescaled campaign as uncertified).
CAM_LAM_FMT=$(printf '%.2f' "$CAM_LAM_DEG")
CAM_ARGS="gamma_acker_in:=$CAM_LAM_RAD gamma_acker_out:=$CAM_LAM_RAD gamma_acker_dir:=$CAM_DIR"
DIFF_ARGS="turn_rate:=0.09 steer_offset:=0.04363 k_steer_yaw:=-0.8 steer_limit:=0.2094"

CELLS=${CELLS:-"cam diff"}

echo "==========================================================="
echo " CAMBER vs DIFFERENTIAL AT MATCHED CURVATURE -- P-M-0..P-M-6 (log S202)"
echo "==========================================================="
echo " cam      : $CAM_ARGS   (lambda $CAM_LAM_DEG deg, dir $CAM_DIR)"
echo " diff     : $DIFF_ARGS"
echo " both     : $ATT_ARGS $FLIGHT_ARGS"
echo " template : $TPL_ARG"
echo " n        : $NPER per cell, interleaved by repetition"
echo " window   : ${GAIT_SIM}s of SIM time per run (NOT wall), ${GAIT_WALL}s timeout"
echo " base     : $BASE"
echo " target   : kappa ~0.33 (R ~3.0 m) on BOTH arms, same sign"
echo
echo " BARS -- registered in S202 BEFORE this ran, and binding (S126):"
echo "   P-M-0  SIGN. Both arms turn the same way (median kappa same sign)."
echo "   P-M-1  MATCHING, gates everything. Median |kappa| over coherent runs"
echo "          within +-20% between arms. Fails -> the comparison is UNSCORED."
echo "   P-M-2  COHERENT-TURN FRACTION, descriptive, NOT powered. cam >= 7/8,"
echo "          diff <= 5/8. Reported as counts with Fisher p beside them."
echo "   P-M-3  REPEATABILITY, the powered bar. sd(kappa) over ALL runs,"
echo "          failures included: diff/cam >= 3x. Exhaustive permutation."
echo "   P-M-3b REGISTERED NULL. sd(kappa) over COHERENT runs only: ratio"
echo "          within [0.5, 2]. Predicts differential is as PRECISE as camber"
echo "          when it works -- the difference is reliability (S129)."
echo "   P-M-4  CROSS-TRACK. Median circle-residual RMS, coherent runs, cam"
echo "          <= 1.25 x diff. (Regularity vs tracking -- see header.)"
echo "   P-M-5  PEAK TORQUE. Median tau p99.5, coherent runs, cam <= 0.90 x diff."
echo "   P-M-6  SPEED. Median v_fwd, coherent runs, cam >= diff."
echo
echo " DECISION RULE: P-M-1 gates. P-M-3 is the repeatability verdict; P-M-4"
echo " and P-M-5 are the cross-track and energy axes. Each scored PASS/FAIL"
echo " independently; a FAIL stands (S126). Coherent = arc >= 90 deg AND"
echo " band v_fwd >= 0.10 m/s (S191's collapse threshold)."
echo

# ---- PREFLIGHT. Everything here runs before any sim time is spent. ---------


BIN="$WS/install/corgi_force_control/lib/corgi_force_control"
for NEED in 'LEG-FRAME GAINS' 'ATTITUDE GAINS' 'gamma_acker' 'Turn: turn_rate'; do
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
for T in cross_track.py matched_kappa.py score_matched_turn.py; do
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
  echo "campaign  matched_turn  $(date -Iseconds)"
  echo "cam   $CAM_ARGS"
  echo "diff  $DIFF_ARGS"
  echo "both  $ATT_ARGS $FLIGHT_ARGS $TPL_ARG"
  echo "n $NPER  gait_sim $GAIT_SIM"
} > "$BASE/DESIGN.txt"

# ---- per-run self-certification -------------------------------------------
certify() {  # certify <cell> <ctl_log> -> 0 ok, 1 INVALID
  local CELL=$1 LOG=$2 ok=1
  grep -q 'k_flight=7150.0 b_flight=115.8' "$LOG" || { echo "  !! flight gains not 7150/115.8"; ok=0; }
  grep -q 'ATTITUDE GAINS: k_yaw=0.0000' "$LOG"    || { echo "  !! k_yaw not certified 0"; ok=0; }
  grep -q 'Loaded 265 template rows' "$LOG"        || { echo "  !! template not 265 rows"; ok=0; }
  case "$CELL" in
    cam)
      grep -q "ACKER CAMBER set: in=${CAM_LAM_FMT} deg" "$LOG" \
        && echo "  ACKER CONFIRMED: $(grep -o 'ACKER CAMBER set: [^"]*' "$LOG" | head -1)" \
        || { echo "  !! ACKER CAMBER not announced at $CAM_LAM_DEG deg"; ok=0; }
      grep -q 'turn_rate=0.0000' "$LOG" || { echo "  !! cam cell has a turn_rate"; ok=0; } ;;
    diff)
      grep -q 'Turn: turn_rate=0.0900' "$LOG" \
        && echo "  TURN CONFIRMED: $(grep -o 'Turn: turn_rate[^"]*' "$LOG" | head -1)" \
        || { echo "  !! turn_rate 0.09 not announced"; ok=0; }
      grep -q 'k_steer_yaw=-0.800' "$LOG" \
        && echo "  STEER CONFIRMED: $(grep -o 'Steering: k_steer[^"]*' "$LOG" | head -1)" \
        || { echo "  !! k_steer_yaw -0.8 not announced"; ok=0; }
      grep -q 'ACKER CAMBER set' "$LOG" && { echo "  !! diff cell has CAMBER"; ok=0; } ;;
  esac
  [ "$ok" = 1 ]
}

run_cell() {  # run_cell <cell> <rep>
  local NAME=$1 REP=$2 OUT="$BASE/$1" ARGS
  case "$NAME" in cam) ARGS="$CAM_ARGS";; diff) ARGS="$DIFF_ARGS";; esac
  mkdir -p "$OUT"
  echo
  echo "################################################################"
  echo "###  rep $REP/$NPER, CELL $NAME"
  if [ "$NAME" = cam ]; then
    echo "###  ON THE RENDER: all four legs cambered LEFT/RIGHT, $CAM_LAM_DEG deg,"
    echo "###  no steer. The path should curl to ONE side. No heading hold."
  else
    echo "###  ON THE RENDER: legs VERTICAL, a commanded turn via differential"
    echo "###  beta. Should curl the SAME way as the cam cell. S129 found this"
    echo "###  arm fails to turn 2-3 times in 5 -- a pirouette or a backwards"
    echo "###  run is a RESULT, not a harness fault. Do not retry on that."
  fi
  echo "################################################################"
  for ATTEMPT in 1 2; do
    N=1 RUN_START=$REP OUTDIR="$OUT" RECORD_ODOM=1 \
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

# ---- REP 1 + the calibration rule ------------------------------------------
for CELL in $CELLS; do run_cell "$CELL" 1; done

echo
echo "-- CALIBRATION CHECK after rep 1 (pre-registered, may fire ONCE) ------"
CAL=$(python3 "$DIAG/score_matched_turn.py" --base "$BASE" --calibrate 2>&1)
echo "$CAL"
case "$CAL" in
  *"CALIBRATION: RESCALE"*)
    NEWLAM=$(printf '%s' "$CAL" | sed -n 's/.*new lambda = \([0-9.]*\).*/\1/p')
    echo
    echo "!! Camber arm outside +-20% of the differential arm at rep 1."
    echo "!! Per S202 S3: STOP, rescale lambda $CAM_LAM_DEG -> $NEWLAM deg, restart"
    echo "!! in a FRESH base. This rule fires at most once. Re-run with:"
    echo "!!   CAM_LAM_DEG=$NEWLAM BASE=${BASE}_cal2 CALIBRATED=1 bash $0"
    [ -n "$CALIBRATED" ] && echo "!! ...but CALIBRATED is already set: the rule has fired. STOP HERE."
    exit 2 ;;
  *"CALIBRATION: OK"*) echo "matched at rep 1; continuing." ;;
  *) echo "!! calibration check inconclusive (a rep-1 run missing or incoherent)."
     echo "!! Continuing -- P-M-1 will gate at the end on all runs." ;;
esac

# ---- REPS 2..N, interleaved -------------------------------------------------
for REP in $(seq 2 "$NPER"); do
  for CELL in $CELLS; do run_cell "$CELL" "$REP"; done
done

# ---- ANALYSIS --------------------------------------------------------------
echo
echo "==========================================================="
echo " ANALYSIS -- P-M-1 gates, then P-M-0, P-M-2..P-M-6"
echo "==========================================================="
python3 "$DIAG/score_matched_turn.py" --base "$BASE"
echo
echo "-- per-run detail (cross_track.py) --------------------------------------"
python3 "$DIAG/cross_track.py" --dir "$BASE/cam" --label cam --dir "$BASE/diff" --label diff
echo
echo "Done. Captures in $BASE. Record the verdicts in the log as they stand."
