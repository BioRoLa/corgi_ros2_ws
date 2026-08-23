#!/bin/bash
# P-M-1..P-M-3: the yaw controller causes the wander. Log S163.
#
# REGISTERED BEFORE THIS RAN -- S163, "not yet run". A failed gate binds (S126).
#
# WHY. S162 measured the front pair cambered -5.15/-5.30 deg and the rear pair
# +4.97/+4.98 -- equal and opposite, ~10 deg apart, with theta within 1.7 deg
# across all four, so it is camber and not leg length. It is gamma_correction()'s
# yaw term: yaw_sign is +1 on the front pair and -1 on the rear, so differential
# front/rear camber IS how this controller steers, and a heading error converts
# straight into that split.
#
# Two things make it pathological rather than merely visible:
#   gamma_limit defaults to 5.0 deg and the measured gamma IS -5.45/-5.65 and
#   +4.97/+5.04 -- the correction is PEGGED AT ITS CLAMP. The heading loop is
#   demanding the maximum lean it is allowed and the robot still drifts -5.5
#   deg/s.
#   gamma_yaw_limit -- the clamp introduced by S89's C2 for exactly this, "a
#   gentle hold... small enough that it cannot saturate into a commanded lean"
#   -- defaults to 0.0, which means OFF, and no campaign has ever set it.
#
# S163 then found the comparison already sitting in banked data: the `lam0` arm
# ran k_yaw:=0.0 d_yaw:=0.0 and wanders 5x less, ~20% straighter, slightly
# faster. n = 4 vs 3, one campaign, two different sessions. THIS campaign is
# the confirmation at n >= 5 in one session at the config of record.
#
# WHAT IS AND IS NOT BEING CLAIMED. Deleting heading feedback is not the
# proposal -- Stage 3 needs commanded turning and `lam0` has no heading hold at
# all. The claim is that THIS law, at THIS gain, UNCLAMPED, is
# counterproductive, and P-M-2 is the usable middle.
#
# ALL SIX CONFIGURATION ELEMENTS ARE NAMED. template_path, k_flight/b_flight and
# the two env vars are S159's four; gamma_yaw_limit is the fifth (S162) and is
# the very thing under test here; GAIT_SIM is the sixth (S166).
#
# No `set -u` -- ROS setup scripts reference unbound variables.

WS=~/corgi_ws/corgi_ros2_ws
DIAG="$WS/src/corgi_force_control/scripts/diag"
CFG="$WS/src/corgi_force_control/config"
# IS THE SIMULATED ROBOT THE RECORDED ROBOT? S198 / open issue #26. The Webots
# proto sat modified-and-uncommitted for six days, and every campaign in that
# window ran on a plant no checkout could rebuild -- `git diff` hid it, because
# it is a 103 MB LFS object and git diff shows only the pointer. This prints the
# plant identity into THIS campaign's own log, and refuses on a dirty or
# unbuilt plant. Self-tested: preflight_plant_selftest.sh, 7 planted cases.
. "$WS/src/corgi_force_control/scripts/diag/preflight_plant.sh"
preflight_plant || exit 1
# IS THE SIMULATOR FREE, AND QUIET? S203. Same one-file treatment as the plant
# guard above, for the same reason: these checks spread by copy-paste and only
# reached 7 of 25 campaigns (the WINDOWS-side webots.exe one) to 11 of 25 (the
# stale-launch one), and the variants disagreed about what to grep. This is the
# union. Self-tested: preflight_sim_selftest.sh, 9 faked-probe cases.
. "$WS/src/corgi_force_control/scripts/diag/preflight_sim.sh"
preflight_sim || exit 1
NPER=${NPER:-5}
# P-M-1 is registered at n >= 5, and it is a yaw-rate claim: identical-settings
# runs on this plant span 33.7-39.4% on front/rear desync, but the yaw-rate
# difference under test is 5x, far outside that.
GAIT_SIM=${GAIT_SIM:-24}
GAIT_WALL=${GAIT_WALL:-420}
BASE=${BASE:-/home/alexc/corgi_runs/yaw_hold}

TPL_ARG="template_path:=$CFG/gslip_pronk_template_v070.csv"
FLIGHT_ARGS="k_flight:=7150.0 b_flight:=115.8"

# name : k_yaw : d_yaw : gamma_yaw_limit
#   default   the config of record as every campaign has actually run it
#   nofb      S163's lam0 condition: heading feedback off entirely
#   clamp     P-M-2's usable middle: the gain stays, the yaw term is clamped
#             to ~1 deg so it cannot saturate into a commanded lean
CELLS=${CELLS:-"default:0.15:0.02:0.0 nofb:0.0:0.0:0.0 clamp:0.15:0.02:0.0175"}

echo "==========================================================="
echo " YAW HOLD campaign -- P-M-1..P-M-3 (log S163)"
echo "==========================================================="
echo " cells    : $CELLS   (name:k_yaw:d_yaw:gamma_yaw_limit)"
echo " n / cell : $NPER"
echo " template : $TPL_ARG"
echo " flight   : $FLIGHT_ARGS"
echo " window   : ${GAIT_SIM}s of SIM time per run (NOT wall), ${GAIT_WALL}s timeout"
echo " base     : $BASE"
echo
echo " BARS, registered in S163 and binding:"
echo "   P-M-1  nofb:  |yaw rate| < 2.0 deg/s  AND  straightness > 0.80"
echo "   P-M-2  clamp: lands BETWEEN default and nofb on straightness"
echo "                 (falsified if no better than default)"
echo "   P-M-3  CONJUNCTIVE: neither cell's v_fwd may fall below the"
echo "                 lam0_default median +0.252 by more than the spread"
echo

# ---- PREFLIGHT. Everything here runs before any sim time is spent. ---------


BIN="$WS/install/corgi_force_control/lib/corgi_force_control"
[ "$(strings "$BIN/gslip_pronk_node" 2>/dev/null | grep -c 'LEG-FRAME GAINS')" != 0 ] || {
  echo "!! gslip_pronk_node lacks the gain announcement -- rebuild"; exit 1; }
echo "controller carries the gain announcement."

# PROOF OF INTENT for the arm that matters. Until 2026-08-22 k_yaw was logged
# ONLY inside the `gamma_yaw_limit > 0` branch, so the nofb cell -- the whole
# point of this campaign -- had nothing to grep, and `k_yaw:=0.0` failing to
# reach the node would read as "the heading controller does nothing".
[ "$(strings "$BIN/gslip_pronk_node" 2>/dev/null | grep -c 'ATTITUDE GAINS')" != 0 ] || {
  echo "!! gslip_pronk_node lacks the ATTITUDE GAINS banner -- k_yaw cannot be"
  echo "!! certified on the nofb arm. Rebuild before running this."; exit 1; }
echo "controller carries the ATTITUDE GAINS banner."

grep -q "sweep_frac_fwd" "$DIAG/touchdown_phase.py" || {
  echo "!! analyser lacks the propulsion screen"; exit 1; }
[ -f "$CFG/gslip_pronk_template_v070.csv" ] || { echo "!! v070 missing"; exit 1; }
echo "v070 present, named explicitly on every cell."

GUARD=/home/alexc/corgi_runs/shift_duty_sweep/v070_db_stop_odo
gv() { python3 "$DIAG/check_menger.py" --odom-csv "$GUARD/odom_run$1.csv" \
         --torque-csv "$GUARD/run$1.csv" --start 12 2>/dev/null \
       | grep -o 'Menger R / R_fit  *[0-9.]*' | grep -o '[0-9.]*$'; }
G1=$(gv 1); G3=$(gv 3)
echo "estimator guard: $G1 / $G3 (expect 0.943 / 0.952)"
[ "$G1" = "0.943" ] && [ "$G3" = "0.952" ] || { echo "!! guard failed"; exit 1; }
echo "estimator guard OK."

# speed_from_odom owns straightness and yaw rate, which ARE the bars here.
python3 "$DIAG/speed_from_odom.py" --selftest > /tmp/yaw_sfo.out 2>&1 || {
  echo "!! speed_from_odom selftest FAILED -- P-M-1 cannot be scored:";
  tail -20 /tmp/yaw_sfo.out; exit 1; }
echo "speed_from_odom selftest PASS (straightness and yaw rate are its output)."
echo

[ -n "${PREFLIGHT_ONLY:-}" ] && { echo "PREFLIGHT_ONLY -- all gates passed."; exit 0; }

mkdir -p "$BASE"

# ---- RUNS, INTERLEAVED -----------------------------------------------------
#
# rep 1: default, nofb, clamp.  rep 2: default, nofb, clamp.  ... not all of
# `default` and then all of `nofb`.
#
# WHY. A blocked campaign confounds cell with TIME. S161 ran blocked and the
# machine changed between cell 1 and cells 2-4 -- a stray Webots appeared --
# so cell 1 captured 27-31 s of sim per run and the other three 10-13 s, and
# no comparison in it survived. Interleaving spreads any drift EVENLY across
# the cells instead of loading it onto the treatment arms, which is where a
# blocked design always puts it.
#
# It is free here. repeat_gain_regime.sh already tears down and relaunches the
# entire stack for every single run, so there is no per-cell setup to amortise
# and interleaving costs exactly zero extra wall clock.
#
# It also fails EARLIER, which is what you want: a catastrophic arm shows up in
# rep 1 rather than after two thirds of the campaign is spent, and every cell
# has at least one run banked when it does.
#
# Requires RUN_START in repeat_gain_regime.sh so a single-run invocation writes
# run$REP.csv rather than run1.csv every time.
if ! grep -q 'RUN_START' "$DIAG/repeat_gain_regime.sh"; then
  echo "!! repeat_gain_regime.sh has no RUN_START -- it would overwrite"
  echo "!! run1.csv on every repetition and silently keep only the last."
  echo "!! Apply the interleave patch first. REFUSING."
  exit 1
fi

for REP in $(seq 1 "$NPER"); do
  echo
  echo "%%%%%%%%%%%%%%%%  REPETITION $REP / $NPER  %%%%%%%%%%%%%%%%"
  for CELL in $CELLS; do
    NAME="${CELL%%:*}"; R1="${CELL#*:}"
    KY="${R1%%:*}"; R2="${R1#*:}"
    DY="${R2%%:*}"; GYL="${R2#*:}"
    echo
    echo "################################################################"
    echo "###  rep $REP, CELL $NAME : k_yaw $KY, d_yaw $DY, gamma_yaw_limit $GYL"
    if [ "$REP" = "1" ]; then
      case "$NAME" in
        nofb)
          echo "###  No heading feedback at all. On the render the front/rear"
          echo "###  camber split should VANISH -- that split IS the yaw"
          echo "###  controller. Expect a straighter path that is not actively"
          echo "###  held, so it may drift slowly whichever way it starts." ;;
        clamp)
          echo "###  Heading feedback on, yaw term clamped to ~1 deg. The split"
          echo "###  should be present but SMALL -- roughly a fifth of the"
          echo "###  default's 10 deg -- and no longer sitting pegged." ;;
        *)
          echo "###  The config of record as every campaign has actually run"
          echo "###  it: the familiar ~10 deg front-inward/rear-outward split,"
          echo "###  pegged at the 5 deg clamp, and ~-5.5 deg/s of drift." ;;
      esac
    fi
    echo "################################################################"
    OUT="$BASE/$NAME"
    mkdir -p "$OUT"
    # RETRY A COLD START, once. PORTED FROM sweep_camber_pattern.sh 2026-08-22
    # (S171 S6). The FIRST launch after an idle gap fails -- webots.exe dies
    # before the driver connects, or the driver never connects -- and the very
    # next launch succeeds (2 of 2 on 2026-08-22). repeat_gain_regime.sh prints
    # "skipping" and RETURNS 0, so a campaign SILENTLY LOSES that cell while
    # every other cell runs and looks fine. That is how a paired control was
    # lost. For THIS campaign it would silently reduce n for one arm of
    # P-M-1..P-M-3, which is worse than a crash because nothing announces it.
    for ATTEMPT in 1 2; do
      N=1 RUN_START=$REP OUTDIR="$OUT" RECORD_ODOM=1 \
        GAIT_SIM=$GAIT_SIM GAIT_WALL=$GAIT_WALL \
        CTL_ARGS="k_yaw:=$KY d_yaw:=$DY gamma_yaw_limit:=$GYL $FLIGHT_ARGS $TPL_ARG" \
        bash "$DIAG/repeat_gain_regime.sh"
      if [ -f "$OUT/run$REP.csv" ]; then
        [ "$ATTEMPT" = 2 ] && echo "  (cell $NAME rep $REP succeeded on RETRY -- cold start, S171 S6)"
        break
      fi
      if [ "$ATTEMPT" = 1 ]; then
        echo "  !! cell $NAME rep $REP produced NO CAPTURE. Cold-start failure"
        echo "  !! mode, not a result. Retrying ONCE before giving up."
      else
        echo "  !! cell $NAME rep $REP produced NO CAPTURE on either attempt."
        echo "  !! n is now REDUCED for this cell -- say so when scoring P-M-*."
      fi
    done
  done
done
# ---- ANALYSIS --------------------------------------------------------------

echo
echo "==========================================================="
echo " ANALYSIS"
echo "==========================================================="
ARGS=""
for CELL in $CELLS; do
  NAME="${CELL%%:*}"; ARGS="$ARGS --dir $BASE/$NAME --label $NAME"
done

echo
echo "--- P-M-1 / P-M-2 / P-M-3: yaw rate, straightness, speed ---"
echo "    P-M-1  nofb  |yaw| < 2.0 deg/s AND straightness > 0.80"
echo "    P-M-2  clamp between default and nofb on straightness"
echo "    P-M-3  CONJUNCTIVE: v_fwd not below +0.252 by more than the spread"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  echo "  cell $NAME:"
  python3 "$DIAG/speed_from_odom.py" "$BASE/$NAME"/odom_run*.csv 2>&1 | tail -8
done

echo
echo "--- the MECHANISM: is the camber split actually gone? (S162) ---"
echo "    default should read ~-5.2/-5.3 front, +5.0/+5.0 rear, pegged at 5 deg"
echo "    This is also the PROOF OF ACTION for k_yaw: with the heading loop off"
echo "    the split must VANISH, and no log line can fake that."
python3 "$DIAG/audit_gamma_decomp.py" --campaign "$BASE" 2>&1 | tail -40

echo
echo "--- validity screen: is each cell still a gait? (S152) ---"
# shellcheck disable=SC2086
python3 "$DIAG/touchdown_phase.py" $ARGS

echo
echo "--- torque, reported not gated ---"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  echo "  cell $NAME:"
  python3 "$DIAG/tau_demand_window.py" "$BASE/$NAME"/run*.csv 2>&1 | tail -4
done

echo
echo "==========================================================="
echo " P-M-1..P-M-3 are REGISTERED. A failed gate binds -- re-register"
echo " rather than rescue. And note what this campaign CANNOT settle:"
echo " it does not test commanded turning, which Stage 3 needs and which"
echo " the nofb cell has no mechanism for at all."
echo "==========================================================="
