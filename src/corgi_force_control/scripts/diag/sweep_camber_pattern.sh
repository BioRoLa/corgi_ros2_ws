#!/bin/bash
# P-S-1..P-S-4: the camber command PATTERN. Stage 3 task 5. Log S171 S5.
#
# REGISTERED BEFORE THIS RAN -- P-R-1..P-R-4 in S169, re-registered as
# P-S-1..P-S-4 in S171 S5 after P-R-4 failed. A failed gate binds (S126): the
# S169 run is NOT rescued by the new bars, it stays unscored. These are new
# predictions on new data, with a PAIRED lam0 control the first design lacked.
#
# WHAT IS BEING CONFIRMED. Thesis Timeline Stage 3 task 5: the camber command
# pattern is gamma = lambda * {+1,-1,-1,+1}, the LEFT/RIGHT selector, NOT the
# front/rear couple gamma_correction() uses. "Worth one confirming run before
# building on it" -- and every lean task sits on it.
#
# WHY IT IS GENUINELY UNRUN. The pattern is already in the controller:
# gamma_openloop() applies lr_sign[4] = {+1,-1,-1,+1}, and with
# gamma_acker_in == gamma_acker_out and gamma_acker_dir = +1 it reduces to
# exactly lambda * lr_sign. But S88/S90 only ever drove this channel as an
# ACKERMANN PAIR -- menger_acker_campaign.sh sets ACK_IN != ACK_OUT
# (0.17453 / 0.13765 at lam10). The UNIFORM left/right pattern has never run
# in the pronk. S33's uniform-camber null was WHEEL mode, explained by the
# four-wheel-cart argument, which does not apply to a gait with flight.
# So this run separates THE PATTERN from THE ACKERMANN SPLIT.
#
# WHY IT IS NEWLY CLEAN. S168 adopted k_yaw = d_yaw = 0.0 at ~14:44 on
# 2026-08-22. gamma_correction()'s YAW term -- which produced the -5.179 deg
# front/rear split that dominated every previous run -- is now identically
# zero. What survives is the ROLL term, roll_sign * (k_roll*roll + d_roll*wx),
# k_roll 0.25, clamped at gamma_limit 5 deg. roll_sign is {+1,-1,-1,+1}: THE
# SAME left/right partition as lr_sign. So the roll feedback acts on exactly
# the channel the open-loop command uses and must oppose it as soon as the
# commanded lean produces body roll. Pricing that opposition is P-R-2, and it
# has no prior -- the yaw term used to swamp it.
#
# The template contributes nothing: v070's gamma column is identically
# 0.000000 on all 266 rows (checked below, not assumed). So cmd->gamma is
# gamma_openloop() plus gamma_correction() and nothing else.
#
# gamma_acker_limit is 20 deg (its OWN clamp), so the 5 deg gamma_limit on
# gamma_correction does NOT clip the open-loop term. Independent clamps are
# what make P-R-2 readable.
#
# ALL SIX CONFIGURATION ELEMENTS ARE NAMED. template_path, k_flight/b_flight
# and the two env vars are S159's four (the env vars default correctly inside
# repeat_gain_regime.sh -- read, not assumed); GAIT_SIM is S166's; k_yaw is now
# 0.0 by default since S168, so it is no longer an element that defaults away.
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
NPER=${NPER:-1}
GAIT_SIM=${GAIT_SIM:-24}
GAIT_WALL=${GAIT_WALL:-420}
BASE=${BASE:-/home/alexc/corgi_runs/camber_pattern}

TPL_ARG="template_path:=$CFG/gslip_pronk_template_v070.csv"
FLIGHT_ARGS="k_flight:=7150.0 b_flight:=115.8"

# lambda = 10 deg = 0.17453 rad, matching S88's lam10 INNER magnitude exactly
# so the achieved camber is comparable to the campaign that measured the
# authority. in == out is what makes it the uniform pattern rather than a pair.
LAM_RAD=${LAM_RAD:-0.17453}
LAM_DEG=${LAM_DEG:-10}
# MUST carry a decimal point. These are declared as DOUBLE parameters, and ROS
# types a bare `1` as an INTEGER: gslip_pronk_node then aborts at startup with
# InvalidParameterTypeException and the run dies before the hold. Cost run 2 on
# 2026-08-22 (S171). menger_acker_campaign.sh writes dir=1.0 for this reason.
ACK_DIR=${ACK_DIR:-1.0}

# PAIRED DESIGN, S171 S5. The first version used an ABSOLUTE flight threshold
# to answer a DIFFERENTIAL question ("did the camber break the gait?"), and the
# threshold it borrowed -- audit_gamma_decomp's flight > 25% -- fails S167's own
# `off` control on 3 of 3 runs. The control belongs IN the campaign, on the same
# plant in the same session, not in a constant imported from an analyser.
#   name : lambda (rad) : dir
#   lam0   camber channel fully OFF (dir 0.0 makes gamma_openloop return
#          literal 0.0), so cmd->gamma is gamma_correction's roll term alone
#   lr10   the uniform left/right pattern under test
CELLS=${CELLS:-"lam0:0.0:0.0 lr10:$LAM_RAD:$ACK_DIR"}

echo "==========================================================="
echo " CAMBER COMMAND PATTERN -- P-S-1..P-S-4 (log S171 S5)"
echo "==========================================================="
echo " pattern  : gamma = lambda * {+1,-1,-1,+1}  (lr_sign, A=FL B=FR C=RR D=RL)"
echo " lambda   : $LAM_RAD rad = $LAM_DEG deg, in == out, dir $ACK_DIR"
echo " n        : $NPER"
echo " template : $TPL_ARG"
echo " flight   : $FLIGHT_ARGS"
echo " window   : ${GAIT_SIM}s of SIM time per run (NOT wall), ${GAIT_WALL}s timeout"
echo " base     : $BASE"
echo
echo " BARS: P-S-1..P-S-4, RE-REGISTERED in S171 S5 and binding."
echo " (P-R-1..P-R-4 of S169 are spent: P-R-4 failed, so P-R-1..3 are"
echo "  UNSCORED and are NOT rescued here. These are new bars on new data.)"
echo "   P-S-1  lr10: achieved |L/R| > 5.0 deg, |F/R| < 1.5, |common| < 1.5"
echo "          lam0: |L/R| < 1.0 deg  (banked prior: S168 nofb +0.116)"
echo "   P-S-2  lr10: ERROR projection on L/R in [-3.0, 0.0] deg"
echo "          (delivery 70-100% of command, shortfall signed AGAINST it)"
echo "   P-S-3  rollMED at lr10 is NEGATIVE and differs from its PAIRED lam0"
echo "          control by more than 1.0 deg"
echo "   P-S-4  CONJUNCTIVE, SCORED FIRST. Both runs: full 24 s sim span,"
echo "          playback in band, flight >= 15%, all-down < 60%."
echo "          AND lr10's flight within 5 points of lam0's."
echo "          A larger drop = the camber degraded the gait, which is a"
echo "          FINDING and is recorded as one, not scored as a pass."
echo "          If P-S-4 fails NOTHING else is scored."
echo
echo " Why 15% and not audit_gamma_decomp's 25%: that gate fails S167's own"
echo " clock_ff/off control on 3 of 3 runs (23.4/22.8/20.0%). This plant's"
echo " flight sits at 19-27%, so 25% splits one population near its middle."
echo " The analyser's own threshold is left ALONE (other sections use it);"
echo " P-S-4 is scored from the printed flight numbers instead. See S171 S2."
echo
echo " NOT claimed at n = 1: steering authority, curvature magnitude, speed."
echo " The plant has no heading hold since S168; drift is unbounded."
echo

# ---- PREFLIGHT. Everything here runs before any sim time is spent. ---------


BIN="$WS/install/corgi_force_control/lib/corgi_force_control"
[ "$(strings "$BIN/gslip_pronk_node" 2>/dev/null | grep -c 'LEG-FRAME GAINS')" != 0 ] || {
  echo "!! gslip_pronk_node lacks the gain announcement -- rebuild"; exit 1; }
echo "controller carries the gain announcement."

# PROOF OF INTENT. k_yaw must be certifiable as 0.0 from the run's own log:
# the whole reason this run is clean is that the yaw term is off, and a k_yaw
# that quietly came back would look exactly like "the pattern leaks into F/R"
# -- i.e. it would falsify P-R-1 for entirely the wrong reason.
[ "$(strings "$BIN/gslip_pronk_node" 2>/dev/null | grep -c 'ATTITUDE GAINS')" != 0 ] || {
  echo "!! gslip_pronk_node lacks the ATTITUDE GAINS banner -- k_yaw cannot be"
  echo "!! certified. Rebuild before running this."; exit 1; }
echo "controller carries the ATTITUDE GAINS banner."

# The open-loop camber channel must exist in the INSTALLED binary. src is not
# install -- S28's ceiling sweep ran nine runs against a stale driver and the
# result looked exactly like the right answer.
[ "$(strings "$BIN/gslip_pronk_node" 2>/dev/null | grep -c 'gamma_acker')" != 0 ] || {
  echo "!! the INSTALLED gslip_pronk_node has no gamma_acker parameters --"
  echo "!! the camber command would silently do nothing. Rebuild. REFUSING."
  exit 1; }
echo "installed controller carries the gamma_acker channel."

[ -f "$CFG/gslip_pronk_template_v070.csv" ] || { echo "!! v070 missing"; exit 1; }
NZ=$(awk -F, 'NR>1 && $4+0 != 0 {n++} END {print n+0}' "$CFG/gslip_pronk_template_v070.csv")
[ "$NZ" = "0" ] || {
  echo "!! v070 gamma column is NOT identically zero ($NZ rows) -- the template"
  echo "!! would contribute camber and P-R-1 would not be readable. REFUSING."
  exit 1; }
echo "v070 present, named explicitly, gamma column identically zero."

# audit_gamma_decomp OWNS P-R-1 and P-R-2. Gate it before spending sim time:
# its selftest plants a known offset on ONE pure partition at a time and must
# recover that projection and put ~0 in the others.
python3 "$DIAG/audit_gamma_decomp.py" --selftest > /tmp/cp_agd.out 2>&1 || {
  echo "!! audit_gamma_decomp selftest FAILED -- P-R-1/P-R-2 cannot be scored:"
  tail -20 /tmp/cp_agd.out; exit 1; }
echo "audit_gamma_decomp selftest PASS (it owns P-S-1 and P-S-2)."

# body_attitude owns P-R-3, and its signed roll columns were added TODAY for
# this run. Refuse if missing -- an unsigned roll cannot answer a question
# whose entire content is a direction.
grep -q "roll_med" "$DIAG/body_attitude.py" || {
  echo "!! body_attitude.py has no signed roll_med -- P-R-3 cannot be scored."
  exit 1; }
echo "body_attitude carries signed roll (P-S-3)."

# DOUBLE-TYPED PARAMETERS MUST LOOK LIKE DOUBLES. A bare integer makes
# gslip_pronk_node abort at startup (InvalidParameterTypeException) BEFORE the
# hold, so the failure reads as "controller never reached hold" and says
# nothing about the type. Refuse here instead, where the message is useful.
for PV in "$LAM_RAD" "$ACK_DIR"; do
  case "$PV" in
    *.*) ;;
    *) echo "!! '$PV' has no decimal point. gamma_acker_* are DOUBLE"
       echo "!! parameters and ROS types a bare integer as int, which aborts"
       echo "!! the controller at startup. Write it as e.g. ${PV}.0. REFUSING."
       exit 1 ;;
  esac
done
echo "double-typed launch args carry decimal points."

echo
[ -n "${PREFLIGHT_ONLY:-}" ] && { echo "PREFLIGHT_ONLY -- all gates passed."; exit 0; }

mkdir -p "$BASE"

# ---- RUN -------------------------------------------------------------------

# Interleaved by repetition, blocked only within a repetition. With NPER = 1
# this is simply control-then-treatment back to back on the same plant, which
# is the whole point: the pair is what P-S-3 and P-S-4 are scored on.
for REP in $(seq 1 "$NPER"); do
  for CELL in $CELLS; do
    NAME="${CELL%%:*}"; REST="${CELL#*:}"
    LAM="${REST%%:*}"; DIR="${REST#*:}"
    OUT="$BASE/$NAME"
    mkdir -p "$OUT"
    echo
    echo "################################################################"
    if [ "$NAME" = "lam0" ]; then
      echo "###  rep $REP, CELL lam0 : PAIRED CONTROL, camber channel OFF"
      echo "###  ON THE RENDER: legs vertical, no visible camber at all."
      echo "###  This is the run S171 says was missing -- without it, a low"
      echo "###  flight fraction in the treatment cannot be blamed on the"
      echo "###  camber rather than on the plant."
    else
      echo "###  rep $REP, CELL lr10 : gamma = $LAM_DEG deg * {+1,-1,-1,+1}, dir $DIR"
      echo "###  ON THE RENDER: all four legs visibly cambered in a LEFT/RIGHT"
      echo "###  pattern -- the two left legs (A=FL, D=RL) one way, the two"
      echo "###  right legs (B=FR, C=RR) the other. Front and rear on the SAME"
      echo "###  side should MATCH. If you see FRONT-vs-REAR instead, the yaw"
      echo "###  term is back and the run is void."
      echo "###  The body should lean and drift right. It is NOT heading-held"
      echo "###  since S168, so slow uncorrected drift is expected."
    fi
    echo "################################################################"
    # RETRY A COLD START, once. Measured 2026-08-22 (S171 S6): the FIRST
    # launch after an idle gap fails -- either webots.exe dies before the
    # driver connects, or the driver never connects at all -- and the very
    # next launch succeeds. repeat_gain_regime.sh prints "skipping" and
    # returns 0, so a campaign SILENTLY LOSES that cell. That is how the
    # paired control was lost on the first attempt at this campaign, which
    # is worse than a crash: the treatment cell still ran and looked fine.
    for ATTEMPT in 1 2; do
      N=1 RUN_START=$REP OUTDIR="$OUT" RECORD_ODOM=1 \
        GAIT_SIM=$GAIT_SIM GAIT_WALL=$GAIT_WALL \
        CTL_ARGS="gamma_acker_in:=$LAM gamma_acker_out:=$LAM gamma_acker_dir:=$DIR $FLIGHT_ARGS $TPL_ARG" \
        bash "$DIAG/repeat_gain_regime.sh"
      if [ -f "$OUT/run$REP.csv" ]; then
        [ "$ATTEMPT" = 2 ] && echo "  (cell $NAME succeeded on RETRY -- cold start, S171 S6)"
        break
      fi
      if [ "$ATTEMPT" = 1 ]; then
        echo "  !! cell $NAME produced NO CAPTURE. This is the cold-start"
        echo "  !! failure mode, not a result. Retrying ONCE before giving up."
      else
        echo "  !! cell $NAME produced NO CAPTURE on either attempt. The pair"
        echo "  !! is incomplete and P-S-4 fails -- do NOT score the other cell."
      fi
    done
  done
done

# ---- ANALYSIS --------------------------------------------------------------

echo
echo "==========================================================="
echo " ANALYSIS -- P-S-4 FIRST, then P-S-1..P-S-3"
echo "==========================================================="
echo
echo "-- P-S-4a: engagement. k_yaw must read 0.0000 on BOTH cells ----------"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  echo "  [$NAME]"
  grep -h -o "ATTITUDE GAINS.*" "$BASE/$NAME"/ctl_run*.log 2>/dev/null | head -2
done
echo
echo "-- P-S-4b: flight fraction, BOTH cells. Bar: >= 15% each, AND lr10 --"
echo "-- within 5 POINTS of lam0. Scored off the NUMBER, not the refusal. --"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"; REST="${CELL#*:}"
  LAM="${REST%%:*}"; DIR="${REST#*:}"
  LD=0; [ "$DIR" != "0.0" ] && LD=$LAM_DEG
  for n in $(seq 1 "$NPER"); do
    TQ="$BASE/$NAME/run$n.csv"
    if [ ! -f "$TQ" ]; then
      echo "  $NAME run$n: NO CAPTURE -- P-S-4 fails for the pair."
      continue
    fi
    OUTP=$(python3 "$DIAG/audit_gamma_decomp.py" --torque-csv "$TQ" \
             --odom-csv "$BASE/$NAME/odom_run$n.csv" \
             --gamma-in "$LD" --gamma-out "$LD" --gamma-dir "$DIR" 2>&1)
    if echo "$OUTP" | grep -q "gait gate FAIL"; then
      echo "  $NAME run$n: $(echo "$OUTP" | grep -o 'flight[^)]*)')"
      echo "     ^ that is audit_gamma_decomp's OWN 25% gate refusing. Score"
      echo "       P-S-4 off the flight NUMBER above, not off the refusal:"
      echo "       the same gate fails S167's clock_ff/off 3 of 3 (S171 S2)."
    else
      echo "  $NAME run$n: audit gate passed (flight > 25%)"
    fi
  done
done
echo
echo "-- P-S-1 / P-S-2: gamma decomposition, BOTH cells --------------------"
echo "-- lr10's commanded L/R is +$LAM_DEG.000; the tool prints the ERROR. --"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"; REST="${CELL#*:}"
  LAM="${REST%%:*}"; DIR="${REST#*:}"
  LD=0; [ "$DIR" != "0.0" ] && LD=$LAM_DEG
  echo "  == $NAME (cmd L/R = $LD deg) =="
  for n in $(seq 1 "$NPER"); do
    [ -f "$BASE/$NAME/run$n.csv" ] || continue
    python3 "$DIAG/audit_gamma_decomp.py" --torque-csv "$BASE/$NAME/run$n.csv" \
      --odom-csv "$BASE/$NAME/odom_run$n.csv" \
      --gamma-in "$LD" --gamma-out "$LD" --gamma-dir "$DIR" 2>&1 | sed 's/^/    /'
  done
done
echo
echo "-- P-S-3: signed roll, PAIRED. lr10 negative AND >1.0 deg from lam0 --"
ARGS=""
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  [ -d "$BASE/$NAME" ] && ARGS="$ARGS --dir $BASE/$NAME --label $NAME"
done
# shellcheck disable=SC2086
python3 "$DIAG/body_attitude.py" $ARGS
echo
echo "-- CONTEXT ONLY, NOT A BAR: speed / straightness / yaw rate ----------"
echo "-- n = 1 per cell and no heading hold. Not a measurement.           --"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  python3 "$DIAG/speed_from_odom.py" --dir "$BASE/$NAME" || \
    echo "  ($NAME: speed_from_odom non-zero -- usually NO odom capture.)"
done
echo
echo "Done. Score P-S-4 FIRST, on the PAIR. A gate-failed or unpaired run is"
echo "re-run material, NOT a result. See S171 S5."
