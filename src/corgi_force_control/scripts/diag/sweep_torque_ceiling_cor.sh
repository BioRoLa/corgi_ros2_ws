#!/bin/bash
# P-Q-1..P-Q-3: is the speed shortfall torque-limited AT THE CONFIG OF RECORD?
#
# REGISTERED BEFORE THIS RAN. A failed gate binds (S126).
#
# WHY RE-RUN SOMETHING S28 ALREADY CLOSED. S28 (2026-08-12) swept the ceiling
# 35 / 70 / 200 at v070 and got 0.44 / 0.46 / 0.44 m/s -- "nearly 6x the ceiling
# changes speed by nothing" -- and closed the torque branch. Reading its harness
# (sweep_torque_ceiling.sh) shows it was measured on a plant that is NOT the
# configuration anything runs on today:
#
#   template_path       v070                         <- correct
#   k_flight/b_flight   NOT SET -> 12000/150         <- the REJECTED default
#   CORGI_DIRBETA_TRANSFORM  NOT SET -> 0            <- config of record is 1
#   CORGI_THETA_STOP         NOT SET -> 0            <- config of record is 1
#   protocol            ramp_cycle.sh, RAMP_UNTIL=10 <- a speed RAMP, not a
#                                                       steady-state pronk
#   plus steering gains steer_offset / k_steer_yaw that no current campaign uses
#
# Three of the five configuration elements S159 identified are wrong, and the
# protocol is a different experiment. That is why S28 reports 0.44-0.46 m/s
# where the config of record gives 0.24-0.32. The CONTRAST inside S28 was
# internally controlled -- all three ceilings shared the same wrong config -- so
# "6x changes nothing" is probably sound about THAT plant. Whether it transfers
# is open, and it is load-bearing: it is the reason "torque is not the
# constraint" is currently treated as settled.
#
# There is also a reason it might come out DIFFERENTLY, and it runs both ways.
# At k_flight 12000 the leg is torque-saturated for an unrelated reason (Open
# Issue #21, S153) -- S28 measured the ceiling effect on an already-saturated
# plant. At the config of record clipping is only 2.73% of samples, which argues
# the ceiling should matter even LESS. Registered accordingly.
#
# ALSO NOTE the default clamp has changed since S28: MAX_TORQUE_LEG is now
# 29.5 N.m (HT-04 stall at 6:1), not the 35.0 S28's footer mentions. The control
# cell here is 29.5, the actual hardware line, not S28's 35.
#
# ⚠ Raising CORGI_MAX_TORQUE does NOT model a stronger motor. It makes clipped
# demand observable. No result here may be read as "the robot would do this with
# better hardware" -- resolve_torque_limits()'s own docstring says so.
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
NPER=${NPER:-3}
GAIT_SIM=${GAIT_SIM:-24}
GAIT_WALL=${GAIT_WALL:-420}
BASE=${BASE:-/home/alexc/corgi_runs/tq_ceiling_cor}

TPL_ARG="template_path:=$CFG/gslip_pronk_template_v070.csv"
FLIGHT_ARGS="k_flight:=7150.0 b_flight:=115.8"

#   29.5   the real HT-04 stall line at 6:1, and today's default
#   70     S28's middle cell, kept so the two campaigns share a rung
#   200    effectively unclipped; S28 saw 80-121 N.m demanded here
CEILINGS=${CEILINGS:-"29.5 70 200"}

echo "==========================================================="
echo " TORQUE CEILING at the CONFIG OF RECORD -- P-Q-1..P-Q-3"
echo "==========================================================="
echo " ceilings : $CEILINGS N.m   (CORGI_MAX_TORQUE, leg and abad)"
echo " n / cell : $NPER"
echo " template : $TPL_ARG"
echo " flight   : $FLIGHT_ARGS"
echo " window   : ${GAIT_SIM}s of SIM time per run"
echo " base     : $BASE"
echo
echo " REGISTERED, and binding:"
echo "   P-Q-1  raising the ceiling 29.5 -> 200 (6.8x) does NOT raise median"
echo "          v_fwd above the 29.5 cell by more than the run-to-run spread."
echo "          FALSIFIED if it does -- which reopens the whole torque branch"
echo "          and means S28's verdict does not transfer off its own config."
echo "   P-Q-2  unclipped peak demand at 200 reproduces S28's 80-121 N.m band."
echo "          Falsified otherwise; a much lower peak would mean the"
echo "          config-of-record plant is a different load case entirely."
echo "   P-Q-3  CONTROL. The 29.5 cell must reproduce today's clock_ff/off:"
echo "          v_fwd inside [+0.241, +0.320] and tau p99.5 near 51.4 N.m."
echo "          If it does not, the harness moved and nothing else is readable."
echo

# ---- PREFLIGHT -------------------------------------------------------------

STALE=$(pgrep -f 'Corgi_launch.py|gslip_pronk_node|webots_ros2_driver' 2>/dev/null | wc -l)
if [ "$STALE" != 0 ]; then
  echo "!! stale sim processes -- REFUSING:"
  pgrep -fa 'Corgi_launch.py|gslip_pronk_node|webots_ros2_driver' | head
  exit 1
fi
FOREIGN=$(pgrep -f 'usr/local/webots' 2>/dev/null | wc -l)
if [ "$FOREIGN" != 0 ]; then
  echo "!! a Linux-side Webots is running that is NOT the Corgi sim -- REFUSING:"
  pgrep -fa 'usr/local/webots' | head
  exit 1
fi
LOAD=$(cut -d' ' -f1 /proc/loadavg)
if awk -v l="$LOAD" 'BEGIN{exit !(l > 4.0)}'; then
  echo "!! load average $LOAD before the campaign started -- REFUSING"; exit 1
fi
echo "machine clean; load average $LOAD."

# PORTED FROM sweep_camber_pattern.sh 2026-08-22 (S171 S6). Webots here is a
# WINDOWS binary invoked through /init, so a surviving instance is invisible to
# every WSL pgrep above. It holds port 1234, the next launch dies with exit 1
# plus a misleading "[Errno 13] Permission denied" on the temp .wbt, and the run
# produces no capture at all.
if command -v powershell.exe > /dev/null 2>&1; then
  WINWB=$(powershell.exe -NoProfile -Command \
      "@(Get-Process webots* -ErrorAction SilentlyContinue).Count" 2>/dev/null \
      | tr -d '\r\n ')
  case "$WINWB" in
    ''|*[!0-9]*) echo "windows-side Webots check: inconclusive ('$WINWB') -- continuing." ;;
    0) echo "windows-side Webots check: none running." ;;
    *) echo "!! $WINWB WINDOWS-side webots.exe still running. It holds port 1234"
       echo "!! and no WSL pgrep can see it. REFUSING."
       powershell.exe -NoProfile -Command \
         "Get-Process webots* | Select-Object Id,ProcessName,StartTime" 2>/dev/null
       exit 1 ;;
  esac
else
  echo "windows-side Webots check: powershell.exe not reachable -- continuing."
fi

# THE GUARD THAT S28 HAD TO LEARN THE HARD WAY. Its first attempt ran all nine
# runs at 35 N.m regardless of the env var, because the INSTALLED driver still
# had the ceiling hardcoded and corgi_sim had never been rebuilt. The result
# looked exactly like the right answer -- "raising it changes nothing" -- and
# was unfalsifiable after the fact. src is not install.
DRV=$WS/install/corgi_sim/local/lib/python3.10/dist-packages/corgi_driver_pkg/corgi_driver.py
[ -f "$DRV" ] || DRV=$(find "$WS/install/corgi_sim" -name corgi_driver.py 2>/dev/null | head -1)
if [ -z "$DRV" ] || ! grep -q "CORGI_MAX_TORQUE" "$DRV"; then
  echo "!! the INSTALLED driver has no CORGI_MAX_TORQUE hook:"
  echo "   ${DRV:-<not found>}"
  echo "!! Run: colcon build --packages-select corgi_sim"
  exit 1
fi
echo "installed driver carries the CORGI_MAX_TORQUE hook."
grep -q "Torque ceilings:" "$DRV" || {
  echo "!! the driver does not announce its ceilings -- the per-run assert"
  echo "!! cannot confirm the value reached the sim. REFUSING."; exit 1; }
echo "installed driver announces its ceilings (per-run assert can bind)."

BIN="$WS/install/corgi_force_control/lib/corgi_force_control"
[ "$(strings "$BIN/gslip_pronk_node" 2>/dev/null | grep -c 'LEG-FRAME GAINS')" != 0 ] || {
  echo "!! controller lacks the gain announcement -- rebuild"; exit 1; }
[ -f "$CFG/gslip_pronk_template_v070.csv" ] || { echo "!! v070 missing"; exit 1; }

GUARD=/home/alexc/corgi_runs/shift_duty_sweep/v070_db_stop_odo
gv() { python3 "$DIAG/check_menger.py" --odom-csv "$GUARD/odom_run$1.csv" \
         --torque-csv "$GUARD/run$1.csv" --start 12 2>/dev/null \
       | grep -o 'Menger R / R_fit  *[0-9.]*' | grep -o '[0-9.]*$'; }
G1=$(gv 1); G3=$(gv 3)
echo "estimator guard: $G1 / $G3 (expect 0.943 / 0.952)"
[ "$G1" = "0.943" ] && [ "$G3" = "0.952" ] || { echo "!! guard failed"; exit 1; }
echo "estimator guard OK."
echo

[ -n "${PREFLIGHT_ONLY:-}" ] && { echo "PREFLIGHT_ONLY -- all gates passed."; exit 0; }

mkdir -p "$BASE"

# ---- RUNS, INTERLEAVED -----------------------------------------------------
# Same reasoning as sweep_yaw.sh: a blocked design confounds cell with time,
# and S161 lost a whole campaign to exactly that. Free here, because every run
# already tears the stack down and relaunches.
if ! grep -q 'RUN_START' "$DIAG/repeat_gain_regime.sh"; then
  echo "!! repeat_gain_regime.sh has no RUN_START -- interleaving would"
  echo "!! overwrite run1.csv every repetition. REFUSING."; exit 1
fi

for REP in $(seq 1 "$NPER"); do
  echo
  echo "%%%%%%%%%%%%%%%%  REPETITION $REP / $NPER  %%%%%%%%%%%%%%%%"
  for TQ in $CEILINGS; do
    NAME="tq${TQ}"
    OUT="$BASE/$NAME"
    mkdir -p "$OUT"
    echo
    echo "################################################################"
    echo "###  rep $REP, CEILING $TQ N.m"
    if [ "$REP" = "1" ]; then
      case "$TQ" in
        29.5) echo "###  The real stall line. This is the control and it must"
              echo "###  reproduce clock_ff/off. Expect ~2.7% of samples clipped." ;;
        200)  echo "###  Effectively unclipped. The motors are ALLOWED to demand"
              echo "###  80-121 N.m here; watch for the robot doing something"
              echo "###  violent rather than fast. Torque it cannot use is the"
              echo "###  expected outcome, not extra speed." ;;
        *)    echo "###  Intermediate rung, shared with S28 for comparability." ;;
      esac
    fi
    echo "################################################################"
    # Exported here so the sim launch inside repeat_gain_regime.sh inherits it.
    # SIM_ASSERT greps the DRIVER's own startup line, so the assert confirms the
    # ceiling reached the simulator rather than merely that we set a variable.
    CEIL_FMT=$(awk -v t="$TQ" 'BEGIN{printf "%.2f", t}')
    # RETRY A COLD START, once. PORTED 2026-08-22 (S171 S6): the first launch
    # after an idle gap fails and the next succeeds. repeat_gain_regime.sh
    # prints "skipping" and RETURNS 0, so a campaign silently loses that cell
    # while the others run and look fine. Here it would quietly reduce n for
    # one ceiling, which is exactly the comparison P-Q-1..P-Q-3 rest on.
    for ATTEMPT in 1 2; do
      CORGI_MAX_TORQUE="$TQ" \
        SIM_ASSERT="Torque ceilings: leg $CEIL_FMT" \
        N=1 RUN_START=$REP OUTDIR="$OUT" RECORD_ODOM=1 \
        GAIT_SIM=$GAIT_SIM GAIT_WALL=$GAIT_WALL \
        CTL_ARGS="$FLIGHT_ARGS $TPL_ARG" \
        bash "$DIAG/repeat_gain_regime.sh"
      if [ -f "$OUT/run$REP.csv" ]; then
        [ "$ATTEMPT" = 2 ] && echo "  (cell tq$TQ rep $REP succeeded on RETRY -- cold start, S171 S6)"
        break
      fi
      if [ "$ATTEMPT" = 1 ]; then
        echo "  !! cell tq$TQ rep $REP produced NO CAPTURE. Cold-start failure"
        echo "  !! mode, not a result. Retrying ONCE before giving up."
      else
        echo "  !! cell tq$TQ rep $REP produced NO CAPTURE on either attempt."
        echo "  !! n is REDUCED for this ceiling -- say so when scoring P-Q-*."
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
for TQ in $CEILINGS; do ARGS="$ARGS --dir $BASE/tq$TQ --label tq$TQ"; done

echo
echo "--- P-Q-1: speed vs ceiling. Flat = S28 transfers; rising = it does not ---"
for TQ in $CEILINGS; do
  printf "  %-8s " "tq$TQ"
  python3 "$DIAG/speed_from_odom.py" "$BASE/tq$TQ"/odom_run*.csv 2>&1 | grep CELL
done

echo
echo "--- P-Q-2: what the motors actually DEMAND when allowed (pre-clamp) ---"
echo "    S28 saw 80-121 N.m at ceiling 200."
for TQ in $CEILINGS; do
  printf "  %-8s " "tq$TQ"
  python3 "$DIAG/tau_demand_window.py" "$BASE/tq$TQ"/run*.csv 2>&1 | grep CELL
done

echo
echo "--- P-Q-3: does the 29.5 control reproduce clock_ff/off? ---"
echo "    bars: v_fwd in [+0.241, +0.320], tau p99.5 ~ 51.4"
python3 "$DIAG/speed_from_odom.py" /home/alexc/corgi_runs/clock_ff/off/odom_run*.csv 2>&1 | grep CELL
python3 "$DIAG/tau_demand_window.py" /home/alexc/corgi_runs/clock_ff/off/run*.csv 2>&1 | grep CELL

echo
echo "--- validity + fidelity ---"
# shellcheck disable=SC2086
python3 "$DIAG/touchdown_phase.py" $ARGS
for TQ in $CEILINGS; do
  printf "  %-8s " "tq$TQ"
  python3 "$DIAG/playback_ratio.py" --dir "$BASE/tq$TQ" 2>&1 | grep -c "playback OK" \
    | xargs -I{} echo "{} runs with playback in band"
done

echo
echo "--- the touchdown angle alpha, since the ceiling changes the hop ---"
# shellcheck disable=SC2086
python3 "$DIAG/touchdown_velocity_angle.py" $ARGS

echo
echo "==========================================================="
echo " ⚠ A raised ceiling is an INSTRUMENT, not a stronger motor."
echo " Nothing here may be read as what the robot would do with"
echo " better hardware (resolve_torque_limits() says so itself)."
echo "==========================================================="
