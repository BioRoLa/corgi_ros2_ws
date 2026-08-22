#!/bin/bash
# P-N-1..P-N-5: the clocked-torque feedforward, Lu & Lin eq 11's D term. Log S164.
#
# REGISTERED BEFORE THIS RAN. A failed gate binds (S126).
#
# WHY. eq 11 (Bioinspir. Biomim. 19 026017 p5, transcribed 2026-08-22 -- the
# paper is in the vault owner's collection, so this no longer rests on a
# docstring) damps the leg angle to the CLOCK'S rate:
#
#     tau = k_P*(theta_fp - theta) + k_D*(theta_fp_dot - theta_dot)
#
# The deployed controller damps to ZERO. The difference is an additive brake
# proportional to the commanded sweep rate and OPPOSING it. Now measurable:
# force_control adds f_clock = bz*(dbeta_ref*|hip->contact|)*e_t.
#
# THE SIZE OF THE TERM, AT THE CONFIG OF RECORD, NOT AT THE DEFAULT.
# S164 priced it at 8.0 N.m/motor from phi_dot_ref = 6.556 rad/s -- which is the
# v~1.20 DEFAULT template. v070 clocks at 3.0098 rad/s:
#
#   stance   bz  30.0 * 3.0098 * L(85.4)^2 = 6.37 N.m leg axis, 3.19 /motor
#   flight   bz 115.8 * -2.0383 * L^2      = 16.65 N.m leg axis, 8.33 /motor
#
# The FLIGHT brake is the larger one, which is why `both` is an arm and not a
# footnote: (b_flight/k_flight)*v_t = 0.0330 rad is 24% of the measured +0.1382
# rad touchdown tracking lag.
#
# ALL FOUR CONFIGURATION ELEMENTS ARE NAMED (S159). Omitting any one silently
# selects a different, older design point:
#   template_path      -> default is the v~1.20 template, NOT v070
#   k_flight/b_flight  -> default 12000/150, not the config of record 7150/115.8
#   CORGI_DIRBETA_TRANSFORM / CORGI_THETA_STOP -> set by repeat_gain_regime.sh
#
# No `set -u` -- ROS setup scripts reference unbound variables.

WS=~/corgi_ws/corgi_ros2_ws
DIAG="$WS/src/corgi_force_control/scripts/diag"
CFG="$WS/src/corgi_force_control/config"
NPER=${NPER:-3}
# Capture window in SIM seconds, not wall seconds. Makes the window a property
# of the experiment rather than of whatever else the machine is doing, which is
# what cost S161 its cross-cell comparability (S166).
#
# GAIT_SIM measures the GAIT span -- S0 is sampled after the settle -- so it is
# NOT comparable to S166's per-run table, which quotes the absolute end-of-run
# sim time (standup + settle + gait). On laggard/base the two differ a lot:
#
#   absolute end-of-run sim   30.84 / 30.44 / 27.36 s   <- S166's table
#   gait span (odom-covered)  22.85 / 22.92 / 21.94 s   <- what GAIT_SIM sets
#
# so the clean real-time factor is 22.9 s / 200 s wall = 0.114, NOT 0.154. An
# earlier version of this comment used the absolute figure, sized the timeout
# off it, and would have left a healthy run 18-23% from the wall -- rejecting
# 200 s for exactly the margin it then chose.
#
# 24 s of gait gives every analyser its full 20 s tail with 4 s of margin, and
# sits just above laggard/base's own 21.9-22.9 s so the `off` arm here is
# comparable to it. At RTF 0.114 that needs ~211 s of wall; GAIT_WALL is a
# TIMEOUT only, set to 420 s for 2x headroom, because a timeout that fires on a
# healthy run truncates it and that is the failure this gate exists to prevent.
GAIT_SIM=${GAIT_SIM:-24}
GAIT_WALL=${GAIT_WALL:-420}
BASE=${BASE:-/home/alexc/corgi_runs/clock_ff}

TPL_ARG="template_path:=$CFG/gslip_pronk_template_v070.csv"
FLIGHT_ARGS="k_flight:=7150.0 b_flight:=115.8"

# name : clock_ff_scale : clock_ff_phase
# `off` FIRST -- it is the reference arm for the engagement check and the
# harness validation gate. `both` LAST -- it puts 8.3 N.m/motor on an UNLOADED
# leg through flight, and if that goes wrong it should not cost the whole
# campaign.
CELLS=${CELLS:-"off:0.0:stance on:1.0:stance both:1.0:both"}

echo "==========================================================="
echo " CLOCK FEEDFORWARD campaign -- P-N-1..P-N-5 (log S164)"
echo "==========================================================="
echo " cells    : $CELLS   (name:clock_ff_scale:clock_ff_phase)"
echo " n / cell : $NPER   (SCREEN -- identical settings span 33.7-39.4% on"
echo "            front/rear desync, so only a large effect is visible)"
echo " template : $TPL_ARG"
echo " flight   : $FLIGHT_ARGS"
echo " base     : $BASE"
echo " window   : ${GAIT_SIM}s of SIM time per run (NOT wall), ${GAIT_WALL}s wall timeout"
echo "            gait span only; banked laggard/base was 21.9-22.9s -- S166"
echo

# ---- PREFLIGHT. Everything here runs before any sim time is spent. ---------

STALE=$(pgrep -f 'Corgi_launch.py|gslip_pronk_node|webots_ros2_driver' 2>/dev/null | wc -l)
if [ "$STALE" != 0 ]; then
  echo "!! stale sim processes -- REFUSING:"
  pgrep -fa 'Corgi_launch.py|gslip_pronk_node|webots_ros2_driver' | head
  echo "!! The simulator is SHARED. Another agent's campaign looks exactly"
  echo "!! like a leftover process from here. Ask before killing anything."
  exit 1
fi
echo "stale-launch check clean."

# The S161 confound was a Webots the stale-launch grep did not look for: a bare
# `webots` with no world argument, holding port 1234 and a third of the CPU for
# an hour. That grep only knew about Corgi_launch / gslip_pronk_node /
# webots_ros2_driver. Widen it, and check the load while we are here.
FOREIGN=$(pgrep -f 'usr/local/webots' 2>/dev/null | wc -l)
if [ "$FOREIGN" != 0 ]; then
  echo "!! a Linux-side Webots is running that is NOT the Corgi sim:"
  pgrep -fa 'usr/local/webots' | head
  echo "!! It will steal CPU and shorten every capture. REFUSING."
  exit 1
fi
LOAD=$(cut -d' ' -f1 /proc/loadavg)
if awk -v l="$LOAD" 'BEGIN{exit !(l > 4.0)}'; then
  echo "!! 1-minute load average is $LOAD before the campaign has started."
  echo "!! Webots throughput scales with this and the harness cannot buy it"
  echo "!! back. Find the load first. REFUSING."
  exit 1
fi
echo "no foreign Webots; load average $LOAD."

BIN="$WS/install/corgi_force_control/lib/corgi_force_control"
# The controller half: the parameter was read and the rate derived.
[ "$(strings "$BIN/gslip_pronk_node" 2>/dev/null | grep -c 'CLOCK FEEDFORWARD')" != 0 ] || {
  echo "!! gslip_pronk_node lacks the CLOCK FEEDFORWARD banner -- rebuild"; exit 1; }
[ "$(strings "$BIN/gslip_pronk_node" 2>/dev/null | grep -c 'LEG-FRAME GAINS')" != 0 ] || {
  echo "!! gslip_pronk_node lacks the gain announcement -- rebuild"; exit 1; }
# The impedance-law half: the term is computed, and its sign was asserted.
[ "$(strings "$BIN/force_control_node" 2>/dev/null | grep -c 'CLOCK FF ACTIVE')" != 0 ] || {
  echo "!! force_control_node lacks the in-law CLOCK FF ACTIVE line -- rebuild"; exit 1; }
[ "$(strings "$BIN/force_control_node" 2>/dev/null | grep -c 'CLOCK FF SIGN CHECK')" != 0 ] || {
  echo "!! force_control_node lacks the sign self-check -- rebuild"; exit 1; }
echo "both binaries carry all four announcements."

grep -q "sweep_frac_fwd" "$DIAG/touchdown_phase.py" || {
  echo "!! analyser lacks the propulsion screen"; exit 1; }
echo "analyser carries the propulsion screen."

[ -f "$CFG/gslip_pronk_template_v070.csv" ] || { echo "!! v070 missing"; exit 1; }
echo "v070 present, named explicitly on every cell."

# Estimator guard: refuse to spend sim time on a toolchain that has changed
# under us. Known reference capture, known answer.
GUARD=/home/alexc/corgi_runs/shift_duty_sweep/v070_db_stop_odo
gv() { python3 "$DIAG/check_menger.py" --odom-csv "$GUARD/odom_run$1.csv" \
         --torque-csv "$GUARD/run$1.csv" --start 12 2>/dev/null \
       | grep -o 'Menger R / R_fit  *[0-9.]*' | grep -o '[0-9.]*$'; }
G1=$(gv 1); G3=$(gv 3)
echo "estimator guard: $G1 / $G3 (expect 0.943 / 0.952)"
[ "$G1" = "0.943" ] && [ "$G3" = "0.952" ] || { echo "!! guard failed"; exit 1; }
echo "estimator guard OK."

# The two new analysers, both of which gate on KNOWN ANSWERS. leg_demand must
# reproduce S160's four min-vLeg cells (0.188/0.056/0.304/-0.030) or it is not
# fit to score P-N-2; clock_ff_engagement must reject a capture with no
# feedforward, reject a sign-inverted one, and confirm an injected one.
python3 "$DIAG/leg_demand.py" --selftest > /tmp/ff_lt.out 2>&1 || {
  echo "!! leg_demand selftest FAILED -- P-N-2 cannot be scored:"; tail -20 /tmp/ff_lt.out; exit 1; }
echo "leg_demand selftest PASS (reproduces S160's min vLeg on all four cells)."
python3 "$DIAG/clock_ff_engagement.py" --selftest > /tmp/ff_ct.out 2>&1 || {
  echo "!! clock_ff_engagement selftest FAILED:"; tail -20 /tmp/ff_ct.out; exit 1; }
echo "clock_ff_engagement selftest PASS (fooling, undetectable, sign-inversion)."
echo

[ -n "${PREFLIGHT_ONLY:-}" ] && { echo "PREFLIGHT_ONLY -- all gates passed."; exit 0; }

mkdir -p "$BASE"

# ---- RUNS ------------------------------------------------------------------

for CELL in $CELLS; do
  NAME="${CELL%%:*}"; REST="${CELL#*:}"
  SCALE="${REST%%:*}"; PHASE="${REST#*:}"
  echo
  echo "################################################################"
  echo "###  CELL $NAME : clock_ff_scale $SCALE, clock_ff_phase $PHASE"
  if [ "$PHASE" = "both" ]; then
    echo "###  ⚠ 8.3 N.m/motor on an UNLOADED leg through flight. Watch the"
    echo "###    render: a snappier return swing is expected; a leg that"
    echo "###    over-swings, hits the theta stop, or falls out of phase with"
    echo "###    the others is a REAL failure, not a rendering artefact."
  fi
  echo "################################################################"
  OUT="$BASE/$NAME"
  mkdir -p "$OUT"
  N=$NPER OUTDIR="$OUT" RECORD_ODOM=1 GAIT_SIM=$GAIT_SIM GAIT_WALL=$GAIT_WALL \
    CTL_ARGS="clock_ff_scale:=$SCALE clock_ff_phase:=$PHASE $FLIGHT_ARGS $TPL_ARG" \
    bash "$DIAG/repeat_gain_regime.sh"
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
echo "--- HARNESS VALIDATION (S113). The off cell must reproduce these or"
echo "--- the configuration is wrong and nothing below means anything:"
echo "---   beta_TD measured -0.0785 | commanded +0.0597 | lag +0.1382"
echo "--- v_fwd is REPORTED, NOT GATED: S113 gives +0.274..+0.284 and S160's"
echo "--- baseline at the same config gives +0.231. They disagree; the three"
echo "--- beta numbers bind."
python3 "$DIAG/touchdown_phase.py" --dir "$BASE/off" --label off

echo
echo "--- P-N-1: swept beta. Bar: > +0.1281 rad on the `on` cell, n >= 3 ---"
# shellcheck disable=SC2086
python3 "$DIAG/touchdown_phase.py" $ARGS

echo
echo "--- P-N-2 (DISCRIMINATING): min vLeg must RISE vs off ---"
echo "    Flat or falling separates this from S155's laggard mechanism."
# shellcheck disable=SC2086
python3 "$DIAG/leg_demand.py" $ARGS --odom

echo
echo "--- P-N-3 / P-N-5: torque, CONJUNCTIVE and binding ---"
echo "    P-N-3  on   cell: tau p99.5/motor <= off cell + 3.2 N.m"
echo "    P-N-5  both cell: tau p99.5/motor <= off cell + 11.5 N.m"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  echo "  cell $NAME:"
  python3 "$DIAG/tau_demand_window.py" "$BASE/$NAME"/run*.csv 2>&1 | tail -4
done

echo
echo "--- P-N-4: touchdown tracking lag on the `both` cell ---"
echo "    Bar: falls by >= 0.015 rad below the off cell's +0.1382"
echo "    (half the 0.0330 rad first-order estimate). Read track_err above."

echo
echo "--- speed and straightness, reported beside every gate ---"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  echo "  cell $NAME:"
  python3 "$DIAG/speed_from_odom.py" "$BASE/$NAME"/odom_run*.csv 2>&1 | tail -4
done

echo
echo "--- ENGAGEMENT, proof of action (the weak third check; the decisive"
echo "--- evidence is the per-run CLOCK FF ACTIVE line above) ---"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  [ "$NAME" = "off" ] && continue
  echo "  cell $NAME vs off:"
  python3 "$DIAG/clock_ff_engagement.py" --ref "$BASE/off" --arm "$BASE/$NAME" \
    --arm-dbeta-ref 3.0098 --b-tangential 30 2>&1 | tail -12
done

echo
echo "==========================================================="
echo " Every gate above is REGISTERED. A failed gate binds -- re-register"
echo " rather than rescue it, and score P-N-2 honestly: it is the clause"
echo " that separates this mechanism from S155's."
echo "==========================================================="
