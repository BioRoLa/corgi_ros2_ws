#!/bin/bash
# P-E-0 .. P-E-3: does raising k_tangential tighten touchdown IN A GAIT THAT
# ACTUALLY PROPELS?
#
# REGISTERED BEFORE THIS RAN -- log S152 sec 5. A failed gate binds (S126).
# This supersedes P-D-1..P-D-3, which were WITHDRAWN BEFORE SCORING in S152
# because their primary statistic was found confounded before any arm
# comparison was made.
#
#   P-E-0  VALIDITY, scored FIRST, CONJUNCTIVE. An arm is admitted only if it
#          PROPELS: median stance sweep > 0, >= 55% of stance episodes rolling
#          forward, and net v_fwd > 0 over the tail. An arm failing any of
#          these is not a slower gait, it IS NOT A GAIT, and does not enter any
#          comparison no matter how good its sd(beta_TD) is.
#   P-E-1  PRIMARY. Among ADMITTED arms only, within-leg sd(beta_TD) falls with
#          k_tangential. The bar is the SAME-SESSION shipped-600 arm -- NOT the
#          banked 0.0547, which came from an arm this screen rejects.
#   P-E-2  THE CONFOUND, scored explicitly. Report sd(beta_TD) AGAINST net
#          forward speed for every arm. If sd falls only where speed falls, the
#          finding is locomotion degradation and is reported as such. A
#          repeatability gain is only a gain AT HELD SPEED.
#   P-E-3  OPERATING POINT, stated not inherited. k_flight/b_flight are passed
#          EXPLICITLY. See below.
#
# WHY P-E-0 EXISTS. S152: the banked kt_sweep's 1200 arm -- the one with the
# BEST sd(beta_TD), 0.0547 -- rolls BACKWARD in stance in both its runs, with
# forward-rolling episodes at 41%. A gait that stops locomoting has tighter
# touchdown angles trivially. Two of the four banked arms fail this screen.
# Alex caught it by watching the Webots render; three automated preflight gates
# and a registered prediction set did not.
#
# ---------------------------------------------------------------------------
# OPERATING POINT. This campaign runs at the CONFIG OF RECORD, explicitly:
#
#     k_flight 7150.0   b_flight 115.8      (SS103-109, adopted 2026-08-20)
#
# NOT the launch default (12000.0 / 150.0), which is a DIFFERENT operating
# point that the project measured at 1.87x the torque and 12% less forward
# speed, and rejected -- see Open Issue #19. The banked kt_sweep inherited that
# default silently, which is a large part of why it does not propel.
#
# This makes the banked 0.0547 non-comparable, which is why P-E-1's bar is the
# same-session 600 arm instead. That is the correct trade: an internally valid
# comparison on a gait that locomotes beats a cross-session comparison against
# a number from a gait that does not.
# ---------------------------------------------------------------------------
#
# No `set -u` -- ROS setup scripts reference unbound variables.

WS=~/corgi_ws/corgi_ros2_ws
DIAG="$WS/src/corgi_force_control/scripts/diag"
# IS THE SIMULATED ROBOT THE RECORDED ROBOT? S198 / open issue #26. The Webots
# proto sat modified-and-uncommitted for six days, and every campaign in that
# window ran on a plant no checkout could rebuild -- `git diff` hid it, because
# it is a 103 MB LFS object and git diff shows only the pointer. This prints the
# plant identity into THIS campaign's own log, and refuses on a dirty or
# unbuilt plant. Self-tested: preflight_plant_selftest.sh, 7 planted cases.
. "$WS/src/corgi_force_control/scripts/diag/preflight_plant.sh"
preflight_plant || exit 1
ARMS=${ARMS:-"600.0 1200.0 2400.0"}
REF_KT=${REF_KT:-600.0}
NPER=${NPER:-5}
BASE=${BASE:-/home/alexc/corgi_runs/kt2400}
# S159: template_path MUST be named. The controller's fallback is the
# v~1.20 template, not v070.
TPL_ARG=${TPL_ARG:-"template_path:=$WS/src/corgi_force_control/config/gslip_pronk_template_v070.csv"}
FLIGHT_ARGS=${FLIGHT_ARGS:-"k_flight:=7150.0 b_flight:=115.8"}

REF_DIR="$BASE/kt$REF_KT"

echo "==========================================================="
echo " k_tangential campaign -- P-E-0..P-E-3 (log S152 sec 5)"
echo "==========================================================="
echo " arms          : $ARMS"
echo " n per arm     : $NPER"
echo " reference     : $REF_KT  ($REF_DIR)"
echo " base          : $BASE"
echo " odom          : ON"
echo " flight gains  : $FLIGHT_ARGS   <- CONFIG OF RECORD, passed explicitly"
echo

# --- stale-launch check. A leftover sim from another session silently
# --- correlates every arm, and the harness's teardown only runs per-run.
STALE=$(pgrep -f 'Corgi_launch.py|gslip_pronk_node|webots_ros2_driver' 2>/dev/null | wc -l)
if [ "$STALE" != 0 ]; then
  echo "!! $STALE stale sim/controller process(es) already running:"
  pgrep -fa 'Corgi_launch.py|gslip_pronk_node|webots_ros2_driver' 2>/dev/null | head
  echo "!! Another agent may be using the simulator. REFUSING to start."
  exit 1
fi
echo "stale-launch check clean."

# --- the controller must carry the startup announcement, or the per-run
# --- proof-of-intent assert silently degrades to "cannot certify".
BIN="$WS/install/corgi_force_control/lib/corgi_force_control/gslip_pronk_node"
if [ -f "$BIN" ] && [ "$(strings "$BIN" | grep -c 'LEG-FRAME GAINS')" != 0 ]; then
  echo "controller carries the LEG-FRAME GAINS announcement."
else
  echo "!! controller binary does NOT carry the LEG-FRAME GAINS line."
  echo "!! Rebuild corgi_force_control first."
  exit 1
fi

# --- the analyser must carry the propulsion screen, or P-E-0 cannot be scored.
if grep -q "sweep_frac_fwd" "$DIAG/touchdown_phase.py"; then
  echo "analyser carries the P-E-0 propulsion screen."
else
  echo "!! touchdown_phase.py has no sweep_frac_fwd -- P-E-0 is unscoreable."
  exit 1
fi

# --- estimator guard, BEFORE any sim time. A check on the ANALYSIS TOOLCHAIN,
# --- so it runs on the known reference capture and must reproduce its recorded
# --- answer. Standing rule: gate analysers against invalid input.
GUARD=/home/alexc/corgi_runs/shift_duty_sweep/v070_db_stop_odo
guard_val() {
  python3 "$DIAG/check_menger.py" --odom-csv "$GUARD/odom_run$1.csv" \
      --torque-csv "$GUARD/run$1.csv" --start 12 2>/dev/null \
    | grep -o 'Menger R / R_fit  *[0-9.]*' | grep -o '[0-9.]*$'
}
G1=$(guard_val 1); G3=$(guard_val 3)
echo "estimator guard: run1 = ${G1:-<none>} (expect 0.943), run3 = ${G3:-<none>} (expect 0.952)"
if [ "$G1" != "0.943" ] || [ "$G3" != "0.952" ]; then
  echo "!! estimator guard FAILED to reproduce. REFUSING to spend sim time."
  exit 1
fi
echo "estimator guard OK."
echo

# PREFLIGHT_ONLY=1 exercises every gate above and stops. All four gates are
# cheap; a fault found here costs nothing, the same fault found afterwards
# costs the whole sweep.
if [ -n "${PREFLIGHT_ONLY:-}" ]; then
  echo "PREFLIGHT_ONLY set -- all gates passed, stopping before any sim time."
  exit 0
fi

mkdir -p "$BASE"

for KT in $ARMS; do
  echo
  echo "################################################################"
  echo "###  k_tangential = $KT   ($NPER runs, $FLIGHT_ARGS)"
  echo "################################################################"
  OUT="$BASE/kt$KT"
  mkdir -p "$OUT"
  N=$NPER OUTDIR="$OUT" RECORD_ODOM=1 \
    CTL_ARGS="k_tangential:=$KT $FLIGHT_ARGS $TPL_ARG" \
    KT_REF_DIR="$REF_DIR" KT_REF="$REF_KT" \
    bash "$DIAG/repeat_gain_regime.sh"
done

echo
echo "==========================================================="
echo " ANALYSIS"
echo "==========================================================="

echo
echo "--- P-E-0 VALIDITY (scored FIRST) + P-E-1 PRIMARY, same table ---"
echo "    An arm with sweep <= 0 or %fwd < 55% is NOT A GAIT and is"
echo "    REJECTED before sd(beta_TD) is read at all."
ARGS=""
for KT in $ARMS; do ARGS="$ARGS --dir $BASE/kt$KT --label kt$KT"; done
# shellcheck disable=SC2086
python3 "$DIAG/touchdown_phase.py" $ARGS

echo
echo "--- P-E-0 third clause + P-E-2: net forward speed per arm ---"
for KT in $ARMS; do
  echo "  k_t = $KT:"
  python3 "$DIAG/speed_from_odom.py" "$BASE/kt$KT"/odom_run*.csv 2>&1 | tail -5
done
echo
echo "  P-E-2: read sd(beta_TD) AGAINST these speeds. If sd falls only where"
echo "  speed falls, the finding is locomotion degradation, not stability."

echo
echo "--- COST (reported, NOT gated) ---"
for KT in $ARMS; do
  echo "  k_t = $KT:"
  python3 "$DIAG/tau_demand_window.py" "$BASE/kt$KT"/run*.csv 2>&1 | tail -6
done

echo
echo "--- gait quality, secondary ---"
for KT in $ARMS; do
  echo "  k_t = $KT  gait mode:"
  python3 "$DIAG/gait_mode.py" "$BASE/kt$KT"/run*.csv 2>&1 | tail -5
  echo "  k_t = $KT  desync:"
  python3 "$DIAG/desync_vs_gain_regime.py" "$BASE/kt$KT"/run*.csv 2>&1 \
      | sed -n '/run  *n/,/^$/p'
done

echo
echo "==========================================================="
echo " Engagement verdicts are printed by repeat_gain_regime.sh above,"
echo " per run (proof of intent) and per arm (proof of action). An arm"
echo " marked INVALID, or REJECTED by P-E-0, enters no comparison."
echo "==========================================================="
