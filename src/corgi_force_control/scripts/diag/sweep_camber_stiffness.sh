#!/usr/bin/env bash
# Does more AB/AD stiffness shrink S33's camber undershoot?
#
# ---------------------------------------------------------------------------
# READ THIS BEFORE ASSUMING THIS TESTS k_lateral. IT DOES NOT, AND CANNOT.
#
# k_lateral is a gslip_pronk parameter. It reaches the AB/AD only as
# gslip_pronk -> force_control -> K_joint(2,2) -> kp_h. **This rig launches
# neither node**: camber_cycle.sh brings up corgi_sim and then camber_roll.py,
# which publishes position commands straight to /motor/command with its own
# hardcoded gains. Passing k_lateral here changes nothing at all -- two arms
# would come back identical and the null would be an artefact.
#
# kp_h IS the lateral-stiffness knob for this rig, so that is what is swept.
# For reference, the implementation log records k_lateral 7500 -> kp_h ~125 and
# 30000 -> kp_h ~972, which are not linearly consistent because K_joint depends
# on pose -- so treat the mapping as indicative and read the result against kp_h.
# The rig's own default is 90, SOFTER than the pronk's 7500-equivalent.
# ---------------------------------------------------------------------------
#
# S33 measured a worst-leg camber undershoot of 1.5 / 3.0 / 4.0 deg at
# lambda = 10 / 20 / 30, and attributed it to "the position loop's finite
# stiffness (kp_h = 90) against the lateral ground reaction, not saturation" --
# peak |tau_h| was only 3.8-7.0 N.m against a 44.25 ceiling. If that attribution
# is right, raising kp_h should shrink the undershoot roughly in proportion,
# and torque should rise but stay well under the ceiling.
#
# PREDICTION, registered before running:
#   Undershoot falls monotonically with kp_h and roughly as 1/kp_h, since a
#   position loop against a near-constant lateral load gives error ~ F/kp.
#   From 90 -> 500 (5.6x) the 3.0 deg undershoot at lambda 20 should drop to
#   well under 1 deg. Torque rises but stays under 44.25, because S33 showed the
#   joint is loafing in wheeled mode.
#
#   What would FALSIFY the attribution: undershoot flat in kp_h. That would mean
#   the error is not the position loop -- backlash, the 6 arcmin gearbox spec, or
#   a kinematic offset -- and no gain change fixes it.
set -o pipefail

HERE=/home/alexc/corgi_ws/corgi_ros2_ws/src/corgi_force_control/scripts/diag
# IS THE SIMULATED ROBOT THE RECORDED ROBOT? S198 / open issue #26. The Webots
# proto sat modified-and-uncommitted for six days, and every campaign in that
# window ran on a plant no checkout could rebuild -- `git diff` hid it, because
# it is a 103 MB LFS object and git diff shows only the pointer. This prints the
# plant identity into THIS campaign's own log, and refuses on a dirty or
# unbuilt plant. Self-tested: preflight_plant_selftest.sh, 7 planted cases.
. "$HERE/preflight_plant.sh"
preflight_plant || exit 1
# IS THE SIMULATOR FREE, AND QUIET? S202. Same one-file treatment as the plant
# guard above, for the same reason: these checks spread by copy-paste and only
# reached 7 of 25 campaigns (the WINDOWS-side webots.exe one) to 11 of 25 (the
# stale-launch one), and the variants disagreed about what to grep. This is the
# union. Self-tested: preflight_sim_selftest.sh, 9 faked-probe cases.
. "$HERE/preflight_sim.sh"
preflight_sim || exit 1
export ROLL_TIME=10        # S33 used 20 for a circle fit; the undershoot is a
                           # steady-state hold and does not need the distance.

run () {   # $1 = lambda deg, $2 = kp_h
    TAG="kph$2_lam$1"
    echo "=== lambda $1, kp_h $2  -> camber_$TAG"
    CAMBER_EXTRA="--kp-h $2" bash "$HERE/camber_cycle.sh" "$1" lr "$TAG" \
        2>&1 | tail -4
    echo
}

# Stiffness response at lambda 20 (S33: 3.0 deg undershoot).
for K in 90 125 250 500; do run 20 "$K"; done

# Confirm the trend at lambda 30, where the undershoot was largest (4.0 deg).
for K in 90 500; do run 30 "$K"; done

echo "CAMBER STIFFNESS SWEEP DONE"
