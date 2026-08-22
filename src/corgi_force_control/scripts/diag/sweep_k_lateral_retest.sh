#!/usr/bin/env bash
# Re-test k_lateral = 7500 under the CORRECTED per-joint ceilings.
#
# WHY THE ORIGINAL 7500 WAS CHOSEN (S10-era): k_lateral was 4x over-stiff at
# 30000, giving kp_h = 972 N.m/rad against an intended 250. The over-stiff AB/AD
# fed coupling torque into the LEG motors -- 33-35 N.m against the 35 N.m
# ceiling -- while the model predicted 14. Dropping to 7500 gave kp_h 125 and
# peak leg torque 3-10 N.m.
#
# WHAT CHANGED: the shared 35 N.m clamp is gone (S32). Per joint it is now
#   leg  29.5 N.m  (HT-04 stall at 6:1)  -- LOWER than the 35 it was tuned against
#   ABAD 44.25 N.m (HT-04 stall at 9:1)  -- HIGHER
#
# So the two ceilings moved in OPPOSITE directions, and the binding constraint in
# the original tuning was the LEG, not the ABAD. That is the whole question here.
#
# PREDICTION, registered before running:
#   The coupling-torque mechanism is unchanged and the leg ceiling got TIGHTER
#   (35 -> 29.5), so raising k_lateral should cost MORE now than it did then.
#   The ABAD's extra headroom is irrelevant because at 7500 the ABAD was only at
#   3-10 N.m -- nowhere near either ceiling. Expect 7500 to stand, and 30000 to
#   look worse against 29.5 than it did against 35.
#
#   What would CHANGE the answer: leg saturation staying flat as k_lateral rises.
#   That would mean the coupling story does not reproduce, and k_lateral could be
#   raised -- which matters because S33 attributes the 1.5-4.0 deg camber
#   UNDERSHOOT to "the position loop's finite stiffness against lateral ground
#   reaction, not saturation". A stiffer AB/AD would hold commanded camber
#   better, and Stage 1 currently has to carry that undershoot as a bias.
#
# Template: v~0.70, the one S27 says the robot actually executes, and the one
# Stage 2a/3 will build on. RAMP_UNTIL=12 with a ~2.7 s anchor gives ~9.3 s of
# template time, about 35 strides -- enough for a stable saturation percentage.
set -o pipefail

WS=/home/alexc/corgi_ws/corgi_ros2_ws
HERE="$WS/src/corgi_force_control/scripts/diag"
TPL="$WS/src/corgi_force_control/config/gslip_pronk_template_v070.csv"
OUT=/home/alexc/corgi_ws/runs/2026-08-17_k_lateral_retest
mkdir -p "$OUT"

DRV="$WS/install/corgi_sim/local/lib/python3.10/dist-packages/corgi_driver_pkg/corgi_driver.py"
if ! grep -q "MAX_TORQUE_ABAD" "$DRV"; then
    echo "ABORT: installed driver has no per-joint ceilings. colcon build --packages-select corgi_sim"
    exit 1
fi

coverage () {   # $1 = npz
    python3 - "$1" <<'PY'
import sys
import numpy as np
try:
    d = np.load(sys.argv[1], allow_pickle=True)
except Exception as e:
    print(f"    COVERAGE: no dump ({e.__class__.__name__})")
    sys.exit(0)
a = float(d["anchor"])
cov = float(d["imu_t"].max()) - a
print(f"    COVERAGE: anchor {a:.2f}, {cov:.2f} s of template "
      f"-- {'OK' if cov >= 6.0 else 'SHORT'}")
PY
}

run () {   # $1 = k_lateral
    for N in 1 2 3; do
        TAG="kl${1%.0}_$N"
        echo "=== $TAG   k_lateral=$1"
        RAMP_UNTIL=12 RAMP_DUMP="$OUT/$TAG.npz" \
            bash "$HERE/ramp_cycle.sh" "$TPL" k_lateral:="$1" 2>&1 | tail -3
        cp /tmp/ramp_ctl.log "$OUT/$TAG.ctl.log" 2>/dev/null
        coverage "$OUT/$TAG.npz"
        echo
    done
}

# Values MUST carry a decimal point: the parameter is declared double, and
# `k_lateral:=7500` is parsed as an integer, which makes gslip_pronk abort with
# InvalidParameterTypeException before the sim even starts.
run 3000.0
run 7500.0
run 15000.0
run 30000.0

echo "K_LATERAL RETEST DONE"
