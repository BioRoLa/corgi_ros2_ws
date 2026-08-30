#!/usr/bin/env bash
# Does the launch file forward every parameter the node declares?
#
# WHY. gslip_pronk.launch.py forwards ONLY DeclareLaunchArgument'ed names. A
# parameter declared in gslip_pronk.cpp but not in the launch file is dropped
# SILENTLY when passed as name:=value -- the node runs at its default, the
# campaign's config line says otherwise, and only a per-run banner grep (if
# one exists) catches it. S128 hit this with the steering parameters;
# S210's first dip campaign hit it again 3 months later with a new channel:
# 2 cells x 14 min wasted before the certify check caught it. Two declare
# sites that nothing reconciled. This reconciles them.
#
# Usage:  preflight_launch_args.sh            -> exit 0 if every declared
#                                                node parameter is forwarded
#         preflight_launch_args.sh --list     -> print the comparison
# Sourceable: `. preflight_launch_args.sh; preflight_launch_args || exit 1`
preflight_launch_args() {
  local WS=~/corgi_ws/corgi_ros2_ws
  local CPP="${PLA_CPP:-$WS/src/corgi_force_control/src/gslip_pronk.cpp}"
  local LP="${PLA_LAUNCH:-$WS/src/corgi_force_control/launch/gslip_pronk.launch.py}"
  local NODE LAUNCH MISSING
  # Joined into one line first: many declarations put the name on the line
  # AFTER declare_parameter<T>( -- a single-line grep missed 20 of them,
  # including the four dip parameters this check was written for.
  NODE=$(tr '
' ' ' < "$CPP" | grep -oE 'declare_parameter<[^>]+>\( *"[a-z0-9_]+"' | grep -oE '"[a-z0-9_]+"' | tr -d '"' | sort -u)
  LAUNCH=$(grep -oE "LaunchConfiguration\(['\"][a-z0-9_]+['\"]\)" "$LP" | grep -oE "['\"][a-z0-9_]+['\"]" | tr -d "'\"" | sort -u)
  MISSING=$(comm -23 <(echo "$NODE") <(echo "$LAUNCH"))
  if [ "${1:-}" = "--list" ]; then
    echo "node declares $(echo "$NODE" | wc -l) parameters; launch forwards $(echo "$LAUNCH" | wc -l)"
    echo "declared but NOT forwarded (passing these as name:=value is silently ignored):"
    echo "$MISSING" | sed 's/^/  /'
  fi
  if [ -n "$MISSING" ]; then
    echo "!! launch-args: $(echo "$MISSING" | wc -l) node parameter(s) are NOT forwarded by gslip_pronk.launch.py:"
    echo "$MISSING" | sed 's/^/!!   /'
    echo "!! A campaign passing any of these runs at the DEFAULT and says otherwise (S128, S210)."
    return 1
  fi
  echo "launch-args: every declared node parameter is forwarded by the launch file."
  return 0
}
preflight_launch_args_selftest() {
  # Plant a launch file that forwards everything but one declared name; the
  # check must fail and name it. Then the real files must pass.
  local WS=~/corgi_ws/corgi_ros2_ws T
  T=$(mktemp)
  # /g: the dict key and its LaunchConfiguration share ONE line; without /g only
  # the key is renamed, the name is still forwarded, and nothing is planted.
  sed 's/gamma_acker_dip_rearm_ms/gamma_acker_dip_rearm_XX/g'       "$WS/src/corgi_force_control/launch/gslip_pronk.launch.py" > "$T"
  if PLA_LAUNCH="$T" preflight_launch_args > /dev/null 2>&1; then
    echo "  FAIL selftest: a dropped parameter was not caught"; rm -f "$T"; return 1
  fi
  if ! PLA_LAUNCH="$T" preflight_launch_args 2>&1 | grep -q 'gamma_acker_dip_rearm_ms'; then
    echo "  FAIL selftest: the dropped parameter was not NAMED"; rm -f "$T"; return 1
  fi
  rm -f "$T"
  echo "  ok  a planted drop is caught and named"
  preflight_launch_args > /dev/null || { echo "  FAIL selftest: real files do not pass"; return 1; }
  echo "  ok  real files pass"
  echo "  SELFTEST PASS"
}
case "${BASH_SOURCE[0]}" in "$0")
  case "${1:-}" in --selftest) preflight_launch_args_selftest ;; *) preflight_launch_args "$@" ;; esac ;;
esac
