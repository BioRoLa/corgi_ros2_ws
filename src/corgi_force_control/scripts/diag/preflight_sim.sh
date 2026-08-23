# preflight_sim.sh -- IS THE SIMULATOR FREE, AND QUIET?
#
# Source this and call preflight_sim. Companion to preflight_plant.sh: that one
# asks "is this the recorded robot?", this one asks "is the machine in a state
# where the answer will mean anything?".
#
# WHY IT IS ONE FILE AND NOT 25 COPIES (S202). These checks spread by
# copy-paste and drifted badly. Before this file, of the 25 campaign scripts:
#   stale-launch check        11
#   Linux-side foreign Webots  6
#   load average               7
#   WINDOWS-side webots.exe    7   <- the one S171 added, and S166 predicted
#                                     "every sweep in this directory has this
#                                     blind spot"
#   nothing at all            13
# and the variants disagreed about WHAT to grep: sweep_contact_lateral matched
# the *_cycle.sh wrappers but NOT gslip_pronk_node, so a bare controller was
# invisible to it. This file implements the UNION, so every campaign gets the
# strictest version anyone has written.
#
# THE FAILURES THESE EXIST FOR, all paid for in real campaigns:
#   - S171 run 1: a WINDOWS-side webots.exe still held port 1234. Webots here is
#     /mnt/c/Program Files/Webots/.../webots.exe invoked through /init, so NO
#     WSL pgrep can see it. The launch died exit 1 with a misleading
#     "[Errno 13] Permission denied", the driver never connected, no capture.
#   - S161/S166: a bare `webots` with no world argument opens the TurtleBot demo
#     world and holds port 1234 plus a third of the CPU. It matches none of the
#     Corgi patterns.
#   - GAIT_WALL is WALL-CLOCK, so background load starves the LATER cells of a
#     campaign and silently shortens their captures -- which confounds exactly
#     the cross-cell comparison the campaign exists to make.
#
# IT REFUSES; IT DOES NOT KILL. The simulator is SHARED and another agent's
# campaign looks exactly like a leftover process from here. The *_cycle.sh
# runners tear down deliberately; a campaign script must not.
#
# No `set -u` -- ROS setup scripts break under it.

# Self-locating, same as preflight_plant.sh.
PREFLIGHT_SIM_WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." 2>/dev/null && pwd)

# Test seams. Honoured ONLY under PREFLIGHT_SIM_SELFTEST=1, which announces
# itself on every line it touches -- a real campaign cannot be quietly bypassed
# with these, and its log would say so if anyone tried.
_pfs_probe() {
  # $1 = probe name, $2.. = the real command
  local name="$1"; shift
  if [ "$PREFLIGHT_SIM_SELFTEST" = "1" ]; then
    local var="PREFLIGHT_SIM_FAKE_$name"
    eval "local faked=\"\$$var\""
    if [ -n "$faked" ]; then
      # "-" means "the probe found nothing" -- distinguishable from unset,
      # which means "use the real probe".
      [ "$faked" = "-" ] || echo "$faked"
      return 0
    fi
  fi
  "$@" 2>/dev/null
}

preflight_sim_stale() {
  # The union of every pattern any sweep used, plus the *_cycle.sh wrappers
  # that sweep_contact_lateral alone thought to look for.
  local pat='Corgi_launch[.]py|gslip_pronk_node|webots_ros2_driver|camber_cycle[.]sh|ramp_cycle[.]sh|sim_cycle[.]sh'
  local hits
  hits=$(_pfs_probe STALE pgrep -f "$pat" | grep -c . )
  if [ "$hits" != 0 ]; then
    echo "!! $hits stale sim/controller process(es) already running -- REFUSING:"
    pgrep -fa "$pat" 2>/dev/null | head
    echo "!! The simulator is SHARED. Another agent's campaign looks exactly like"
    echo "!! a leftover process from here. ASK before killing anything."
    return 1
  fi
  echo "sim: stale-launch check clean."
  return 0
}

preflight_sim_webots() {
  # 1. A Linux-side Webots that is not the Corgi sim -- e.g. a bare `webots`
  #    holding the TurtleBot demo world, S161's confound.
  local foreign
  foreign=$(_pfs_probe FOREIGN pgrep -f 'usr/local/webots' | grep -c . )
  if [ "$foreign" != 0 ]; then
    echo "!! a Linux-side Webots is running that is NOT the Corgi sim:"
    pgrep -fa 'usr/local/webots' 2>/dev/null | head
    echo "!! It holds port 1234 and steals CPU from every control loop. REFUSING."
    return 1
  fi

  # 2. The WINDOWS-side one, which no WSL pgrep above can see. This is the
  #    check S171 added and that only 7 of 25 campaigns ever got.
  if [ "$PREFLIGHT_SIM_SELFTEST" != "1" ] && ! command -v powershell.exe > /dev/null 2>&1; then
    echo "sim: windows-side Webots check SKIPPED -- powershell.exe not reachable."
    return 0
  fi
  local winwb
  winwb=$(_pfs_probe WINWB powershell.exe -NoProfile -Command \
            "@(Get-Process webots* -ErrorAction SilentlyContinue).Count" | tr -d '\r\n ')
  case "$winwb" in
    ''|*[!0-9]*)
      echo "sim: windows-side Webots check INCONCLUSIVE ('$winwb') -- continuing." ;;
    0)
      echo "sim: no Webots running, Linux-side or Windows-side." ;;
    *)
      echo "!! $winwb WINDOWS-side webots.exe still running. It holds port 1234 and"
      echo "!! NO WSL pgrep can see it. The launch will die with exit 1 and a"
      echo "!! misleading '[Errno 13] Permission denied' on the temp .wbt, and the"
      echo "!! driver will never connect. REFUSING."
      powershell.exe -NoProfile -Command \
        "Get-Process webots* | Select-Object Id,ProcessName,StartTime" 2>/dev/null
      return 1 ;;
  esac
  return 0
}

preflight_sim_load() {
  local max="${CORGI_MAX_LOAD:-4.0}"
  local load
  load=$(_pfs_probe LOAD cut -d' ' -f1 /proc/loadavg)
  if awk -v l="$load" -v m="$max" 'BEGIN{exit !(l > m)}'; then
    echo "!! 1-minute load average is $load (limit $max) before the campaign has"
    echo "!! started. GAIT_WALL is WALL-CLOCK, so this starves the LATER cells of"
    echo "!! sim time and silently shortens their captures -- it confounds the"
    echo "!! cross-cell comparison rather than just slowing it. REFUSING."
    return 1
  fi
  echo "sim: load average $load (limit $max)."
  return 0
}

preflight_sim() {
  [ "$PREFLIGHT_SIM_SELFTEST" = "1" ] && \
    echo "!! PREFLIGHT_SIM_SELFTEST=1 -- PROBES ARE FAKED, THIS IS NOT A REAL PREFLIGHT."
  preflight_sim_stale  || return 1
  preflight_sim_webots || return 1
  preflight_sim_load   || return 1
  return 0
}
