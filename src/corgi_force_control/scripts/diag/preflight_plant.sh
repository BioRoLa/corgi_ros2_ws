# preflight_plant.sh -- IS THE SIMULATED ROBOT THE RECORDED ROBOT?
#
# Source this and call preflight_plant. It is a plant-side sibling of the
# LEG-FRAME GAINS banner (S151): that made the CONFIG announce itself, this
# makes the PLANT announce itself.
#
# WHY IT EXISTS. S198: protos/CorgiRobotABAD.proto in the corgi_sim submodule
# was edited on 2026-08-17 18:20 (S41's leg-inertia patch, the four *_LEG
# tensors) and left UNCOMMITTED for six days. S88 and S164-197 all ran against
# that working-tree proto while the recorded submodule pointer resolved to a
# DIFFERENT robot -- so none of those results could be reproduced from a fresh
# checkout. Nobody noticed because the proto is a 103 MB git-LFS object and
# `git diff` on it shows only a 3-line pointer. Same shape as open issue #19:
# what ran was not what was recorded. Open issue #26 asks for exactly this.
#
# It prints the plant identity on EVERY run, pass or fail. That line in the
# campaign log is the point: a run should record its own plant the way S159
# argued a run should record its own operating point.
#
# No `set -u` anywhere in this file -- ROS setup scripts break under it, and
# these scripts are sourced after those.

# Self-locating: this file lives at
#   <ws>/src/corgi_force_control/scripts/diag/preflight_plant.sh
# so the workspace is five directories up. That means callers need neither $WS
# nor $DIAG -- the three sweeps that define only $HERE can source it too.
PREFLIGHT_PLANT_WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." 2>/dev/null && pwd)

preflight_plant() {
  local ws="${1:-${WS:-$PREFLIGHT_PLANT_WS}}"
  local sm="$ws/src/corgi_sim"
  local rel="protos/CorgiRobotABAD.proto"
  local src="$sm/$rel"
  local inst="$ws/install/corgi_sim/share/corgi_sim/$rel"

  [ -d "$sm/.git" ] || [ -f "$sm/.git" ] || {
    echo "!! $sm is not a git checkout -- the plant cannot be identified."
    echo "!! REFUSING."; return 1; }

  local sha oid
  sha=$(git -C "$sm" rev-parse --short HEAD 2>/dev/null)
  # `git show` on an LFS path yields the POINTER, which is what we want here
  # and costs nothing -- no smudge, no 103 MB read.
  oid=$(git -C "$sm" show "HEAD:$rel" 2>/dev/null | awk '/^oid /{print substr($2,8,12)}')
  echo "PLANT: corgi_sim $sha  proto oid ${oid:-unknown}"

  # 1. The proto must not be dirty. This is the check that was missing.
  local dirty
  dirty=$(git -C "$sm" status --porcelain -- "$rel" 2>/dev/null)
  if [ -n "$dirty" ]; then
    echo "!! THE WEBOTS MODEL IS MODIFIED AND UNCOMMITTED:"
    echo "!!   $dirty"
    echo "!! Whatever this campaign measures will NOT be reproducible from git."
    echo "!! That is exactly how S198 happened -- six days of results on a plant"
    echo "!! no checkout could rebuild. Commit it (see S198.6 for how to diff a"
    echo "!! 103 MB LFS blob), or set CORGI_ALLOW_DIRTY_PLANT=1 to proceed"
    echo "!! deliberately and have this campaign's log say so."
    if [ "$CORGI_ALLOW_DIRTY_PLANT" = "1" ]; then
      echo "!! CORGI_ALLOW_DIRTY_PLANT=1 -- PROCEEDING ON AN UNRECORDED PLANT."
    else
      echo "!! REFUSING."; return 1
    fi
  fi

  # 2. src is not install. Editing the proto without rebuilding leaves Webots
  #    loading the OLD robot while the repo shows the new one -- S28's stale-
  #    driver trap, one level down, and it looks exactly like a real result.
  if [ -f "$inst" ]; then
    if ! cmp -s "$inst" "$src"; then
      echo "!! install/ and src/ disagree about the robot:"
      echo "!!   src    : $(stat -c '%s bytes  %y' "$src" 2>/dev/null)"
      echo "!!   install: $(stat -c '%s bytes  %y' "$inst" 2>/dev/null)"
      echo "!! Webots loads the INSTALLED proto, so the plant is not what the"
      echo "!! repo says. colcon build --packages-select corgi_sim. REFUSING."
      return 1
    fi
    echo "plant: install matches src."
  else
    echo "plant: no installed proto at $inst -- cannot verify src vs install."
  fi

  # 3. Does the PARENT repo record the submodule commit that is checked out?
  #    A stale pointer is not a reason to refuse -- the plant is still
  #    identified and reproducible -- but it means `git checkout` of the parent
  #    would hand someone a different robot.
  local recorded head
  recorded=$(git -C "$ws" ls-tree HEAD src/corgi_sim 2>/dev/null | awk '{print $3}')
  head=$(git -C "$sm" rev-parse HEAD 2>/dev/null)
  if [ -n "$recorded" ] && [ "$recorded" != "$head" ]; then
    echo "!! WARN: parent records src/corgi_sim ${recorded:0:7}, checked out ${head:0:7}."
    echo "!! WARN: the campaign is readable, but the parent pointer is stale."
  fi

  # 4. Freeze the installed artifacts, so plant_verify can catch a rebuild that
  #    lands MID-campaign (S204). Exported, so each run's child process inherits
  #    it without the campaign script having to pass anything along.
  plant_freeze "$ws" || return 1
  return 0
}

# ---------------------------------------------------------------------------
# THE MID-CAMPAIGN CASE (S204). preflight_plant runs ONCE, at campaign start.
# S181 is what that misses: sweep_torque_ceiling_cor.sh was written at 14:19,
# S168 rebuilt the controller at 14:44, and the campaign's control bar was then
# scored against a plant that had moved underneath it. P-Q-3 failed and
# P-Q-1/P-Q-2 went unscored -- a whole campaign, for a change nothing measured.
#
# So: FREEZE a fingerprint of the installed artifacts at campaign start, and
# RE-CHECK it before every run. A campaign that straddles a rebuild now stops
# at the run where the plant moved, instead of producing a mixed dataset that
# looks clean.
#
# The four artifacts are the ones that actually decide what a run does:
#   proto       the robot                     (S41/S198's case)
#   driver      contact, torque, publishing   (installed, not src)
#   controller  gslip_pronk_node              (S168's case, 2026-08-22 14:44)
#   force_ctl   force_control_node            (the impedance law)
#
# NOT the template CSV: sweep_template_speed.sh legitimately varies it per cell,
# and it is a launch argument, already covered by open issue #19's discipline.
#
# `stat -L`, NOT `stat`. corgi_force_control is a --symlink-install, so the
# path in install/lib is an 81-byte SYMLINK into build/ whose own mtime is
# 2026-08-09 and never changes on rebuild. Without -L this check would compare
# two constants forever and pass through every rebuild it exists to catch.
# Cost is four stat(2) calls and ZERO file reads, which is deliberate: it runs
# while a campaign is live, and analysis load starves later cells (S166).

_plant_artifacts() {
  local ws="${1:-${WS:-$PREFLIGHT_PLANT_WS}}"
  echo "proto=$ws/install/corgi_sim/share/corgi_sim/protos/CorgiRobotABAD.proto"
  echo "driver=$ws/install/corgi_sim/local/lib/python3.10/dist-packages/corgi_driver_pkg/corgi_driver.py"
  echo "controller=$ws/install/corgi_force_control/lib/corgi_force_control/gslip_pronk_node"
  echo "force_ctl=$ws/install/corgi_force_control/lib/corgi_force_control/force_control_node"
}

plant_fingerprint() {
  local ws="${1:-${WS:-$PREFLIGHT_PLANT_WS}}"
  local out="" name path sig
  while IFS= read -r spec; do
    name="${spec%%=*}"; path="${spec#*=}"
    sig=$(stat -L -c '%s:%Y' "$path" 2>/dev/null) || sig="MISSING"
    [ -n "$sig" ] || sig="MISSING"
    out="$out,$name=$sig"
  done <<< "$(_plant_artifacts "$ws")"
  # Comma-separated and space-free ON PURPOSE: this value is exported and
  # read back by child processes, and a fingerprint containing spaces
  # truncates at the first space anywhere it is passed unquoted.
  echo "${out#,}"
}

plant_freeze() {
  local ws="${1:-${WS:-$PREFLIGHT_PLANT_WS}}"
  CORGI_PLANT_LOCK=$(plant_fingerprint "$ws")
  export CORGI_PLANT_LOCK
  echo "plant lock: $CORGI_PLANT_LOCK"
  return 0
}

plant_verify() {
  local ws="${1:-${WS:-$PREFLIGHT_PLANT_WS}}"
  local now; now=$(plant_fingerprint "$ws")

  if [ -z "$CORGI_PLANT_LOCK" ]; then
    # Standalone invocation -- nothing to compare against. Still announce, so
    # the run's log records which plant it used.
    echo "plant lock: (none set) $now"
    return 0
  fi
  [ "$now" = "$CORGI_PLANT_LOCK" ] && return 0

  echo "!! ================= THE PLANT MOVED MID-CAMPAIGN ================="
  echo "!! An installed artifact changed AFTER this campaign started. Runs"
  echo "!! before and after this point are NOT comparable, and a campaign that"
  echo "!! straddles a rebuild is exactly how S181 lost P-Q-1/P-Q-2."
  local f nowv wasv
  for f in proto driver controller force_ctl; do
    wasv=$(echo ",$CORGI_PLANT_LOCK" | sed -n "s/.*,$f=\([^,]*\).*/\1/p")
    nowv=$(echo ",$now"              | sed -n "s/.*,$f=\([^,]*\).*/\1/p")
    if [ "$wasv" != "$nowv" ]; then
      echo "!!   $f  CHANGED  size:mtime  $wasv  ->  $nowv"
    fi
  done
  echo "!! at campaign start : $CORGI_PLANT_LOCK"
  echo "!! now               : $now"

  # Leave the evidence WITH the captures, so this cannot be lost to a harness
  # that swallows a non-zero exit (repeat_gain_regime.sh returns 0 on a failed
  # launch -- see its `skipping` path).
  if [ -n "$OUTDIR" ] && [ -d "$OUTDIR" ]; then
    {
      echo "campaign start : $CORGI_PLANT_LOCK"
      echo "detected at run: $now"
    } > "$OUTDIR/PLANT_CHANGED_MID_CAMPAIGN.txt" 2>/dev/null \
      && echo "!! recorded in $OUTDIR/PLANT_CHANGED_MID_CAMPAIGN.txt"
  fi
  echo "!! REFUSING. Re-register the campaign against the new plant (S126: a"
  echo "!! failed gate binds -- re-run, do not rescue), or export"
  echo "!! CORGI_PLANT_LOCK= to start a fresh lock deliberately."
  echo "!! ==============================================================="
  return 1
}
