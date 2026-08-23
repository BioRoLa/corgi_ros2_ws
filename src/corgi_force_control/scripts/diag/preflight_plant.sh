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
  return 0
}
