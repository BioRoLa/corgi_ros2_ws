#!/bin/bash
# Grant CAP_SYS_NICE to the two 1 kHz control binaries, so they can actually
# take the SCHED_FIFO priority they ask for at startup (log S313).
#
# ******************************************************************
# RE-RUN THIS AFTER EVERY colcon build.
#
# setcap is an attribute of the binary FILE. A rebuild writes a NEW file,
# and the capability does not come with it. The nodes are fail-safe -- they
# warn and run at normal priority rather than refusing to start -- so a
# rebuild silently disarms the mitigation and the only sign is one WARN line
# scrolling past in the launch output. This is the easiest way for the
# 15 Hz starvation of S313 to come back after it was "fixed".
# ******************************************************************
#
# Alternative that needs no capability and no sudo afterwards:
#     chrt -f 80 ros2 launch corgi_force_control gslip_pronk.launch.py ...
# but that has to prefix every launch, and it is easy to forget on one.
#
# Usage:   ./grant_realtime.sh            # grant, then verify
#          ./grant_realtime.sh --check    # verify only, no sudo
#
# Run it WITHOUT sudo -- it calls `sudo setcap` itself, and only for that.
# Running the whole script under sudo also works now, but is not needed.
# If the executable bit is missing on a fresh checkout:  bash ./grant_realtime.sh
set -u

# Derive the workspace from where THIS FILE lives, not from $HOME: under
# sudo, $HOME is /root, and this script exists to be run in a context where
# sudo is involved. Layout: <ws>/src/corgi_force_control/scripts/<this>
SELF_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS_GUESS=$(cd "$SELF_DIR/../../.." && pwd)
WS="${CORGI_WS:-$WS_GUESS}"

if [ ! -d "$WS/install" ]; then
    echo "note: $WS has no install/ -- set CORGI_WS if the workspace is elsewhere"
fi
LIB="$WS/install/corgi_force_control/lib/corgi_force_control"
# force_estimation_node is here because it is also a 1 kHz loop and was
# missing from this list (log S318.5). Its install path differs.
FELIB="$WS/install/corgi_force_estimation/lib/corgi_force_estimation"
BINS=("$LIB/force_control_node" "$LIB/gslip_pronk_node" "$FELIB/force_estimation_node")

CHECK_ONLY=0
[ "${1:-}" = "--check" ] && CHECK_ONLY=1

if ! command -v getcap >/dev/null 2>&1; then
    echo "getcap/setcap not found -- install libcap2-bin:"
    echo "    sudo apt-get install -y libcap2-bin"
    exit 3
fi

rc=0
for b in "${BINS[@]}"; do
    name=$(basename "$b")
    if [ ! -f "$b" ]; then
        echo "MISSING   $name  ($b)"
        echo "          build the workspace first, or set CORGI_WS"
        rc=3
        continue
    fi

    have=$(getcap "$b" 2>/dev/null)
    if printf '%s' "$have" | grep -q "cap_sys_nice"; then
        echo "OK        $name  already has cap_sys_nice"
        continue
    fi

    if [ "$CHECK_ONLY" = 1 ]; then
        echo "NOT SET   $name  -- run without --check to grant it"
        rc=1
        continue
    fi

    echo "granting  $name ..."
    if sudo setcap cap_sys_nice+ep "$b"; then
        if getcap "$b" 2>/dev/null | grep -q "cap_sys_nice"; then
            echo "OK        $name  granted"
        else
            echo "FAILED    $name  setcap reported success but the cap is absent"
            echo "          (a filesystem mounted nosuid or without xattr support"
            echo "           will do this -- use 'chrt -f 80 ...' instead)"
            rc=2
        fi
    else
        echo "FAILED    $name  setcap failed"
        rc=2
    fi
done

echo
if [ "$rc" = 0 ]; then
    echo "Both control binaries can take SCHED_FIFO."
    echo "Confirm at launch: the banner must say"
    echo "    REALTIME: SCHED_FIFO priority 80 ACQUIRED"
    echo "If it says 'could NOT set', this did not take effect."
else
    echo "NOT fully granted -- the 1 kHz loops will run at normal priority"
    echo "and commands can degrade to ~15 Hz under CPU load (S313)."
fi
exit $rc
