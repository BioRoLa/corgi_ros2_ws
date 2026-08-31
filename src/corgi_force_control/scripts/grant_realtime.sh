#!/bin/bash
# Let the 1 kHz control loops take SCHED_FIFO, WITHOUT breaking ROS.
#
# ============================================================================
# WHY THIS NO LONGER USES setcap  --  READ BEFORE "FIXING" IT BACK
#
# The first version ran `setcap cap_sys_nice+ep` on the node binaries. That
# works, and it also makes every one of them fail to start:
#
#   error while loading shared libraries: librclcpp.so: cannot open shared
#   object file: No such file or directory                    (exit code 127)
#
# A binary carrying file capabilities is executed with AT_SECURE=1, and
# glibc's dynamic loader then IGNORES LD_LIBRARY_PATH (and LD_PRELOAD) for
# it. ROS 2 finds all of its libraries through LD_LIBRARY_PATH. So capability
# and ROS are mutually exclusive on the same binary -- observed on the Orin
# 2026-09-01, all three nodes dead at once.
#
# The right mechanism is RLIMIT_RTPRIO: raise the user's real-time priority
# ceiling, and an ordinary unprivileged process can call sched_setscheduler()
# itself. No capability, no AT_SECURE, LD_LIBRARY_PATH untouched.
# ============================================================================
#
# Usage:  ./grant_realtime.sh            # set the limit, revoke stale caps
#         ./grant_realtime.sh --check    # report only, no changes, no sudo
#         ./grant_realtime.sh --revoke   # remove capabilities and stop
#
# Run WITHOUT sudo; it calls sudo only for the two things that need it.
# After granting you MUST start a NEW LOGIN SESSION (log out and back in, or
# reboot): PAM applies limits at login, so the shell you ran this from keeps
# the old ceiling.
set -u

PRIO=80
LIMITS_FILE=/etc/security/limits.d/99-corgi-realtime.conf

# Derive the workspace from where THIS FILE lives, not from $HOME: under sudo
# $HOME is /root. Layout: <ws>/src/corgi_force_control/scripts/<this>
SELF_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS_GUESS=$(cd "$SELF_DIR/../../.." && pwd)
WS="${CORGI_WS:-$WS_GUESS}"

BINS=(
    "$WS/install/corgi_force_control/lib/corgi_force_control/force_control_node"
    "$WS/install/corgi_force_control/lib/corgi_force_control/gslip_pronk_node"
    "$WS/install/corgi_force_estimation/lib/corgi_force_estimation/force_estimation_node"
)

MODE=grant
case "${1:-}" in
    --check)  MODE=check ;;
    --revoke) MODE=revoke ;;
    "")       ;;
    *) echo "unknown option: $1"; exit 2 ;;
esac

[ -d "$WS/install" ] || echo "note: $WS has no install/ -- set CORGI_WS if the workspace is elsewhere"

# ---------------------------------------------------------------------------
# Stale capabilities are actively harmful: they stop the nodes loading at all.
# Always look, in every mode.
# ---------------------------------------------------------------------------
stale=()
if command -v getcap >/dev/null 2>&1; then
    for b in "${BINS[@]}"; do
        [ -f "$b" ] || continue
        if getcap "$b" 2>/dev/null | grep -q cap_sys_nice; then
            stale+=("$b")
        fi
    done
fi

if [ ${#stale[@]} -gt 0 ]; then
    echo "FOUND capabilities on ${#stale[@]} binary(ies). These BREAK ROS:"
    echo "  a binary with file caps runs AT_SECURE, so the loader ignores"
    echo "  LD_LIBRARY_PATH and librclcpp.so is not found (exit 127)."
    if [ "$MODE" = check ]; then
        for b in "${stale[@]}"; do echo "  STALE CAP  $(basename "$b")"; done
        echo "  Run without --check (or with --revoke) to remove them."
    else
        for b in "${stale[@]}"; do
            if sudo setcap -r "$b" 2>/dev/null; then
                echo "  revoked    $(basename "$b")"
            else
                echo "  FAILED to revoke $(basename "$b")"
            fi
        done
    fi
    echo
fi

[ "$MODE" = revoke ] && exit 0

# ---------------------------------------------------------------------------
# The actual mechanism: RLIMIT_RTPRIO for this user.
# ---------------------------------------------------------------------------
CUR_RT=$(ulimit -r 2>/dev/null || echo 0)
echo "current shell's real-time priority ceiling (ulimit -r): $CUR_RT"
echo "needed for SCHED_FIFO $PRIO: at least $PRIO"

configured=no
if [ -f "$LIMITS_FILE" ] && grep -q rtprio "$LIMITS_FILE" 2>/dev/null; then
    configured=yes
fi
echo "limits file $LIMITS_FILE: $([ "$configured" = yes ] && echo present || echo absent)"

if [ "$MODE" = check ]; then
    if [ "$CUR_RT" != unlimited ] && [ "${CUR_RT:-0}" -lt "$PRIO" ] 2>/dev/null; then
        echo
        echo "NOT granted in this session -- the nodes will warn and run at normal"
        echo "priority, and commands can degrade to ~15 Hz under load (log S313)."
        [ "$configured" = yes ] && echo "The limit IS configured; you have not started a new login session yet."
        exit 1
    fi
    echo
    echo "This session can take SCHED_FIFO $PRIO."
    exit 0
fi

if [ "$configured" = no ]; then
    echo
    echo "writing $LIMITS_FILE (needs sudo):"
    printf '%s\n' \
        "# Corgi 1 kHz control loops need SCHED_FIFO (log S313)." \
        "# NOT setcap: file capabilities make the loader ignore LD_LIBRARY_PATH" \
        "# and every ROS node then fails with 'librclcpp.so: cannot open'." \
        "$USER  -  rtprio  99" \
        "$USER  -  memlock unlimited" \
        | sudo tee "$LIMITS_FILE" >/dev/null && echo "  written" || {
            echo "  FAILED to write $LIMITS_FILE"; exit 2; }
fi

echo
if [ "$CUR_RT" = unlimited ] || [ "${CUR_RT:-0}" -ge "$PRIO" ] 2>/dev/null; then
    echo "Ready: this session can already take SCHED_FIFO $PRIO."
else
    echo "*** LOG OUT AND BACK IN (or reboot) before launching. ***"
    echo "PAM applies limits at login, so this shell still has the old ceiling."
    echo "Confirm afterwards with:  ulimit -r      (want 99)"
fi
echo
echo "Then at the controller launch the banner must say:"
echo "    REALTIME: SCHED_FIFO priority $PRIO ACQUIRED"
echo "If it says 'could NOT set', this did not take effect."
