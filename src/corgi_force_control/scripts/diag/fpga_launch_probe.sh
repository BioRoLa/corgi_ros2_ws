#!/bin/bash
# Which launch method does the sbRIO's FPGA driver actually survive?
#
# The panel's launch aborts with
#     terminate called ... basic_string::_M_construct null not valid
# while Alex's manual `ssh in, ./run_fpga_driver.sh` works. Two candidate
# differences -- login environment, and having a TTY -- and adding a login
# shell alone did NOT fix it. So stop guessing and test the four cells.
#
# Run this ON THE ORIN. It touches only the sbRIO, one driver at a time,
# and cleans up after each cell.
#
#   ./fpga_launch_probe.sh
#
# Each cell: launch, wait, report whether the process is alive and whether
# the log contains an abort. The cell that survives is the one the panel
# should use.
set -u

HOST="${CORGI_SBRIO_HOST:-192.168.0.104}"
USER_="${CORGI_SBRIO_USER:-admin}"
SCRIPT="${CORGI_SBRIO_SCRIPT:-/home/admin/run_fpga_driver.sh}"
DIR=$(dirname "$SCRIPT")
KEY_OPT=""
[ -f "$HOME/.ssh/id_sbrio" ] && KEY_OPT="-i $HOME/.ssh/id_sbrio"
SSH="ssh -o BatchMode=yes -o ConnectTimeout=5 $KEY_OPT"

cleanup() {
    $SSH "$USER_@$HOST" \
        'kill -9 $(ps ax | awk "/[f]pga_driver|[g]rpccore/ {print \$1}") 2>/dev/null; true' \
        >/dev/null 2>&1
    sleep 2
}

report() {   # $1 = cell name, $2 = remote log path
    sleep 8
    local out
    out=$($SSH "$USER_@$HOST" "
        alive=\$(ps ax | awk '/[f]pga_driver/ {print \$1}' | tr '\n' ' ')
        crash=\$(grep -m1 -E 'terminate called|logic_error' '$2' 2>/dev/null)
        reached=\$(grep -c 'starting fpga_driver' '$2' 2>/dev/null)
        session=\$(grep -c -E 'Session opened|IRQ reserved' '$2' 2>/dev/null)
        echo \"alive=[\$alive]\"
        echo \"reached_driver_start=\$reached  logged_fpga_session=\$session\"
        echo \"crash=\$crash\"
    " 2>/dev/null | grep -v 'NI Linux Real-Time\|NI-Auth')
    echo "$out" | sed 's/^/    /'
    if echo "$out" | grep -q 'alive=\[\s*\]'; then
        echo "    VERDICT: dead"
    elif echo "$out" | grep -q 'crash=.\+'; then
        echo "    VERDICT: alive but crashed once"
    else
        echo "    VERDICT: *** SURVIVED ***"
    fi
    echo
}

echo "sbRIO $USER_@$HOST   script $SCRIPT"
echo

# ---------------------------------------------------------------- cell A ---
echo "A. non-login shell, no TTY   (the panel's ORIGINAL launch)"
cleanup
$SSH "$USER_@$HOST" \
    "cd '$DIR' && : > /tmp/probe_a.log; setsid '$SCRIPT' > /tmp/probe_a.log 2>&1 < /dev/null &" \
    >/dev/null 2>&1
report A /tmp/probe_a.log

# ---------------------------------------------------------------- cell B ---
echo "B. LOGIN shell, no TTY       (the panel's CURRENT launch, 6d4ad3f)"
cleanup
$SSH "$USER_@$HOST" \
    ": > /tmp/probe_b.log; setsid bash -lc \"cd '$DIR' && '$SCRIPT'\" > /tmp/probe_b.log 2>&1 < /dev/null &" \
    >/dev/null 2>&1
report B /tmp/probe_b.log

# ---------------------------------------------------------------- cell C ---
echo "C. LOGIN shell WITH a TTY    (closest to typing it yourself)"
cleanup
ssh -tt -o BatchMode=yes -o ConnectTimeout=5 $KEY_OPT "$USER_@$HOST" \
    ": > /tmp/probe_c.log; setsid bash -lc \"cd '$DIR' && '$SCRIPT'\" > /tmp/probe_c.log 2>&1 < /dev/null & sleep 1; exit" \
    >/dev/null 2>&1
report C /tmp/probe_c.log

# ---------------------------------------------------------------- cell D ---
echo "D. foreground under a TTY    (exactly the manual recipe, then killed)"
cleanup
ssh -tt -o BatchMode=yes -o ConnectTimeout=5 $KEY_OPT "$USER_@$HOST" \
    "cd '$DIR' && ./$(basename "$SCRIPT") > /tmp/probe_d.log 2>&1" \
    >/dev/null 2>&1 &
SSH_PID=$!
report D /tmp/probe_d.log
kill $SSH_PID 2>/dev/null

cleanup
echo "All cells cleaned up. Nothing left running on the sbRIO."
echo
echo "If D survives and B does not, it is the TTY, not the environment."
echo "If nothing survives, the sbRIO itself needs attention (reboot) and no"
echo "launch method will help."
