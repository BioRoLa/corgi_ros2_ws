#!/usr/bin/env python3
"""Bring the sbRIO's FPGA driver up from the panel.

The FPGA driver is the bottom of the stack: grpccore plus fpga_driver on the
sbRIO, with the ROS bridge on the Orin talking to them over gRPC. Until this
session it was started by hand once per power cycle -- ssh admin@<sbrio>,
./run_fpga_driver.sh, and leave that terminal open -- which is one more thing
to forget at the start of a session, and it dies silently if the terminal is
ever closed.

Two rules shape what is here:

* ``BatchMode=yes`` on every ssh call. The panel has no terminal to type a
  password into, so an ssh that decides to prompt would hang the GUI until the
  timeout. BatchMode turns "would prompt" into an immediate, legible failure,
  and the caller turns that into a log line saying to install a key.
* The driver is detached on the remote side (setsid, or nohup where setsid is
  missing) so it outlives the ssh session. Without that it would be killed the
  moment this call returns -- the same reason the manual recipe needed a
  terminal left open.

This needs key-based ssh from the Orin to the sbRIO. One-time setup, run on
the Orin:

    ssh-keygen -t ed25519 -f ~/.ssh/id_sbrio -N ""
    ssh-copy-id -i ~/.ssh/id_sbrio.pub admin@192.168.0.104   # asks for the
                                                             # password ONCE

Note that ``id_sbrio`` is not one of the names ssh offers by default (it
tries id_rsa / id_ecdsa / id_ed25519 / id_dsa and nothing else), so
``ssh-copy-id`` installing it on the sbRIO is only half the job: the client
has to be told to use it. We pass ``-i`` explicitly below whenever the file
exists, which also means no ~/.ssh/config entry is required. If you would
rather configure it once for every tool, this is the equivalent stanza:

    Host 192.168.0.104
        User admin
        IdentityFile ~/.ssh/id_sbrio
        IdentitiesOnly yes

Host/user/script path/key are overridable by environment so a re-addressed
sbRIO does not need a rebuild:  CORGI_SBRIO_HOST, CORGI_SBRIO_USER,
CORGI_SBRIO_SCRIPT, CORGI_SBRIO_KEY.
"""

import os
import subprocess

SBRIO_HOST = os.environ.get('CORGI_SBRIO_HOST', '192.168.0.104')
SBRIO_USER = os.environ.get('CORGI_SBRIO_USER', 'admin')
SBRIO_SCRIPT = os.environ.get('CORGI_SBRIO_SCRIPT',
                              '/home/admin/run_fpga_driver.sh')
SBRIO_KEY = os.path.expanduser(
    os.environ.get('CORGI_SBRIO_KEY', '~/.ssh/id_sbrio'))

KEY_HINT = ("no passwordless ssh to the sbRIO -- run once on the Orin "
            "(ssh-copy-id asks for the sbRIO password, once):  "
            "ssh-keygen -t ed25519 -f ~/.ssh/id_sbrio -N '' && "
            "ssh-copy-id -i ~/.ssh/id_sbrio.pub %s@%s" % (SBRIO_USER, SBRIO_HOST))

# Run on the sbRIO, fed to `bash -s` over stdin with the script path as $1.
# Sent as text rather than installed as a file so there is nothing to get out
# of date on the sbRIO, and nothing for colcon to fail to install on the Orin.
#
# HISTORY, because it cost a bring-up: the first version detected the driver
# with `pgrep -x fpga_driver`. That does not work on the sbRIO. The driver
# launched perfectly every time and was reported FAILED every time -- and
# because a failed start is not a start, each press launched ANOTHER instance.
# Nothing here may depend on one process tool existing.
_COMMON = r"""
set -u
SCRIPT="${1:-/home/admin/run_fpga_driver.sh}"
DIR=$(dirname "$SCRIPT")
LOG="$DIR/panel_fpga_start.log"
PIDFILE="$DIR/panel_fpga.pid"
SELF=$$

# Union of every detector available, deduped. Each contributes nothing when
# its tool is missing or unsupported, so the answer degrades instead of lying.
running_pids() {
    {
        if command -v pgrep >/dev/null 2>&1; then
            pgrep -x fpga_driver 2>/dev/null
        fi
        ps -eo pid=,comm= 2>/dev/null | awk '$2 == "fpga_driver" { print $1 }'
        ps ax 2>/dev/null | awk -v self="$SELF" \
            '$1 != self && /fpga_driver/ && !/run_fpga_driver/ { print $1 }'
        if [ -f "$PIDFILE" ]; then
            p=$(cat "$PIDFILE" 2>/dev/null)
            if [ -n "$p" ] && kill -0 "$p" 2>/dev/null; then echo "$p"; fi
        fi
    } 2>/dev/null | sort -u | tr '\n' ' '
}

core_pids() {
    {
        if command -v pgrep >/dev/null 2>&1; then
            pgrep -x grpccore 2>/dev/null
        fi
        ps -eo pid=,comm= 2>/dev/null | awk '$2 == "grpccore" { print $1 }'
    } 2>/dev/null | sort -u | tr '\n' ' '
}

have_driver() { [ -n "$(running_pids | tr -d ' ')" ]; }

# The driver's own words. Independent of every process tool, and the only
# evidence that it reached a working FPGA session rather than merely existing.
log_says_up() {
    grep -q "Session opened" "$LOG" 2>/dev/null || \
    grep -q "IRQ reserved"  "$LOG" 2>/dev/null
}

report_pids() {
    echo "fpga_driver pid: $(running_pids)"
    echo "grpccore    pid: $(core_pids)"
}

# A C++ abort in the log is a definite answer and should not wait out the
# whole start window.
crash_line() {
    grep -m1 -E "terminate called|std::logic_error|std::bad_alloc|Segmentation fault|error while loading shared libraries|command not found" \
        "$LOG" 2>/dev/null
}
"""

_REMOTE_START = _COMMON + r"""
if have_driver; then
    echo "ALREADY_RUNNING"
    report_pids
    exit 0
fi

if [ ! -f "$SCRIPT" ]; then
    echo "NO_SCRIPT"
    echo "not found on the sbRIO: $SCRIPT"
    exit 4
fi

cd "$DIR" || { echo "NO_SCRIPT"; echo "cannot cd to $DIR"; exit 5; }

# Truncate first: log_says_up must not read a PREVIOUS run's success line.
: > "$LOG" 2>/dev/null || true

# bash -lc, so /etc/profile and ~/.profile run exactly as they do in the
# interactive session where this script is known to work. Without it the
# driver aborts on a null getenv (see this module's docstring).
if [ -x "$SCRIPT" ]; then
    CMD="cd '$DIR' && '$SCRIPT'"
else
    CMD="cd '$DIR' && sh '$SCRIPT'"
fi
if command -v setsid >/dev/null 2>&1; then
    setsid bash -lc "$CMD" >"$LOG" 2>&1 </dev/null &
else
    nohup bash -lc "$CMD" >"$LOG" 2>&1 </dev/null &
fi
echo $! > "$PIDFILE" 2>/dev/null || true

EVIDENCE=""
n=0
while [ $n -lt 15 ]; do
    sleep 1
    C=$(crash_line)
    if [ -n "$C" ]; then
        echo "FAILED"
        echo "the driver ABORTED on startup: $C"
        echo "--- last lines of $LOG ---"
        tail -n 15 "$LOG" 2>/dev/null
        exit 6
    fi
    if log_says_up; then EVIDENCE="the driver logged its FPGA session"; break; fi
    if have_driver;  then EVIDENCE="process table"; break; fi
    n=$((n + 1))
done

if [ -z "$EVIDENCE" ]; then
    echo "FAILED"
    echo "no FPGA session logged and no process found after 15 s"
    echo "--- last lines of $LOG ---"
    tail -n 15 "$LOG" 2>/dev/null
    exit 6
fi

# It EXISTED. That is not the same as it is running: the driver has been
# seen alive at 1 s and gone by 50 s. Make it prove it survived.
sleep 4
C=$(crash_line)
if [ -n "$C" ]; then
    echo "FAILED"
    echo "the driver started and then ABORTED: $C"
    echo "--- last lines of $LOG ---"
    tail -n 15 "$LOG" 2>/dev/null
    exit 6
fi
if ! have_driver; then
    echo "FAILED"
    echo "the driver started and then EXITED within 5 s (no process left)"
    echo "--- last lines of $LOG ---"
    tail -n 15 "$LOG" 2>/dev/null
    exit 6
fi

echo "STARTED"
echo "evidence: $EVIDENCE, still alive after a settle"
report_pids
if ! log_says_up; then
    echo "NOTE: no \"Session opened\"/\"IRQ reserved\" line yet -- the process"
    echo "      is up but has not confirmed a working FPGA session."
fi
sed -n '1,10p' "$LOG" 2>/dev/null
exit 0
"""

_REMOTE_STOP = _COMMON + r"""
D=$(running_pids)
C=$(core_pids)

if [ -z "$(printf '%s' "$D$C" | tr -d ' ')" ]; then
    echo "NOT_RUNNING"
    exit 0
fi

# SIGTERM first, and give it real time. The driver closes its FPGA session and
# releases the reserved IRQ on the way out; a killed one can leave the session
# held, and then the NEXT start fails with the board still owned by a corpse.
kill -TERM $D $C 2>/dev/null

n=0
while [ $n -lt 10 ]; do
    sleep 1
    if [ -z "$(printf '%s' "$(running_pids)$(core_pids)" | tr -d ' ')" ]; then
        rm -f "$PIDFILE" 2>/dev/null
        echo "STOPPED"
        exit 0
    fi
    n=$((n + 1))
done

echo "did not exit on SIGTERM after 10 s -- escalating to SIGKILL"
echo "the FPGA session may be left held; if the next start fails, reboot the sbRIO"
kill -KILL $(running_pids) $(core_pids) 2>/dev/null
sleep 1

if [ -n "$(printf '%s' "$(running_pids)$(core_pids)" | tr -d ' ')" ]; then
    echo "FAILED"
    report_pids
    exit 7
fi

rm -f "$PIDFILE" 2>/dev/null
echo "STOPPED_HARD"
exit 0
"""

_REMOTE_STATUS = _COMMON + r"""
if have_driver; then
    echo "ALREADY_RUNNING"
    report_pids
    if log_says_up; then
        echo "the driver logged a working FPGA session"
    else
        echo "WARNING: process is up but no FPGA session line in $LOG"
    fi
else
    echo "NOT_RUNNING"
    report_pids
fi
"""


_BANNER = ('NI Linux Real-Time', 'Log in with your NI-Auth credentials')


def _is_banner(line: str) -> bool:
    return any(b in line for b in _BANNER)


def _ssh_argv(connect_timeout: int = 5):
    argv = [
        'ssh',
        '-o', 'BatchMode=yes',                     # never sit at a password prompt
        '-o', 'ConnectTimeout=%d' % connect_timeout,
        '-o', 'StrictHostKeyChecking=accept-new',
    ]
    # ssh's default identity list is id_rsa / id_ecdsa / id_ed25519 / id_dsa --
    # id_sbrio is not on it, so a key installed by `ssh-copy-id -i` would never
    # be OFFERED and the connection would fail as though setup had not happened.
    # Named explicitly here; a ~/.ssh/config entry still works for anyone who
    # prefers one, and is used unchanged when this file is absent.
    if os.path.exists(SBRIO_KEY):
        argv += ['-i', SBRIO_KEY]
    argv += [
        '%s@%s' % (SBRIO_USER, SBRIO_HOST),
        'bash', '-s', '--', SBRIO_SCRIPT,
    ]
    return argv


def _run(remote_script: str, timeout: float):
    """Returns (token, lines). token is one of
    ALREADY_RUNNING / STARTED / STOPPED / STOPPED_HARD / NOT_RUNNING /
    FAILED / NO_SCRIPT / NO_KEY / UNREACHABLE / TIMEOUT / NO_SSH."""
    try:
        proc = subprocess.run(
            _ssh_argv(),
            input=remote_script,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return 'TIMEOUT', ['sbRIO did not answer within %.0f s' % timeout]
    except FileNotFoundError:
        return 'NO_SSH', ['ssh client not found on this machine']
    except Exception as exc:                                   # pragma: no cover
        return 'FAILED', ['ssh failed: %s' % exc]

    out = [ln.rstrip() for ln in proc.stdout.splitlines() if ln.strip()]
    # The sbRIO prints a login banner on every connection; it is not output
    # and it made the panel log look like the driver was talking.
    err = [ln.rstrip() for ln in proc.stderr.splitlines()
           if ln.strip() and not _is_banner(ln)]

    for token in ('ALREADY_RUNNING', 'STARTED', 'STOPPED', 'STOPPED_HARD',
                  'NOT_RUNNING', 'NO_SCRIPT', 'FAILED'):
        if token in out:
            return token, [ln for ln in out if ln != token] + err

    # Never got as far as the remote script.
    blob = ' '.join(err).lower()
    if 'permission denied' in blob or 'publickey' in blob:
        return 'NO_KEY', [KEY_HINT] + err
    if ('timed out' in blob or 'no route to host' in blob
            or 'could not resolve' in blob or 'connection refused' in blob):
        return 'UNREACHABLE', ['cannot reach %s@%s' % (SBRIO_USER, SBRIO_HOST)] + err
    return 'FAILED', (out + err) or ['ssh exited %d with no output' % proc.returncode]


def start_fpga_driver(timeout: float = 30.0):
    """Start the driver on the sbRIO if it is not already up.

    Never kills a running driver: a panel button that can silently restart the
    FPGA layer mid-experiment is worse than one that occasionally says 'already
    running'.
    """
    return _run(_REMOTE_START, timeout)


def stop_fpga_driver(timeout: float = 40.0):
    """Stop the driver on the sbRIO.

    The timeout is generous because the remote side deliberately waits out
    a full SIGTERM before escalating; the ordinary case returns in a second
    or two.
    """
    return _run(_REMOTE_STOP, timeout)


def fpga_driver_status(timeout: float = 12.0):
    return _run(_REMOTE_STATUS, timeout)


def target_description() -> str:
    return '%s@%s:%s' % (SBRIO_USER, SBRIO_HOST, SBRIO_SCRIPT)
