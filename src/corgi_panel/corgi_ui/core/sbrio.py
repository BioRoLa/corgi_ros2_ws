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
    ssh-copy-id -i ~/.ssh/id_sbrio.pub admin@192.168.0.104

Host/user/script path are overridable by environment so a re-addressed sbRIO
does not need a rebuild:  CORGI_SBRIO_HOST, CORGI_SBRIO_USER,
CORGI_SBRIO_SCRIPT.
"""

import os
import subprocess

SBRIO_HOST = os.environ.get('CORGI_SBRIO_HOST', '192.168.0.104')
SBRIO_USER = os.environ.get('CORGI_SBRIO_USER', 'admin')
SBRIO_SCRIPT = os.environ.get('CORGI_SBRIO_SCRIPT',
                              '/home/admin/run_fpga_driver.sh')

KEY_HINT = ("no passwordless ssh to the sbRIO -- run once on the Orin:  "
            "ssh-keygen -t ed25519 -f ~/.ssh/id_sbrio -N '' && "
            "ssh-copy-id -i ~/.ssh/id_sbrio.pub %s@%s" % (SBRIO_USER, SBRIO_HOST))

# Run on the sbRIO, fed to `bash -s` over stdin with the script path as $1.
# Sent as text rather than installed as a file so there is nothing to get out
# of date on the sbRIO, and nothing for colcon to fail to install on the Orin.
#
# pgrep -x (exact process NAME) rather than -f (full command line): -f would
# match this very bash, whose argv carries the script path, and every start
# would report ALREADY_RUNNING.
_REMOTE_START = r'''
set -u
SCRIPT="${1:-/home/admin/run_fpga_driver.sh}"
DIR=$(dirname "$SCRIPT")
LOG="$DIR/panel_fpga_start.log"

if pgrep -x fpga_driver >/dev/null 2>&1; then
    echo "ALREADY_RUNNING"
    echo "fpga_driver pid: $(pgrep -x fpga_driver | tr '\n' ' ')"
    echo "grpccore    pid: $(pgrep -x grpccore | tr '\n' ' ')"
    exit 0
fi

if [ ! -f "$SCRIPT" ]; then
    echo "NO_SCRIPT"
    echo "not found on the sbRIO: $SCRIPT"
    exit 4
fi

cd "$DIR" || { echo "NO_SCRIPT"; echo "cannot cd to $DIR"; exit 5; }

if [ -x "$SCRIPT" ]; then RUN="$SCRIPT"; else RUN="sh $SCRIPT"; fi
if command -v setsid >/dev/null 2>&1; then
    setsid $RUN >"$LOG" 2>&1 </dev/null &
else
    nohup $RUN >"$LOG" 2>&1 </dev/null &
fi

# The driver opens its FPGA session about 2.5 s in; wait for the process, not
# for a fixed sleep, so a healthy start returns as soon as it is up.
n=0
while [ $n -lt 12 ]; do
    sleep 1
    if pgrep -x fpga_driver >/dev/null 2>&1; then break; fi
    n=$((n + 1))
done

if pgrep -x fpga_driver >/dev/null 2>&1; then
    echo "STARTED"
    echo "fpga_driver pid: $(pgrep -x fpga_driver | tr '\n' ' ')"
    echo "grpccore    pid: $(pgrep -x grpccore | tr '\n' ' ')"
    sed -n '1,10p' "$LOG" 2>/dev/null
    exit 0
fi

echo "FAILED"
tail -n 20 "$LOG" 2>/dev/null
exit 6
'''

_REMOTE_STATUS = r'''
if pgrep -x fpga_driver >/dev/null 2>&1; then
    echo "ALREADY_RUNNING"
    echo "fpga_driver pid: $(pgrep -x fpga_driver | tr '\n' ' ')"
    echo "grpccore    pid: $(pgrep -x grpccore | tr '\n' ' ')"
else
    echo "NOT_RUNNING"
fi
'''


def _ssh_argv(connect_timeout: int = 5):
    return [
        'ssh',
        '-o', 'BatchMode=yes',                     # never sit at a password prompt
        '-o', 'ConnectTimeout=%d' % connect_timeout,
        '-o', 'StrictHostKeyChecking=accept-new',
        '%s@%s' % (SBRIO_USER, SBRIO_HOST),
        'bash', '-s', '--', SBRIO_SCRIPT,
    ]


def _run(remote_script: str, timeout: float):
    """Returns (token, lines). token is one of
    ALREADY_RUNNING / STARTED / NOT_RUNNING / FAILED / NO_SCRIPT /
    NO_KEY / UNREACHABLE / TIMEOUT / NO_SSH."""
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
    err = [ln.rstrip() for ln in proc.stderr.splitlines() if ln.strip()]

    for token in ('ALREADY_RUNNING', 'STARTED', 'NOT_RUNNING',
                  'NO_SCRIPT', 'FAILED'):
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


def fpga_driver_status(timeout: float = 12.0):
    return _run(_REMOTE_STATUS, timeout)


def target_description() -> str:
    return '%s@%s:%s' % (SBRIO_USER, SBRIO_HOST, SBRIO_SCRIPT)
