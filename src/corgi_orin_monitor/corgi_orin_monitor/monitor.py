#!/usr/bin/env python3
"""
corgi_orin_monitor.monitor -- always-on health / power logger for the Orin.

Why it exists: the Orin reboots unexpectedly (also at standstill with motors
powered, temperatures normal).  When the cause is a supply dip or a hard reset
the kernel never gets to write anything, so this daemon keeps a continuous,
fsync'ed record and, at every boot, collects the hardware reset reason and a
summary of the previous boot's last seconds.

Pure standard library, no ROS dependency: it runs from systemd before anything
else is up and keeps running while ROS crashes, restarts, or is never launched.

Layout of --log-dir:
    index.json                           boots in order (seq, boot_id, dir)
    boot_0007_20261005_113000_1ea1a98/   one directory per Linux boot
        boot_info.txt        facts collected at start: reset reason, previous
                             boot's journal tail, nvpmodel, IMU autostart probe
        prev_boot_summary.txt  report.py output for the previous boot
        late_probe.txt       IMU / service state N s after start (default 90)
        heartbeat.csv        1 Hz   wall, epoch, monotonic, boottime, ntp, event
        power.csv            20 Hz  INA3221 rails (mV, mA, mW), CPU/GPU clocks
        system.csv           1 Hz   load, cpu%, mem, temps, ttyTHS1, imu_node, net
        tegrastats.log       tegrastats --interval 500, prefixed with our clock
        dmesg.log            kernel log, live (whole buffer first, then follow)
        journal.log          journalctl -b 0 -f, live
        monitor.log          this daemon's own notes
        pstore/              copy of /sys/fs/pstore if it held anything

Every file is appended with flush+fsync (interval --fsync-interval, default
0.1 s), so after a power loss at most ~100 ms of data is missing.
"""
import argparse
import datetime
import glob
import json
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

VERSION = '1.0.0'
DEFAULT_LOG_DIR = '/var/log/corgi_orin_monitor'
FALLBACK_LOG_DIR = '~/corgi_orin_monitor_logs'
HERE = os.path.dirname(os.path.abspath(__file__))


# --------------------------------------------------------------------------- helpers
def wall() -> float:
    return time.time()


def mono() -> float:
    return time.monotonic()


def boottime() -> float:
    """Seconds since boot including suspend (CLOCK_BOOTTIME), i.e. uptime."""
    try:
        return time.clock_gettime(time.CLOCK_BOOTTIME)
    except Exception:
        try:
            return float(read_text('/proc/uptime', '0 0').split()[0])
        except Exception:
            return -1.0


def iso(ts: Optional[float] = None) -> str:
    if ts is None:
        ts = wall()
    return datetime.datetime.fromtimestamp(ts).isoformat(timespec='milliseconds')


def read_text(path: str, default: Optional[str] = None) -> Optional[str]:
    try:
        with open(path, 'r', errors='replace') as f:
            return f.read().strip()
    except Exception:
        return default


def run(cmd, timeout: float = 15.0, shell: bool = False) -> str:
    """Run a command, return stdout+stderr; never raise."""
    try:
        p = subprocess.run(cmd, shell=shell, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           timeout=timeout, text=True, errors='replace')
        out = p.stdout.rstrip()
        if p.returncode != 0:
            out += f'\n[exit {p.returncode}]'
        return out
    except FileNotFoundError:
        return f'[not found: {cmd[0] if isinstance(cmd, list) else cmd}]'
    except subprocess.TimeoutExpired:
        return f'[timeout after {timeout}s]'
    except Exception as e:  # pragma: no cover
        return f'[error: {e}]'


def which(name: str) -> Optional[str]:
    return shutil.which(name)


def fsync_dir(path: str) -> None:
    try:
        fd = os.open(path, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    except Exception:
        pass


def atomic_write_json(path: str, obj) -> None:
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(obj, f, indent=1)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)
    fsync_dir(os.path.dirname(path))


class SyncWriter:
    """Append-only text file whose writes reach the disk quickly.

    fsync_interval <= 0  -> flush+fsync on every write (heartbeat, boot_info)
    fsync_interval  > 0  -> fsync at most every that many seconds; a background
                            syncer also forces a sync so a lone line never waits.
    """

    def __init__(self, path: str, fsync_interval: float = 0.0, header: Optional[str] = None):
        self.path = path
        self.interval = fsync_interval
        self.lock = threading.Lock()
        self.dirty = False
        self.last_sync = 0.0
        is_new = not os.path.exists(path) or os.path.getsize(path) == 0
        self.fd = os.open(path, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o644)
        self.f = os.fdopen(self.fd, 'w', buffering=1 << 16, errors='replace')
        if is_new:
            fsync_dir(os.path.dirname(path))  # make the new directory entry durable
            if header:
                self.write(header, force_sync=True)

    def write(self, line: str, force_sync: bool = False) -> None:
        with self.lock:
            try:
                self.f.write(line + '\n')
                self.dirty = True
                self._maybe_sync(force_sync)
            except Exception:
                pass

    def _maybe_sync(self, force: bool) -> None:
        if not self.dirty:
            return
        now = mono()
        if force or self.interval <= 0 or (now - self.last_sync) >= self.interval:
            self.f.flush()
            os.fsync(self.fd)
            self.dirty = False
            self.last_sync = now

    def sync(self) -> None:
        with self.lock:
            try:
                self._maybe_sync(True)
            except Exception:
                pass

    def close(self) -> None:
        self.sync()
        try:
            self.f.close()
        except Exception:
            pass


# --------------------------------------------------------------------------- probes
def decode_dt_value(raw: bytes) -> str:
    """Render a device-tree property: string if printable, else int/hex."""
    txt = raw.rstrip(b'\x00')
    if txt and all(32 <= b < 127 or b == 0 for b in txt):
        return ' | '.join(s.decode('ascii', 'replace') for s in txt.split(b'\x00') if s)
    if len(raw) == 4:
        return f'0x{int.from_bytes(raw, "big"):08x} ({int.from_bytes(raw, "big")})'
    return raw.hex() if raw else '(empty)'


def dump_reset_reason() -> str:
    """Everything under /proc/device-tree/chosen whose path mentions 'reset'.

    On Tegra (JetPack 5/6) the bootloader records the PMC reset source here,
    e.g. chosen/reset/pmc-reset-reason/reset-source and reset-level.
    """
    out: List[str] = []
    root = '/proc/device-tree/chosen'
    if not os.path.isdir(root):
        return f'{root}: absent (not a Tegra device-tree system?)'
    for dirpath, _dirs, files in os.walk(root):
        for fn in sorted(files):
            p = os.path.join(dirpath, fn)
            if 'reset' not in p.lower():
                continue
            try:
                with open(p, 'rb') as f:
                    out.append(f'{p[len(root) + 1:]}: {decode_dt_value(f.read())}')
            except Exception as e:
                out.append(f'{p[len(root) + 1:]}: [error {e}]')
    if not out:
        out.append('(no *reset* entries under /proc/device-tree/chosen)')
    return '\n'.join(out)


def dump_pstore(dest_dir: str) -> str:
    src = '/sys/fs/pstore'
    if not os.path.isdir(src):
        return f'{src}: absent'
    try:
        files = sorted(os.listdir(src))
    except PermissionError:
        return f'{src}: not readable (run as root to see panic records)'
    if not files:
        return f'{src}: empty (no kernel panic / oops record from last boot)'
    os.makedirs(dest_dir, exist_ok=True)
    lines = [f'{src}: {len(files)} file(s) -> copied to {dest_dir}']
    for fn in files:
        try:
            shutil.copy2(os.path.join(src, fn), os.path.join(dest_dir, fn))
            lines.append(f'  {fn}  {os.path.getsize(os.path.join(src, fn))} B')
        except Exception as e:
            lines.append(f'  {fn}  [copy failed: {e}]')
    fsync_dir(dest_dir)
    return '\n'.join(lines)


def imu_probe() -> str:
    """How is the IMU supposed to start, and is it running right now?"""
    pat = r'imu|corgi|ros2|ros_|ros-|\bros\b'
    parts = []
    parts.append('## enabled unit files matching ' + pat)
    parts.append(run(f"systemctl list-unit-files --no-pager --no-legend 2>/dev/null | grep -iE '{pat}' || true",
                     shell=True) or '(none)')
    parts.append('## loaded units matching ' + pat)
    parts.append(run(f"systemctl list-units --all --no-pager --no-legend 2>/dev/null | grep -iE '{pat}' || true",
                     shell=True) or '(none)')
    units = []
    for line in run(f"systemctl list-units --all --no-pager --no-legend --plain 2>/dev/null | grep -iE '{pat}'",
                    shell=True).splitlines():
        tok = line.split()
        if tok and tok[0].endswith(('.service', '.timer')):
            units.append(tok[0])
    for u in units[:8]:
        parts.append(f'## systemctl status {u}')
        parts.append(run(['systemctl', 'status', u, '--no-pager', '-n', '15'], timeout=10))
    parts.append('## root crontab')
    parts.append(run(['crontab', '-l']))
    for home in sorted(glob.glob('/home/*')):
        user = os.path.basename(home)
        parts.append(f'## crontab of {user}')
        parts.append(run(['crontab', '-u', user, '-l']))
        for d in glob.glob(os.path.join(home, '.config/autostart/*.desktop')):
            parts.append(f'## {d}')
            parts.append(read_text(d, '') or '')
    parts.append('## /etc/rc.local')
    parts.append(read_text('/etc/rc.local', '(absent)') or '(empty)')
    parts.append('## /etc/xdg/autostart entries mentioning ' + pat)
    parts.append(run(f"grep -ilE '{pat}' /etc/xdg/autostart/*.desktop 2>/dev/null || true", shell=True) or '(none)')
    parts.append('## serial devices')
    parts.append(run("ls -l /dev/ttyTHS* /dev/ttyUSB* /dev/ttyACM* 2>&1", shell=True))
    parts.append('## processes matching ' + pat)
    parts.append(run(f"ps -eo pid,ppid,etimes,user,stat,cmd | grep -iE '{pat}' | grep -v grep || true", shell=True)
                 or '(none)')
    return '\n'.join(parts)


class Rail:
    """One INA3221 channel: voltage and current sysfs files kept open."""

    def __init__(self, label: str, v_path: Optional[str], i_path: Optional[str],
                 p_path: Optional[str] = None, alarm_path: Optional[str] = None):
        self.label = label
        self.paths = {'mV': v_path, 'mA': i_path, 'mW': p_path, 'alarm': alarm_path}
        self.fds: Dict[str, Optional[int]] = {}
        self.reopen()

    def reopen(self) -> None:
        for k, fd in list(self.fds.items()):
            if fd is not None:
                try:
                    os.close(fd)
                except Exception:
                    pass
        self.fds = {}
        for k, p in self.paths.items():
            if p and os.path.exists(p):
                try:
                    self.fds[k] = os.open(p, os.O_RDONLY)
                except Exception:
                    self.fds[k] = None
            else:
                self.fds[k] = None

    def columns(self) -> List[str]:
        cols = []
        if self.fds.get('mV') is not None:
            cols.append(f'{self.label}_mV')
        if self.fds.get('mA') is not None:
            cols.append(f'{self.label}_mA')
        if self.fds.get('mV') is not None and self.fds.get('mA') is not None or self.fds.get('mW') is not None:
            cols.append(f'{self.label}_mW')
        if self.fds.get('alarm') is not None:
            cols.append(f'{self.label}_alarm')
        return cols

    @staticmethod
    def _read(fd: Optional[int]) -> Optional[float]:
        if fd is None:
            return None
        try:
            return float(os.pread(fd, 64, 0).strip())
        except Exception:
            return None

    def sample(self) -> List[str]:
        v = self._read(self.fds.get('mV'))
        i = self._read(self.fds.get('mA'))
        p = self._read(self.fds.get('mW'))
        a = self._read(self.fds.get('alarm'))
        vals = []
        if self.fds.get('mV') is not None:
            vals.append('' if v is None else f'{v:.0f}')
        if self.fds.get('mA') is not None:
            vals.append('' if i is None else f'{i:.0f}')
        if self.fds.get('mV') is not None and self.fds.get('mA') is not None:
            vals.append('' if (v is None or i is None) else f'{v * i / 1000.0:.0f}')
        elif self.fds.get('mW') is not None:
            vals.append('' if p is None else f'{p / 1000.0:.0f}')  # iio reports uW
        if self.fds.get('alarm') is not None:
            vals.append('' if a is None else f'{a:.0f}')
        return vals


def discover_rails() -> Tuple[List[Rail], str]:
    """Find INA3221 channels. JetPack 5/6: hwmon. JetPack 4: ina3221x iio."""
    rails: List[Rail] = []
    notes: List[str] = []
    seen: Dict[str, int] = {}
    for h in sorted(glob.glob('/sys/class/hwmon/hwmon*')):
        name = read_text(os.path.join(h, 'name'), '')
        if name not in ('ina3221', 'ina3221x', 'ina219', 'ina226', 'ina230'):
            continue
        for lab in sorted(glob.glob(os.path.join(h, 'in*_label'))):
            n = re.search(r'in(\d+)_label$', lab).group(1)
            label = (read_text(lab, '') or '').strip().replace(' ', '_')
            if not label or label.upper() == 'NC':
                continue
            if label in seen:
                seen[label] += 1
                label = f'{label}_{seen[label]}'
            else:
                seen[label] = 0
            alarm = os.path.join(h, f'curr{n}_crit_alarm')
            rails.append(Rail(label, os.path.join(h, f'in{n}_input'), os.path.join(h, f'curr{n}_input'),
                              alarm_path=alarm if os.path.exists(alarm) else None))
            notes.append(f'{h} ({name}) in{n}: {label}')
    if not rails:
        for d in sorted(glob.glob('/sys/bus/i2c/drivers/ina3221x/*/iio:device*')):
            for lab in sorted(glob.glob(os.path.join(d, 'rail_name_*'))):
                n = lab.rsplit('_', 1)[-1]
                label = (read_text(lab, '') or '').strip().replace(' ', '_')
                if not label or label.upper() == 'NC':
                    continue
                rails.append(Rail(label, os.path.join(d, f'in_voltage{n}_input'),
                                  os.path.join(d, f'in_current{n}_input'),
                                  p_path=os.path.join(d, f'in_power{n}_input')))
                notes.append(f'{d} rail {n}: {label}')
    if not rails:
        notes.append('no INA3221 found: power.csv will carry only clocks '
                     '(expected on a non-Jetson machine)')
    return rails, '\n'.join(notes)


def discover_clocks() -> List[Tuple[str, str]]:
    cols = []
    for p in sorted(glob.glob('/sys/devices/system/cpu/cpufreq/policy*/scaling_cur_freq'),
                    key=lambda q: int(re.sub(r'\D', '', q.split('/')[-2]) or 0)):
        name = p.split('/')[-2]  # policy0
        cols.append((f'cpu_{name}_kHz', p))
    for p in sorted(glob.glob('/sys/class/devfreq/*/cur_freq')):
        dev = p.split('/')[-2].lower()
        if any(k in dev for k in ('gpu', 'ga10b', 'gv11b', 'gp10b')):
            cols.append(('gpu_Hz', p))
            break
    return cols


def discover_thermal() -> List[Tuple[str, str]]:
    cols = []
    for z in sorted(glob.glob('/sys/class/thermal/thermal_zone*'),
                    key=lambda s: int(re.sub(r'\D', '', s) or 0)):
        t = (read_text(os.path.join(z, 'type'), '') or os.path.basename(z)).replace(' ', '_')
        cols.append((f'T_{t}_C', os.path.join(z, 'temp')))
    return cols


def discover_netifs() -> List[str]:
    return sorted(n for n in os.listdir('/sys/class/net') if n != 'lo') if os.path.isdir('/sys/class/net') else []


def ntp_synced() -> str:
    if os.path.exists('/run/systemd/timesync/synchronized'):
        return '1'
    if which('timedatectl'):
        v = run(['timedatectl', 'show', '-p', 'NTPSynchronized', '--value'], timeout=3).strip().lower()
        if v.startswith('yes'):
            return '1'
        if v.startswith('no'):
            return '0'
    if which('chronyc'):
        v = run(['chronyc', '-c', 'tracking'], timeout=3)
        if v and not v.startswith('['):
            return '1' if 'Leap status     : Normal' in run(['chronyc', 'tracking'], timeout=3) else '0'
    return '?'


def scan_procs() -> Tuple[int, int, int, int]:
    """(process count, imu_node running, corgi_ros_bridge running, corgi_* process count)"""
    n = imu = bridge = corgi = 0
    for d in os.listdir('/proc'):
        if not d.isdigit():
            continue
        n += 1
        try:
            with open(f'/proc/{d}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\x00', b' ').decode('utf-8', 'replace')
        except Exception:
            continue
        if not cmd:
            continue
        if 'imu_node' in cmd:
            imu = 1
        if 'corgi_ros_bridge' in cmd:
            bridge = 1
        if 'corgi_' in cmd and 'corgi_orin_monitor' not in cmd:
            corgi += 1
    return n, imu, bridge, corgi


# --------------------------------------------------------------------------- monitor
class Monitor:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.stop_event = threading.Event()
        self.stop_reason = ''
        self.writers: List[SyncWriter] = []
        self.threads: List[threading.Thread] = []
        self.procs: List[subprocess.Popen] = []
        self.proc_lock = threading.Lock()
        self.boot_id = args.fake_boot_id or read_text('/proc/sys/kernel/random/boot_id', 'unknown')
        self.log_dir = self._resolve_log_dir(args.log_dir)
        self.boot_dir, self.seq, self.new_boot = self._register_boot()
        self.mlog = self._writer('monitor.log', 0.0)
        self.note(f'corgi_orin_monitor v{VERSION} start pid={os.getpid()} uid={os.getuid()} '
                  f'boot_id={self.boot_id} seq={self.seq} new_boot_dir={self.new_boot} argv={sys.argv}')
        if os.getuid() != 0:
            self.note('WARNING: not root. dmesg/journal/nvpmodel/crontab may be unreadable. '
                      'Install the systemd service for full coverage.')
        self.hb: Optional[SyncWriter] = None

    # ---- setup
    @staticmethod
    def _resolve_log_dir(requested: Optional[str]) -> str:
        candidates = [requested] if requested else [DEFAULT_LOG_DIR, FALLBACK_LOG_DIR]
        for c in candidates:
            path = os.path.abspath(os.path.expanduser(c))
            try:
                os.makedirs(path, exist_ok=True)
                probe = os.path.join(path, '.write_test')
                with open(probe, 'w') as f:
                    f.write('ok')
                os.remove(probe)
                return path
            except Exception as e:
                sys.stderr.write(f'log dir {path} unusable ({e})\n')
        raise SystemExit('no writable log directory')

    def _load_index(self) -> List[dict]:
        p = os.path.join(self.log_dir, 'index.json')
        try:
            with open(p) as f:
                data = json.load(f)
            boots = data.get('boots', [])
        except Exception:
            boots = []
        # self-heal: pick up directories created by an older index or by hand
        known = {b.get('dir') for b in boots}
        for d in sorted(glob.glob(os.path.join(self.log_dir, 'boot_*'))):
            name = os.path.basename(d)
            if name in known or not os.path.isdir(d):
                continue
            m = re.match(r'boot_(\d+)_\d+_\d+_([0-9a-f]+)$', name)
            if m:
                boots.append({'seq': int(m.group(1)), 'boot_id_prefix': m.group(2), 'dir': name,
                              'recovered': True})
        boots.sort(key=lambda b: b.get('seq', 0))
        return boots

    def _save_index(self, boots: List[dict]) -> None:
        atomic_write_json(os.path.join(self.log_dir, 'index.json'),
                          {'version': VERSION, 'boots': boots})

    def _register_boot(self) -> Tuple[str, int, bool]:
        boots = self._load_index()
        for b in boots:
            if b.get('boot_id') == self.boot_id and os.path.isdir(os.path.join(self.log_dir, b['dir'])):
                return os.path.join(self.log_dir, b['dir']), b['seq'], False
        seq = (boots[-1]['seq'] + 1) if boots else 1
        name = f'boot_{seq:04d}_{time.strftime("%Y%m%d_%H%M%S")}_{self.boot_id[:7]}'
        path = os.path.join(self.log_dir, name)
        os.makedirs(path, exist_ok=True)
        fsync_dir(self.log_dir)
        boots.append({'seq': seq, 'boot_id': self.boot_id, 'dir': name,
                      'first_start_wall': wall(), 'first_start_iso': iso(),
                      'uptime_at_start_s': round(boottime(), 3), 'hostname': socket.gethostname()})
        self._save_index(boots)
        return path, seq, True

    def _writer(self, filename: str, interval: float, header: Optional[str] = None) -> SyncWriter:
        w = SyncWriter(os.path.join(self.boot_dir, filename), interval, header)
        self.writers.append(w)
        return w

    def note(self, msg: str) -> None:
        line = f'{iso()} {msg}'
        try:
            self.mlog.write(line, force_sync=True)
        except Exception:
            pass
        sys.stderr.write(line + '\n')
        sys.stderr.flush()

    def spawn(self, name: str, target, *a) -> None:
        t = threading.Thread(target=self._guard, args=(name, target) + a, name=name, daemon=True)
        t.start()
        self.threads.append(t)

    def _guard(self, name, target, *a) -> None:
        try:
            target(*a)
        except Exception as e:
            self.note(f'thread {name} died: {e!r}')

    # ---- streams
    def heartbeat_loop(self) -> None:
        hb = self._writer('heartbeat.csv', 0.0, 'wall_iso,epoch,monotonic,boottime,ntp_synced,event')
        self.hb = hb
        ntp = ntp_synced()
        hb.write(f'{iso()},{wall():.3f},{mono():.3f},{boottime():.3f},{ntp},'
                 f'START v{VERSION} pid={os.getpid()} uid={os.getuid()} seq={self.seq}', force_sync=True)
        n = 0
        t0 = mono()
        while not self.stop_event.is_set():
            n += 1
            if self.stop_event.wait(max(0.0, t0 + n - mono())):
                break
            if n % 5 == 0:
                ntp = ntp_synced()
            hb.write(f'{iso()},{wall():.3f},{mono():.3f},{boottime():.3f},{ntp},', force_sync=True)
            if n % 3600 == 0:
                self.enforce_budget()
        hb.write(f'{iso()},{wall():.3f},{mono():.3f},{boottime():.3f},{ntp},STOP {self.stop_reason}',
                 force_sync=True)

    def power_loop(self) -> None:
        rails, notes = discover_rails()
        clocks = discover_clocks()
        self.rail_notes = notes
        cols = ['wall_iso', 'epoch', 'monotonic']
        for r in rails:
            cols += r.columns()
        cols += [c for c, _ in clocks]
        w = self._writer('power.csv', self.args.fsync_interval, ','.join(cols))
        self.note(f'power.csv: {len(rails)} rail(s), {len(clocks)} clock(s), {self.args.power_hz} Hz')
        clock_fds = []
        for _c, p in clocks:
            try:
                clock_fds.append(os.open(p, os.O_RDONLY))
            except Exception:
                clock_fds.append(None)
        period = 1.0 / max(0.5, self.args.power_hz)
        next_t = mono()
        last_reopen = mono()
        while not self.stop_event.is_set():
            next_t += period
            if self.stop_event.wait(max(0.0, next_t - mono())):
                break
            row = [iso(), f'{wall():.3f}', f'{mono():.3f}']
            missing = False
            for r in rails:
                vals = r.sample()
                missing = missing or ('' in vals)
                row += vals
            for fd in clock_fds:
                try:
                    row.append(os.pread(fd, 32, 0).strip().decode() if fd is not None else '')
                except Exception:
                    row.append('')
            w.write(','.join(row))
            if missing and mono() - last_reopen > 5.0:
                last_reopen = mono()
                for r in rails:
                    r.reopen()

    def system_loop(self) -> None:
        thermal = discover_thermal()
        ifaces = discover_netifs()
        pings = list(self.args.ping or [])
        cols = ['wall_iso', 'epoch', 'monotonic', 'load1', 'cpu_pct', 'mem_avail_MB', 'swap_used_MB',
                'oom_kill_total', 'procs', 'imu_node', 'ros_bridge', 'corgi_procs', 'ttyTHS1', 'usb_devs']
        cols += [c for c, _ in thermal]
        cols += [f'net_{i}' for i in ifaces]
        cols += [f'ping_{h}_ms' for h in pings]
        w = self._writer('system.csv', 0.0, ','.join(cols))
        self.note(f'system.csv: {len(thermal)} thermal zone(s), ifaces={ifaces}, ping={pings}')
        prev_cpu = self._cpu_times()
        n = 0
        t0 = mono()
        while not self.stop_event.is_set():
            n += 1
            if self.stop_event.wait(max(0.0, t0 + n - mono())):
                break
            cur = self._cpu_times()
            cpu_pct = ''
            if prev_cpu and cur:
                tot = cur[0] - prev_cpu[0]
                idle = cur[1] - prev_cpu[1]
                cpu_pct = f'{100.0 * (tot - idle) / tot:.1f}' if tot > 0 else ''
            prev_cpu = cur
            mem = self._meminfo()
            vm = read_text('/proc/vmstat', '') or ''
            m = re.search(r'^oom_kill (\d+)', vm, re.M)
            procs, imu, bridge, corgi = scan_procs()
            row = [iso(), f'{wall():.3f}', f'{mono():.3f}',
                   (read_text('/proc/loadavg', '') or '').split(' ')[0], cpu_pct,
                   mem.get('MemAvailable', ''), mem.get('SwapUsed', ''), m.group(1) if m else '',
                   str(procs), str(imu), str(bridge), str(corgi),
                   '1' if os.path.exists('/dev/ttyTHS1') else '0',
                   str(len([d for d in glob.glob('/sys/bus/usb/devices/*') if ':' not in os.path.basename(d)
                            and not os.path.basename(d).startswith('usb')]))]
            for _c, p in thermal:
                t = read_text(p)
                try:
                    row.append(f'{int(t) / 1000.0:.1f}' if t is not None else '')
                except Exception:
                    row.append('')
            for i in ifaces:
                st = read_text(f'/sys/class/net/{i}/operstate', '?')
                car = read_text(f'/sys/class/net/{i}/carrier', '?')
                row.append(f'{st}/{car}')
            for h in pings:
                row.append(self._ping_ms(h))
            w.write(','.join(row))

    @staticmethod
    def _cpu_times() -> Optional[Tuple[int, int]]:
        line = (read_text('/proc/stat', '') or '').splitlines()[0:1]
        if not line or not line[0].startswith('cpu '):
            return None
        f = [int(x) for x in line[0].split()[1:]]
        return sum(f), f[3] + (f[4] if len(f) > 4 else 0)

    @staticmethod
    def _meminfo() -> Dict[str, str]:
        d: Dict[str, str] = {}
        txt = read_text('/proc/meminfo', '') or ''
        for key in ('MemAvailable', 'SwapTotal', 'SwapFree'):
            m = re.search(rf'^{key}:\s+(\d+)', txt, re.M)
            if m:
                d[key] = m.group(1)
        if 'MemAvailable' in d:
            d['MemAvailable'] = f'{int(d["MemAvailable"]) / 1024:.0f}'
        if 'SwapTotal' in d and 'SwapFree' in d:
            d['SwapUsed'] = f'{(int(d["SwapTotal"]) - int(d["SwapFree"])) / 1024:.0f}'
        return d

    @staticmethod
    def _ping_ms(host: str) -> str:
        out = run(['ping', '-n', '-c', '1', '-W', '1', host], timeout=3)
        m = re.search(r'time=([\d.]+) ms', out)
        return m.group(1) if m else 'x'

    def stream_loop(self, name: str, candidates: List[List[str]], filename: str) -> None:
        """Run the first working command of `candidates`, timestamp every line.

        A candidate that dies within 3 s with a non-zero exit (e.g. dmesg under
        dmesg_restrict without root) is skipped for the next one.  A stream that
        worked and later dies is restarted, up to 20 times.
        """
        w = self._writer(filename, self.args.fsync_interval)
        avail = [c for c in candidates if which(c[0])]
        if not avail:
            self.note(f'{name}: none of {[c[0] for c in candidates]} found; stream disabled')
            w.write(f'{iso()} {wall():.3f} | [{name} unavailable on this machine]')
            return
        ci = 0
        restarts = 0
        while not self.stop_event.is_set() and ci < len(avail):
            cmd = avail[ci]
            try:
                proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        text=True, errors='replace', bufsize=1)
            except Exception as e:
                self.note(f'{name}: failed to start {cmd}: {e}')
                ci += 1
                continue
            with self.proc_lock:
                self.procs.append(proc)
            t_start = mono()
            self.note(f'{name}: started {" ".join(cmd)} pid={proc.pid}')
            w.write(f'{iso()} {wall():.3f} | [{name} started: {" ".join(cmd)}]')
            try:
                for line in proc.stdout:  # type: ignore[union-attr]
                    w.write(f'{iso()} {wall():.3f} | {line.rstrip()}')
                    if self.stop_event.is_set():
                        break
            except Exception as e:
                self.note(f'{name}: read error {e}')
            rc = proc.wait()
            if self.stop_event.is_set():
                return
            w.write(f'{iso()} {wall():.3f} | [{name} exited rc={rc}]')
            if rc != 0 and mono() - t_start < 3.0:
                ci += 1
                self.note(f'{name}: {cmd[0]} failed immediately (rc={rc}); '
                          + (f'trying {avail[ci][0]}' if ci < len(avail) else 'no fallback left, stream disabled'))
                continue
            restarts += 1
            if restarts >= 20:
                self.note(f'{name}: exited rc={rc} too many times; giving up')
                return
            self.note(f'{name}: exited rc={rc}; restart {restarts}/20 in 5 s')
            if self.stop_event.wait(5):
                return

    # ---- one-shot collections
    def collect_boot_info(self) -> None:
        bi = self._writer('boot_info.txt', 0.0)
        if not self.new_boot:
            bi.write(f'\n===== monitor restarted {iso()} (same boot, seq {self.seq}) =====')
            bi.write(f'uptime_s: {boottime():.1f}  ntp_synced: {ntp_synced()}')
            return

        def sec(title: str, fn) -> None:
            """One section; a failing probe writes its error instead of killing the thread."""
            try:
                body = fn() if callable(fn) else fn
            except Exception as e:
                body = f'[probe failed: {e!r}]'
            bi.write(f'\n### {title}\n{(body or "").rstrip()}', force_sync=True)

        bi.write(f'corgi_orin_monitor v{VERSION} boot_info  seq={self.seq}  boot_id={self.boot_id}\n'
                 f'collected_at: {iso()}  epoch={wall():.3f}  uptime_s={boottime():.1f}  '
                 f'host={socket.gethostname()}  uid={os.getuid()}', force_sync=True)
        # Most valuable facts first, in case power goes again within seconds.
        sec('reset_reason (device-tree, written by the bootloader for THIS boot)', dump_reset_reason)
        try:
            self.write_prev_boot_summary()
        except Exception as e:
            self.note(f'prev_boot_summary failed: {e!r}')
        sec('pstore (kernel panic/oops left by the PREVIOUS boot)',
            lambda: dump_pstore(os.path.join(self.boot_dir, 'pstore')))
        sec('clock', lambda: run('date -Is; echo; timedatectl 2>&1 | head -12', shell=True))
        sec('kernel.panic / panic_on_oops (0 = a panic hangs instead of rebooting)',
            lambda: run('sysctl kernel.panic kernel.panic_on_oops kernel.hung_task_panic 2>&1', shell=True))
        sec('watchdogs', lambda: run(
            'for w in /sys/class/watchdog/*; do [ -e "$w" ] || continue; echo "$w: identity=$(cat $w/identity 2>/dev/null) '
            'state=$(cat $w/state 2>/dev/null) timeout=$(cat $w/timeout 2>/dev/null)"; done; '
            'systemctl show -p RuntimeWatchdogUSec -p RebootWatchdogUSec 2>/dev/null', shell=True) or '(none)')
        sec('journal storage (/var/log/journal present => journalctl -b -1 works across reboots)',
            lambda: run('ls -ld /var/log/journal 2>&1; journalctl --disk-usage 2>&1; '
                        'grep -E "^#?Storage" /etc/systemd/journald.conf 2>/dev/null', shell=True))
        sec('journalctl --list-boots (tail)',
            lambda: run('journalctl --list-boots --no-pager 2>&1 | tail -15', shell=True))
        sec('journal_prev_boot_tail (last 150 lines of previous boot; a shutdown sequence here = software reboot)',
            lambda: run('journalctl -b -1 -n 150 --no-pager -o short-iso 2>&1', shell=True, timeout=30))
        sec('journal_prev_boot_errors',
            lambda: run('journalctl -b -1 -p err -n 60 --no-pager -o short-iso 2>&1', shell=True, timeout=30))
        sec('last -x (reboot/shutdown records from wtmp)',
            lambda: run('last -x -n 25 shutdown reboot 2>&1', shell=True))
        sec('platform', lambda: run(
            'uname -a; cat /etc/nv_tegra_release 2>/dev/null; '
            'dpkg-query -W nvidia-jetpack nvidia-l4t-core 2>/dev/null; '
            'grep PRETTY_NAME /etc/os-release; cat /proc/device-tree/model 2>/dev/null | tr -d "\\0"; echo',
            shell=True))
        sec('/proc/cmdline', lambda: read_text('/proc/cmdline', '') or '')
        sec('nvpmodel -q (power mode; MAXN draws the highest peak current)',
            lambda: run(['nvpmodel', '-q'], timeout=10))
        sec('jetson_clocks --show', lambda: run(['jetson_clocks', '--show'], timeout=15))
        sec('power rails found for power.csv',
            lambda: getattr(self, 'rail_notes', '(power thread not started yet)'))
        sec('thermal zones', lambda: '\n'.join(f'{c}: {p}' for c, p in discover_thermal()) or '(none)')
        sec('storage', lambda: run('df -h / /var/log 2>&1; echo; findmnt -no SOURCE,FSTYPE,OPTIONS / 2>&1',
                                   shell=True))
        sec('network', lambda: run('ip -brief link 2>&1; echo; ip -brief addr 2>&1', shell=True))
        sec('usb', lambda: run(['lsusb'], timeout=10))
        sec('pci', lambda: run(['lspci'], timeout=10))
        sec('imu_autostart_probe', imu_probe)
        sec('top processes', lambda: run('ps -eo pid,etimes,pcpu,pmem,stat,cmd --sort=-pcpu | head -25', shell=True))
        sec('dmesg_boot_errors (this boot so far)',
            lambda: run('dmesg --level=emerg,alert,crit,err 2>&1 | tail -60', shell=True))
        bi.write(f'\n### done {iso()}', force_sync=True)
        self.note('boot_info.txt complete')
        self.enforce_budget()

    def write_prev_boot_summary(self) -> None:
        if not self.new_boot or self.seq <= 1:
            return
        report = os.path.join(HERE, 'report.py')
        if not os.path.exists(report):
            self.note('report.py not found next to monitor.py; skipping prev_boot_summary')
            return
        out = run([sys.executable, report, '--log-dir', self.log_dir, '--boot', 'prev', '--tail-sec', '30'],
                  timeout=60)
        w = self._writer('prev_boot_summary.txt', 0.0)
        w.write(f'generated {iso()} by seq {self.seq}\n{out}', force_sync=True)
        self.note('prev_boot_summary.txt written')

    def late_probe(self) -> None:
        if self.stop_event.wait(self.args.late_probe_sec):
            return
        w = self._writer('late_probe.txt', 0.0)
        w.write(f'===== late probe at {iso()} uptime_s={boottime():.1f} ntp_synced={ntp_synced()} =====')
        w.write(imu_probe())
        w.write('\n## journal lines mentioning imu / ttyTHS1 in this boot')
        w.write(run("journalctl -b 0 --no-pager -o short-iso 2>&1 | grep -iE 'imu|ttyTHS1' | tail -80",
                    shell=True, timeout=30) or '(none)')
        w.sync()
        self.note('late_probe.txt written')

    def enforce_budget(self) -> None:
        """Delete oldest boot directories beyond --max-total-gb; never the last two."""
        try:
            boots = self._load_index()
            limit = self.args.max_total_gb * (1 << 30)
            sizes = {}
            total = 0
            for b in boots:
                p = os.path.join(self.log_dir, b['dir'])
                s = sum(os.path.getsize(os.path.join(dp, f)) for dp, _d, fs in os.walk(p) for f in fs
                        if os.path.exists(os.path.join(dp, f)))
                sizes[b['dir']] = s
                total += s
            removed = []
            while total > limit and len(boots) > 2:
                victim = boots.pop(0)
                shutil.rmtree(os.path.join(self.log_dir, victim['dir']), ignore_errors=True)
                total -= sizes.get(victim['dir'], 0)
                removed.append(victim['dir'])
            if removed:
                self._save_index(boots)
                self.note(f'budget: removed {removed}, now {total / (1 << 30):.2f} GB')
            else:
                self.note(f'budget: {total / (1 << 30):.2f} GB of {self.args.max_total_gb} GB in {len(boots)} boot dir(s)')
        except Exception as e:
            self.note(f'budget check failed: {e}')

    def syncer_loop(self) -> None:
        while not self.stop_event.wait(0.5):
            for w in list(self.writers):
                w.sync()

    # ---- lifecycle
    def start(self) -> None:
        disabled = set(self.args.disable or [])
        self.spawn('heartbeat', self.heartbeat_loop)
        if 'power' not in disabled:
            self.spawn('power', self.power_loop)
        if 'system' not in disabled:
            self.spawn('system', self.system_loop)
        if 'tegrastats' not in disabled:
            self.spawn('tegrastats', self.stream_loop, 'tegrastats',
                       [['tegrastats', '--interval', str(self.args.tegrastats_ms)]], 'tegrastats.log')
        if 'dmesg' not in disabled:
            self.spawn('dmesg', self.stream_loop, 'dmesg',
                       [['dmesg', '--follow', '--time-format=iso'], ['dmesg', '-w'],
                        ['journalctl', '-k', '-f', '-o', 'short-iso-precise']], 'dmesg.log')
        if 'journal' not in disabled:
            self.spawn('journal', self.stream_loop, 'journal',
                       [['journalctl', '-b', '0', '-f', '-o', 'short-iso-precise']], 'journal.log')
        self.spawn('syncer', self.syncer_loop)
        time.sleep(0.3)  # let power_loop fill rail_notes before boot_info reads it
        self.spawn('boot_info', self.collect_boot_info)
        self.spawn('late_probe', self.late_probe)

    def stop(self, reason: str) -> None:
        if self.stop_event.is_set():
            return
        self.stop_reason = reason
        self.note(f'stopping: {reason}')
        self.stop_event.set()
        with self.proc_lock:
            for p in self.procs:
                try:
                    p.terminate()
                except Exception:
                    pass
        for t in self.threads:
            t.join(timeout=3.0)
        for w in self.writers:
            w.close()

    def run_forever(self) -> None:
        def on_signal(signum, _frame):
            self.stop(f'signal {signal.Signals(signum).name}')

        for s in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
            signal.signal(s, on_signal)
        self.start()
        while not self.stop_event.is_set():
            time.sleep(0.5)
        time.sleep(0.2)


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description='Always-on Orin health/power logger (post-mortem for unexpected reboots).',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument('--log-dir', default=None,
                    help=f'where to write; default {DEFAULT_LOG_DIR} if writable, else {FALLBACK_LOG_DIR}')
    ap.add_argument('--power-hz', type=float, default=20.0, help='INA3221 sample rate')
    ap.add_argument('--fsync-interval', type=float, default=0.1,
                    help='max seconds between fsyncs for high-rate files (heartbeat always syncs each row)')
    ap.add_argument('--tegrastats-ms', type=int, default=500, help='tegrastats interval')
    ap.add_argument('--late-probe-sec', type=float, default=90.0,
                    help='seconds after start to re-check IMU/service state')
    ap.add_argument('--max-total-gb', type=float, default=8.0,
                    help='delete oldest boot directories beyond this total (never the last two)')
    ap.add_argument('--ping', action='append', metavar='HOST',
                    help='also ping HOST once per second in system.csv (e.g. the sbRIO); repeatable')
    ap.add_argument('--disable', action='append', choices=['power', 'system', 'tegrastats', 'dmesg', 'journal'],
                    help='turn a stream off; repeatable')
    ap.add_argument('--fake-boot-id', default=None, help='testing only: pretend this is a different boot')
    return ap.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    os.umask(0o022)
    args = parse_args(argv)
    m = Monitor(args)
    m.run_forever()
    return 0


if __name__ == '__main__':
    sys.exit(main())
