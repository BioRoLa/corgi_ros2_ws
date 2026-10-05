#!/usr/bin/env python3
"""
corgi_orin_monitor.report -- post-mortem for an unexpected Orin reboot.

    orin_monitor_report --all                 one line per boot: how did it end?
    orin_monitor_report                       details of the previous boot (default)
    orin_monitor_report --boot 7              details of boot seq 7
    orin_monitor_report --boot cur            the running boot

"How did it end" comes from three independent sources, printed side by side:
  1. the last heartbeat row: a STOP marker means the monitor was told to stop
     (systemd shutdown) -> software reboot; no marker -> the power or a reset
     line went away under it (or kill -9 / monitor crash, which you would know).
  2. the NEXT boot's journal_prev_boot_tail: a systemd shutdown sequence there
     is the second witness for a software reboot.
  3. the NEXT boot's reset_reason from the device tree (bootloader-written).
Calibrate once: do a `sudo reboot` and a deliberate power pull while the
monitor runs, then compare those two rows with the unexpected ones.
"""
import argparse
import datetime
import glob
import json
import os
import re
import sys
from typing import Dict, List, Optional, Tuple

DEFAULT_LOG_DIR = '/var/log/corgi_orin_monitor'
FALLBACK_LOG_DIR = '~/corgi_orin_monitor_logs'
ERR_RE = re.compile(r'error|fail|timeout|SError|overcurrent|over-current|throttl|reset|voltage|brown|'
                    r'under|oom|hung|watchdog|panic|BUG|Oops|i2c|usb .*disconnect|link is down|'
                    r'ttyTHS|imu', re.I)


# --------------------------------------------------------------------------- io
def fmt_ts(epoch: Optional[float]) -> str:
    if epoch is None:
        return '-'
    return datetime.datetime.fromtimestamp(epoch).strftime('%Y-%m-%d %H:%M:%S')


def fmt_dur(s: Optional[float]) -> str:
    if s is None:
        return '-'
    s = int(round(s))
    if s < 0:
        return f'{s}s(!)'
    h, r = divmod(s, 3600)
    m, r = divmod(r, 60)
    return f'{h}h{m:02d}m{r:02d}s' if h else (f'{m}m{r:02d}s' if m else f'{r}s')


def tail_text(path: str, nbytes: int = 1 << 20) -> str:
    try:
        size = os.path.getsize(path)
        with open(path, 'rb') as f:
            if size > nbytes:
                f.seek(size - nbytes)
                f.readline()  # drop the partial line
            return f.read().decode('utf-8', 'replace')
    except Exception:
        return ''


def tail_lines(path: str, n: int) -> List[str]:
    return tail_text(path, max(64 << 10, n * 400)).splitlines()[-n:]


def head_lines(path: str, n: int) -> List[str]:
    try:
        out = []
        with open(path, 'r', errors='replace') as f:
            for line in f:
                out.append(line.rstrip('\n'))
                if len(out) >= n:
                    break
        return out
    except Exception:
        return []


def resolve_log_dir(requested: Optional[str]) -> str:
    for c in ([requested] if requested else [DEFAULT_LOG_DIR, FALLBACK_LOG_DIR]):
        p = os.path.abspath(os.path.expanduser(c))
        if os.path.isdir(p):
            return p
    raise SystemExit(f'log dir not found: {requested or DEFAULT_LOG_DIR}')


def load_boots(log_dir: str) -> List[dict]:
    boots: List[dict] = []
    try:
        with open(os.path.join(log_dir, 'index.json')) as f:
            boots = json.load(f).get('boots', [])
    except Exception:
        pass
    known = {b.get('dir') for b in boots}
    for d in sorted(glob.glob(os.path.join(log_dir, 'boot_*'))):
        name = os.path.basename(d)
        m = re.match(r'boot_(\d+)_', name)
        if m and name not in known and os.path.isdir(d):
            boots.append({'seq': int(m.group(1)), 'dir': name})
    boots = [b for b in boots if os.path.isdir(os.path.join(log_dir, b['dir']))]
    boots.sort(key=lambda b: b.get('seq', 0))
    for b in boots:
        b['path'] = os.path.join(log_dir, b['dir'])
    return boots


# --------------------------------------------------------------------------- facts per boot
def heartbeat_facts(boot: dict) -> dict:
    """start/last epochs, boot wall time, ending classification."""
    f: Dict[str, object] = {'has_hb': False}
    p = os.path.join(boot['path'], 'heartbeat.csv')
    head = head_lines(p, 2)
    tail = tail_lines(p, 6)
    if len(head) < 2:
        return f
    f['has_hb'] = True

    def parse(row: str) -> Optional[dict]:
        c = row.split(',', 5)
        if len(c) < 6 or c[0] == 'wall_iso':
            return None
        try:
            return {'epoch': float(c[1]), 'mono': float(c[2]), 'boottime': float(c[3]),
                    'ntp': c[4], 'event': c[5]}
        except ValueError:
            return None

    first = parse(head[1])
    rows = [r for r in (parse(x) for x in tail) if r]
    if not first or not rows:
        return f
    last = rows[-1]
    f['first'] = first
    f['last'] = last
    f['boot_wall'] = first['epoch'] - first['boottime']       # when Linux started, by this boot's clock
    f['start_epoch'] = first['epoch']
    f['last_epoch'] = last['epoch']
    f['last_uptime'] = last['boottime']
    try:
        with open(p, 'rb') as fh:
            f['starts'] = sum(1 for ln in fh if b',START' in ln)
    except Exception:
        f['starts'] = sum(1 for r in rows if r['event'].startswith('START'))
    stop = next((r for r in reversed(rows) if r['event'].startswith('STOP')), None)
    if stop is not None and stop is last:
        f['ending'] = 'CLEAN'
        f['ending_detail'] = last['event']
    elif stop is not None:
        f['ending'] = 'CLEAN?'  # STOP then more rows: monitor restarted and then died silently
        f['ending_detail'] = f'STOP seen but heartbeats continued after it: {stop["event"]}'
    else:
        f['ending'] = 'ABRUPT'
        f['ending_detail'] = 'no STOP marker: power loss / hard reset / hang+watchdog (or kill -9)'
    return f


def boot_info_section(boot: dict, title_prefix: str) -> str:
    p = os.path.join(boot['path'], 'boot_info.txt')
    try:
        txt = open(p, errors='replace').read()
    except Exception:
        return ''
    ms = re.findall(r'^### ' + re.escape(title_prefix) + r'.*?\n(.*?)(?=^### |^===== |\Z)', txt, re.S | re.M)
    return ms[-1].strip() if ms else ''  # a restart re-reads some sections; the latest wins


def reset_reason(boot: dict) -> str:
    sec = boot_info_section(boot, 'reset_reason')
    if not sec:
        return '-'
    keep = [ln for ln in sec.splitlines()
            if re.search(r'reset[-_](source|level|reason)', ln, re.I) and not re.search(r'no reset_reason|no \*reset\*', ln)]
    keep = keep or sec.splitlines()[:2]
    short = []
    for ln in keep:
        ln = ln.strip()
        if ':' in ln:
            path, _, val = ln.partition(':')
            ln = f'{path.rstrip().rsplit("/", 1)[-1]}={val.strip()}'
        short.append(ln)
    return '; '.join(short)[:160]


def shutdown_evidence(boot: dict) -> str:
    sec = boot_info_section(boot, 'journal_prev_boot_tail')
    if not sec:
        return '-'
    if re.search(r'Reached target .*(Power-Off|Reboot|Shutdown|Final Step|System Reboot)|'
                 r'systemd-shutdown|Shutting down\.|reboot: Restarting system|Stopped target', sec):
        return 'shutdown sequence present'
    if '[exit' in sec or 'No journal' in sec or 'Specifying boot ID' in sec:
        return 'journal of previous boot unavailable (volatile journal?)'
    return 'no shutdown sequence (log just stops)'


def pstore_evidence(boot: dict) -> str:
    sec = boot_info_section(boot, 'pstore')
    if not sec:
        return '-'
    if 'copied to' in sec:
        return 'PANIC RECORD'
    if 'not readable' in sec:
        return 'unreadable'
    return 'empty' if 'empty' in sec else 'absent'


def csv_tail_by_seconds(path: str, seconds: float, max_rows: int = 200000) -> Tuple[List[str], List[List[str]]]:
    txt = tail_text(path, 6 << 20)
    lines = txt.splitlines()
    header = head_lines(path, 1)
    cols = header[0].split(',') if header else []
    rows = [ln.split(',') for ln in lines if ln and not ln.startswith('wall_iso')]
    rows = [r for r in rows if len(r) == len(cols)]
    if not rows:
        return cols, []
    try:
        last_epoch = float(rows[-1][1])
    except (ValueError, IndexError):
        return cols, rows[-50:]
    keep = []
    for r in reversed(rows):
        try:
            if float(r[1]) < last_epoch - seconds:
                break
        except ValueError:
            continue
        keep.append(r)
        if len(keep) >= max_rows:
            break
    keep.reverse()
    return cols, keep


def summarize_numeric(cols: List[str], rows: List[List[str]], skip: int = 3) -> List[str]:
    out = []
    for j in range(skip, len(cols)):
        vals = []
        for r in rows:
            try:
                vals.append(float(r[j]))
            except (ValueError, IndexError):
                pass
        if not vals:
            strs = [r[j] for r in rows if j < len(r) and r[j] != '']
            if strs:
                changed = '' if len(set(strs)) == 1 else f'  (changed: {" -> ".join(dict.fromkeys(strs))})'
                out.append(f'  {cols[j]:<28} last {strs[-1]!r}{changed}')
            else:
                out.append(f'  {cols[j]:<28} (no data)')
            continue
        out.append(f'  {cols[j]:<28} min {min(vals):>10.1f}  max {max(vals):>10.1f}  '
                   f'last {vals[-1]:>10.1f}  (n={len(vals)})')
    return out


# --------------------------------------------------------------------------- printing
def print_all(boots: List[dict]) -> None:
    print(f'{"seq":>4}  {"boot (Linux start)":<19}  {"last heartbeat":<19}  {"up":>9}  {"ending":<7}  '
          f'{"gap->next":>9}  {"next: journal says":<34}  pstore   next: reset reason')
    for i, b in enumerate(boots):
        hb = heartbeat_facts(b)
        nxt = boots[i + 1] if i + 1 < len(boots) else None
        gap = None
        if nxt and hb.get('has_hb'):
            nhb = heartbeat_facts(nxt)
            if nhb.get('has_hb'):
                gap = nhb['boot_wall'] - hb['last_epoch']
        flag = ''
        if hb.get('has_hb') and (hb['last']['ntp'] != '1'):
            flag = '~'  # clock not NTP-synced at the end; gap unreliable
        print(f'{b.get("seq", 0):>4}  {fmt_ts(hb.get("boot_wall")):<19}  {fmt_ts(hb.get("last_epoch")):<19}  '
              f'{fmt_dur(hb.get("last_uptime")):>9}  {str(hb.get("ending", "-")):<7}  '
              f'{(flag + fmt_dur(gap)) if gap is not None else ("running" if nxt is None else "-"):>9}  '
              f'{(shutdown_evidence(nxt) if nxt else "-"):<34}  {(pstore_evidence(nxt) if nxt else "-"):<8} '
              f'{reset_reason(nxt) if nxt else "-"}')
    print('\n  up = uptime at last heartbeat.  ~ before a gap = clock was not NTP-synced, gap unreliable.')
    print('  ending: CLEAN = monitor got SIGTERM (systemd shutdown) ; ABRUPT = heartbeat just stopped.')


def print_boot(boots: List[dict], idx: int, tail_sec: float) -> None:
    b = boots[idx]
    nxt = boots[idx + 1] if idx + 1 < len(boots) else None
    hb = heartbeat_facts(b)
    print(f'=== boot seq {b.get("seq")}  dir {b["dir"]}  ({"previous" if nxt and idx == len(boots) - 2 else "selected"}) ===')
    if not hb.get('has_hb'):
        print('no heartbeat.csv rows; nothing to say about this boot')
        return
    print(f'Linux started        {fmt_ts(hb["boot_wall"])}   (monitor start {fmt_ts(hb["start_epoch"])}, '
          f'ntp_synced={hb["first"]["ntp"]}, monitor starts in this boot: {hb["starts"]})')
    print(f'last heartbeat       {fmt_ts(hb["last_epoch"])}   uptime {fmt_dur(hb["last_uptime"])}   '
          f'ntp_synced={hb["last"]["ntp"]}')
    print(f'ending               {hb["ending"]}: {hb["ending_detail"]}')
    if nxt:
        nhb = heartbeat_facts(nxt)
        if nhb.get('has_hb'):
            gap = nhb['boot_wall'] - hb['last_epoch']
            warn = '' if (hb['last']['ntp'] == '1' and nhb['first']['ntp'] == '1') else \
                '   (!) a clock was not NTP-synced; treat the gap as approximate'
            print(f'next boot (seq {nxt.get("seq")}) Linux started {fmt_ts(nhb["boot_wall"])}; '
                  f'gap after last heartbeat {fmt_dur(gap)}{warn}')
            print('   a gap of only seconds = the board came straight back (auto power-on after a dip, or a reset)')
        print(f'next boot journal:   {shutdown_evidence(nxt)}')
        print(f'next boot pstore:    {pstore_evidence(nxt)}')
        print(f'next boot reset reason (device tree):')
        sec = boot_info_section(nxt, 'reset_reason')
        for ln in (sec.splitlines() or ['-']):
            print('   ' + ln)
    else:
        print('no later boot recorded (this is the running boot, or the monitor has not started since)')

    # --- last seconds of each stream
    def show_csv(name: str, seconds: float, raw_rows: int) -> None:
        p = os.path.join(b['path'], name)
        if not os.path.exists(p):
            print(f'\n--- {name}: absent')
            return
        cols, rows = csv_tail_by_seconds(p, seconds)
        print(f'\n--- {name}: last {seconds:.0f} s before the end ({len(rows)} rows)')
        if not rows:
            return
        for ln in summarize_numeric(cols, rows):
            print(ln)
        print(f'  last {raw_rows} rows:')
        print('  ' + ','.join(cols))
        for r in rows[-raw_rows:]:
            print('  ' + ','.join(r))

    show_csv('power.csv', tail_sec, 10)
    show_csv('system.csv', tail_sec, 5)

    def show_log(name: str, n: int, only_errors: bool = False) -> None:
        p = os.path.join(b['path'], name)
        if not os.path.exists(p):
            print(f'\n--- {name}: absent')
            return
        lines = tail_lines(p, 400 if only_errors else n)
        if only_errors:
            lines = [ln for ln in lines if ERR_RE.search(ln)][-n:]
        print(f'\n--- {name}: last {len(lines)} {"error-ish " if only_errors else ""}lines')
        for ln in lines:
            print('  ' + ln[:300])

    show_log('tegrastats.log', 8)
    show_log('dmesg.log', 25)
    show_log('dmesg.log', 15, only_errors=True)
    show_log('journal.log', 25)
    show_log('monitor.log', 10)


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--log-dir', default=None)
    ap.add_argument('--boot', default='prev', help='prev (default) | cur | <seq> | <dir name>')
    ap.add_argument('--tail-sec', type=float, default=30.0, help='seconds before the end to summarize')
    ap.add_argument('--all', action='store_true', help='one-line table of every boot')
    a = ap.parse_args(argv)
    log_dir = resolve_log_dir(a.log_dir)
    boots = load_boots(log_dir)
    if not boots:
        print(f'no boot directories in {log_dir}')
        return 1
    print(f'log dir: {log_dir}   boots recorded: {len(boots)}\n')
    if a.all:
        print_all(boots)
        return 0
    if a.boot == 'cur':
        idx = len(boots) - 1
    elif a.boot == 'prev':
        if len(boots) < 2:
            print('only one boot recorded; nothing previous to report. Use --boot cur for the running one.')
            return 1
        idx = len(boots) - 2
    else:
        idx = next((i for i, b in enumerate(boots) if str(b.get('seq')) == a.boot or b['dir'] == a.boot), None)
        if idx is None:
            print(f'boot {a.boot!r} not found; have seqs {[b.get("seq") for b in boots]}')
            return 1
    print_boot(boots, idx, a.tail_sec)
    return 0


if __name__ == '__main__':
    sys.exit(main())
