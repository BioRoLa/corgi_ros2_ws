# corgi_orin_monitor

Always-on health / power logger for the Orin, plus a post-mortem report for
unexpected reboots.

**Why.** The Orin reboots by itself, also at standstill with the motors powered
and with normal temperatures. When the cause is a supply dip or a hard reset the
kernel never gets to write anything, so no log on the Orin explains it. This
package keeps a continuous record that reaches the disk within ~100 ms
(`fsync`), and at every boot it collects the bootloader's **reset reason** and
summarises the **previous boot's last seconds**. Run it for a week, then read
the table.

Pure Python standard library. No ROS dependency: it starts from systemd a few
seconds after the kernel, and keeps running whether or not ROS is launched.

## Install on the Orin (once)

```bash
cd ~/corgi_ws/corgi_ros2_ws/src/corgi_orin_monitor
scripts/install_service.sh --persistent-journal
```

`--persistent-journal` makes journald keep logs across reboots
(`/var/log/journal`), so `journalctl -b -1` works too. Optional extras:

```bash
EXTRA_ARGS="--ping 192.168.1.10" scripts/install_service.sh   # also ping the sbRIO at 1 Hz
LOG_DIR=/data/orin_monitor scripts/install_service.sh         # another log location
scripts/uninstall_service.sh                                   # remove the service, keep logs
```

The service runs as root straight from this source tree (no `colcon build`
needed). `colcon build` still works and gives `ros2 run corgi_orin_monitor
orin_monitor` / `orin_monitor_report` for manual use; without root some
sources (dmesg, journal, nvpmodel, pstore) are unreadable.

## After a reboot: what to run

```bash
python3 ~/corgi_ws/corgi_ros2_ws/src/corgi_orin_monitor/corgi_orin_monitor/report.py --all
```

One row per Linux boot:

```
 seq  boot (Linux start)   last heartbeat              up  ending   gap->next  next: journal says         pstore  next: reset reason
  12  2026-10-06 09:12:03  2026-10-06 10:41:27   1h29m24s  ABRUPT         14s  no shutdown sequence       empty   reset/pmc-reset-reason/reset-source: SYS_RESET_N; reset-level: ...
  13  2026-10-06 10:41:41  2026-10-06 12:03:10   1h21m29s  CLEAN          31s  shutdown sequence present  empty   ... MAINSWRST ...
```

Three independent witnesses say how a boot ended:

| column | source | meaning |
|---|---|---|
| `ending` | last row of `heartbeat.csv` | `CLEAN` = the monitor received SIGTERM, i.e. systemd was shutting down (software reboot). `ABRUPT` = the heartbeat simply stops: power loss, reset line, or hang + watchdog. |
| `next: journal says` | next boot's `journalctl -b -1` tail | a systemd shutdown sequence is the second witness for a software reboot |
| `next: reset reason` | next boot's `/proc/device-tree/chosen/reset/...` (bootloader-written) | the PMC's own record of why it reset |
| `pstore` | next boot's `/sys/fs/pstore` | `PANIC RECORD` = the kernel panicked |

Details of one boot (power rails, temperatures, kernel log in the last 30 s):

```bash
python3 .../report.py                 # previous boot (the one that died)
python3 .../report.py --boot 12       # a specific one
python3 .../report.py --tail-sec 120  # longer window
```

**Calibrate first.** With the monitor running, do one `sudo reboot` and one
deliberate power pull. Those two rows tell you what "software" and "power"
look like on this exact board (reset-source string, reset-level, gap). Compare
every unexpected row against them; that beats any legend.

## What is logged

One directory per Linux boot under `/var/log/corgi_orin_monitor/`:

| file | rate | content |
|---|---|---|
| `heartbeat.csv` | 1 Hz, fsync every row | wall clock, epoch, monotonic, uptime, NTP-synced flag, `START`/`STOP` events. The last row is the time of death. |
| `power.csv` | 20 Hz | every INA3221 rail (`VDD_GPU_SOC`, `VDD_CPU_CV`, `VIN_SYS_5V0`, `VDDQ_VDD2_1V8AO` on AGX Orin): mV, mA, mW; CPU cluster and GPU clocks |
| `system.csv` | 1 Hz | load, CPU %, available memory, swap, OOM kills, process count, `imu_node` running, `corgi_ros_bridge` running, number of `corgi_*` processes, `/dev/ttyTHS1` present, USB device count, every thermal zone, link state of every network interface, optional pings |
| `tegrastats.log` | 2 Hz | raw `tegrastats` with our timestamp prefixed (RAM, EMC, GR3D, temps, rail power) |
| `dmesg.log` | live | kernel log of this boot, whole buffer then follow |
| `journal.log` | live | `journalctl -b 0 -f`: service starts/failures, including the IMU unit |
| `boot_info.txt` | once at start | reset reason, pstore, clock sync, `kernel.panic`, watchdogs, previous boot's journal tail and errors, `last -x`, platform/JetPack, `nvpmodel -q`, `jetson_clocks --show`, rails found, storage, network, USB, PCI, **IMU autostart probe**, top processes |
| `prev_boot_summary.txt` | once at start | `report.py` output for the previous boot, so the post-mortem exists even if nobody runs the tool |
| `late_probe.txt` | once, 90 s after start | IMU autostart probe again, plus every journal line mentioning `imu` or `ttyTHS1` in this boot |
| `monitor.log` | | the daemon's own notes (streams started, fallbacks, budget) |

Disk: roughly 350 MB/day. Oldest boot directories are deleted beyond
`--max-total-gb` (default 8), never the last two.

### Limits to keep in mind

- The on-board INA3221 does **not** measure the DC input jack. `VIN_SYS_5V0`
  is the 5 V derived from it; a sag there is an indirect sign of an input dip.
  A direct view of the 12 V needs an external meter: the sbRIO's analog input
  through a divider, sampled at kHz, is the best instrument on the robot.
- Without an RTC battery the wall clock after a power loss is wrong until NTP
  syncs. `heartbeat.csv` carries a `ntp_synced` flag and a monotonic column;
  the report marks gaps computed from an unsynced clock with `~`.
- `ABRUPT` also results from `kill -9` of the monitor or a monitor crash. The
  `monitor.log` tail and systemd's `Restart=always` make that distinguishable.
- Tegra-specific paths (INA3221 hwmon, `tegrastats`, `/proc/device-tree/chosen/reset`)
  are discovered at runtime and degrade to "absent" notes elsewhere. This
  package was developed on an x86 laptop; the first run on the Orin should be
  checked with `report.py --boot cur` and a look at `boot_info.txt`.

## Experiments to run while it logs

Change one thing at a time; each needs no code.

1. `sudo nvpmodel -m <30 W mode>`. If the reboots stop, the supply cannot
   deliver the MAXN peak current.
2. Orin on a bench supply, motors on the battery as usual. Still reboots =
   noise or a reset line; stops = the robot's 12 V sags.
3. Standstill, motors **unpowered**, as the control group.

## IMU autostart

`corgi_imu/scripts/imu_node.sh` does `sudo chmod 777 /dev/ttyTHS1` before
`ros2 run`. Inside a systemd unit without passwordless sudo that line blocks
forever, and the IMU "does not start". `boot_info.txt` records which
mechanism launches it (systemd unit, crontab, rc.local, desktop autostart) and
`late_probe.txt` shows whether `imu_node` is alive 90 s after boot. A udev rule
makes the chmod unnecessary:

```
# /etc/udev/rules.d/99-corgi-imu.rules
KERNEL=="ttyTHS1", MODE="0666"
```
