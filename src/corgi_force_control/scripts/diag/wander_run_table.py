#!/usr/bin/env python3
"""One row per run: engagement, gait mode, curvature, speed, straightness.

Offline only. No simulator time.

Why this exists. The wander question -- does the plant's UNCOMMANDED path
curvature track its flight fraction? -- needs air% and |kappa| for the SAME
run, and the three tools that own those numbers each report per cell. This
joins them, keyed on run, so the correlation can be taken at all.

It computes nothing itself. It shells out to check_menger.py, gait_mode.py and
speed_from_odom.py and parses their output, deliberately: those are the tools
whose numbers are in the log, and a second implementation of the same
statistic is how a project ends up with two numbers for one quantity.

REFUSAL, not best effort. Every run must yield every field and must
self-certify its configuration from its own ctl_run<N>.log. A run that does not
is dropped with a printed reason and counted -- because "we analysed 16 runs"
and "16 runs existed" are different claims, and that difference is exactly what
went wrong with the handover's "38 uncambered-anchor runs" (38 is the total
across three trees; 16 are uncambered anchors).

Usage:
    python3 wander_run_table.py --cell ~/corgi_runs/camber_snr/k7150/lam0_default \
                                --cell ~/corgi_runs/camber_snr/k12000/lam0_default \
                                --csv /tmp/wander_runs.csv
    python3 wander_run_table.py --selftest
"""

from __future__ import annotations

import argparse
import csv
import os
import re
import subprocess
import sys
from dataclasses import asdict, dataclass, fields

DIAG = os.path.dirname(os.path.abspath(__file__))
START_S = 12.0          # steady band, same as the runbook's scoring order
CHORD_M = 1.0

RE_KAPPA = re.compile(r"median kappa\s+([-+]?\d+\.\d+)\s+1/m")
RE_GATE = re.compile(r"GATE .*?->\s*(PASS|FAIL)")
RE_TRIPLE_ROW = re.compile(r"^\s*\d+\s+\d+\.\d+\s+[-+]\d", re.M)
RE_AIR = re.compile(r"air_idx\s+([\d.]+)\s+air\s+([\d.]+)%")
RE_SPEED = re.compile(
    r"v_fwd\s+([-+]\d+\.\d+).*?v_path\s+([\d.]+)\s+straight\s+([\d.]+)"
    r"\s+yaw\s+([-+]?\s*\d+\.\d+)")
RE_FLIGHT_GAINS = re.compile(r"FLIGHT GAINS set: k_flight=([\d.]+) b_flight=([\d.]+)")
RE_RUN_N = re.compile(r"(?<!odom_)run(\d+)\.csv")
RE_ODOM_N = re.compile(r"odom_run(\d+)\.csv")


@dataclass
class Row:
    tree: str
    cell: str
    run: int
    k_flight: float
    b_flight: float
    yaw_hold: int
    cambered: int
    steered: int
    gait_gate: str
    n_triples: int
    kappa_med: float
    air_index: float
    air_pct: float
    v_fwd: float
    v_path: float
    straightness: float
    yaw_deg_s: float


def _run(cmd):
    out = subprocess.run(cmd, capture_output=True, text=True)
    return out.stdout + out.stderr


def _engagement(log_path):
    """Read the run's own configuration off its controller log. None if absent.

    Absence is the certification for camber and steering: an uncambered anchor
    is a run whose own log never announced camber.
    """
    if not os.path.isfile(log_path):
        return None
    txt = open(log_path, errors="replace").read()
    m = RE_FLIGHT_GAINS.search(txt)
    if not m:
        return None
    steered = ("STEER CONFIRMED" in txt) or ("TURN CONFIRMED" in txt)
    if not steered:
        for tr in re.findall(r"turn_rate[=: ]+([\d.]+)", txt):
            if float(tr) != 0.0:
                steered = True
                break
    return {
        "k_flight": float(m.group(1)),
        "b_flight": float(m.group(2)),
        "yaw_hold": int("GENTLE YAW CLAMP set" in txt),
        "cambered": int("ACKER CAMBER set" in txt),
        "steered": int(steered),
    }


def collect(cell, start, chord):
    cell = os.path.expanduser(cell.rstrip("/"))
    tree = os.path.basename(os.path.dirname(os.path.dirname(cell)))
    point = os.path.basename(os.path.dirname(cell))
    label = point + "/" + os.path.basename(cell)

    gm = _run([sys.executable, DIAG + "/gait_mode.py", "--dir", cell])
    sp = _run([sys.executable, DIAG + "/speed_from_odom.py", "--dir", cell])

    gm_by_run = {}
    for ln in gm.splitlines():
        m, n = RE_AIR.search(ln), RE_RUN_N.search(ln)
        if m and n:
            gm_by_run[int(n.group(1))] = m.groups()
    sp_by_run = {}
    for ln in sp.splitlines():
        m, n = RE_SPEED.search(ln), RE_ODOM_N.search(ln)
        if m and n:
            sp_by_run[int(n.group(1))] = m.groups()

    rows, dropped = [], []
    runs = sorted(int(RE_ODOM_N.search(f).group(1)) for f in os.listdir(cell)
                  if re.fullmatch(r"odom_run\d+\.csv", f))
    for n in runs:
        odom = "%s/odom_run%d.csv" % (cell, n)
        tq = "%s/run%d.csv" % (cell, n)
        eng = _engagement("%s/ctl_run%d.log" % (cell, n))
        if eng is None:
            dropped.append("%s run%d: no engagement lines -- cannot self-certify"
                           % (label, n))
            continue
        if not os.path.isfile(tq):
            dropped.append("%s run%d: no torque csv" % (label, n))
            continue
        cm = _run([sys.executable, DIAG + "/check_menger.py", "--odom-csv", odom,
                   "--torque-csv", tq, "--start", str(start), "--chord", str(chord)])
        mk, mg = RE_KAPPA.search(cm), RE_GATE.search(cm)
        if not mk or not mg:
            dropped.append("%s run%d: check_menger gave no median kappa or no gate"
                           % (label, n))
            continue
        if n not in gm_by_run or n not in sp_by_run:
            dropped.append("%s run%d: missing gait_mode or speed_from_odom row"
                           % (label, n))
            continue
        ai, ap = gm_by_run[n]
        vf, vp, st, yw = sp_by_run[n]
        rows.append(Row(tree=tree, cell=label, run=n,
                        k_flight=eng["k_flight"], b_flight=eng["b_flight"],
                        yaw_hold=eng["yaw_hold"], cambered=eng["cambered"],
                        steered=eng["steered"], gait_gate=mg.group(1),
                        n_triples=len(RE_TRIPLE_ROW.findall(cm)),
                        kappa_med=float(mk.group(1)),
                        air_index=float(ai), air_pct=float(ap),
                        v_fwd=float(vf), v_path=float(vp), straightness=float(st),
                        yaw_deg_s=float(yw.replace(" ", ""))))
    return rows, dropped


def selftest():
    """Parsers must bite on real output shapes, and refuse malformed ones."""
    ok = True
    cases = [
        (RE_KAPPA, "    median kappa -0.37111 1/m   IQR [-0.74426, +0.09917]",
         "-0.37111"),
        (RE_GATE, "  GATE  flight  36.2%   all-down  19.0%   theta max 144.27 deg "
                  "(gate 100..165)   -> PASS", "PASS"),
        (RE_AIR, "k7150/lam0_default run1.csv air_idx  0.36   air 26.2%  all4 22.3%",
         "0.36"),
        (RE_SPEED, "lam0_default/odom_run1.csv     v_fwd +0.307 (naive +0.300)  "
                   "v_path 0.309  straight 0.994  yaw  -0.43 deg/s", "+0.307"),
        (RE_FLIGHT_GAINS, "FLIGHT GAINS set: k_flight=7150.0 b_flight=115.8", "7150.0"),
    ]
    for rx, line, want in cases:
        m = rx.search(line)
        got = m.group(1) if m else None
        status = "PASS" if got == want else "FAIL"
        ok = ok and status == "PASS"
        print("  %-4s %-42s -> %r (want %r)" % (status, rx.pattern[:40], got, want))

    # The run-number patterns must not confuse run1.csv with odom_run1.csv.
    checks = [
        (RE_RUN_N.search("odom_run3.csv") is None, "RE_RUN_N ignores odom_run3.csv"),
        (RE_RUN_N.search("run3.csv").group(1) == "3", "RE_RUN_N reads run3.csv"),
        (RE_ODOM_N.search("odom_run3.csv").group(1) == "3", "RE_ODOM_N reads odom"),
        (RE_KAPPA.search("median kappa nan 1/m") is None, "refuses kappa nan"),
        (RE_AIR.search("air_idx -- air --%") is None, "refuses air --"),
        (RE_SPEED.search("v_fwd -- v_path -- straight --") is None, "refuses speed --"),
    ]
    for good, what in checks:
        status = "PASS" if good else "FAIL"
        ok = ok and good
        print("  %-4s %s" % (status, what))

    print("SELFTEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cell", action="append", default=[],
                    help="a capture cell directory; repeatable")
    ap.add_argument("--start", type=float, default=START_S)
    ap.add_argument("--chord", type=float, default=CHORD_M)
    ap.add_argument("--csv", help="write the joined table here")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()

    if a.selftest:
        return selftest()
    if not a.cell:
        ap.error("give at least one --cell")

    rows, dropped = [], []
    for c in a.cell:
        r, d = collect(c, a.start, a.chord)
        rows += r
        dropped += d

    print("%-11s%-26s%4s%9s%6s%6s%6s%6s%11s%9s%7s%8s%9s%8s" % (
        "tree", "cell", "run", "k_flight", "camb", "steer", "gate", "trip",
        "kappa_med", "air_idx", "air%", "v_fwd", "straight", "yaw"))
    for r in rows:
        print("%-11s%-26s%4d%9.0f%6d%6d%6s%6d%+11.4f%9.2f%7.1f%+8.3f%9.3f%+8.2f" % (
            r.tree, r.cell, r.run, r.k_flight, r.cambered, r.steered, r.gait_gate,
            r.n_triples, r.kappa_med, r.air_index, r.air_pct, r.v_fwd,
            r.straightness, r.yaw_deg_s))

    print("\nadmitted %d runs, dropped %d" % (len(rows), len(dropped)))
    for d in dropped:
        print("  DROPPED  " + d)

    if a.csv:
        with open(a.csv, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=[f.name for f in fields(Row)])
            w.writeheader()
            for r in rows:
                w.writerow(asdict(r))
        print("\nwrote " + a.csv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
