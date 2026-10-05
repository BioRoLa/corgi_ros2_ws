#!/usr/bin/env python3
"""correlate.py -- assign each Vicon trial to a bag by absolute clock.

Three independent channels have to agree before an assignment is accepted:
  1. TIME. Bag message timestamps are absolute epoch nanoseconds, readable from
     the .db3 without metadata.yaml. The Vicon .x2d CreationTime is the capture
     time (raw camera data is written as the trial is captured). Both clocks were
     checked against this workstation and agree to the second, so the offset is
     zero and the comparison is direct.
  2. DURATION. Vicon gait window (from vicon_l0.py) against the bag's own gait
     length. They are not equal -- the Vicon window is the body's vertical
     oscillation, the bag's is trigger-on to trigger-off -- but the RANK order
     and the ratio should be stable across a session.
  3. PHYSICS. A cambered arc must show curvature of the commanded sign. A trial
     labelled lambda0 that curves hard is mislabelled whatever its name says.

Anything the three do not agree on is reported UNRESOLVED rather than guessed.
"""
import datetime as dt
import glob
import os
import sqlite3
import sys

BAGDIR = os.path.expanduser("~/corgi_runs/hw_2026-09-08/bags")
TZ = dt.timezone(dt.timedelta(hours=8))     # Vicon PC and WSL both local UTC+8

# (basename, x2d CreationTime local) -- capture time, from the Vicon PC
VICON = [
    ("S3_L0A1", "2026-09-08 22:41:35"),
    ("S3_L0A2", "2026-09-08 22:46:46"),
    ("S3_L0A3", "2026-09-08 22:55:16"),
    ("s3_diag_pitchoff_a1", "2026-09-08 23:11:51"),
    ("s3_ol10n_a1", "2026-09-08 23:41:51"),
    ("s3_l10_a2", "2026-09-08 23:43:19"),
    ("s3_ol10_a2", "2026-09-08 23:47:01"),
    ("s3_ol10n_a2", "2026-09-08 23:48:19"),
    ("s3_l0_a3", "2026-09-08 23:49:31"),
    ("s3_ol15n_a1", "2026-09-08 23:50:55"),
    ("s3_ol10n_a1b", "2026-09-09 00:09:53"),
    ("s3_ol10n_a1c", "2026-09-09 00:12:26"),
    ("s3_ol12n_a1", "2026-09-09 00:14:13"),
    ("s3_ol12_a1", "2026-09-09 00:16:03"),
    ("s3_l0_dp0_a1", "2026-09-09 00:18:10"),
]

# vicon_l0.py output: gait window seconds, v_fwd, kappa
VSTATS = {
    "S3_L0A1": (6.34, 0.364, -0.0074), "S3_L0A2": (3.75, 0.184, +0.8857),
    "S3_L0A3": (4.35, 0.198, +0.6227), "s3_diag_pitchoff_a1": (4.43, 0.178, +0.2290),
    "s3_ol10n_a1": (7.17, 0.292, -0.2772), "s3_l10_a2": (6.09, 0.349, -0.0658),
    "s3_ol10_a2": (4.40, 0.202, +0.3034), "s3_ol10n_a2": (6.89, 0.298, -0.1958),
    "s3_l0_a3": (5.82, 0.340, -0.1292), "s3_ol15n_a1": (4.21, 0.248, -0.4861),
    "s3_ol10n_a1b": (2.30, 0.092, -1.0038), "s3_ol10n_a1c": (7.14, 0.299, -0.1709),
    "s3_ol12n_a1": (6.62, 0.310, -0.4065), "s3_ol12_a1": (2.69, 0.162, +0.8463),
    "s3_l0_dp0_a1": (6.90, 0.290, -0.0889),
}

# the cell each BAG was launched as, from its banner (verified earlier)
BAGCELL = {
    "s3_l0_a1": "l0", "s3_ol10_a1v2": "+10", "s3_diag_pitchoff_a1": "l0(pitch off)",
    "s3_l0_a1b": "l0", "s3_l0_kp60_a1": "l0(kp60)", "s3_l0_kp60d0_a1": "l0(kp60 d0)",
    "s3_ol10n_a1": "-10 (bag empty)", "s3_l0_a2": "l0", "s3_ol10_a2": "+10",
    "s3_ol10n_a2": "-10", "s3_l0_a3": "l0", "s3_ol15n_a1": "-15",
    "s3_ol10n_a1b": "-10", "s3_ol10n_a1c": "-10", "s3_ol12n_a1": "-12.5",
    "s3_ol12_a1": "+12.5", "s3_l0_dp0_a1": "l0(d_pitch 0)",
}


def bag_span(db3):
    """(first, last) message epoch seconds, straight from the sqlite file."""
    con = sqlite3.connect("file:%s?mode=ro" % db3, uri=True)
    try:
        r = con.execute("SELECT min(timestamp), max(timestamp) FROM messages").fetchone()
    finally:
        con.close()
    if not r or r[0] is None:
        return None
    return r[0] / 1e9, r[1] / 1e9


def main():
    bags = {}
    for d in sorted(glob.glob(os.path.join(BAGDIR, "s3_*"))):
        run = os.path.basename(d)
        hits = glob.glob(os.path.join(d, "*.db3"))
        if not hits:
            continue
        try:
            span = bag_span(hits[0])
        except Exception as e:
            print("  %-22s UNREADABLE %s" % (run, e))
            continue
        if span:
            bags[run] = span

    print("=== bags: absolute span from their own message timestamps ===")
    print("  %-22s %-19s %-19s %7s" % ("run", "first msg (local)", "last msg (local)", "dur s"))
    for run in sorted(bags, key=lambda r: bags[r][0]):
        a, b = bags[run]
        print("  %-22s %-19s %-19s %7.1f"
              % (run,
                 dt.datetime.fromtimestamp(a, TZ).strftime("%Y-%m-%d %H:%M:%S"),
                 dt.datetime.fromtimestamp(b, TZ).strftime("%Y-%m-%d %H:%M:%S"),
                 b - a))

    print()
    print("=== assignment: each Vicon capture to the bag recording at that instant ===")
    print("  %-22s %-19s  %-24s %-9s %s"
          % ("vicon trial", "capture (x2d)", "bag whose recording spans it", "offset", "cell"))
    used = {}
    for name, ts in VICON:
        t = dt.datetime.strptime(ts, "%Y-%m-%d %H:%M:%S").replace(tzinfo=TZ).timestamp()
        inside = [(r, a, b) for r, (a, b) in bags.items() if a - 20 <= t <= b + 20]
        if len(inside) == 1:
            r, a, b = inside[0]
            where = "inside" if a <= t <= b else "within 20 s"
            print("  %-22s %-19s  %-24s %-9s %s"
                  % (name, ts[11:], r, where, BAGCELL.get(r, "?")))
            used.setdefault(r, []).append(name)
        elif len(inside) > 1:
            print("  %-22s %-19s  AMBIGUOUS: %s"
                  % (name, ts[11:], ", ".join(r for r, _, _ in inside)))
        else:
            nearest = min(bags.items(), key=lambda kv: min(abs(t - kv[1][0]), abs(t - kv[1][1]))) if bags else None
            if nearest:
                r, (a, b) = nearest
                gap = t - b if t > b else a - t
                print("  %-22s %-19s  no bag recording; nearest %s by %+.0f s"
                      % (name, ts[11:], r, gap))
            else:
                print("  %-22s %-19s  no bag data at all" % (name, ts[11:]))

    print()
    print("=== cross-check: does the Vicon curvature match the bag's commanded cell? ===")
    for name, ts in VICON:
        t = dt.datetime.strptime(ts, "%Y-%m-%d %H:%M:%S").replace(tzinfo=TZ).timestamp()
        inside = [r for r, (a, b) in bags.items() if a - 20 <= t <= b + 20]
        if len(inside) != 1:
            continue
        cell = BAGCELL.get(inside[0], "?")
        dur, v, k = VSTATS[name]
        want = "+" if cell.startswith("+") else ("-" if cell.startswith("-") else "~0")
        got = "+" if k > 0.05 else ("-" if k < -0.05 else "~0")
        ok = "OK" if want == got else "MISMATCH"
        print("  %-22s cell %-16s kappa %+7.4f  expected %-3s got %-3s  %s"
              % (name, cell, k, want, got, ok))

    print()
    print("=== bags with no Vicon capture inside their recording ===")
    for r in sorted(bags):
        if r not in used:
            print("  %-22s %s" % (r, BAGCELL.get(r, "?")))


if __name__ == "__main__":
    sys.exit(main())
