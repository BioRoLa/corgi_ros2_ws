#!/usr/bin/env python3
"""S326 read-outs P-A295-2/3: ABAD demand, clip fraction, and gamma delivery
per run, comparable across the 44.25 (S216) and 29.5 (S326) campaigns.

Usage: abad_clip_report.py <cam_dir> <ceiling_Nm> [t_start]

Per run and leg, over the steady band [t_start, end] (default 12 s, the gate's
settle), on motor==ABAD rows:
  p99.5 |tau_demand|          -- what the controller asked for
  clip% of stance             -- fraction of in_contact samples with
                                 |tau_applied| >= 0.999*ceiling
  gamma delivery              -- mean |gamma| in stance / commanded 0.24435 rad
"""
import csv
import glob
import math
import os
import sys

CMD_GAMMA = 0.24435  # 14 deg


def pctl(xs, q):
    if not xs:
        return float("nan")
    s = sorted(xs)
    i = min(len(s) - 1, int(math.ceil(q / 100.0 * len(s))) - 1)
    return s[max(i, 0)]


def one_run(path, ceiling, t_start):
    per = {}  # leg -> dict(lists)
    thr = 0.999 * ceiling
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            if row["motor"] != "ABAD":
                continue
            t = float(row["t"])
            if t < t_start:
                continue
            d = per.setdefault(row["leg"], {"dem": [], "clip": 0, "stance": 0,
                                            "gam": []})
            dem = abs(float(row["tau_demand"]))
            d["dem"].append(dem)
            if row["in_contact"] == "1":
                d["stance"] += 1
                if abs(float(row["tau_applied"])) >= thr:
                    d["clip"] += 1
                d["gam"].append(abs(float(row["gamma"])))
    return per


def main():
    cam = sys.argv[1]
    ceiling = float(sys.argv[2])
    t_start = float(sys.argv[3]) if len(sys.argv) > 3 else 12.0
    runs = sorted(glob.glob(os.path.join(cam, "run[0-9]*.csv")))
    runs = [p for p in runs if "uncertified" not in p]
    print(f"{cam}  ceiling {ceiling} N.m  band t>={t_start}s  ({len(runs)} runs)")
    print(f"{'run':4s} {'leg':3s} {'p99.5 dem':>9s} {'clip%stance':>11s} "
          f"{'gamma%':>7s}")
    agg = {}
    for p in runs:
        name = os.path.basename(p).replace(".csv", "")
        per = one_run(p, ceiling, t_start)
        for leg in sorted(per):
            d = per[leg]
            clip = 100.0 * d["clip"] / d["stance"] if d["stance"] else float("nan")
            gam = (100.0 * (sum(d["gam"]) / len(d["gam"])) / CMD_GAMMA
                   if d["gam"] else float("nan"))
            print(f"{name:4s} {leg:3s} {pctl(d['dem'], 99.5):9.1f} "
                  f"{clip:11.2f} {gam:7.1f}")
            a = agg.setdefault(leg, {"dem": [], "clip": [], "gam": []})
            a["dem"].append(pctl(d["dem"], 99.5))
            a["clip"].append(clip)
            a["gam"].append(gam)
    print("-- per-leg across runs (min..max) --")
    for leg in sorted(agg):
        a = agg[leg]
        print(f"  {leg}: p99.5 dem {min(a['dem']):.1f}..{max(a['dem']):.1f}  "
              f"clip {min(a['clip']):.2f}..{max(a['clip']):.2f}%  "
              f"gamma {min(a['gam']):.1f}..{max(a['gam']):.1f}%")


if __name__ == "__main__":
    main()
