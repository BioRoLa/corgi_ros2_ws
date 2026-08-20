"""Body pitch and roll, and whether pitch AT TOUCHDOWN wrecks the stance.

Alex, 2026-08-20: "when it lands in a very pitched position, it messes up
the gait significantly."

That is a per-episode claim, so it gets a per-episode test rather than a
run average. For every debounced stance episode:

    pitch_TD    body pitch at the touchdown edge (rad, + = nose up)
    sweep       beta swept by that leg while down -- S111 established
                that the SIGN of this predicts direction of travel and
                its magnitude ranks with speed, so it is the natural
                per-stance quality measure

then Spearman rho(|pitch_TD|, sweep). Alex's hypothesis predicts a
clearly NEGATIVE rho: land pitched, get a worse stroke.

Attitude comes from the odom quaternion (Supervisor ground truth) with
the same formulas the controller's update_attitude uses, so the numbers
are comparable to k_roll/k_yaw's inputs. Odom and torque CSVs share the
sim clock; pitch is interpolated onto the touchdown times.

Read-only.
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_menger import debounce, DEBOUNCE, load_odom_csv  # noqa: E402

LEGS = "ABCD"
TAIL_S = 20.0


class Unfit(Exception):
    pass


def rpy(q):
    """-> (roll, pitch, yaw), matching gslip_pronk.cpp update_attitude."""
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def spearman(a, b):
    ra = np.argsort(np.argsort(a)).astype(float)
    rb = np.argsort(np.argsort(b)).astype(float)
    ra -= ra.mean(); rb -= rb.mean()
    den = np.sqrt((ra ** 2).sum() * (rb ** 2).sum())
    return float((ra * rb).sum() / den) if den > 0 else 0.0


def load_contact_beta(path):
    per = {}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            if r["motor"] != "L_Motor":
                continue
            try:
                per.setdefault(r["leg"], []).append(
                    (float(r["t"]), int(r["in_contact"]), float(r["beta"])))
            except (ValueError, KeyError):
                continue
    out = {}
    for leg, rows in per.items():
        rows.sort()
        a = np.array(rows)
        if len(a) > 500:
            out[leg] = (a[:, 0], a[:, 1].astype(bool), a[:, 2])
    if not out:
        raise Unfit("no usable legs")
    return out


def analyse(torque_csv, odom_csv, tail_s=TAIL_S):
    ot, oxy, oq = load_odom_csv(odom_csv)
    if len(ot) < 200:
        raise Unfit(f"only {len(ot)} odom samples")
    roll, pitch, _ = rpy(oq)
    legs = load_contact_beta(torque_csv)

    tmax = min(ot.max(), max(v[0].max() for v in legs.values()))
    om = ot >= (tmax - tail_s)
    if int(om.sum()) < 100:
        raise Unfit("odom does not cover the tail")
    r_t, r_roll, r_pitch = ot[om], roll[om], pitch[om]

    pit_td, sweeps, rolls_td = [], [], []
    for leg, (t, c, b) in legs.items():
        m = t >= (tmax - tail_s)
        t, c, b = t[m], c[m], b[m]
        if len(t) < 200:
            continue
        d = debounce(c, DEBOUNCE)
        chg = np.diff(d.astype(int))
        rise = np.flatnonzero(chg > 0) + 1
        fall = np.flatnonzero(chg < 0) + 1
        for r0 in rise:
            nxt = fall[fall > r0]
            if not len(nxt) or nxt[0] - r0 < 3:
                continue
            f0 = nxt[0]
            if t[r0] < r_t[0] or t[r0] > r_t[-1]:
                continue
            pit_td.append(float(np.interp(t[r0], r_t, r_pitch)))
            rolls_td.append(float(np.interp(t[r0], r_t, r_roll)))
            sweeps.append(float(b[f0] - b[r0]))
    if len(pit_td) < 20:
        raise Unfit(f"only {len(pit_td)} joinable stance episodes")
    p = np.array(pit_td)
    s = np.array(sweeps)
    return {
        "pitch_rms": float(np.sqrt(np.mean(r_pitch ** 2))),
        "pitch_p2p": float(r_pitch.max() - r_pitch.min()),
        "roll_rms": float(np.sqrt(np.mean(r_roll ** 2))),
        "roll_p2p": float(r_roll.max() - r_roll.min()),
        "abs_pitch_TD": float(np.median(np.abs(p))),
        "pitch_TD_p90": float(np.percentile(np.abs(p), 90)),
        "abs_roll_TD": float(np.median(np.abs(rolls_td))),
        "rho_pitch_sweep": spearman(np.abs(p), s),
        "n": len(p),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", action="append", required=True)
    ap.add_argument("--label", action="append", default=[])
    args = ap.parse_args()
    d2r = 180.0 / np.pi
    print("body attitude, last 20 s. pitch + = nose up. angles in DEGREES.")
    print("rho = Spearman(|pitch at touchdown|, beta swept that stance).")
    print("Alex's hypothesis predicts rho clearly NEGATIVE.\n")
    print(f"{'cell':26} {'pitchRMS':>9} {'pitchP2P':>9} {'rollRMS':>8} "
          f"{'|pitTD|':>8} {'pitTD90':>8} {'rho':>7} {'n':>5}")
    for i, d in enumerate(args.dir):
        lab = args.label[i] if i < len(args.label) else os.path.basename(
            d.rstrip("/"))
        vals = []
        for tq in sorted(glob.glob(os.path.join(os.path.expanduser(d),
                                                "run[0-9].csv"))):
            od = os.path.join(os.path.dirname(tq),
                              "odom_" + os.path.basename(tq))
            if not os.path.exists(od) or os.path.getsize(od) < 10000:
                continue
            try:
                vals.append(analyse(tq, od))
            except (Unfit, SystemExit) as e:
                print(f"  {os.path.basename(tq)}: REFUSED -- {e}")
        if not vals:
            continue
        f = lambda k: float(np.median([v[k] for v in vals]))
        print(f"{lab:26} {d2r*f('pitch_rms'):9.2f} {d2r*f('pitch_p2p'):9.2f} "
              f"{d2r*f('roll_rms'):8.2f} {d2r*f('abs_pitch_TD'):8.2f} "
              f"{d2r*f('pitch_TD_p90'):8.2f} {f('rho_pitch_sweep'):+7.2f} "
              f"{sum(v['n'] for v in vals):5d}")


if __name__ == "__main__":
    main()
