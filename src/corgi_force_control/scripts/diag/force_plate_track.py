#!/usr/bin/env python3
"""Score a force rock: commanded vs estimated vs force-plate force, per leg.

Pairs with exp_h24 rock_mode:=diag_fz | pitch_fz | roll_fz | fore_fx | lateral_fy,
which stands still and adds a known sinusoid to one force axis. This is the 3D
successor of Eason's 2025-03 force-plate test (imp_cmd_pub.cpp, deleted): Fy
rocked +/-20 N diagonally on four plates, one leg per plate.

From a rosbag2 with /impedance/command and /force/state it finds the rock axis
and frequency from the command, then for each leg fits
    y(t) = c0 + a*sin(wt) + b*cos(wt)
to the command, the estimate and the plate inside the steady rock window:
  gain    amplitude(signal) / amplitude(command)        1.00 = tracks
  lag     phase lag behind the command, deg and ms     + = signal is late
  offset  mean(signal) - mean(command) over the window  N
  rms     rms(signal - command) over the window         N
plus the mean of every axis in the 2 s hold before the rock (the weight split).

Plates, in the command's convention (force the leg puts ON the ground, body
frame, x forward, z up, so a standing leg is fz < 0):
  sim       /sensor/force_plate_{1..4} in the bag (Corgi_ABAD_force_plate.wbt,
            corgi_sim's ForcePlateDriver). Same sim clock, so lag is exact.
  hardware  --plate-c3d trial.c3d, the Vicon Nexus capture of the 4 Bertecs
            (ANALOG Force.Fx<n>/Fy/Fz, plate frames from FORCE_PLATFORM:CORNERS).
  other     --plate-csv with --plate-col 'F{axis}_{i}' columns and a time column
            or --plate-rate; assumed already in the lab frame, z up.
Lab frame -> body frame uses --robot-yaw-deg (robot heading in the lab, 0 =
facing +x). Each leg is assigned the plate whose centre is nearest its hip, for
a robot centred on the plate grid; override with --plate-map a=1,b=4,c=3,d=2.
If the plates read the opposite way to the command in the hold (action vs
reaction), the whole plate vector is flipped and the report says so.

Hardware plates have no ROS clock. With --plate-offset S (bag time of plate
t = 0, e.g. from a tap at a known time) lag and rms are reported. Without it
the rock ENVELOPE is cross-correlated with the command's: gain and offset stay
valid (the sine fit has a free phase), lag and rms are not reported, because
aligning on the signal would absorb the lag being measured.

Usage:
  force_plate_track.py <bag_dir>                         # sim, plates in the bag
  force_plate_track.py <bag_dir> --plate-c3d s1_rock.c3d [--robot-yaw-deg 180]
  force_plate_track.py <bag_dir> --plate-csv daq.csv --plate-time-col time
  force_plate_track.py --self-test
Needs ROS sourced for bags (rosbag2_py + corgi_msgs) and the c3d package for
--plate-c3d. --self-test needs neither.
"""
import argparse
import csv
import json
import os
import sys

import numpy as np

LEGS = "abcd"
AXES = ("fx", "fy", "fz")
HIP_XY = {"a": (0.255, 0.12), "b": (0.255, -0.12), "c": (-0.255, -0.12), "d": (-0.255, 0.12)}
# lab/sim plate centres, m: 1 at +x+y, 2 at -x+y, 3 at -x-y, 4 at +x-y (c3d CORNERS, 2026-09)
DEFAULT_CENTRES = {1: (0.3, 0.2), 2: (-0.3, 0.2), 3: (-0.3, -0.2), 4: (0.3, -0.2)}


# -- inputs ------------------------------------------------------------------------
def read_bag(path):
    try:
        import rosbag2_py
        import yaml
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        sys.exit("the rosbag2 python API is not importable (%s).\n"
                 "  source /opt/ros/humble/setup.bash && source install/setup.bash" % exc)
    storage = "sqlite3"
    meta = os.path.join(path, "metadata.yaml")
    if os.path.exists(meta):
        with open(meta) as fh:
            storage = yaml.safe_load(fh)["rosbag2_bagfile_information"].get("storage_identifier", storage)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = {"/impedance/command": "cmd", "/force/state": "est"}
    missing = [t for t in want if t not in types]
    if missing:
        sys.exit("bag has no %s -- record impedance/command and force/state" % ", ".join(missing))
    plate_topics = {"/sensor/force_plate_%d" % n: n for n in range(1, 5) if "/sensor/force_plate_%d" % n in types}
    rows = {"cmd": [], "est": []}
    prow = {n: [] for n in plate_topics.values()}
    while reader.has_next():
        topic, data, tns = reader.read_next()
        if topic not in want and topic not in plate_topics:
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        st = msg.header.stamp
        t = st.sec + st.nanosec * 1e-9 if (st.sec or st.nanosec) else tns * 1e-9
        if topic in plate_topics:
            f = msg.wrench.force
            prow[plate_topics[topic]].append((t, f.x, f.y, f.z))
            continue
        rows[want[topic]].append([t] + [getattr(getattr(msg, "module_" + l), ax) for l in LEGS for ax in AXES])
    out = {}
    for k, r in rows.items():
        a = np.asarray(r)
        if len(a) < 100:
            sys.exit("only %d %s messages in the bag" % (len(a), k))
        a = a[np.argsort(a[:, 0])]
        out[k] = {"t": a[:, 0], **{l + ax: a[:, 1 + 3 * i + j] for i, l in enumerate(LEGS) for j, ax in enumerate(AXES)}}
    if len(prow) == 4 and all(len(v) > 100 for v in prow.values()):
        t = np.asarray([r[0] for r in prow[1]])
        plate = {"t": t}
        for n, r in prow.items():
            a = np.asarray(r)
            for j, ax in enumerate(AXES):
                plate["%d%s" % (n, ax)] = np.interp(t, a[:, 0], a[:, 1 + j])
        out["plate"] = plate
    elif prow:
        print("note: bag has %d of 4 plate topics -- ignoring them" % len(prow))
    return out


def read_c3d(path):
    """Plate forces in the lab frame (z up) and plate centres, from a Nexus c3d."""
    try:
        import c3d
    except ImportError:
        sys.exit("--plate-c3d needs the c3d package (pip install c3d)")
    with open(path, "rb") as fh:
        r = c3d.Reader(fh)
        if not r.analog_used:
            sys.exit("%s has no analog channels -- were the plates live in Nexus?" % path)
        labels = [s.strip() for s in r.get("ANALOG").get("LABELS").string_array]
        corners = np.asarray(r.get("FORCE_PLATFORM").get("CORNERS").float_array) / 1000.0  # (plate, corner, xyz) m
        an = np.concatenate([a for _, _, a in r.read_frames()], axis=1)
        rate = r.analog_rate
    plate = {"t": np.arange(an.shape[1]) / rate}
    centres = {}
    for n in range(1, corners.shape[0] + 1):
        c = corners[n - 1]
        # C3D: corners 1..4 lie in the plate's +x+y, -x+y, -x-y, +x-y quadrants
        ex = (c[0] + c[3]) / 2 - (c[1] + c[2]) / 2
        ey = (c[0] + c[1]) / 2 - (c[2] + c[3]) / 2
        ex, ey = ex / np.linalg.norm(ex), ey / np.linalg.norm(ey)
        R = np.column_stack([ex, ey, np.cross(ex, ey)])  # local -> lab
        try:
            local = np.vstack([an[labels.index("Force.F%s%d" % (ax, n))] for ax in "xyz"])
        except ValueError:
            sys.exit("no Force.F?%d channels in %s (have: %s)" % (n, path, ", ".join(labels)))
        lab = R @ local
        for j, ax in enumerate(AXES):
            plate["%d%s" % (n, ax)] = lab[j]
        centres[n] = tuple(c[:, :2].mean(axis=0))
    loaded = max(np.ptp(plate["%dfz" % n]) for n in centres)
    if loaded < 5.0:
        print("warning: every plate stays within %.1f N -- nothing stood on them in this capture" % loaded)
    return plate, centres


def read_csv(args):
    with open(args.plate_csv, newline="") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        sys.exit("%s is empty" % args.plate_csv)
    if args.plate_time_col:
        t = np.array([float(r[args.plate_time_col]) for r in rows])
    elif args.plate_rate:
        t = np.arange(len(rows)) / args.plate_rate
    else:
        sys.exit("--plate-csv needs --plate-time-col or --plate-rate")
    plate = {"t": t}
    for n in range(1, 5):
        for ax in AXES:
            col = args.plate_col.format(axis=ax[1], AXIS=ax[1].upper(), i=n)
            if col not in rows[0]:
                sys.exit("no column %s in %s (have: %s)" % (col, args.plate_csv, ", ".join(rows[0])))
            plate["%d%s" % (n, ax)] = np.array([float(r[col]) for r in rows])
    return plate


def plate_map(centres, yaw, override=None):
    if override:
        m = {kv.split("=")[0].strip().lower(): int(kv.split("=")[1]) for kv in override.split(",")}
        if sorted(m) != list(LEGS) or len(set(m.values())) != 4:
            sys.exit("--plate-map needs each of a, b, c, d on a different plate")
        return m
    c, s = np.cos(yaw), np.sin(yaw)
    m = {}
    for l, (x, y) in HIP_XY.items():
        hx, hy = c * x - s * y, s * x + c * y
        m[l] = min(centres, key=lambda n: (centres[n][0] - hx) ** 2 + (centres[n][1] - hy) ** 2)
    if len(set(m.values())) != 4:
        sys.exit("two legs land on the same plate (%s) -- pass --plate-map" % m)
    return m


def plates_to_legs(plate, pmap, yaw):
    """Lab-frame plates -> per-leg body-frame forces."""
    c, s = np.cos(yaw), np.sin(yaw)
    out = {"t": plate["t"]}
    for l, n in pmap.items():
        fx, fy = plate["%dfx" % n], plate["%dfy" % n]
        out[l + "fx"] = c * fx + s * fy
        out[l + "fy"] = -s * fx + c * fy
        out[l + "fz"] = plate["%dfz" % n]
    return out


# -- analysis ---------------------------------------------------------------------
def resample(series, t_new):
    return {k: (t_new if k == "t" else np.interp(t_new, series["t"], v)) for k, v in series.items()}


def find_rock(cmd, axis=None):
    """Rock axis, angular frequency, steady window and pre-rock hold, from the command alone."""
    t = cmd["t"]
    dt = np.median(np.diff(t))
    if axis is None:
        axis = max(AXES, key=lambda ax: sum(np.ptp(cmd[l + ax] - np.median(cmd[l + ax])) for l in LEGS))
    # the rock pattern sums to zero, so the per-leg deviation from its own median is the rock
    dev = np.vstack([cmd[l + axis] - np.median(cmd[l + axis]) for l in LEGS])
    lead = dev[np.argmax(np.ptp(dev, axis=1))]
    spec = np.abs(np.fft.rfft(lead - lead.mean()))
    freqs = np.fft.rfftfreq(len(lead), dt)
    spec[freqs < 0.05] = 0
    f = freqs[np.argmax(spec)]
    period = max(1, int(round(1.0 / f / dt)))
    env = np.sqrt(np.convolve(lead ** 2, np.ones(period) / period, mode="same")) * np.sqrt(2)
    idx = np.flatnonzero(env > 0.95 * env.max())
    # trim half a period off each end so the window holds only full-amplitude cycles
    lo, hi = idx[0] + period // 2, idx[-1] - period // 2
    if hi - lo < period:
        sys.exit("no steady rock window found (rock < 2 periods?)")
    first = np.flatnonzero(env > 0.05 * env.max())[0]
    hold = (t[first] - 2.0, t[first] - 0.1) if t[first] - 2.0 >= t[0] else None
    return axis, 2 * np.pi * f, (t[lo], t[hi]), hold


def sine_fit(t, y, w):
    A = np.column_stack([np.ones_like(t), np.sin(w * t), np.cos(w * t)])
    (c0, a, b), *_ = np.linalg.lstsq(A, y, rcond=None)
    return c0, np.hypot(a, b), np.arctan2(b, a)


def score(t, y, ycmd, w, phase_ok=True):
    c0c, ampc, phc = sine_fit(t, ycmd, w)
    c0, amp, ph = sine_fit(t, y, w)
    r = {"gain": amp / ampc if ampc > 1e-9 else float("nan"), "amp": amp, "amp_cmd": ampc,
         "offset": c0 - c0c}
    if phase_ok:
        lag = (phc - ph + np.pi) % (2 * np.pi) - np.pi  # + = y late
        r["lag_deg"] = float(np.degrees(lag))
        r["lag_ms"] = float(lag / w * 1000)
        r["rms"] = float(np.sqrt(np.mean((y - ycmd) ** 2)))
    return {k: float(v) for k, v in r.items()}


def align_envelope(plate, cmd, axis, w, t_grid):
    """Plate time offset that best overlays the plates' rock envelope on the command's."""
    dt = t_grid[1] - t_grid[0]
    period = max(1, int(round(2 * np.pi / w / dt)))

    def env(sig):
        d = sig - np.median(sig)
        return np.sqrt(np.convolve(d ** 2, np.ones(period) / period, mode="same"))
    ec = sum(env(cmd[l + axis]) for l in LEGS)
    tp = np.arange(plate["t"][0], plate["t"][-1], dt)
    ep = sum(env(np.interp(tp, plate["t"], plate[l + axis])) for l in LEGS)
    ec, ep = ec - ec.mean(), ep - ep.mean()
    xc = np.correlate(ep, ec, mode="full")
    k = np.argmax(xc) - (len(ec) - 1)
    return t_grid[0] - (tp[0] + k * dt)


def analyse(cmd, est, plate=None, plate_offset=None, axis=None, rate=1000.0):
    """plate: per-leg body-frame forces (plates_to_legs), or None."""
    t0, t1 = max(cmd["t"][0], est["t"][0]), min(cmd["t"][-1], est["t"][-1])
    grid = np.arange(t0, t1, 1.0 / rate)
    cmd, est = resample(cmd, grid), resample(est, grid)
    axis, w, (wlo, whi), hold = find_rock(cmd, axis)
    win = (grid >= wlo) & (grid <= whi)
    res = {"axis": axis, "hz": w / (2 * np.pi), "window_s": [float(wlo - t0), float(whi - t0)], "legs": {}}

    pl, phase_ok = None, False
    if plate is not None:
        if plate_offset is None:
            plate_offset = align_envelope(plate, cmd, axis, w, grid)
            res["plate_alignment"] = "envelope (lag/rms not reported)"
        else:
            phase_ok = True
            res["plate_alignment"] = "given"
        res["plate_offset_s"] = float(plate_offset)
        shifted = dict(plate, t=plate["t"] + plate_offset)
        if shifted["t"][0] > wlo or shifted["t"][-1] < whi:
            print("warning: plate data does not cover the rock window after alignment")
        pl = resample(shifted, grid)
        # action vs reaction: in the hold (or the window) a standing leg pushes down
        ref = ((grid >= hold[0]) & (grid <= hold[1])) if hold else win
        if np.mean([pl[l + "fz"][ref].mean() for l in LEGS]) * np.mean([cmd[l + "fz"][ref].mean() for l in LEGS]) < 0:
            pl = {k: (v if k == "t" else -v) for k, v in pl.items()}
            res["plate_flipped"] = True

    for l in LEGS:
        k = l + axis
        leg = {"est": score(grid[win], est[k][win], cmd[k][win], w)}
        if pl is not None:
            leg["plate"] = score(grid[win], pl[k][win], cmd[k][win], w, phase_ok)
            leg["plate_vs_est"] = score(grid[win], est[k][win], pl[k][win], w, phase_ok)
        if hold:
            h = (grid >= hold[0]) & (grid <= hold[1])
            leg["hold"] = {ax: {"cmd": float(cmd[l + ax][h].mean()), "est": float(est[l + ax][h].mean()),
                                **({"plate": float(pl[l + ax][h].mean())} if pl is not None else {})}
                           for ax in AXES}
        res["legs"][l] = leg
    return res


def report(res):
    print("rock axis %s at %.3f Hz, steady window %.1f..%.1f s after bag start"
          % (res["axis"], res["hz"], *res["window_s"]))
    if "plate_offset_s" in res:
        print("plate offset %+.3f s (%s)%s" % (res["plate_offset_s"], res["plate_alignment"],
              "; plate sign flipped to the command's convention" if res.get("plate_flipped") else ""))
    if "plate_map" in res:
        print("plates: " + ", ".join("%s on %d" % (l.upper(), n) for l, n in res["plate_map"].items()))
    print("\nleg  signal vs reference   amp_ref    gain   lag deg   lag ms   offset N    rms N")
    names = {"est": "est   vs cmd  ", "plate": "plate vs cmd  ", "plate_vs_est": "est   vs plate"}
    for l, leg in res["legs"].items():
        for src in ("est", "plate", "plate_vs_est"):
            if src not in leg:
                continue
            s = leg[src]
            lag = ("%7.1f  %7.1f" % (s["lag_deg"], s["lag_ms"])) if "lag_deg" in s else "      -        -"
            rms = "%7.2f" % s["rms"] if "rms" in s else "      -"
            flag = "" if 0.85 <= s["gain"] <= 1.15 else "   <-- gain off by > 15 %"
            print(" %s   %s   %7.2f   %5.2f   %s   %+8.2f  %s%s"
                  % (l.upper(), names[src], s["amp_cmd"], s["gain"], lag, s["offset"], rms, flag))
    if any("hold" in leg for leg in res["legs"].values()):
        print("\nstatic hold before the rock, mean N:  cmd / est%s" %
              (" / plate" if "plate" in next(iter(res["legs"].values())) else ""))
        for l, leg in res["legs"].items():
            parts = []
            for ax in AXES:
                h = leg["hold"][ax]
                parts.append("%s %+7.1f / %+7.1f%s" % (ax, h["cmd"], h["est"],
                                                        " / %+7.1f" % h["plate"] if "plate" in h else ""))
            print(" %s   %s" % (l.upper(), "   ".join(parts)))


# -- self-test ----------------------------------------------------------------------
def self_test():
    """Synthetic diag_fz rock with known gain / lag / offset, through the lab-frame plate path."""
    rate, hz, amp = 1000.0, 1.0, 10.0
    t = np.arange(0, 20, 1 / rate) + 100.0  # bag clock does not start at 0
    tr = t - t[0] - 2.5
    env = np.clip(tr, 0, 1) * np.clip(15.0 - tr, 0, 1)
    u = amp * env * np.sin(2 * np.pi * hz * np.maximum(tr, 0))
    pat = {"a": 1, "b": -1, "c": 1, "d": -1}
    base = {"a": -60.0, "b": -60.0, "c": -53.0, "d": -53.0}
    truth = {"a": (0.9, 30.0, 2.0), "b": (1.0, 10.0, 0.0), "c": (1.1, 50.0, -3.0), "d": (0.8, 0.0, 1.0)}
    fx_body = {"a": 4.0, "b": -2.0, "c": 3.0, "d": -1.0}  # static fore-aft load, to check the yaw rotation
    rng = np.random.default_rng(0)
    cmd, est = {"t": t}, {"t": t}
    for l in LEGS:
        g, lag_ms, off = truth[l]
        ul = amp * np.interp(tr - lag_ms / 1000, tr, env) * np.sin(2 * np.pi * hz * (np.maximum(tr, 0) - lag_ms / 1000))
        for ax in AXES:
            cmd[l + ax] = np.zeros_like(t)
            est[l + ax] = rng.normal(0, 0.3, len(t))
        cmd[l + "fz"] = base[l] + pat[l] * u
        est[l + "fz"] = base[l] + off + pat[l] * g * ul + rng.normal(0, 0.5, len(t))
    ok = True
    for yaw_deg, given in ((0.0, None), (180.0, t[0] - 0.004)):
        yaw = np.radians(yaw_deg)
        pmap = plate_map(DEFAULT_CENTRES, yaw)
        # lab-frame plates, reaction convention (+z under load) -- analyse() must flip them
        plate = {"t": t - t[0] + 0.004}
        for l, n in pmap.items():
            lag_ms = truth[l][1]
            ul = amp * np.interp(tr - lag_ms / 1000, tr, env) * np.sin(2 * np.pi * hz * (np.maximum(tr, 0) - lag_ms / 1000))
            plate["%dfx" % n] = -(np.cos(yaw) * fx_body[l]) + rng.normal(0, 0.3, len(t))
            plate["%dfy" % n] = -(np.sin(yaw) * fx_body[l]) + rng.normal(0, 0.3, len(t))
            plate["%dfz" % n] = -(base[l] + pat[l] * ul) + rng.normal(0, 0.5, len(t))
        res = analyse(cmd, est, plates_to_legs(plate, pmap, yaw), given)
        res["plate_map"] = pmap
        ok &= res.get("plate_flipped", False)
        ok &= pmap == ({"a": 1, "b": 4, "c": 3, "d": 2} if yaw_deg == 0 else {"a": 3, "b": 2, "c": 1, "d": 4})
        for l in LEGS:
            g, lag_ms, off = truth[l]
            e = res["legs"][l]["est"]
            ok &= abs(e["gain"] - g) < 0.02 and abs(e["lag_ms"] - lag_ms) < 3 and abs(e["offset"] - off) < 0.3
            pf = res["legs"][l]["plate"]
            ok &= abs(pf["gain"] - 1.0) < 0.02
            ok &= abs(res["legs"][l]["hold"]["fx"]["plate"] - fx_body[l]) < 0.1
            if given is not None:
                ok &= abs(pf["lag_ms"] - lag_ms) < 3
        ok &= res["axis"] == "fz" and abs(res["hz"] - hz) < 0.02
        if given is None:
            ok &= abs(res["plate_offset_s"] - (t[0] - 0.004)) < 0.1
        report(res)
        print()
    print("SELF-TEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", nargs="?")
    ap.add_argument("--axis", choices=AXES, help="rock axis (default: the one the command moves most)")
    ap.add_argument("--plate-c3d", help="Vicon Nexus capture with the Bertec analog channels")
    ap.add_argument("--plate-csv")
    ap.add_argument("--plate-rate", type=float, help="CSV samples per second, if no time column")
    ap.add_argument("--plate-time-col")
    ap.add_argument("--plate-col", default="F{axis}_{i}", help="CSV column per plate/axis: {axis} x|y|z, {AXIS} X|Y|Z, {i} plate")
    ap.add_argument("--plate-map", help="leg=plate, e.g. a=1,b=4,c=3,d=2 (default: nearest plate to each hip)")
    ap.add_argument("--robot-yaw-deg", type=float, default=0.0, help="robot heading in the plate/lab frame")
    ap.add_argument("--plate-offset", type=float, help="bag time of plate t = 0, s (hardware; see above)")
    ap.add_argument("--json", help="also write the result here")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        return self_test()
    if not args.bag:
        ap.error("bag is required")
    if args.plate_c3d and args.plate_csv:
        ap.error("--plate-c3d or --plate-csv, not both")

    bag = read_bag(args.bag)
    yaw = np.radians(args.robot_yaw_deg)
    plate, centres, offset = None, DEFAULT_CENTRES, args.plate_offset
    if args.plate_c3d:
        plate, centres = read_c3d(args.plate_c3d)
    elif args.plate_csv:
        plate = read_csv(args)
    elif "plate" in bag:
        plate = bag["plate"]
        offset = 0.0 if offset is None else offset  # sim: plates share the bag's clock
    pmap = plate_map(centres, yaw, args.plate_map) if plate is not None else None
    res = analyse(bag["cmd"], bag["est"], plates_to_legs(plate, pmap, yaw) if plate is not None else None,
                  offset, args.axis)
    if pmap:
        res["plate_map"] = pmap
    report(res)
    if args.json:
        with open(args.json, "w") as fh:
            json.dump(res, fh, indent=2)
        print("saved %s" % args.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
