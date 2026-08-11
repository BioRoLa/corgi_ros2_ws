"""One line per dump: the authority sweep as a table, and the fitted slope.

The per-run detail is in check_turn.py; this exists so a sweep can be read as a
curve rather than as six separate reports, and so the slope -- which is the
actual deliverable of the authority experiment -- is computed once, from all the
points, instead of by eye off two of them.

The parasitic yaw is a constant offset on every run (it is present at
steer_offset = 0 and does not depend on the command), so it drops out of the
SLOPE and survives in the intercept. Both are reported: the slope is the
channel's authority, the intercept is the disturbance the loop has to cancel.

Usage:
    python3 summarise_sweep.py <tag>=<offset_deg>:<dump.npz> ...
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_yaw_phase import dedupe_time, yaw_from_quat  # noqa: E402
from ramp_segments import load as load_template  # noqa: E402

START = 3.5  # see check_turn.py --start: nothing flies before this
END = None


def _rp(q):
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    return np.rad2deg(roll), np.rad2deg(pitch)


def one(path):
    d = np.load(path, allow_pickle=True)
    ot, ov = dedupe_time(d["odom_t"], d["odom"])
    ct, cv, mt, mv = d["contact_t"], d["contact"], d["motor_t"], d["motor_deg"]
    m = (ot >= START) & ((ot <= END) if END is not None else True)
    t, seg = ot[m], ov[m]
    if len(t) < 100:
        return None
    x, y = seg[:, 0], seg[:, 1]
    yaw = np.rad2deg(yaw_from_quat(seg[:, 6:10]))
    span = t[-1] - t[0]
    v = float(np.hypot(np.diff(x), np.diff(y)).sum() / span)
    slope, _ = np.polyfit(t, yaw, 1)

    tt, _, _, _, in_stance = load_template(str(d["template"]))
    stride_s = float(tt[-1])
    design_air = 100.0 * float(1.0 - in_stance.mean())

    cm = (ct >= t[0]) & (ct <= t[-1])
    air = 100.0 * float((~cv[cm].any(axis=1)).mean())
    alld = 100.0 * float(cv[cm].all(axis=1).mean())
    mm = (mt >= t[0]) & (mt <= t[-1])
    th = float(mv[mm][:, :, 0].max())
    roll, pitch = _rp(seg[:, 6:10])
    att = float(max(np.abs(roll).max(), np.abs(pitch).max()))

    # theta needs BOTH bounds. Too low is a leg that never extended; too high is
    # the collapse signature section 19 records -- theta overshooting to 131.6
    # and 117.9 deg against a 100 deg command, with flight falling and speed
    # collapsing with it. A one-sided test passes a robot thrashing at 163 deg,
    # which is how the first pass of this sweep came back six-for-six PASS on
    # runs that had plainly come apart.
    return dict(v=v, dpsi=slope * stride_s, rate=slope, air=air, alld=alld,
                th=th, design=design_air, span=span, att=att,
                alive=(air > 25.0 and 97.0 < th < 110.0 and alld < 60.0
                       and att < 10.0))


def main():
    global START, END
    args = []
    for a in sys.argv[1:]:
        if a.startswith("--start="):
            START = float(a.split("=", 1)[1])
        elif a.startswith("--end="):
            END = float(a.split("=", 1)[1])
        else:
            args.append(a)

    print(f"window {START:.2f} .. "
          f"{'end' if END is None else format(END, '.2f')} s")
    rows = []
    for arg in args:
        tag, rest = arg.split("=", 1)
        off_deg, path = rest.split(":", 1)
        r = one(path)
        if r is None:
            print(f"{tag}: too few samples")
            continue
        r["tag"], r["off"] = tag, float(off_deg)
        rows.append(r)

    print(f"{'tag':>5} {'offset':>8} {'v':>7} {'dpsi/stride':>12} {'rate':>9} "
          f"{'flight':>8} {'design':>7} {'all-down':>9} {'theta':>8} "
          f"{'|att|':>6} {'gate':>6}")
    for r in sorted(rows, key=lambda r: r["off"]):
        print(f"{r['tag']:>5} {r['off']:+7.2f}d {r['v']:7.3f} "
              f"{r['dpsi']:+12.3f} {r['rate']:+8.2f}/s "
              f"{r['air']:7.1f}% {r['design']:6.1f}% {r['alld']:8.1f}% "
              f"{r['th']:8.2f} {r['att']:6.1f} "
              f"{'PASS' if r['alive'] else 'FAIL':>6}")

    good = [r for r in rows if r["alive"]]
    if len(good) >= 3:
        o = np.array([r["off"] for r in good])
        p = np.array([r["dpsi"] for r in good])
        a, b = np.polyfit(o, p, 1)
        resid = p - (a * o + b)
        print()
        print(f"  authority   {a:+.4f} deg/stride per degree of command")
        print(f"  intercept   {b:+.4f} deg/stride at zero command "
              f"(this is the parasitic yaw)")
        print(f"  RMS residual {np.sqrt(np.mean(resid**2)):.4f} deg/stride "
              f"over n={len(good)}")
        if b != 0:
            print(f"  trim to cancel the parasitic yaw: {-b/a:+.2f} deg")


if __name__ == "__main__":
    main()
