"""Recover the speed-ramp template's segment boundaries from the CSV itself.

The ramp concatenates six fixed points (hop, then v~ 0.50 -> 1.20). Nothing in
the CSV marks where one ends and the next begins, so we recover the boundaries
from the gait: each stride is one in_stance rising edge, and the segments differ
in duty factor and beta amplitude.

Deriving them from the file rather than hard-coding times means a regenerated
template stays measurable without editing this script.

Usage:  python3 ramp_segments.py <template.csv>
Import: from ramp_segments import segment_template
"""
import sys

import numpy as np

# Strides per segment, in order, as built by export_speed_ramp_csv.py.
# Used only to group strides; the boundary *times* come from the data.
STRIDES_PER_SEGMENT = [8, 6, 6, 6, 6, 6]
SEGMENT_NAMES = ["hop 30mm", "v~0.50", "v~0.70", "v~0.90", "v~1.05", "v~1.20"]


def load(path):
    raw = np.genfromtxt(path, delimiter=",", names=True)
    return (raw["t"], raw["theta"], raw["beta"], raw["gamma"],
            raw["in_stance"] > 0.5)


def stride_starts(in_stance):
    """Indices where stance begins. One per stride."""
    prev = np.roll(in_stance, 1)
    prev[0] = in_stance[0]
    edges = np.flatnonzero(in_stance & ~prev)
    # A template starting mid-stance has no rising edge at row 0, but row 0 is
    # still the start of the first stride.
    if in_stance[0] and (len(edges) == 0 or edges[0] != 0):
        edges = np.concatenate(([0], edges))
    return edges


def segment_template(path, strides_per_segment=None):
    """-> list of dicts, one per segment, with t_start/t_end and design duty."""
    t, theta, beta, gamma, in_stance = load(path)
    starts = stride_starts(in_stance)
    counts = list(strides_per_segment or STRIDES_PER_SEGMENT)

    if sum(counts) != len(starts):
        # Fall back to treating the whole template as a single segment rather
        # than silently mis-slicing it -- a wrong cut is worse than no cut.
        return [{
            "name": "whole template",
            "t_start": float(t[0]),
            "t_end": float(t[-1]),
            "strides": len(starts),
            "duty": float(in_stance.mean()),
            "theta_deg": (float(np.rad2deg(theta.min())),
                          float(np.rad2deg(theta.max()))),
            "beta_deg": (float(np.rad2deg(beta.min())),
                         float(np.rad2deg(beta.max()))),
            "mismatch": f"{len(starts)} strides found, {sum(counts)} expected",
        }]

    segments = []
    cut = 0
    for name, n in zip(SEGMENT_NAMES, counts):
        i0 = starts[cut]
        i1 = starts[cut + n] if cut + n < len(starts) else len(t)
        sl = slice(i0, i1)
        segments.append({
            "name": name,
            "t_start": float(t[i0]),
            "t_end": float(t[i1 - 1]),
            "strides": n,
            "duty": float(in_stance[sl].mean()),
            "theta_deg": (float(np.rad2deg(theta[sl].min())),
                          float(np.rad2deg(theta[sl].max()))),
            "beta_deg": (float(np.rad2deg(beta[sl].min())),
                         float(np.rad2deg(beta[sl].max()))),
            "mismatch": None,
        })
        cut += n
    return segments


def stride_boundaries(path, strides_per_segment=None):
    """-> list of (t_start, t_end, segment_name), one entry per stride.

    Per-stride resolution is what separates "the ramp is wrong" from "the gait
    had not converged yet": a rung that starts poor and climbs is a settling
    transient, a rung that is flat and poor is a broken rung.
    """
    t, _, _, _, in_stance = load(path)
    starts = stride_starts(in_stance)
    counts = list(strides_per_segment or STRIDES_PER_SEGMENT)

    names = []
    for name, n in zip(SEGMENT_NAMES, counts):
        names.extend([name] * n)
    if len(names) != len(starts):
        names = ["?"] * len(starts)

    out = []
    for k, i0 in enumerate(starts):
        i1 = starts[k + 1] if k + 1 < len(starts) else len(t)
        out.append((float(t[i0]), float(t[i1 - 1]), names[k]))
    return out


def main():
    path = sys.argv[1]
    segs = segment_template(path)
    print(f"{'segment':<12} {'t_start':>8} {'t_end':>8} {'strides':>8} "
          f"{'duty':>6} {'expect air':>11} {'theta range':>16} {'beta range':>16}")
    for s in segs:
        if s["mismatch"]:
            print(f"  WARNING: {s['mismatch']} -- not slicing into segments")
        print(f"{s['name']:<12} {s['t_start']:8.3f} {s['t_end']:8.3f} "
              f"{s['strides']:8d} {s['duty']:6.3f} {100*(1-s['duty']):10.1f}% "
              f"{s['theta_deg'][0]:7.2f}..{s['theta_deg'][1]:<7.2f} "
              f"{s['beta_deg'][0]:7.2f}..{s['beta_deg'][1]:<7.2f}")
    print(f"\ntotal template duration: {segs[-1]['t_end']:.3f} s")


if __name__ == "__main__":
    main()
