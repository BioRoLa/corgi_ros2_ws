#!/usr/bin/env python3
"""The challenge's arithmetic, and the distribution-level version of it."""
import numpy as np

off = {"A": [0.816, 0.643, 0.697], "B": [0.638, 0.604, 0.665],
       "C": [0.263, 0.202, 0.236], "D": [0.284, 0.106, 0.208]}
on = {"A": [0.418, 0.444, 0.467], "B": [0.388, 0.317, 0.401],
      "C": [0.263, 0.235, 0.235], "D": [0.251, 0.241, 0.260]}
vf_off, vf_on = 0.242, 0.213

print("1. Per-leg cell medians and per-cell change")
for l in "ABCD":
    a, b = np.median(off[l]), np.median(on[l])
    print("   %s  off %.3f -> on %.3f   delta %+.3f  (%+.1f%%)"
          % (l, a, b, b - a, 100 * (b - a) / a))
print("   body v_fwd off %.3f -> on %.3f   delta %+.3f  (%+.1f%%)"
      % (vf_off, vf_on, vf_on - vf_off, 100 * (vf_on - vf_off) / vf_off))

print("\n2. If legs merely tracked the body slowdown (-12.0%), predicted on-cell:")
k = vf_on / vf_off
for l in "ABCD":
    a = np.median(off[l])
    print("   %s  predicted %.3f   observed %.3f   miss %+.3f"
          % (l, a * k, np.median(on[l]), np.median(on[l]) - a * k))

print("\n3. Demand-to-body ratio (>1 = leg outruns body and scrubs)")
for l in "ABCD":
    print("   %s  off %.2f   on %.2f"
          % (l, np.median(off[l]) / vf_off, np.median(on[l]) / vf_on))

print("\n4. Distribution level: ALL 12 leg-run values per cell")
for name, cell in (("off", off), ("on", on)):
    vals = sorted(v for l in cell for v in cell[l])
    print("   %-4s min %.3f  sorted %s" % (name, vals[0],
          " ".join("%.3f" % v for v in vals)))
lo = [v for l in off for v in off[l] if v < 0.235]
print("   off leg-runs below the on-cell floor 0.235: %d of 12  %s"
      % (len(lo), sorted(lo)))
hi = [v for l in on for v in on[l] if v < 0.235]
print("   on  leg-runs below 0.235: %d of 12" % len(hi))

print("\n5. Run-to-run sd of min vLeg (n=3)")
for name, mins in (("off", [0.263, 0.106, 0.208]), ("on", [0.251, 0.235, 0.235])):
    print("   %-4s median %.3f  sd %.4f  range %.3f-%.3f"
          % (name, np.median(mins), np.std(mins, ddof=1), min(mins), max(mins)))

print("\n6. The challenge's ceiling, both ways")
print("   using C's OFF median 0.236: ceiling rise %.3f, observed 0.027 -> %.0f%%"
      % (0.236 - 0.208, 100 * 0.027 / 0.028))
print("   using C's ON  median 0.235: ceiling rise %.3f, observed 0.027 -> %.0f%%"
      % (0.235 - 0.208, 100 * 0.027 / 0.027))
print("   same test applied to the OFF cell itself (min=D 0.208, 2nd=C 0.236):")
print("   off's min vLeg is likewise 100%% of the ceiling set by its own 2nd-lowest leg.")
