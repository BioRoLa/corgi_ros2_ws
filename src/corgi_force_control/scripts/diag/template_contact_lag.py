"""How far out of phase is the template's gain schedule with actual contact?

The regime fault is confirmed (n = 4, ratio 0.66-0.78, below chance) and is NOT
primarily desync-driven -- halving desync moved the ratio only 0.61 -> 0.76.
Anti-correlation below chance requires a SYSTEMATIC offset. This measures it.

METHOD. The commanded regime is read from kp (bimodal: stance mode < 100,
flight mode >= 100; the schedule is global so any leg's kp carries it). Per-leg
measured contact is in the same rows, on the same clock. Cross-correlate the
two binary signals per leg over +/- ~1.5 stride periods and report the lag that
maximises agreement.

SIGN CONVENTION, stated once and used everywhere: POSITIVE lag means the
template runs AHEAD of reality -- the schedule enters stance before the foot
actually lands -- and delaying the template clock by that much would align them.

THE DECISION THIS FEEDS:

    all four legs share one lag  ->  the fix is a phase shift of the template
                                     clock: one line, no scheduler
    front and rear lags OPPOSE   ->  no global shift can fix both pairs; the
                                     per-leg scheduler is the honest path
    correlation low at ALL lags  ->  the schedule is not merely shifted but
                                     shaped wrong (period mismatch), and a
                                     shift fixes nothing

Also reported: the regime ratio each run WOULD have at its best global shift --
the direct payoff estimate for the one-line fix -- and at per-leg shifts, which
upper-bounds what any scheduler could recover at these dynamics.

Self-test: --selftest builds signals with known lags (+30 ms front, -30 ms
rear) and must recover them; given this session's record, no analyser output is
trusted before it has failed to fool itself.

Run:
    python3 template_contact_lag.py run1.csv [run2.csv ...]
    python3 template_contact_lag.py --selftest
"""
import csv
import sys

import numpy as np

LEGS = "ABCD"
KP_THRESH = 100.0
MAX_LAG_S = 0.40          # +/- window, ~1.5 stride periods
SETTLE_SKIP_S = 6.0       # skip after the gait window opens (standup remnants)


def load(path):
    per_t = {}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            if r["motor"] != "L_Motor":
                continue
            t = float(r["t"])
            d = per_t.setdefault(t, {"c": [None] * 4, "kp": None, "tf": 0.0})
            d["c"][LEGS.index(r["leg"])] = int(r["in_contact"])
            if d["kp"] is None:
                d["kp"] = float(r["kp"])
            d["tf"] = max(d["tf"], abs(float(r["t_ff"])))
    ts = sorted(k for k, v in per_t.items() if all(x is not None for x in v["c"]))
    t = np.array(ts)
    c = np.array([per_t[k]["c"] for k in ts], dtype=float)
    kp = np.array([per_t[k]["kp"] for k in ts])
    tf = np.array([per_t[k]["tf"] for k in ts])
    return t, c, kp, tf


def best_lag(cmd, con, max_lag):
    """(lag_samples, corr_at_best, corr_at_zero) for cmd shifted onto con.

    Positive lag: cmd leads -- cmd[i] matches con[i + lag].
    """
    a = cmd - cmd.mean()
    b = con - con.mean()
    denom = np.sqrt((a * a).sum() * (b * b).sum())
    if denom < 1e-12:
        return 0, float("nan"), float("nan")
    lags = np.arange(-max_lag, max_lag + 1)
    cc = np.array([
        (a[:len(a) - k] * b[k:]).sum() if k >= 0 else (a[-k:] * b[:k]).sum()
        for k in lags]) / denom
    i = int(np.argmax(cc))
    return int(lags[i]), float(cc[i]), float(cc[max_lag])


def ratio_at_shift(cmd, con4, lag):
    """Regime ratio with the command delayed by `lag` samples.

    PER-LEG definition -- pooled P(stance-cmd | that leg down) over its own
    baseline -- to be commensurable with gain_regime_crosstab.py. An earlier
    version used majority-down and printed 0.36-0.51 where the crosstab
    measured 0.76 on the same runs, which would have read as a contradiction.
    """
    if lag > 0:
        cmd_s, con_s = cmd[:len(cmd) - lag], con4[lag:]
    elif lag < 0:
        cmd_s, con_s = cmd[-lag:], con4[:lag]
    else:
        cmd_s, con_s = cmd, con4
    base = cmd_s.mean()
    if base < 1e-9:
        return float("nan")
    num = den = 0.0
    for i in range(con_s.shape[1]):
        down = con_s[:, i] > 0.5
        if down.sum() < 50:
            continue
        num += cmd_s[down].sum()
        den += down.sum()
    if den < 200:
        return float("nan")
    return float((num / den) / base)


def stride_period(con, dt):
    """Stride period from the contact autocorrelation's first off-zero peak.

    Search restricted to 0.12-0.40 s: the templates run 0.22-0.27 s, and a
    wider window let one ratty run pick a 590 ms harmonic, which then poisons
    the modular unwrap below.
    """
    a = con - con.mean()
    n = len(a)
    ac = np.correlate(a, a, "full")[n - 1:] / (a * a).sum()
    lo = max(3, int(0.12 / dt))
    hi = min(n - 1, int(0.40 / dt))
    if hi <= lo:
        return float("nan")
    return (lo + int(np.argmax(ac[lo:hi]))) * dt


def unwrap_ms(lag_ms, period_ms):
    """Map a lag onto (-period/2, +period/2].

    Cross-correlation of two PERIODIC signals is periodic in the lag, so raw
    peak positions are only defined modulo the stride. The first run of this
    script printed leg A at -160 ms beside B at +60 ms on a 220 ms stride --
    the same phase, aliased -- and the verdict logic read it as front/rear
    disagreement. Every lag comparison below happens after this unwrap.
    """
    if not np.isfinite(period_ms) or period_ms <= 0:
        return lag_ms
    x = lag_ms % period_ms
    if x > period_ms / 2:
        x -= period_ms
    return x


def analyse(t, c, kp, tf, label):
    nz = np.flatnonzero(tf > 1e-9)
    if not nz.size:
        print(f"{label:>10}   no gait window")
        return None
    sel = t >= t[nz[0]] + SETTLE_SKIP_S
    t, c, kp = t[sel], c[sel], kp[sel]
    if len(t) < 800:
        print(f"{label:>10}   too few samples ({len(t)})")
        return None
    dt = float(np.median(np.diff(t)))
    max_lag = int(round(MAX_LAG_S / dt))
    cmd = (kp < KP_THRESH).astype(float)
    period = stride_period(c[:, 0], dt)

    lags_ms, corrs, zeros = [], [], []
    for i in range(4):
        lag, cb, c0 = best_lag(cmd, c[:, i], max_lag)
        lags_ms.append(unwrap_ms(lag * dt * 1e3, period * 1e3))
        corrs.append(cb)
        zeros.append(c0)

    # Best single global shift: maximise mean per-leg correlation.
    a = cmd - cmd.mean()
    denoms, bs = [], []
    for i in range(4):
        b = c[:, i] - c[:, i].mean()
        bs.append(b)
        denoms.append(np.sqrt((a * a).sum() * (b * b).sum()))
    lags = np.arange(-max_lag, max_lag + 1)
    mean_cc = np.zeros(len(lags))
    for j, k in enumerate(lags):
        for b, dn in zip(bs, denoms):
            if dn < 1e-12:
                continue
            v = (a[:len(a) - k] * b[k:]).sum() if k >= 0 else (a[-k:] * b[:k]).sum()
            mean_cc[j] += v / dn / 4.0
    gl = int(lags[int(np.argmax(mean_cc))])
    gl_ms = unwrap_ms(gl * dt * 1e3, period * 1e3)

    # The payoff column is found by SCANNING the ratio itself over all lags,
    # not by evaluating it only at the correlation peak -- the two optima need
    # not coincide when contact durations differ per leg.
    r0 = ratio_at_shift(cmd, c, 0)
    scan = [(ratio_at_shift(cmd, c, int(k)), int(k)) for k in lags[::2]]
    scan = [(r, k) for r, k in scan if np.isfinite(r)]
    r_best, k_best = max(scan) if scan else (float("nan"), 0)
    k_best_ms = unwrap_ms(k_best * dt * 1e3, period * 1e3)

    print(f"{label:>10} {period*1e3:7.0f} "
          + " ".join(f"{v:+7.0f}" for v in lags_ms)
          + "  |" + " ".join(f"{v:5.2f}" for v in corrs)
          + f" | {gl_ms:+6.0f} {r0:6.2f} {r_best:6.2f} {k_best_ms:+7.0f}")
    return (np.array(lags_ms), np.array(corrs), np.array(zeros), gl_ms, r0,
            r_best, period * 1e3, k_best_ms)


def selftest():
    """Known lags must be recovered: front +30 ms, rear -30 ms, dt = 10 ms."""
    rng = np.random.default_rng(0)
    n, dt = 4000, 0.010
    period_s, duty = 0.24, 0.44
    t = np.arange(n) * dt
    phase = (t / period_s) % 1.0
    cmd = (phase < duty).astype(float)
    c = np.zeros((n, 4))
    for i, shift in enumerate([3, 3, -3, -3]):       # +30 ms front, -30 ms rear
        rolled = np.roll(cmd, shift)
        noise = rng.random(n) < 0.05
        c[:, i] = np.where(noise, 1 - rolled, rolled)
    kp = np.where(cmd > 0.5, 36.0, 289.0)
    tf = np.ones(n)
    print("\nSELFTEST  expect front ~ +30 ms, rear ~ -30 ms, corr ~ 0.9:")
    print(f"{'run':>10} {'stride':>7} {'lag A':>7} {'lag B':>7} {'lag C':>7} "
          f"{'lag D':>7}  |{'corr x4':>23} | {'glob':>6} {'r@0':>6} {'r@gl':>6}")
    res = analyse(t + 100.0, c, kp, np.concatenate([[0], tf[1:]]), "synth")
    ok = (res is not None and abs(res[0][0] - 30) <= 10 and abs(res[0][2] + 30) <= 10)
    print("SELFTEST", "PASS" if ok else "FAIL -- do not trust the real output")
    return ok


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--selftest":
        selftest()
        return
    paths = sys.argv[1:]
    if not paths:
        print("usage: template_contact_lag.py run1.csv [...] | --selftest")
        return

    print()
    print("=" * 96)
    print("TEMPLATE-vs-CONTACT LAG   (positive = template AHEAD of reality)")
    print("=" * 96)
    print(f"{'run':>10} {'stride':>7} {'lag A':>7} {'lag B':>7} {'lag C':>7} "
          f"{'lag D':>7}  |{'corr at best x4':>23} | {'glob':>6} {'r@0':>6} "
          f"{'r-best':>6} {'@ms':>7}")
    print(f"{'':>10} {'ms':>7} {'ms':>7} {'ms':>7} {'ms':>7} {'ms':>7}  |"
          f"{'':>23} | {'ms':>6} {'':>6} {'':>6} {'':>7}")

    out = []
    for p in paths:
        t, c, kp, tf = load(p)
        r = analyse(t, c, kp, tf, p.split("/")[-1])
        if r:
            out.append(r)

    if not out:
        return
    L = np.array([r[0] for r in out])          # runs x 4 legs, ms, UNWRAPPED
    C = np.array([r[1] for r in out])
    stride = float(np.mean([r[6] for r in out]))
    print()
    print("=" * 96)
    print(f"  unwrapped lag per leg (ms):   A {L[:,0].mean():+6.0f}   "
          f"B {L[:,1].mean():+6.0f}   C {L[:,2].mean():+6.0f}   D {L[:,3].mean():+6.0f}")
    front = L[:, :2].mean()
    rear = L[:, 2:].mean()
    print(f"  front pair {front:+6.0f} ms   rear pair {rear:+6.0f} ms   "
          f"stride {stride:.0f} ms")
    print(f"  best-corr per leg (mean): " + " ".join(f"{C[:,i].mean():.2f}" for i in range(4)))
    print(f"  regime ratio: measured {np.mean([r[4] for r in out]):.2f} -> "
          f"best achievable with ONE global shift {np.mean([r[5] for r in out]):.2f}"
          f" (at ~{np.mean([r[7] for r in out]):+.0f} ms)")
    print()
    spread = abs(front - rear)
    r_gain = np.mean([r[5] for r in out]) - np.mean([r[4] for r in out])
    if C.mean() < 0.35:
        print("  LOW CORRELATION AT EVERY LAG: the schedule is not merely shifted,")
        print("  it is SHAPED wrong against real contact (period/duty mismatch or")
        print("  contact too irregular). A phase shift fixes nothing here.")
    elif spread < 0.15 * stride and abs(L.mean()) > 10:
        print(f"  ONE SHARED LAG (~{L.mean():+.0f} ms, {100*abs(L.mean())/stride:.0f}% of a stride):")
        print("  the template clock is out of phase as a whole. The one-line fix --")
        print("  shifting the clock -- applies. Verify with the r-best column above.")
    elif spread >= 0.15 * stride:
        print(f"  FRONT AND REAR DISAGREE by {spread:.0f} ms after unwrapping "
              f"(~{100*spread/stride:.0f}% of a stride):")
        print("  no global shift aligns both pairs; per-leg (pair) phase is the")
        print("  honest fix. BUT check r-best first -- if a global shift already")
        print("  recovers most of the ratio, the disagreement is second-order.")
    else:
        print("  lags are small and roughly shared -- the offset alone does not")
        print("  explain the regime fault; if r-best is still well below 1 the")
        print("  fault is in the SHAPE of the schedule, not its phase.")
    print("=" * 96)
    print()


if __name__ == "__main__":
    main()
