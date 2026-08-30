#!/usr/bin/env python3
"""Can body roll close S190's 7.25 mm residual? Decompose it and find out.
Log S197.

S190 left `r sin lambda` as the best-founded contact law (7.25 mm mean residual
against effects of 4-76 mm) with a named next step:

    "Something real is still missing -- most likely body roll (0.455 lambda,
     S189), which none of these predictions include. To close it properly: add
     body roll to the forward model and re-run -- no new simulator time."

Before building that model, test whether it CAN work. Body roll is a rotation
of the whole body about the fore-aft axis, so its effect on body-frame `by` is
COMMON-MODE in the world and therefore ANTISYMMETRIC between the two sides
(SY = {A:+1, B:-1, C:-1, D:+1}, the L/R pattern): whatever it adds to one side
it subtracts from the other. It structurally CANNOT produce a residual that is
the same sign on both sides.

So split the per-leg residual into

    symmetric      = (res_AD + res_BC) / 2     <- roll cannot touch this
    antisymmetric  = (res_AD - res_BC) / 2     <- roll's entire budget

and check two things: how much of the residual is antisymmetric at all, and
whether the antisymmetric part has roll's SHAPE (it must grow with lambda,
since rho = 0.455 lambda).

Numbers are taken from contact_forward_predict.py's own printed table so the
two agree by construction.

Usage:
    residual_decompose.py --selftest
    residual_decompose.py
"""
import argparse
import math
import sys

import numpy as np

# --- verbatim from contact_forward_predict.py -------------------------------
# lam -> leg -> (measured, r_sin prediction), mm, delta from lambda = 0
TABLE = {
    5:  {"A": (3.90, 13.46), "B": (11.70, 12.68),
         "C": (11.90, 12.86), "D": (3.80, 13.21)},
    10: {"A": (15.70, 25.72), "B": (25.30, 26.79),
         "C": (25.80, 27.39), "D": (15.50, 25.32)},
    15: {"A": (25.65, 37.29), "B": (37.80, 40.71),
         "C": (38.30, 41.19), "D": (25.50, 36.93)},
    20: {"A": (35.20, 48.43), "B": (49.70, 53.48),
         "C": (50.40, 54.02), "D": (34.80, 47.82)},
    30: {"A": (50.75, 65.36), "B": (75.30, 80.15),
         "C": (75.90, 80.88), "D": (50.50, 64.77)},
}
SIDE = {"A": "AD", "D": "AD", "B": "BC", "C": "BC"}
RHO_PER_LAM = 0.455          # S189: measured body roll, deg per deg of lambda
BODY_H = 0.25                # nominal body height, m -- the roll lever arm


def decompose():
    out = []
    for lam in sorted(TABLE):
        res = {leg: pred - meas for leg, (meas, pred) in TABLE[lam].items()}
        ad = 0.5 * (res["A"] + res["D"])
        bc = 0.5 * (res["B"] + res["C"])
        out.append({
            "lam": lam,
            "ad": ad, "bc": bc,
            "sym": 0.5 * (ad + bc),
            "anti": 0.5 * (ad - bc),
            "spread_ad": abs(res["A"] - res["D"]),
            "spread_bc": abs(res["B"] - res["C"]),
        })
    return out


def roll_budget(lam, h=BODY_H):
    """Antisymmetric shift, mm, that a body roll of 0.455*lam would produce."""
    return 1000.0 * h * math.sin(math.radians(RHO_PER_LAM * lam))


def selftest():
    ok = True

    def chk(name, good):
        nonlocal ok
        ok = ok and good
        print("  %s %s" % ("ok " if good else "FAIL", name))

    # the decomposition must be lossless
    d = decompose()[0]
    chk("sym + anti reconstructs the A/D residual",
        abs((d["sym"] + d["anti"]) - d["ad"]) < 1e-12)
    chk("sym - anti reconstructs the B/C residual",
        abs((d["sym"] - d["anti"]) - d["bc"]) < 1e-12)
    # reproduce S190's headline: mean |residual| over the 20 leg-points
    allres = [abs(p - m) for lam in TABLE
              for (m, p) in TABLE[lam].values()]
    chk("mean |residual| = 7.25 mm, as S190 reports (got %.2f)"
        % np.mean(allres), abs(np.mean(allres) - 7.25) < 0.01)
    # a pure roll must be exactly antisymmetric and leave no symmetric part
    fake = {}
    for leg, (m, p) in TABLE[30].items():
        s = +1.0 if SIDE[leg] == "AD" else -1.0
        fake[leg] = (m, m + s * 6.0)
    r = {leg: p - m for leg, (m, p) in fake.items()}
    sym = 0.25 * (r["A"] + r["D"] + r["B"] + r["C"])
    chk("a synthetic pure-roll residual has ZERO symmetric part (got %.3f)"
        % sym, abs(sym) < 1e-12)
    return ok


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        print("residual_decompose.py selftest\n")
        good = selftest()
        print("\n  SELFTEST %s" % ("PASS" if good else "FAIL"))
        return 0 if good else 1

    rows = decompose()
    print("S190's `r sin lambda` residual (prediction - measurement), mm.")
    print("Roll is a body rotation, so it can only ever fill the ANTI column.\n")
    print("  %5s %9s %9s %9s %9s %11s" %
          ("lam", "A/D", "B/C", "SYM", "ANTI", "anti share"))
    for r in rows:
        share = abs(r["anti"]) / (abs(r["sym"]) + abs(r["anti"]))
        print("  %5d %9.2f %9.2f %9.2f %9.2f %10.0f%%" %
              (r["lam"], r["ad"], r["bc"], r["sym"], r["anti"], 100 * share))

    print("\n-- within-side spread, as a noise floor --")
    for r in rows:
        print("     lam %2d   |A-D| %.2f   |B-C| %.2f"
              % (r["lam"], r["spread_ad"], r["spread_bc"]))

    print("\n-- does the ANTI part have roll's SHAPE? --")
    print("   roll rho = %.3f * lambda (S189); a rotation about a lever arm h"
          % RHO_PER_LAM)
    print("   shifts body-frame by h*sin(rho), which GROWS with lambda.\n")
    print("  %5s %9s %13s %13s" %
          ("lam", "ANTI", "roll @ h=0.25", "h needed (mm)"))
    for r in rows:
        need = 1e-3 * r["anti"] / math.sin(math.radians(RHO_PER_LAM * r["lam"]))
        print("  %5d %9.2f %13.1f %13.0f"
              % (r["lam"], r["anti"], roll_budget(r["lam"]), 1000 * need))

    anti = np.array([r["anti"] for r in rows])
    sym = np.array([r["sym"] for r in rows])
    print("\n  ANTI across lambda 5->30: %s" % np.array2string(anti, precision=2))
    print("    range %.2f mm on a mean of %.2f -- %s"
          % (anti.max() - anti.min(), anti.mean(),
             "FLAT" if (anti.max() - anti.min()) < 0.25 * anti.mean()
             else "growing"))
    print("  SYM  across lambda 5->30: %s" % np.array2string(sym, precision=2))
    print("    range %.2f mm on a mean of %.2f -- %s"
          % (sym.max() - sym.min(), sym.mean(),
             "FLAT" if (sym.max() - sym.min()) < 0.25 * sym.mean()
             else "growing"))

    # What DOES the symmetric part look like? Roll is out; the two candidates
    # left are a constant baseline offset and an r_eff that is too large.
    # Fit sym = a + b*sin(lambda): `a` is the lambda = 0 baseline's flat band,
    # `b` is the millimetres of r_eff being overpredicted (the model's term is
    # exactly r_eff*sin lambda, so an error in r_eff appears with that shape).
    sl = np.array([math.sin(math.radians(r["lam"])) for r in rows])
    b, a = np.polyfit(sl, sym, 1)
    resid = sym - (a + b * sl)
    print()
    print("-- so what IS the symmetric part? --")
    print("   fit  sym = a + b*sin(lambda)   (b is mm of excess r_eff)")
    print("     a = %+.2f mm   (constant -- the lambda=0 baseline band)" % a)
    print("     b = %+.2f mm   (shape of the r_eff term)" % b)
    print("     max |misfit| %.2f mm over 5 points" % np.max(np.abs(resid)))
    print("   => r_eff %.4f m fits better than the %.3f m in use (%+.1f%%)"
          % (0.145 - b / 1000.0, 0.145, -100.0 * b / 145.0))
    print("   S21 already flagged that 0.145 is the WHEELED-mode radius and")
    print("   the legged value is a separate question. This is an estimate of")
    print("   it, and a loaded tread being SMALLER is the expected direction.")

    print("\n-- verdict --")
    if abs(sym).mean() > abs(anti).mean():
        print("  The residual is majority SYMMETRIC (%.2f vs %.2f mm)."
              % (abs(sym).mean(), abs(anti).mean()))
        print("  Body roll cannot produce a symmetric residual at all, so")
        print("  adding it to the forward model CANNOT close S190. The")
        print("  proposed next step does not work and should not be built.")
    else:
        print("  The residual is majority antisymmetric -- roll is still live.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
