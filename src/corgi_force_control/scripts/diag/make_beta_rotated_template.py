"""Generate a template whose BETA column is rotated against the rest.

Log S112. Measured on the config of record: the foot lands when the
template commands beta ~ +0.060 rad, against an intended -0.1614 -- the
touchdown falls roughly 0.22 rad (~74 rows) late in the beta cycle -- and
the leg additionally trails its command by ~0.14 rad. Net, the leg is at
beta ~ -0.079 when it lands and sweeps only +0.13 of the designed +0.322.

S111 showed the sign of the beta swept WHILE DOWN predicts the direction
of travel in every cell measured, so the quantity to fix is the MEASURED
beta at touchdown. Delaying the beta column by k rows makes the leg meet
the ground earlier in its stroke without touching theta (which sets WHEN
contact happens), gamma, or the in_stance labels (which set the gains,
and which S62/S108 already showed move only torque, not direction).

Rotating the WHOLE template would be a no-op -- a free-running periodic
player has no absolute phase. Rotating beta ALONE is not, because it
changes beta's phase relative to the theta/height dynamics that decide
touchdown.

    beta_new[i] = beta_old[(i - k) mod n]      k > 0 delays beta

theta, gamma and in_stance are untouched. The wrap row is regenerated so
the file keeps the loader's "last row is the wrap of the first" contract.

Usage:
    make_beta_rotated_template.py --rows 27 --out <path>
"""
import argparse
import csv
import os


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=os.path.expanduser(
        "~/corgi_ws/corgi_ros2_ws/src/corgi_force_control/config/"
        "gslip_pronk_template_v070.csv"))
    ap.add_argument("--rows", type=int, required=True,
                    help="rows to DELAY beta by (positive = later)")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    with open(args.src, newline="") as fh:
        rdr = csv.reader(fh)
        header = next(rdr)
        rows = [r for r in rdr if r]
    # The file carries n+1 rows: the last is the wrap of the first.
    body = rows[:-1]
    n = len(body)
    k = args.rows % n
    beta = [r[2] for r in body]
    rot = [beta[(i - k) % n] for i in range(n)]
    out_rows = []
    for i, r in enumerate(body):
        q = list(r)
        q[2] = rot[i]
        out_rows.append(q)
    # The wrap row: keep the ORIGINAL row's t/theta/gamma/in_stance and
    # replace only beta, so the file differs from the source in exactly
    # one column. (The loader pops this row before use, so it cannot
    # affect a run either way -- but a file that differs where it claims
    # not to is the kind of thing that costs an afternoon later.)
    wrap = list(rows[-1])
    wrap[2] = rot[0]
    out_rows.append(wrap)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(header)
        w.writerows(out_rows)

    # Report what the rotation did at the labelled stance boundaries, so
    # the intent is checkable without re-deriving it.
    onset = next(i for i, r in enumerate(body) if float(r[4]) > 0.5)
    last = max(i for i, r in enumerate(body) if float(r[4]) > 0.5)
    print(f"wrote {args.out}  ({n} rows + wrap, beta delayed {k} rows)")
    print(f"  beta at stance onset row {onset}: "
          f"{float(beta[onset]):+.4f} -> {float(rot[onset]):+.4f}")
    print(f"  beta at last stance row {last}: "
          f"{float(beta[last]):+.4f} -> {float(rot[last]):+.4f}")
    print(f"  theta/gamma/in_stance unchanged")


if __name__ == "__main__":
    main()
