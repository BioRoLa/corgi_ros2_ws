"""Stage 0: measure the wheel tread profile from the geometry Webots contacts.

Why this and not the parameter file. `legwheel/config/robot_params.py` describes
a toroidal tyre with a 15 mm "corner fillet", which reads as a flat tread with
rounded shoulders and would mean there is almost no crown for camber steering to
work with. That is a thesis-gating conclusion resting on a comment in a config
file. The sim has been contacting an `IndexedFaceSet` mesh this whole time, and
that mesh -- not the parameter file -- is what produced every result recorded so
far. So measure it.

What this reads: every `contactMaterial` in the proto, with the `boundingObject`
IndexedFaceSet that follows it. Those bounding objects ARE the collision
geometry; the visual meshes elsewhere in the file are not what ODE touches.

The profile is then recovered as a surface of revolution: the axial direction is
the one the body is thinnest along (the wheel is ~40 mm wide against a ~290 mm
diameter), the radius of each vertex is its distance from that axis, and the
tread profile is radius as a function of axial position.

    python3 measure_tread_profile.py [path/to/CorgiRobotABAD.proto]
"""
import re
import sys

import numpy as np

DEFAULT_PROTO = ("/home/alexc/corgi_ws/corgi_ros2_ws/src/corgi_sim/protos/"
                 "CorgiRobotABAD.proto")

CONTACT_RE = re.compile(r'contactMaterial\s+"([^"]+)"')


def parse_bounding_objects(path):
    """-> list of (contact_material, def_name, points Nx3).

    A hand-rolled scan rather than a VRML parser: the file is 100 MB and only
    the point block immediately after each contactMaterial is wanted.
    """
    out = []
    with open(path, "r", errors="replace") as f:
        text = f.read()

    for m in CONTACT_RE.finditer(text):
        mat = m.group(1)
        # The boundingObject follows its contactMaterial immediately. Searching
        # further than that walks into the next node's VISUAL IndexedFaceSet --
        # which is how a first pass reported nine bodies of 17202 points each,
        # all with identical extents, none of them collision geometry.
        tail = text[m.end():m.end() + 300]
        bo = tail.find("boundingObject")
        if bo < 0:
            continue
        dm = re.search(r"boundingObject\s+DEF\s+(\S+)", tail[bo:])
        if dm is None:
            continue
        name = dm.group(1)
        tail = text[m.end():m.end() + 4_000_000]
        bo = tail.find("boundingObject")
        p0 = tail.find("point [", bo)
        if p0 < 0:
            continue
        p1 = tail.find("]", p0)
        block = tail[p0 + len("point ["):p1]
        vals = np.fromstring(block.replace(",", " "), sep=" ")
        if vals.size < 9 or vals.size % 3:
            continue
        out.append((mat, name, vals.reshape(-1, 3)))
    return out


def fit_arc_centre(u, v, iters=6, keep=0.35):
    """-> (cu, cv) of the tread arc, by iterated Kasa fit on the outer envelope.

    Distance from the BODY'S OWN ORIGIN is not the tread radius. These
    collision bodies are arc SEGMENTS -- foot_tip spans a 0.199 m chord, which
    is the ±40 deg foot arc at r = 0.145 (2·0.145·sin40 = 0.186) -- and their
    local origin is nowhere near the arc centre. Measuring from it produced a
    profile that dipped in the middle, which reads as a groove down the centre
    of the tread and is purely an artefact of the wrong origin.

    So fit the centre. Kasa on everything is dragged inward by the body's
    internal structure, so refit repeatedly on the outermost fraction, which
    converges onto the tread surface itself.
    """
    cu, cv = u.mean(), v.mean()
    sel = np.ones(len(u), dtype=bool)
    for _ in range(iters):
        uu, vv = u[sel], v[sel]
        A = np.column_stack((2 * uu, 2 * vv, np.ones_like(uu)))
        b = uu * uu + vv * vv
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        cu, cv = float(sol[0]), float(sol[1])
        r = np.hypot(u - cu, v - cv)
        sel = r >= np.quantile(r, 1.0 - keep)
    return cu, cv


def profile(points):
    """-> (axial axis index, axial coords, radii from the fitted arc centre)."""
    extent = points.max(axis=0) - points.min(axis=0)
    axis = int(np.argmin(extent))
    radial = [i for i in range(3) if i != axis]
    w = points[:, axis]
    w = w - 0.5 * (w.max() + w.min())        # centre the width
    cu, cv = fit_arc_centre(points[:, radial[0]], points[:, radial[1]])
    r = np.hypot(points[:, radial[0]] - cu, points[:, radial[1]] - cv)
    return axis, w, r


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PROTO
    bodies = parse_bounding_objects(path)
    print(f"{len(bodies)} contact bodies with bounding geometry in "
          f"{path.split('/')[-1]}")
    print()
    print(f"  {'material':<12} {'DEF':<38} {'pts':>6} "
          f"{'x extent':>9} {'y extent':>9} {'z extent':>9}")
    for mat, name, pts in bodies:
        e = pts.max(axis=0) - pts.min(axis=0)
        print(f"  {mat:<12} {name[:38]:<38} {len(pts):6d} "
              f"{e[0]:9.4f} {e[1]:9.4f} {e[2]:9.4f}")

    print()
    print("The axial (width) extent is the one that matters: a collision body")
    print("with zero width is a knife edge, and a cambered knife edge has no")
    print("crown to traverse no matter what the tyre parameters say.")
    print()

    for mat, name, pts in bodies:
        e = pts.max(axis=0) - pts.min(axis=0)
        if e.min() < 1e-6:
            print(f"  {mat} / {name[:30]}: PLANAR -- zero extent along "
                  f"axis {int(np.argmin(e))}. No tread width at all.")
            continue
        axis, w, r = profile(pts)
        print(f"  {mat} / {name[:30]}: axial axis {axis}, "
              f"width {1000*(w.max()-w.min()):.1f} mm, "
              f"fitted tread radius {r.max():.4f} m")
        # Bin the vertices by axial position and report the max radius in each
        # bin: that upper envelope IS the tread cross-section.
        nb = 9
        edges = np.linspace(w.min(), w.max(), nb + 1)
        cells, ws = [], []
        for k in range(nb):
            sel = (w >= edges[k]) & (w <= edges[k + 1])
            cells.append(f"{1000*r[sel].max():6.1f}" if sel.any() else "   -- ")
            ws.append(f"{1000*0.5*(edges[k]+edges[k+1]):6.1f}")
        print(f"      w (mm): {' '.join(ws)}")
        print(f"      r (mm): {' '.join(cells)}")
        drop = 1000 * (r.max() - min(
            r[(w >= edges[k]) & (w <= edges[k + 1])].max()
            for k in (0, nb - 1)))
        print(f"      crown drop centre -> shoulder: {drop:.1f} mm "
              f"({'flat-ish' if drop < 2 else 'crowned'})")


if __name__ == "__main__":
    main()
