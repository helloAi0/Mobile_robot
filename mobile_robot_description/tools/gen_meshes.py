#!/usr/bin/env python3
"""Generates the VISUAL meshes (meshes/*.stl, metres, ASCII STL) from the same dimensions as the mass table.

   python3 tools/gen_meshes.py

Visual meshes are detailed (bolt heads, tyre tread lugs, cable conduit, heat-sink fins, sensor lenses); collision
shapes in the URDF are a handful of boxes/cylinders. To use your own CAD: export each part from Blender/CAD as .dae
(with its PBR textures next to it) or .stl, drop it into meshes/ with the same file name, done. Nothing else changes
as long as the part origin is the link frame documented in docs/MODEL.md.
Only numpy is needed. Primitives overlap (it is a render mesh, it does not need to be watertight)."""
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "meshes")


def box(size, c=(0, 0, 0)):
    sx, sy, sz = [s / 2 for s in size]
    v = np.array([[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)]) + np.array(c)
    q = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return [(v[a], v[b], v[d]) for a, b, c_, d in q for (a, b, d) in ((a, b, c_), (a, c_, d))]


def cyl(r, h, c=(0, 0, 0), axis="z", n=28, ri=0.0):
    """Solid cylinder (or tube when ri>0) along `axis`."""
    t = []
    a = [2 * math.pi * i / n for i in range(n + 1)]
    for i in range(n):
        for rad in ([r, ri] if ri > 0 else [r]):
            p0, p1 = (rad * math.cos(a[i]), rad * math.sin(a[i])), (rad * math.cos(a[i + 1]), rad * math.sin(a[i + 1]))
            t += [((*p0, -h / 2), (*p1, -h / 2), (*p1, h / 2)), ((*p0, -h / 2), (*p1, h / 2), (*p0, h / 2))]
        o0, o1 = (r * math.cos(a[i]), r * math.sin(a[i])), (r * math.cos(a[i + 1]), r * math.sin(a[i + 1]))
        i0, i1 = (ri * math.cos(a[i]), ri * math.sin(a[i])), (ri * math.cos(a[i + 1]), ri * math.sin(a[i + 1]))
        for z in (-h / 2, h / 2):
            if ri > 0:
                t += [((*o0, z), (*o1, z), (*i1, z)), ((*o0, z), (*i1, z), (*i0, z))]
            else:
                t += [((0, 0, z), (*o0, z), (*o1, z))]
    perm = {"z": (0, 1, 2), "x": (2, 0, 1), "y": (1, 2, 0)}[axis]
    return [tuple(np.array([p[perm[0]], p[perm[1]], p[perm[2]]]) + np.array(c) if axis == "z" else
                  np.array([p[{"x": (2, 0, 1), "y": (0, 2, 1)}[axis][i]] for i in range(3)]) + np.array(c) for p in tri)
            for tri in t]


def write(name, tris):
    path = os.path.join(OUT, name + ".stl")
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for a, b, c in tris:
            a, b, c = map(np.asarray, (a, b, c))
            n = np.cross(b - a, c - a)
            n = n / (np.linalg.norm(n) or 1)
            f.write(f" facet normal {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}\n  outer loop\n")
            for p in (a, b, c):
                f.write(f"   vertex {p[0]:.5f} {p[1]:.5f} {p[2]:.5f}\n")
            f.write("  endloop\n endfacet\n")
        f.write(f"endsolid {name}\n")
    return len(tris)


def chassis():
    t = box((0.30, 0.24, 0.006), (0.15, 0, 0.023))                       # base plate
    t += box((0.14, 0.065, 0.045), (0.08, 0, 0.0485))                    # battery
    t += box((0.10, 0.075, 0.040), (0.20, 0, 0.046))                     # computer case
    for i in range(6):                                                   # heat-sink fins on the case
        t += box((0.09, 0.004, 0.012), (0.20, -0.03 + 0.012 * i, 0.072))
    t += box((0.26, 0.20, 0.004), (0.15, 0, 0.185))                      # upper deck
    for x, y in [(0.02, 0.105), (0.02, -0.105), (0.28, 0.105), (0.28, -0.105)]:
        t += cyl(0.008, 0.157, (x, y, 0.1045))                           # deck posts
        t += cyl(0.011, 0.004, (x, y, 0.025))                            # bolt heads
        t += cyl(0.011, 0.004, (x, y, 0.187))
    for y in (0.115, -0.115):
        t += cyl(0.010, 0.057, (0.0, y, 0.2155))                         # rear pillars (they cross the lidar plane)
    for y in (0.04, -0.04):
        t += cyl(0.006, 0.140, (0.285, y, 0.096))                        # OAK-D bracket uprights
    t += box((0.012, 0.100, 0.012), (0.285, 0, 0.026))                   # bracket foot
    t += cyl(0.005, 0.30, (0.15, 0.09, 0.034), axis="x", n=12)           # wire conduit along the plate
    return t


def wheel():
    t = cyl(0.05, 0.040, ri=0.035)                                       # tyre
    for i in range(24):                                                  # tread lugs
        a = 2 * math.pi * i / 24
        lug = box((0.008, 0.042, 0.004), (0.0, 0.0, 0.0))
        c, s = math.cos(a), math.sin(a)
        t += [tuple(np.array([c * p[0] - s * p[1] + c * 0.050, s * p[0] + c * p[1] + s * 0.050, p[2]]) for p in tri) for tri in
              [tuple(np.array([q[2], q[1], q[0]]) for q in tri) for tri in lug]]
    t += cyl(0.035, 0.030)                                               # rim
    t += cyl(0.012, 0.050)                                               # hub
    for i in range(5):                                                   # wheel bolts
        a = 2 * math.pi * i / 5
        t += cyl(0.003, 0.046, (0.020 * math.cos(a), 0.020 * math.sin(a), 0), n=10)
    return t


def caster_fork():
    t = cyl(0.015, 0.006, (0, 0, 0.0))                                   # bearing plate
    t += box((0.006, 0.040, 0.045), (-0.0125, 0, -0.0175))               # fork spine (frame: pivot)
    for y in (0.0185, -0.0185):
        t += box((0.004, 0.003, 0.040), (-0.025, y, -0.035))             # fork legs hold the axle
    return t


def caster_wheel():
    return cyl(0.035, 0.020, axis="y") + cyl(0.012, 0.030, axis="y")


def laser():
    t = cyl(0.048, 0.040, (0, 0, -0.002))                                # RPLidar-A1-style body
    t += cyl(0.030, 0.012, (0, 0, 0.026))                                # rotating head
    t += box((0.02, 0.02, 0.012), (0.034, 0, 0.026))                     # optics window
    return t


def oakd():
    t = box((0.030, 0.110, 0.0545))                                      # housing
    for y in (0.0375, -0.0375):
        t += cyl(0.0085, 0.004, (0.016, y, 0.0), axis="x", n=16)         # stereo lenses
    t += cyl(0.0085, 0.004, (0.016, 0.0, 0.0), axis="x", n=16)           # RGB lens
    t += box((0.012, 0.030, 0.006), (-0.020, 0, 0.030))                  # heat-sink on the back
    return t


def main():
    os.makedirs(OUT, exist_ok=True)
    for name, fn in [("chassis", chassis), ("wheel", wheel), ("caster_fork", caster_fork), ("caster_wheel", caster_wheel),
                     ("laser", laser), ("oakd", oakd)]:
        print(f"{name + '.stl':<20}{write(name, fn()):>6} triangles")


if __name__ == "__main__":
    main()
