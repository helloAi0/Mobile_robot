#!/usr/bin/env python3
"""Import textured props from the osrf/gazebo_models repository as clean, production-style Gazebo Harmonic models.

   git clone --depth 1 https://github.com/osrf/gazebo_models /tmp/gazebo_models
   python3 tools/import_gazebo_models.py /tmp/gazebo_models            # needs: pip install trimesh pycollada numpy

For every model in SPECS it:
  * copies the textured COLLADA mesh (+ textures) to models/<name>/meshes and models/<name>/materials/textures,
  * sets the mesh unit to metres and fixes bare texture file names (the originals relied on Gazebo-classic material scripts),
  * scales the mesh to real-world target dimensions (the sources mix inches, millimetres and metres),
  * writes model.sdf with the detailed .dae as the VISUAL only and a few cheap primitives as the COLLISION,
  * writes model.config and records authorship in models/ATTRIBUTION.md (the source models are CC BY 3.0).
Mesh bounds are measured with trimesh, so the collision shapes match the visuals. The output is committed: you only need
to run this again to add or change a model.
"""
import os
import re
import shutil
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.normpath(os.path.join(HERE, "..", "models"))

# name: source dir, mesh (relative to source dir), size target, collision shape, mass (None = static), description
#   size: ("h", metres)  uniform scale to this height     |   ("xyz", (x, y, z))  scale each axis to these dimensions
SPECS = {
    "cafe_table":        ("cafe_table", "meshes/cafe_table.dae", ("h", 0.775), "table", None, "Round cafe table, maple top"),
    "cardboard_box":     ("cardboard_box", "meshes/cardboard_box.dae", ("xyz", (0.5, 0.4, 0.3)), "box", 1.5, "Cardboard box (1.5 kg, pushable)"),
    "euro_pallet":       ("euro_pallet", "meshes/pallet.dae", ("xyz", (0.8, 1.2, 0.144)), "box", None, "EUR-pallet 800 x 1200 mm"),
    "trash_can":         ("first_2015_trash_can", "meshes/trash_can.dae", ("h", 0.62), "cyl", 1.5, "Plastic waste bin (1.5 kg, pushable)"),
    "grey_tote":         ("grey_tote", "meshes/tote.dae", ("xyz", (0.68, 0.41, 0.31)), "box", 2.0, "Storage tote (2 kg, pushable)"),
    "construction_cone": ("construction_cone", "meshes/construction_cone.dae", ("h", 0.75), "cone", 0.8, "Traffic cone (0.8 kg, pushable)"),
    "construction_barrel": ("construction_barrel", "meshes/construction_barrel.dae", ("h", 0.9), "cyl", 4.0, "Construction barrel (4 kg, pushable)"),
}


def sanitize_dae(src, dst):
    """Copy a DAE with unit = metre (so no loader has to guess) and texture paths relative to ../materials/textures."""
    text = open(src, encoding="utf-8", errors="replace").read()
    text = re.sub(r"<unit\b[^>]*/>", '<unit name="meter" meter="1"/>', text)

    def fix(m):
        p = m.group(1).strip()
        if p.lower().endswith((".png", ".jpg", ".jpeg", ".tga")) and "/" not in p and not p.startswith("file:"):
            p = "../materials/textures/" + p
        return f"<init_from>{p}</init_from>"
    text = re.sub(r"<init_from>([^<]*)</init_from>", fix, text)
    open(dst, "w", encoding="utf-8").write(text)


def mesh_bounds(path):
    scene = trimesh.load(path, force="scene")
    b = trimesh.util.concatenate(scene.dump()).bounds
    return b[0], b[1]


def f(v):
    return f"{v:.4f}".rstrip("0").rstrip(".") if abs(v) > 1e-9 else "0"


def collision_xml(shape, ext, zc):
    """Primitive collision shapes in the model frame (origin on the floor, z up). ext = final (x, y, z) size."""
    ex, ey, ez = ext
    if shape == "box":
        return [("body", f"0 0 {f(zc)} 0 0 0", f"<box><size>{f(ex)} {f(ey)} {f(ez)}</size></box>")]
    if shape == "cyl":
        r = 0.45 * min(ex, ey)
        return [("body", f"0 0 {f(zc)} 0 0 0", f"<cylinder><radius>{f(r)}</radius><length>{f(ez)}</length></cylinder>")]
    if shape == "cone":                                    # wide thin base + slim shaft: the tip is not fattened for the lidar
        return [("base", "0 0 0.02 0 0 0", f"<cylinder><radius>{f(0.45 * min(ex, ey))}</radius><length>0.04</length></cylinder>"),
                ("shaft", f"0 0 {f(ez / 2)} 0 0 0", f"<cylinder><radius>{f(0.17 * min(ex, ey))}</radius><length>{f(ez)}</length></cylinder>")]
    if shape == "table":                                   # top slab + column + foot: three boxes, no mesh collision
        return [("top", f"0 0 {f(ez - 0.02)} 0 0 0", f"<box><size>{f(ex)} {f(ey)} 0.04</size></box>"),
                ("column", f"0 0 {f(ez / 2)} 0 0 0", f"<box><size>0.05 0.05 {f(ez)}</size></box>"),
                ("foot", "0 0 0.02 0 0 0", f"<box><size>{f(0.6 * ex)} {f(0.6 * ey)} 0.04</size></box>")]
    raise ValueError(shape)


def inertia(shape, mass, ext):
    ex, ey, ez = ext
    if shape in ("cyl", "cone"):
        r = 0.45 * min(ex, ey) if shape == "cyl" else 0.25 * min(ex, ey)
        ixx = mass * (3 * r * r + ez * ez) / 12
        return ixx, ixx, mass * r * r / 2
    return mass * (ey * ey + ez * ez) / 12, mass * (ex * ex + ez * ez) / 12, mass * (ex * ex + ey * ey) / 12


def build(name, spec, repo):
    src, mesh_rel, size, shape, mass, desc = spec
    sdir = os.path.join(repo, src)
    ddir = os.path.join(OUT, name)
    shutil.rmtree(ddir, ignore_errors=True)
    os.makedirs(os.path.join(ddir, "meshes"))
    mesh_name = os.path.basename(mesh_rel)
    sanitize_dae(os.path.join(sdir, mesh_rel), os.path.join(ddir, "meshes", mesh_name))
    tex_src = os.path.join(sdir, "materials", "textures")
    if os.path.isdir(tex_src):
        shutil.copytree(tex_src, os.path.join(ddir, "materials", "textures"))

    lo, hi = mesh_bounds(os.path.join(ddir, "meshes", mesh_name))
    raw = hi - lo
    if size[0] == "h":
        scale = np.full(3, size[1] / raw[2])
    else:
        scale = np.array(size[1]) / raw
    ext = raw * scale
    # visual pose: centre the footprint on the model origin and put the lowest point on the floor
    off = -np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]]) * scale

    cols = "".join(
        f'      <collision name="{n}"><pose>{p}</pose><geometry>{g}</geometry>'
        f'<surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface></collision>\n'
        for n, p, g in collision_xml(shape, ext, ext[2] / 2))
    if mass:
        ixx, iyy, izz = inertia(shape, mass, ext)
        inert = (f'      <inertial><pose>0 0 {f(ext[2] / 2)} 0 0 0</pose><mass>{mass}</mass><inertia><ixx>{ixx:.5f}</ixx>'
                 f'<iyy>{iyy:.5f}</iyy><izz>{izz:.5f}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>\n')
    else:
        inert = ""
    sdf = f'''<?xml version="1.0" ?>
<!-- GENERATED by tools/import_gazebo_models.py from osrf/gazebo_models/{src} (CC BY 3.0, see ../ATTRIBUTION.md).
     {desc}. Visual: textured COLLADA mesh. Collision: primitives only. Size {ext[0]:.3f} x {ext[1]:.3f} x {ext[2]:.3f} m. -->
<sdf version="1.9">
  <model name="{name}">
    <static>{"false" if mass else "true"}</static>
    <link name="link">
{inert}      <visual name="visual">
        <pose>{f(off[0])} {f(off[1])} {f(off[2])} 0 0 0</pose>
        <geometry><mesh><uri>model://{name}/meshes/{mesh_name}</uri><scale>{f(scale[0])} {f(scale[1])} {f(scale[2])}</scale></mesh></geometry>
      </visual>
{cols}    </link>
  </model>
</sdf>
'''
    open(os.path.join(ddir, "model.sdf"), "w").write(sdf)
    open(os.path.join(ddir, "model.config"), "w").write(
        f'<?xml version="1.0"?>\n<model>\n  <name>{name}</name>\n  <version>1.0</version>\n  <sdf version="1.9">model.sdf</sdf>\n'
        f'  <description>{desc}</description>\n</model>\n')
    return ext, mass


def main():
    if len(sys.argv) != 2 or not os.path.isdir(sys.argv[1]):
        sys.exit(__doc__)
    repo = sys.argv[1]
    rows = []
    for name, spec in SPECS.items():
        ext, mass = build(name, spec, repo)
        rows.append((name, spec[0], ext, mass))
        print(f"{name:20s} {ext[0]:.3f} x {ext[1]:.3f} x {ext[2]:.3f} m   {'static' if not mass else str(mass) + ' kg'}")
    lic = os.path.join(repo, "LICENSE")
    with open(os.path.join(OUT, "ATTRIBUTION.md"), "w") as a:
        a.write("# Third-party models\n\nThe meshes and textures in these folders come from the Open Source Robotics Foundation model\n"
                "database https://github.com/osrf/gazebo_models, licensed **Creative Commons Attribution 3.0 Unported**\n"
                "(https://creativecommons.org/licenses/by/3.0/). Changes: unit set to metres, texture paths fixed, scaled to the\n"
                "sizes below, new SDF with primitive collisions.\n\n| model here | source model | size (m) |\n|---|---|---|\n")
        for n, s, e, _ in rows:
            a.write(f"| `{n}` | `{s}` | {e[0]:.2f} x {e[1]:.2f} x {e[2]:.2f} |\n")
        if os.path.exists(lic):
            a.write("\nOriginal licence header: " + open(lic).read().split("\n")[0] + " (Copyright 2012 Nathan Koenig and contributors)\n")


if __name__ == "__main__":
    main()
