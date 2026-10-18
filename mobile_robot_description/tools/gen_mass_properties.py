#!/usr/bin/env python3
"""Generates urdf/mass_properties.xacro: mass, centre of mass and the FULL inertia tensor of every link.

   python3 tools/gen_mass_properties.py            # rewrites ../urdf/mass_properties.xacro, prints the budget

Why a generator: every link is a COMPOSITE of real components (plate, battery, compute board, motors, posts ...).
Each component has a mass, a size and a position. The tensor of the whole link is the sum of the component
tensors moved to the link's centre of mass with the parallel-axis theorem (Steiner). So the numbers follow the
physical layout: move the battery and the centre of mass, the weight on the wheels and the suspension sag move
with it.

HOW TO USE YOUR CAD NUMBERS: the masses below are a budget from typical parts (aluminium 2.7 g/cm3, 4S2P Li-ion
pack, Raspberry-Pi/Jetson class computer, 12 V gear motors, RPLidar A1, OAK-D). Weigh your real parts and edit the
PARTS table; or, if CAD gives you a link's mass/COM/inertia directly, replace that body's entry in OVERRIDE
(mass, com, (ixx, iyy, izz, ixy, ixz, iyz)) and re-run. Nothing else in the repo needs to change.

The same script also sizes the drive-wheel suspension from the static load on each wheel, so the spring and
damper always match the mass budget.
"""
import math
import os

import numpy as np

G = 9.81
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "urdf", "mass_properties.xacro")

# ---------------------------------------------------------------------------------------------- geometry
SAG = 0.009            # target static spring compression of each drive wheel [m]
ZETA = 0.6             # suspension damping ratio (0.6: settles in ~1 bounce, no ringing)
WHEEL_Y = 0.175        # drive wheel track half-width [m]
CHASSIS_X = -0.10      # chassis frame origin in base_link [m]
CASTER_WHEEL_BASE_X = 0.135    # caster wheel centre in base_link x [m]


def box(m, sx, sy, sz, c):
    return dict(m=m, kind="box", size=(sx, sy, sz), c=c)


def cyl(m, r, h, c, axis="z", ri=0.0):
    return dict(m=m, kind="cyl", size=(r, h, ri), axis=axis, c=c)


# PARTS[body] = list of components, positions in that body's own frame. Masses in kg.
PARTS = {
    # sprung chassis. Frame: chassis (x forward, z up), origin 0.10 m behind the drive axle.
    "chassis": [
        box(1.170, 0.30, 0.24, 0.006, (0.15, 0.0, 0.023)),        # 6 mm aluminium base plate
        box(0.800, 0.14, 0.065, 0.045, (0.08, 0.0, 0.0485)),      # 4S2P Li-ion pack
        box(0.380, 0.10, 0.075, 0.040, (0.20, 0.0, 0.046)),       # computer in a case
        box(0.090, 0.07, 0.050, 0.015, (0.10, 0.0, 0.0785)),      # motor driver board
        box(0.560, 0.26, 0.20, 0.004, (0.15, 0.0, 0.185)),        # 4 mm upper deck
        cyl(0.085, 0.008, 0.157, (0.02, 0.105, 0.1045)),          # 4 deck posts (below the lidar plane)
        cyl(0.085, 0.008, 0.157, (0.02, -0.105, 0.1045)),
        cyl(0.085, 0.008, 0.157, (0.28, 0.105, 0.1045)),
        cyl(0.085, 0.008, 0.157, (0.28, -0.105, 0.1045)),
        cyl(0.048, 0.010, 0.057, (0.0, 0.115, 0.2155)),           # 2 rear pillars: cross the lidar plane
        cyl(0.048, 0.010, 0.057, (0.0, -0.115, 0.2155)),          #   (they create the lidar blind spots)
        cyl(0.020, 0.006, 0.140, (0.285, 0.04, 0.096)),           # OAK-D bracket uprights (reach up into the camera body)
        cyl(0.020, 0.006, 0.140, (0.285, -0.04, 0.096)),
        box(0.030, 0.04, 0.04, 0.02, (0.285, 0.0, 0.035)),        # bumper-side cable gland / switch block
    ],
    # unsprung: gear motor + hub bracket, one per side (frame: parallel to chassis, origin on the axle).
    "left_carrier": [box(0.250, 0.060, 0.035, 0.035, (0.0, -0.0375, 0.0)), box(0.080, 0.04, 0.02, 0.05, (0.0, -0.02, 0.0))],
    "right_carrier": [box(0.250, 0.060, 0.035, 0.035, (0.0, 0.0375, 0.0)), box(0.080, 0.04, 0.02, 0.05, (0.0, 0.02, 0.0))],
    # wheel: tyre ring + rim disc. Frame z = axle.
    "wheel": [cyl(0.090, 0.050, 0.040, (0, 0, 0), ri=0.035), cyl(0.050, 0.035, 0.040, (0, 0, 0))],
    # swivel caster
    "caster_fork": [box(0.060, 0.03, 0.03, 0.045, (-0.0125, 0.0, -0.0175))],
    "caster_wheel": [cyl(0.040, 0.035, 0.020, (0, 0, 0), axis="y")],
    # sensors (each a rigid body of its own so the lidar/camera can be re-mounted)
    "laser": [cyl(0.170, 0.048, 0.040, (0, 0, -0.002))],
    "camera": [box(0.114, 0.030, 0.110, 0.0545, (-0.003, 0, 0))],
    "imu": [box(0.005, 0.02, 0.02, 0.003, (0, 0, 0))],
}

# Link positions in the CHASSIS frame (x of the origin), only used for the static load calculation.
POS_X = {"chassis": 0.0, "laser": 0.122, "camera": 0.285, "imu": 0.15, "left_carrier": 0.10, "right_carrier": 0.10,
         "caster_fork": 0.26}

# CAD override: body -> dict(mass=, com=(x,y,z), inertia=(ixx, iyy, izz, ixy, ixz, iyz)). Empty = use the parts table.
OVERRIDE = {}


def part_tensor(p):
    """Inertia of one component about its own COM, axes aligned with the body frame."""
    m = p["m"]
    if p["kind"] == "box":
        x, y, z = p["size"]
        return np.diag([m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12])
    r, h, ri = p["size"]
    axial = m * (r * r + ri * ri) / 2
    trans = m * (3 * (r * r + ri * ri) + h * h) / 12
    d = {"x": [axial, trans, trans], "y": [trans, axial, trans], "z": [trans, trans, axial]}[p["axis"]]
    return np.diag(d)


def composite(parts):
    M = sum(p["m"] for p in parts)
    com = sum(p["m"] * np.array(p["c"]) for p in parts) / M
    I = np.zeros((3, 3))
    for p in parts:
        r = np.array(p["c"]) - com
        I += part_tensor(p) + p["m"] * ((r @ r) * np.eye(3) - np.outer(r, r))
    return M, com, I


def body_props(name):
    if name in OVERRIDE:
        o = OVERRIDE[name]
        ixx, iyy, izz, ixy, ixz, iyz = o["inertia"]
        return o["mass"], np.array(o["com"]), np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
    return composite(PARTS[name])


def inertial_xml(name, ind="        "):
    M, com, I = body_props(name)
    f = lambda v: f"{v:.6g}"
    return (f'{ind}<inertial>\n{ind}    <origin xyz="{f(com[0])} {f(com[1])} {f(com[2])}" rpy="0 0 0"/>\n'
            f'{ind}    <mass value="{f(M)}"/>\n'
            f'{ind}    <inertia ixx="{f(I[0,0])}" ixy="{f(I[0,1])}" ixz="{f(I[0,2])}" iyy="{f(I[1,1])}" '
            f'iyz="{f(I[1,2])}" izz="{f(I[2,2])}"/>\n{ind}</inertial>\n')


def suspension():
    """Static load per drive wheel -> spring k, damper c, free-hang offset. All derived from the mass budget."""
    masses = {n: body_props(n)[0] for n in PARTS}
    # centre of mass of everything in base_link x
    xs = {"chassis": CHASSIS_X + body_props("chassis")[1][0]}
    for n in ("laser", "camera", "imu", "caster_fork"):
        xs[n] = CHASSIS_X + POS_X[n] + body_props(n)[1][0]
    xs["left_carrier"] = xs["right_carrier"] = 0.0
    xs["left_wheel"] = xs["right_wheel"] = 0.0
    xs["caster_wheel"] = CASTER_WHEEL_BASE_X
    mm = dict(masses, left_wheel=masses["wheel"], right_wheel=masses["wheel"])
    M = sum(mm[k] for k in xs)
    x_com = sum(mm[k] * xs[k] for k in xs) / M
    axle_load = M * G * (CASTER_WHEEL_BASE_X - x_com) / CASTER_WHEEL_BASE_X      # both drive wheels together [N]
    unsprung = (masses["left_carrier"] + masses["wheel"]) * G                    # per side, rests directly on the floor
    f_spring = axle_load / 2 - unsprung                                          # what each spring must carry [N]
    k = round(f_spring / SAG / 50) * 50
    sag = f_spring / k
    m_sprung_wheel = f_spring / G
    c = 2 * ZETA * math.sqrt(k * m_sprung_wheel)
    return dict(total_mass=M, x_com=x_com, axle_load=axle_load, caster_load=M * G - axle_load, f_spring=f_spring,
                k=k, c=round(c, 1), sag=sag, free_z=-sag, caster_share=1 - axle_load / (M * G))


def write():
    s = suspension()
    lines = ['<?xml version="1.0"?>',
             '<!-- GENERATED by tools/gen_mass_properties.py. Do not edit by hand: edit the PARTS table and re-run. -->',
             '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">', '']
    lines += [f'    <xacro:property name="total_mass" value="{s["total_mass"]:.4f}"/>',
              f'    <xacro:property name="susp_k" value="{s["k"]:.0f}"/>          <!-- N/m, per drive wheel -->',
              f'    <xacro:property name="susp_c" value="{s["c"]}"/>           <!-- N*s/m, damping ratio {ZETA} -->',
              f'    <xacro:property name="susp_sag" value="{s["sag"]:.5f}"/>   <!-- m, static compression under load -->',
              f'    <xacro:property name="susp_free_z" value="{s["free_z"]:.5f}"/>  <!-- carrier hangs this far below the loaded pose -->',
              '']
    for name, macro in [("chassis", "chassis"), ("left_carrier", "left_carrier"), ("right_carrier", "right_carrier"),
                        ("wheel", "wheel"), ("caster_fork", "caster_fork"), ("caster_wheel", "caster_wheel"),
                        ("laser", "laser"), ("camera", "camera"), ("imu", "imu")]:
        lines += [f'    <xacro:macro name="inertial_{macro}">', inertial_xml(name).rstrip("\n"), '    </xacro:macro>', '']
    lines.append('</robot>')
    with open(OUT, "w") as f:
        f.write("\n".join(lines) + "\n")
    return s


def main():
    s = write()
    print("wrote", os.path.normpath(OUT))
    print(f"{'body':<14}{'mass kg':>9}   COM (x, y, z) m              Ixx / Iyy / Izz kg*m^2")
    for n in PARTS:
        M, c, I = body_props(n)
        print(f"{n:<14}{M:9.3f}   ({c[0]:+.3f}, {c[1]:+.3f}, {c[2]:+.3f})   {I[0,0]:.2e} {I[1,1]:.2e} {I[2,2]:.2e}")
    print(f"\ntotal {s['total_mass']:.2f} kg | COM x {s['x_com']:+.3f} m from the axle | caster carries {100*s['caster_share']:.0f}%")
    print(f"drive wheel spring: k {s['k']:.0f} N/m, c {s['c']} N*s/m, sag {1000*s['sag']:.1f} mm under {s['f_spring']:.1f} N")


if __name__ == "__main__":
    main()
