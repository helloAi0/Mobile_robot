#!/usr/bin/env python3
"""Generates worlds/house_world.sdf: an indoor floor plan (corridor + 5 rooms) made for SLAM and navigation.

   python3 tools/make_house_world.py            # rewrites worlds/house_world.sdf

Layout (metres, robot starts at 0,0 in the corridor facing +x):
     y= 5.5 +-----------+-----------+-----------+
            | LIVING    | DINING    | BEDROOM   |      north rooms, doors (1.2 m) onto the corridor
     y= 1.0 +--  --------+--  -------+--  -------+
            |              CORRIDOR (2 m wide)  |
     y=-1.0 +--  -----------------  -----  ------+
            | OFFICE (x<0)      | WORKSHOP (x>0)|     south rooms
     y=-5.5 +-------------------+---------------+
          x=-8                 0                x=8
Walls, furniture, pillars and ramps are static (solid). Boxes, crates, barrels, cones and balls are free bodies
(real physics, the robot can push them). People are Gazebo actors (animated, no collision, seen by camera + lidar).
"""
import os

out, names = [], set()
WALL_H, WALL_T = 2.5, 0.15


def rgba(c, a=1):
    return f"{c[0]} {c[1]} {c[2]} {a}"


def mat(c):
    return f"<material><ambient>{rgba(c)}</ambient><diffuse>{rgba(c)}</diffuse></material>"


def geom(kind, size):
    if kind == "box":
        return f"<box><size>{size[0]} {size[1]} {size[2]}</size></box>"
    if kind == "cyl":
        return f"<cylinder><radius>{size[0]}</radius><length>{size[1]}</length></cylinder>"
    if kind == "sph":
        return f"<sphere><radius>{size[0]}</radius></sphere>"
    raise ValueError(kind)


def part(name, kind, size, pose, color, collide=False):
    p = " ".join(str(v) for v in pose)
    s = f'      <visual name="{name}"><pose>{p}</pose><geometry>{geom(kind, size)}</geometry>{mat(color)}</visual>\n'
    if collide:
        s += f'      <collision name="{name}_col"><pose>{p}</pose><geometry>{geom(kind, size)}</geometry></collision>\n'
    return s


def model(name, pose, parts, comment=None, dyn=None):
    """dyn=(mass, ixx, iyy, izz, com_z) -> free rigid body, otherwise static."""
    assert name not in names, name
    names.add(name)
    c = f"    <!-- {comment} -->\n" if comment else ""
    p = " ".join(str(v) for v in pose)
    if dyn:
        m, ix, iy, iz, cz = dyn
        inertial = (f'      <inertial><pose>0 0 {cz} 0 0 0</pose><mass>{m}</mass><inertia><ixx>{ix}</ixx><iyy>{iy}</iyy>'
                    f'<izz>{iz}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>\n')
        static = "false"
    else:
        inertial, static = "", "true"
    out.append(f'{c}    <model name="{name}">\n      <static>{static}</static>\n      <pose>{p}</pose>\n'
               f'      <link name="link">\n{inertial}{"".join(parts)}      </link>\n    </model>\n')


def box_inertia(m, x, y, z):
    return (m, round(m * (y * y + z * z) / 12, 4), round(m * (x * x + z * z) / 12, 4), round(m * (x * x + y * y) / 12, 4))


# ------------------------------------------------------------------ colours
WALL = (0.90, 0.88, 0.82); OUTER = (0.80, 0.78, 0.72); TRIM = (0.35, 0.3, 0.25)
WOOD = (0.55, 0.38, 0.20); WOOD2 = (0.40, 0.26, 0.14); SOFA = (0.25, 0.35, 0.50); BED = (0.7, 0.75, 0.85)
METAL = (0.45, 0.47, 0.5); PLANT = (0.15, 0.45, 0.15); POT = (0.55, 0.3, 0.2); CARD = (0.60, 0.45, 0.25)
RED = (0.7, 0.12, 0.1); ORANGE = (1.0, 0.45, 0.05); WHITE = (0.92, 0.92, 0.92); CONC = (0.62, 0.62, 0.6)
FLOOR_HALL = (0.72, 0.70, 0.66); FLOOR_WOOD = (0.62, 0.45, 0.28); FLOOR_TILE = (0.78, 0.80, 0.82)
FLOOR_CARPET = (0.45, 0.50, 0.58); FLOOR_CONC = (0.55, 0.55, 0.55)

# ------------------------------------------------------------------ floors (visual only; the ground plane is the collision)
def floor(name, x0, x1, y0, y1, color):
    model(name, ((x0 + x1) / 2, (y0 + y1) / 2, 0.005, 0, 0, 0),
          [part("f", "box", (x1 - x0, y1 - y0, 0.01), (0, 0, 0, 0, 0, 0), color)])


floor("floor_corridor", -8, 8, -1, 1, FLOOR_HALL)
floor("floor_living", -8, -2.7, 1, 5.5, FLOOR_WOOD)
floor("floor_dining", -2.7, 2.7, 1, 5.5, FLOOR_TILE)
floor("floor_bedroom", 2.7, 8, 1, 5.5, FLOOR_CARPET)
floor("floor_office", -8, 0, -5.5, -1, FLOOR_CARPET)
floor("floor_workshop", 0, 8, -5.5, -1, FLOOR_CONC)

# ------------------------------------------------------------------ walls
def wall(name, x0, y0, x1, y1, color=WALL):
    """Straight wall from (x0,y0) to (x1,y1), axis aligned."""
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    sx, sy = max(abs(x1 - x0), WALL_T), max(abs(y1 - y0), WALL_T)
    model(name, (cx, cy, WALL_H / 2, 0, 0, 0), [part("w", "box", (sx, sy, WALL_H), (0, 0, 0, 0, 0, 0), color, collide=True)])


wall("outer_north", -8, 5.5, 8, 5.5, OUTER)
wall("outer_south", -8, -5.5, 8, -5.5, OUTER)
wall("outer_west", -8, -5.5, -8, 5.5, OUTER)
wall("outer_east", 8, -5.5, 8, 5.5, OUTER)
# north wall of the corridor (y=1) with 3 doors (1.2 m wide) at x=-5.35, 0, 5.35
for i, (a, b) in enumerate([(-8, -5.95), (-4.75, -0.6), (0.6, 4.75), (5.95, 8)]):
    wall(f"hall_north_{i}", a, 1.0, b, 1.0)
# south wall of the corridor (y=-1) with 2 doors at x=-4 and x=4
for i, (a, b) in enumerate([(-8, -4.6), (-3.4, 3.4), (4.6, 8)]):
    wall(f"hall_south_{i}", a, -1.0, b, -1.0)
wall("div_north_1", -2.7, 1.0, -2.7, 5.5)
wall("div_north_2", 2.7, 1.0, 2.7, 5.5)
wall("div_south", 0, -5.5, 0, -1.0)
# door frames (visual) so the doorways are easy to spot
for i, x in enumerate([-5.35, 0, 5.35]):
    model(f"frame_n{i}", (x, 1.0, 2.2, 0, 0, 0), [part("lintel", "box", (1.3, 0.2, 0.15), (0, 0, 0, 0, 0, 0), TRIM)])
for i, x in enumerate([-4, 4]):
    model(f"frame_s{i}", (x, -1.0, 2.2, 0, 0, 0), [part("lintel", "box", (1.3, 0.2, 0.15), (0, 0, 0, 0, 0, 0), TRIM)])

# ------------------------------------------------------------------ furniture (static, solid)
def furniture(name, x, y, sx, sy, sz, color, yaw=0.0):
    model(name, (x, y, sz / 2, 0, 0, yaw), [part("b", "box", (sx, sy, sz), (0, 0, 0, 0, 0, 0), color, collide=True)])


def plant(name, x, y):
    model(name, (x, y, 0, 0, 0, 0), [part("pot", "cyl", (0.2, 0.4), (0, 0, 0.2, 0, 0, 0), POT, collide=True),
                                     part("leaf", "sph", (0.32,), (0, 0, 0.75, 0, 0, 0), PLANT)])


# living room
furniture("sofa", -5.3, 4.9, 2.2, 0.9, 0.75, SOFA)
furniture("coffee_table", -5.3, 3.3, 1.1, 0.6, 0.4, WOOD2)
furniture("tv_stand", -7.7, 3.2, 0.4, 1.6, 0.5, WOOD2)
furniture("bookshelf", -3.4, 5.25, 1.2, 0.35, 1.9, WOOD2)
plant("plant_living", -7.3, 1.6)
# dining room
furniture("dining_table", 0, 3.3, 1.8, 0.9, 0.75, WOOD)
for i, (cx, cy) in enumerate([(-0.5, 2.5), (0.5, 2.5), (-0.5, 4.1), (0.5, 4.1)]):
    furniture(f"chair_{i}", cx, cy, 0.4, 0.4, 0.45, WOOD2)
furniture("sideboard", 0, 5.2, 2.0, 0.4, 0.9, WOOD2)
plant("plant_dining", 2.1, 1.7)
# bedroom
furniture("bed", 6.6, 4.3, 2.0, 1.6, 0.5, BED, yaw=1.5708)
furniture("wardrobe", 7.7, 2.2, 0.6, 1.8, 2.0, WOOD2)
furniture("nightstand", 5.2, 5.1, 0.4, 0.4, 0.5, WOOD)
# office (south west)
furniture("desk", -5.0, -5.0, 1.8, 0.7, 0.75, WOOD)
model("office_chair", (-5.0, -4.1, 0, 0, 0, 0), [part("seat", "cyl", (0.22, 0.08), (0, 0, 0.45, 0, 0, 0), METAL, collide=True),
                                               part("stem", "cyl", (0.04, 0.45), (0, 0, 0.22, 0, 0, 0), METAL)])
furniture("filing_cabinet", -7.6, -2.0, 0.5, 0.6, 1.3, METAL)
furniture("office_shelf", -7.7, -4.0, 0.4, 2.0, 1.8, WOOD2)
plant("plant_office", -1.0, -4.8)
# workshop (south east)
furniture("workbench", 6.5, -5.0, 2.4, 0.8, 0.9, WOOD2)
furniture("tool_shelf", 7.7, -3.0, 0.4, 2.4, 1.8, METAL)
model("pillar_1", (3.2, -4.0, 1.25, 0, 0, 0), [part("p", "cyl", (0.25, 2.5), (0, 0, 0, 0, 0, 0), CONC, collide=True)])
model("pillar_2", (4.8, -4.6, 1.25, 0, 0, 0), [part("p", "cyl", (0.25, 2.5), (0, 0, 0, 0, 0, 0), CONC, collide=True)])
# Ramp: 7 degrees, 2.4 m long slopes, flat top at 0.25 m. The low ends are buried in the floor so there is NO lip
# (a 5 cm wheel cannot climb a vertical edge: an earlier version with a lip trapped the robot).
RAMP_C = (0.55, 0.5, 0.4)
model("ramp_ws", (1.5, -2.3, 0, 0, 0, 0), [
    part("slope", "box", (2.4, 1.2, 0.06), (0, 0, 0.077, 0, -0.12, 0), RAMP_C, collide=True),
    part("top", "box", (1.0, 1.2, 0.06), (1.69, 0, 0.22, 0, 0, 0), RAMP_C, collide=True),
    part("down", "box", (2.4, 1.2, 0.06), (3.38, 0, 0.077, 0, 0.12, 0), RAMP_C, collide=True)],
      "Gentle ramp (7 degrees): up, flat top, down. Drive over it to show the physics.")

# ------------------------------------------------------------------ decoration (visual only, no collision)
def rug(name, x, y, sx, sy, color):
    model(name, (x, y, 0.012, 0, 0, 0), [part("r", "box", (sx, sy, 0.012), (0, 0, 0, 0, 0, 0), color)])


rug("rug_living", -5.3, 3.3, 2.8, 1.8, (0.60, 0.25, 0.25))
rug("rug_dining", 0, 3.3, 2.6, 1.8, (0.30, 0.40, 0.55))
rug("rug_bedroom", 6.0, 4.0, 2.4, 1.6, (0.45, 0.35, 0.50))
rug("rug_corridor", -2.0, 0, 3.0, 0.9, (0.55, 0.20, 0.20))


def painting(name, x, y, yaw, w=1.0, h=0.7, color=(0.25, 0.45, 0.65)):
    model(name, (x, y, 1.55, 0, 0, yaw), [part("frame", "box", (w + 0.08, 0.03, h + 0.08), (0, 0, 0, 0, 0, 0), TRIM),
                                          part("img", "box", (w, 0.035, h), (0, 0, 0, 0, 0, 0), color)])


painting("art_1", -6.0, 5.4, 0.0)
painting("art_2", 1.0, 5.4, 0.0, color=(0.65, 0.5, 0.25))
painting("art_3", 5.0, 5.4, 0.0, color=(0.3, 0.55, 0.4))
painting("art_4", 3.0, -5.4, 0.0, color=(0.6, 0.3, 0.3))
painting("art_5", -3.0, -5.4, 0.0, color=(0.3, 0.35, 0.6))
# ------------------------------------------------------------------ free bodies (real physics)
def crate(name, x, y, z, size=0.4, yaw=0.0, mass=1.0, color=CARD):
    model(name, (x, y, z, 0, 0, yaw), [part("c", "box", (size, size, size), (0, 0, 0, 0, 0, 0), color, collide=True)],
          "Free-standing box: the robot can push it", dyn=box_inertia(mass, size, size, size) + (0,))


crate("box_1", -2.0, -2.6, 0.25, 0.5, 0.3)
crate("box_2", -3.4, -4.4, 0.2, 0.4, 0.8)
crate("box_3", 3.0, -0.55, 0.2, 0.4, 0.1)          # in the corridor: something to drive around
crate("crate_a", 6.8, -1.8, 0.2, 0.4, 0.1, color=WOOD)
crate("crate_b", 6.8, -2.3, 0.2, 0.4, 0.0, color=WOOD)
crate("crate_c", 6.8, -2.05, 0.6, 0.4, 0.2, color=WOOD)
for i, (bx, by) in enumerate([(5.6, -2.6), (7.0, -3.9)]):
    model(f"barrel_{i+1}", (bx, by, 0, 0, 0, 0), [part("b", "cyl", (0.3, 0.9), (0, 0, 0.45, 0, 0, 0), RED, collide=True),
                                                   part("r", "cyl", (0.31, 0.06), (0, 0, 0.6, 0, 0, 0), WHITE)],
          "Barrel (3 kg free body)", dyn=(3.0, 0.27, 0.27, 0.135, 0.45))
for i, (cx, cy) in enumerate([(-6.0, 0.5), (-5.0, -0.5), (-4.0, 0.5)]):   # slalom in the corridor
    model(f"cone_{i+1}", (cx, cy, 0, 0, 0, 0), [part("base", "box", (0.34, 0.34, 0.03), (0, 0, 0.015, 0, 0, 0), (0.1, 0.1, 0.1)),
                                                 part("c1", "cyl", (0.13, 0.30), (0, 0, 0.18, 0, 0, 0), ORANGE, collide=True),
                                                 part("c2", "cyl", (0.08, 0.30), (0, 0, 0.46, 0, 0, 0), ORANGE),
                                                 part("s", "cyl", (0.105, 0.07), (0, 0, 0.34, 0, 0, 0), WHITE)],
          "Traffic cone (free body)", dyn=(0.4, 0.014, 0.014, 0.0034, 0.17))
for i, (bx, by, c) in enumerate([(-6.5, -3.0, (0.9, 0.2, 0.2)), (1.5, 2.0, (0.2, 0.4, 0.9))]):
    model(f"ball_{i+1}", (bx, by, 0.2, 0, 0, 0), [part("b", "sph", (0.2,), (0, 0, 0, 0, 0, 0), c, collide=True)],
          "Ball (rolls when pushed)", dyn=(0.5, 0.008, 0.008, 0.008, 0))

# ------------------------------------------------------------------ people (actors)
MAN = "https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae"
WOMAN = "https://fuel.gazebosim.org/1.0/Luca/models/FemaleVisitorWalk/tip/files/meshes/FemaleVisitorWalk.dae"
MZ = 1.0     # root height for the man mesh (verified: feet on the floor)
WZ = 0.0     # root height for the woman mesh (its feet are at z=0). Tune: ros2 run mobile_robot tune_actor_height.py <z>


def actor(name, mesh, pts, comment):
    wps = "\n".join(f'          <waypoint><time>{t}</time><pose>{x} {y} {z} 0 0 {yaw}</pose></waypoint>' for t, x, y, z, yaw in pts)
    return f'''    <!-- {comment} -->
    <actor name="{name}">
      <skin><filename>{mesh}</filename><scale>1.0</scale></skin>
      <animation name="walk"><filename>{mesh}</filename><interpolate_x>true</interpolate_x></animation>
      <script>
        <loop>true</loop>
        <delay_start>0.0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walk" tension="0.6">
{wps}
        </trajectory>
      </script>
    </actor>
'''


def patrol(x0, y0, x1, y1, z, speed=0.9, t0=0.0):
    """Walk (x0,y0)->(x1,y1), turn, walk back, forever. Returns waypoints."""
    import math
    d = math.hypot(x1 - x0, y1 - y0)
    yaw = math.atan2(y1 - y0, x1 - x0)
    T = d / speed
    back = yaw + math.pi if yaw <= 0 else yaw - math.pi
    return [(t0, x0, y0, z, round(yaw, 3)), (round(t0 + T, 2), x1, y1, z, round(yaw, 3)), (round(t0 + T + 0.5, 2), x1, y1, z, round(back, 3)),
            (round(t0 + 2 * T + 0.5, 2), x0, y0, z, round(back, 3)), (round(t0 + 2 * T + 1.0, 2), x0, y0, z, round(yaw, 3))]


people = (
    actor("man_corridor", MAN, patrol(-6.5, 0.45, 6.5, 0.45, MZ), "Man walking up and down the corridor") + "\n" +
    actor("woman_corridor", WOMAN, patrol(6.5, -0.45, -6.5, -0.45, WZ, speed=0.8), "Woman walking the other way along the corridor") + "\n" +
    actor("woman_workshop", WOMAN, patrol(1.2, -4.9, 6.0, -3.6, WZ, speed=0.7), "Woman walking through the workshop") + "\n" +
    actor("man_dining", MAN, patrol(-2.0, 1.8, -2.0, 4.8, MZ, speed=0.7), "Man pacing in the living/dining area (near the wall)"))

lights = ""
for i, (lx, ly) in enumerate([(-5.35, 3.3), (0, 3.3), (5.35, 3.3), (0, 0), (-6, 0), (6, 0), (-4, -3.3), (4, -3.3)]):
    lights += f'''    <light type="point" name="lamp_{i}"><pose>{lx} {ly} 2.3 0 0 0</pose><cast_shadows>false</cast_shadows>
      <diffuse>0.55 0.52 0.45 1</diffuse><specular>0.05 0.05 0.05 1</specular>
      <attenuation><range>9</range><constant>0.6</constant><linear>0.05</linear><quadratic>0.01</quadratic></attenuation></light>\n'''

header = '''<?xml version="1.0" ?>
<!--
  House world for SLAM / navigation (Gazebo Harmonic, gz-sim 8). GENERATED by tools/make_house_world.py: edit that
  script and re-run it instead of editing this file by hand.

  Corridor (2 m wide) with 5 rooms, doors 1.2 m wide, furniture, pillars, a ramp, free bodies (boxes, barrels, cones,
  balls: real physics) and 4 walking people (actors: animated, no collision, but seen by camera and lidar).
  The robot starts at (0, 0) in the corridor facing +x. Needs internet the first time (people come from Gazebo Fuel).
-->
<sdf version="1.9">
  <world name="default">
    <physics name="default_physics" type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.62 0.78 0.95 1</background>
      <shadows>true</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.7 0.7 0.68 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.35 0.4 0.3 1</ambient><diffuse>0.35 0.4 0.3 1</diffuse></material></visual>
      </link>
    </model>
'''
footer = "\n  </world>\n</sdf>\n"
dst = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "worlds", "house_world.sdf")
open(dst, "w").write(header + lights + "".join(out) + "\n" + people + footer)
print("wrote", os.path.normpath(dst), "| models:", len(names))
