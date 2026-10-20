import os
import math

out = []
names = set()

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
    """One visual (and optionally collision) inside a link. pose = (x,y,z,roll,pitch,yaw) relative to model."""
    p = " ".join(str(v) for v in pose)
    s = f'      <visual name="{name}"><pose>{p}</pose><geometry>{geom(kind, size)}</geometry>{mat(color)}</visual>\n'
    if collide:
        s += f'      <collision name="{name}_col"><pose>{p}</pose><geometry>{geom(kind, size)}</geometry></collision>\n'
    return s

def model(name, pose, parts, comment=None, dyn=None):
    """dyn = (mass, ixx, iyy, izz, com_z) makes the model a free rigid body (it can be pushed, tipped over, rolled)."""
    assert name not in names, name
    names.add(name)
    c = f"    <!-- {comment} -->\n" if comment else ""
    p = " ".join(str(v) for v in pose)
    if dyn:
        m, ix, iy, iz, cz = dyn
        inertial = (f'      <inertial><pose>0 0 {cz} 0 0 0</pose><mass>{m}</mass><inertia><ixx>{ix}</ixx><iyy>{iy}</iyy><izz>{iz}</izz>'
                    f'<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>\n')
        static = "false"
    else:
        inertial, static = "", "true"
    out.append(f'{c}    <model name="{name}">\n      <static>{static}</static>\n      <pose>{p}</pose>\n      <link name="link">\n{inertial}{"".join(parts)}      </link>\n    </model>\n')

# ------------------------------------------------------------------ colours
GRASS=(0.30,0.55,0.22); ASPHALT=(0.16,0.16,0.18); WHITE=(0.92,0.92,0.92); CURB=(0.75,0.75,0.72)
TRUNK=(0.38,0.25,0.13); LEAF1=(0.15,0.45,0.15); LEAF2=(0.12,0.40,0.14); LEAF3=(0.22,0.52,0.18)
GLASS=(0.25,0.38,0.55); ROOF=(0.25,0.25,0.30); DOOR=(0.35,0.22,0.12); BLUE=(0.12,0.28,0.75)
METAL=(0.25,0.27,0.30); WOOD=(0.55,0.38,0.20); LAMP=(1.0,0.95,0.7)

# ------------------------------------------------------------------ ground areas (visual only)
model("grass", (12, 9, 0.01, 0, 0, 0), [part("v","box",(48,9,0.02),(0,0,0,0,0,0),GRASS)], "Grass strip left of the sidewalk (visual only)")
model("road", (12, -11, 0.005, 0, 0, 0), [part("v","box",(48,10,0.01),(0,0,0,0,0,0),ASPHALT)], "Road on the right (visual only)")
model("curb", (12, -6.1, 0.06, 0, 0, 0), [part("v","box",(48,0.3,0.12),(0,0,0,0,0,0),CURB)], "Curb between sidewalk and road")

dash = []
for i, x in enumerate(range(-8, 36, 4)):
    dash.append(part(f"d{i}","box",(2,0.15,0.012),(x-12,0,0.0,0,0,0),(0.95,0.85,0.2)))
model("road_centre_line", (12, -11, 0.012, 0, 0, 0), dash, "Dashed centre line")

zebra = []
for i in range(8):
    zebra.append(part(f"z{i}","box",(3.0,0.5,0.012),(0, -3.5 + i*1.0, 0,0,0,0),WHITE))
model("crosswalk", (16, -11, 0.012, 0, 0, 0), zebra, "Zebra crossing")

# ------------------------------------------------------------------ buildings with windows
def building(name, cx, cy, w, d, h, wall, win_cols, door=True, sign=False, comment=None):
    front = cy - d/2
    parts = [part("walls","box",(w,d,h),(0,0,h/2,0,0,0),wall,collide=True),
             part("roof","box",(w+0.6,d+0.6,0.25),(0,0,h+0.12,0,0,0),ROOF)]
    rows = [1.9 + 1.7*k for k in range(int((h-2.2)//1.7)+1) if 1.9+1.7*k < h-0.6]
    for r, z in enumerate(rows):
        for c, wx in enumerate(win_cols):
            if door and r == 0 and abs(wx) < 1.2:   # leave the door free
                continue
            parts.append(part(f"win_{r}_{c}","box",(0.9,0.05,1.1),(wx,-d/2-0.02,z,0,0,0),GLASS))
    if door:
        parts.append(part("door","box",(1.4,0.06,2.2),(0,-d/2-0.03,1.1,0,0,0),DOOR))
    if sign:
        parts.append(part("blue_sign","box",(w,0.08,0.6),(0,-d/2-0.05,2.6,0,0,0),BLUE))
    model(name, (cx, cy, 0, 0, 0, 0), parts, comment)

building("building_post", 14, 17, 10, 6, 5, (0.62,0.70,0.86), [-4,-2,0,2,4], sign=True, comment="Building with a blue sign (centre)")
building("building_left", 0, 17, 8, 6, 7, (0.80,0.62,0.45), [-3,-1,1,3], comment="Taller brown building")
building("building_right", 28, 17, 9, 6, 4, (0.90,0.90,0.86), [-3,-1,1,3], comment="Low white building")

# ------------------------------------------------------------------ trees and bushes
def tree(name, x, y, trunk_h, leaf_r, leaf_c):
    model(name, (x, y, 0, 0, 0, 0), [
        part("trunk","cyl",(0.16,trunk_h),(0,0,trunk_h/2,0,0,0),TRUNK,collide=True),
        part("leaves","sph",(leaf_r,),(0,0,trunk_h+leaf_r*0.6,0,0,0),leaf_c),
        part("leaves2","sph",(leaf_r*0.7,),(0.5*leaf_r,0.3*leaf_r,trunk_h+leaf_r*1.0,0,0,0),leaf_c)])

for i,(x,y,th,lr,lc) in enumerate([(3,6,2.0,1.2,LEAF1),(9,5.6,2.2,1.3,LEAF2),(16,6.2,2.0,1.2,LEAF3),(22,5.8,2.4,1.5,LEAF1),(29,6.0,2.1,1.3,LEAF2),(-4,6.0,2.2,1.3,LEAF3)]):
    tree(f"tree_{i+1}", x, y, th, lr, lc)

bushes = [part(f"b{i}","sph",(0.55+0.1*(i%3),),(x,0,0.45,0,0,0),LEAF2 if i%2 else LEAF1) for i,x in enumerate(range(-8,16,2))]
model("bushes", (14, 13.2, 0, 0, 0, 0), bushes, "Row of bushes in front of the buildings")

# ------------------------------------------------------------------ street furniture
def lamp(name, x, y):
    model(name, (x, y, 0, 0, 0, 0), [
        part("pole","cyl",(0.06,4.0),(0,0,2.0,0,0,0),METAL,collide=True),
        part("arm","box",(0.6,0.06,0.06),(0.25,0,4.0,0,0,0),METAL),
        part("lamp","sph",(0.18,),(0.5,0,3.92,0,0,0),LAMP)])
for i, x in enumerate([2, 12, 22, 32]):
    lamp(f"lamp_{i+1}", x, -4.9)

def bench(name, x, y, yaw):
    model(name, (x, y, 0, 0, 0, yaw), [
        part("seat","box",(1.5,0.45,0.08),(0,0,0.45,0,0,0),WOOD,collide=True),
        part("back","box",(1.5,0.06,0.45),(0,0.2,0.75,0.15,0,0),WOOD),
        part("leg1","box",(0.08,0.4,0.45),(-0.65,0,0.22,0,0,0),METAL),
        part("leg2","box",(0.08,0.4,0.45),(0.65,0,0.22,0,0,0),METAL)])
bench("bench_1", 10, 3.9, 3.14159)
bench("bench_2", 20, 3.9, 3.14159)

model("mailbox", (7, -3.9, 0, 0, 0, 0), [
    part("body","box",(0.45,0.35,1.1),(0,0,0.55,0,0,0),BLUE,collide=True),
    part("slot","box",(0.3,0.02,0.06),(0,-0.18,0.9,0,0,0),(0.05,0.05,0.1))], "Blue mailbox")
model("trash_can", (13, -3.9, 0, 0, 0, 0), [part("can","cyl",(0.25,0.8),(0,0,0.4,0,0,0),(0.2,0.45,0.25),collide=True)], "Trash can")
model("cardboard_box", (6, 1.8, 0.25, 0, 0, 0.3), [part("box","box",(0.5,0.5,0.5),(0,0,0,0,0,0),(0.60,0.45,0.25),collide=True)], "Cardboard box (free body, 1 kg)", dyn=(1.0, 0.042, 0.042, 0.042, 0))
model("planter", (-3, -3.6, 0, 0, 0, 0), [
    part("pot","box",(0.8,0.8,0.5),(0,0,0.25,0,0,0),(0.55,0.3,0.2),collide=True),
    part("plant","sph",(0.45,),(0,0,0.8,0,0,0),LEAF3)], "Flower planter")


# ------------------------------------------------------------------ obstacle course around the start (lidar targets)
ORANGE=(1.0,0.45,0.05); CONC=(0.62,0.62,0.6); RED=(0.7,0.12,0.1); YEL=(0.9,0.75,0.1)
def cone(name, x, y):
    model(name, (x, y, 0, 0, 0, 0), [
        part("base","box",(0.34,0.34,0.03),(0,0,0.015,0,0,0),(0.1,0.1,0.1)),
        part("c1","cyl",(0.13,0.30),(0,0,0.18,0,0,0),ORANGE,collide=True),
        part("c2","cyl",(0.08,0.30),(0,0,0.46,0,0,0),ORANGE),
        part("stripe","cyl",(0.105,0.07),(0,0,0.34,0,0,0),WHITE)],
        "Traffic cone (free body: the robot can knock it over)", dyn=(0.4, 0.014, 0.014, 0.0034, 0.17))
for i in range(6):                      # slalom of traffic cones straight ahead
    cone(f"cone_{i+1}", 2.0 + i*1.2, 0.55 if i % 2 == 0 else -0.55)
def barrier(name, x, y, yaw):
    model(name, (x, y, 0, 0, 0, yaw), [
        part("b","box",(1.6,0.35,0.7),(0,0,0.35,0,0,0),CONC,collide=True),
        part("s","box",(1.6,0.36,0.12),(0,0,0.55,0,0,0),YEL)])
barrier("barrier_1", 3.0, -2.0, 0.0)
barrier("barrier_2", 8.0, 1.2, 0.5)
barrier("barrier_3", 1.5, 2.6, 1.5708)
def barrel(name, x, y, c):
    model(name, (x, y, 0, 0, 0, 0), [part("b","cyl",(0.3,0.9),(0,0,0.45,0,0,0),c,collide=True),
                                      part("r","cyl",(0.31,0.06),(0,0,0.6,0,0,0),WHITE)],
          "Barrel (free body, 3 kg)", dyn=(3.0, 0.27, 0.27, 0.135, 0.45))
barrel("barrel_1", -2.5, 1.5, RED); barrel("barrel_2", -2.5, -1.2, RED); barrel("barrel_3", 5.5, -1.5, (0.1,0.35,0.7))
def pillar(name, x, y):
    model(name, (x, y, 0, 0, 0, 0), [part("p","cyl",(0.25,2.5),(0,0,1.25,0,0,0),CONC,collide=True)])
pillar("pillar_1", 4.0, 0.0); pillar("pillar_2", -3.5, 3.0)
def fence(name, x, y, yaw, n=6):
    parts = []
    for i in range(n):
        parts.append(part(f"post{i}","box",(0.08,0.08,1.0),(i*0.9,0,0.5,0,0,0),WOOD,collide=True))
    parts.append(part("rail1","box",(n*0.9,0.04,0.08),((n-1)*0.45,0,0.8,0,0,0),WOOD))
    parts.append(part("rail2","box",(n*0.9,0.04,0.08),((n-1)*0.45,0,0.4,0,0,0),WOOD))
    model(name, (x, y, 0, 0, 0, yaw), parts)
fence("fence_1", 13, 4.6, 0); fence("fence_2", 20, 4.6, 0)
# a small kiosk and a bus stop on the pavement
model("kiosk", (17, -3.6, 0, 0, 0, 0), [
    part("walls","box",(2.0,1.4,2.2),(0,0,1.1,0,0,0),(0.85,0.7,0.3),collide=True),
    part("roof","box",(2.3,1.7,0.15),(0,0,2.28,0,0,0),RED),
    part("win","box",(1.2,0.02,0.8),(0,0.71,1.3,0,0,0),GLASS)], "Small kiosk")
model("bus_stop", (24, -3.7, 0, 0, 0, 0), [
    part("roof","box",(3.0,1.2,0.1),(0,0,2.4,0,0,0),METAL),
    part("p1","box",(0.08,0.08,2.4),(-1.4,-0.5,1.2,0,0,0),METAL,collide=True),
    part("p2","box",(0.08,0.08,2.4),(1.4,-0.5,1.2,0,0,0),METAL,collide=True),
    part("back","box",(2.8,0.04,1.3),(0,-0.55,1.3,0,0,0),GLASS),
    part("bench","box",(1.8,0.4,0.08),(0,0,0.45,0,0,0),WOOD,collide=True)], "Bus stop")


# ------------------------------------------------------------------ physics playground (free bodies and slopes)
WOODC=(0.72,0.55,0.32)
crate_k = 0
for (cx, cy, cz) in [(11,-1.6,0.2),(11,-2.2,0.2),(11,-1.9,0.6),(12.2,1.0,0.2)]:
    crate_k += 1
    model(f"crate_{crate_k}", (cx, cy, cz, 0, 0, 0.1*crate_k), [part("c","box",(0.4,0.4,0.4),(0,0,0,0,0,0),WOODC,collide=True)],
          "Wooden crate (free body, 1 kg): knock the stack over", dyn=(1.0, 0.027, 0.027, 0.027, 0))
for i,(bx,by,bc) in enumerate([(1.3,0.9,(0.9,0.2,0.2)),(1.3,-0.9,(0.2,0.4,0.9)),(5.2,0.0,(0.95,0.85,0.2))]):
    model(f"ball_{i+1}", (bx, by, 0.2, 0, 0, 0), [part("b","sph",(0.2,),(0,0,0,0,0,0),bc,collide=True)],
          "Ball (free body, rolls when pushed)", dyn=(0.5, 0.008, 0.008, 0.008, 0))
model("ramp", (14.5, 0.5, 0, 0, 0, 0), [
    part("slope","box",(2.4,1.4,0.06),(0,0,0.18,0,-0.17,0),(0.55,0.5,0.4),collide=True),
    part("top","box",(1.0,1.4,0.06),(1.65,0,0.38,0,0,0),(0.55,0.5,0.4),collide=True),
    part("down","box",(2.4,1.4,0.06),(3.33,0,0.18,0,0.17,0),(0.55,0.5,0.4),collide=True)], "Small ramp: up (10 degrees), flat top, down again")
for i,x in enumerate([-4.0, -5.5, -7.0]):
    model(f"speed_bump_{i+1}", (x, 0, 0, 0, 0, 0), [part("b","cyl",(0.07,2.4),(0,0,0.0,1.5708,0,0),(0.9,0.75,0.1),collide=True)], "Speed bump: a bump for the wheels")

# ------------------------------------------------------------------ cars on the road
def car(name, x, y, yaw, body):
    model(name, (x, y, 0, 0, 0, yaw), [
        part("body","box",(4.2,1.8,0.7),(0,0,0.65,0,0,0),body,collide=True),
        part("cabin","box",(2.2,1.6,0.6),(-0.2,0,1.3,0,0,0),(0.2,0.3,0.4)),
        part("w1","cyl",(0.32,0.25),(1.3,0.9,0.32,1.5708,0,0),(0.05,0.05,0.05)),
        part("w2","cyl",(0.32,0.25),(1.3,-0.9,0.32,1.5708,0,0),(0.05,0.05,0.05)),
        part("w3","cyl",(0.32,0.25),(-1.3,0.9,0.32,1.5708,0,0),(0.05,0.05,0.05)),
        part("w4","cyl",(0.32,0.25),(-1.3,-0.9,0.32,1.5708,0,0),(0.05,0.05,0.05))])
car("car_red", 8, -8.6, 0, (0.75,0.12,0.12))
car("car_silver", 26, -13.4, 3.14159, (0.72,0.74,0.78))

# ------------------------------------------------------------------ people (Gazebo "actors")
def waypoints(pts):
    return "\n".join(f'          <waypoint><time>{t}</time><pose>{x} {y} {z} 0 0 {yaw}</pose></waypoint>' for t,x,y,z,yaw in pts)

MAN = "https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae"
WOMAN = "https://fuel.gazebosim.org/1.0/Luca/models/FemaleVisitorWalk/tip/files/meshes/FemaleVisitorWalk.dae"

def actor(name, mesh, anim, pts, comment, tension="0.6"):
    return f'''    <!-- {comment} -->
    <actor name="{name}">
      <skin><filename>{mesh}</filename><scale>1.0</scale></skin>
      <animation name="{anim}">
        <filename>{mesh}</filename>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>0.0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="{anim}" tension="{tension}">
{waypoints(pts)}
        </trajectory>
      </script>
    </actor>
'''

# Man crosses the sidewalk at x = 4.5 m, 1 m/s, turns around at both ends.
man_pts = [(0,4.5,-3.2,1.0,1.57),(6.4,4.5,3.2,1.0,1.57),(6.9,4.5,3.2,1.0,-1.57),(13.3,4.5,-3.2,1.0,-1.57),(13.8,4.5,-3.2,1.0,1.57)]
# Woman walks along the sidewalk at y = 2.8 m from x = 7 to x = 15 and back (past the bench).
WZ = 0.0   # woman mesh has its feet at z=0; men use 1.0. Tune with: ros2 run mobile_robot_gazebo tune_actor_height.py <z>
woman_pts = [(0,7,2.8,WZ,0.0),(8,15,2.8,WZ,0.0),(8.5,15,2.8,WZ,3.14),(16.5,7,2.8,WZ,3.14),(17,7,2.8,WZ,0.0)]

people = actor("pedestrian_man", MAN, "walk", man_pts, "Man: standard Gazebo walking actor, crosses in front of the robot") + "\n" + \
         actor("woman_walking", WOMAN, "walk", woman_pts, "Woman walking along the sidewalk (Fuel: Luca/FemaleVisitorWalk). If she floats or sinks, change the z=1.0 values below (try 0.9 to 1.1).")

man2_pts = [(0,-1,-4.0,1.0,0.0),(7,6,-4.0,1.0,0.0),(7.5,6,-4.0,1.0,1.57),(12,6,3.0,1.0,1.57),(12.5,6,3.0,1.0,3.14),(19.5,-1,3.0,1.0,3.14),(20,-1,3.0,1.0,-1.57),(24.5,-1,-4.0,1.0,-1.57),(25,-1,-4.0,1.0,0.0)]
woman2_pts = [(0,20,-2.0,WZ,3.14),(10,10,-2.0,WZ,3.14),(10.5,10,-2.0,WZ,0.0),(20.5,20,-2.0,WZ,0.0),(21,20,-2.0,WZ,3.14)]
woman3_pts = [(0,0.5,-5.0,WZ,0.0),(6,6.5,-5.0,WZ,0.0),(6.5,6.5,-5.0,WZ,3.14),(12.5,0.5,-5.0,WZ,3.14),(13,0.5,-5.0,WZ,0.0)]
people += "\n" + actor("pedestrian_man_2", MAN, "walk", man2_pts, "Second man: patrols a big rectangle around the obstacle course") + "\n" + \
          actor("woman_walking_2", WOMAN, "walk", woman2_pts, "Second woman: walks along the kerb side") + "\n" + \
          actor("woman_walking_3", WOMAN, "walk", woman3_pts, "Third woman: walks next to the kerb in front of the robot")

standing = '''    <!-- A woman standing near the mailbox (Fuel: OpenRobotics "Casual female"; standing, not animated) -->
    <include>
      <name>woman_standing</name>
      <pose>9.5 -2.6 0 0 0 1.57</pose>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Casual female</uri>
    </include>
'''

header = '''<?xml version="1.0" ?>
<!--
  Street world for Gazebo Harmonic (gz-sim 8). Generated layout:
    sidewalk (ground, grey) | grass + trees + 3 buildings on the left | curb + road with crosswalk and cars on the right
    lamp posts, benches, mailbox, trash can, planter, a cardboard box
    people: a man crossing at x=4.5 m, a woman walking along the sidewalk (x 7..15 m, y=2.8 m), three women walking (Fuel FemaleVisitorWalk)
    obstacle course near the start: traffic-cone slalom, concrete barriers, barrels, pillars (give the lidar plenty to hit)
    2 more walking people, kiosk, bus stop, fences

  - The robot spawns at the origin facing +x.
  - People are Gazebo "actors": animated meshes without physics. No collision (the robot can drive through
    them) but the camera and the lidar DO see them. They are downloaded from Gazebo Fuel the first time
    (internet needed, can take a minute, cached in ~/.gz/fuel). If a download fails the world still loads.
  - Static with collision: ground, trees, lamps, benches, buildings, cars, barriers, pillars, fences, ramp, speed bumps.
  - FREE BODIES (real physics, the robot can push / topple them): traffic cones, barrels, crates, balls, cardboard box.
  - The Sensors system below is required, otherwise camera and lidar never render.
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
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.62 0.78 0.95 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.95 0.95 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <!-- Ground = grey sidewalk (the only big collision surface, so the robot sits on it) -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material>
            <ambient>0.62 0.62 0.62 1</ambient>
            <diffuse>0.62 0.62 0.62 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

'''
footer = "\n  </world>\n</sdf>\n"
open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "worlds", "street_world.sdf"), "w").write(header + "".join(out) + "\n" + people + "\n" + footer)
print("models:", len(names))
