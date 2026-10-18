#!/usr/bin/env python3
"""Independent physics check of the robot description (MuJoCo), no ROS or Gazebo needed.

   pip install mujoco xacro numpy
   python3 mobile_robot_description/tools/physics_check.py            # prints a PASS/FAIL table with measured numbers

It builds the URDF from the xacro, drops the robot on a floor and measures what a real robot would do:
ride height and tilt at rest, suspension sag and settling, straight-line driving, turning in place, acceleration pitch,
top speed and climbing the 7 degree ramp, with the SAME mass/inertia tensors, spring and damper constants, friction values and
the 2 N*m motor limit as the Gazebo model. It cannot prove Gazebo behaves identically (different engine), but it catches the
mistakes that matter: wrong wheel axis signs, a robot that sinks or tips, a spring that is too soft or too stiff, a caster
that drags, motors that are too weak for the ramp.
"""
import math
import os
import re
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DESC = os.path.normpath(os.path.join(HERE, ".."))

DT = 0.002
WHEEL_R, TRACK = 0.05, 0.35
MAX_TORQUE = 2.0                                        # N*m per wheel (robot_core.xacro: wheel_max_torque)
MAX_SPEED = 12.0                                        # rad/s (wheel_max_speed)
MU_DRIVE, MU_CASTER, MU_FLOOR = 1.0, 0.6, 0.9           # robot_core.xacro / house_world.sdf


def _prop(text, name):
    return float(re.search(rf'name="{name}" value="([-\d.eE+]+)"', text).group(1))


class Sim:
    def __init__(self, ramp_deg=None, z0=0.08, xacro_args=None):
        import mujoco
        import xacro
        self.mj = mujoco
        maps = {"hardware": "mock", "hw_plugin": "mock_components/GenericSystem", "hw_device": "/dev/null",
                "hw_baud": "1", "hw_counts_per_rev": "1"}
        maps.update(xacro_args or {})
        urdf = xacro.process_file(os.path.join(DESC, "urdf", "robot.urdf.xacro"), mappings=maps).toxml()
        urdf = urdf.replace("package://mobile_robot_description/meshes/", os.path.join(DESC, "meshes") + "/")
        mass = open(os.path.join(DESC, "urdf", "mass_properties.xacro")).read()
        self.k, self.free_z = _prop(mass, "susp_k"), _prop(mass, "susp_free_z")
        self.total_mass = _prop(mass, "total_mass")
        with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as f:
            f.write(urdf)
            path = f.name
        spec = mujoco.MjSpec.from_file(path)
        os.unlink(path)
        spec.compiler.discardvisual = True              # collisions are what the physics engine sees
        spec.option.timestep = DT
        base = spec.body("base_link")
        base.add_freejoint()
        base.pos = [0, 0, z0]
        for g in spec.geoms:                            # friction as in the Gazebo model (sliding, torsional, rolling)
            owner = g.parent.name if hasattr(g, "parent") else ""
            mu = MU_DRIVE if "wheel" in owner and "caster" not in owner else (MU_CASTER if "caster" in owner else 0.5)
            g.friction = [mu, 0.005, 0.0001]
            g.solref = [0.01, 1.0]                      # contact time constant >= 2 * timestep, else the solver chatters
        world = spec.worldbody
        world.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[30, 30, 0.1], pos=[0, 0, 0],
                       friction=[MU_FLOOR, 0.005, 0.0001], solref=[0.01, 1.0], name="floor")
        if ramp_deg is not None:                        # 7 degree ramp, top surface starts at x = 1.0 m on the floor (no lip)
            a = math.radians(ramp_deg)
            length, thick = 3.0, 0.06
            c = [1.0 + length / 2 * math.cos(a) + thick / 2 * math.sin(a), 0, length / 2 * math.sin(a) - thick / 2 * math.cos(a)]
            quat = [math.cos(-a / 2), 0, math.sin(-a / 2), 0]       # pitch up towards +x
            world.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[length / 2, 0.6, thick / 2], pos=c, quat=quat,
                           friction=[MU_FLOOR, 0.005, 0.0001], solref=[0.01, 1.0], name="ramp")
        self.spec = spec
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        m = self.model
        jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)  # noqa: E731
        self.dof = {n: m.jnt_dofadr[jid(n)] for n in ("left_wheel_joint", "right_wheel_joint", "left_carrier_joint",
                                                      "right_carrier_joint", "caster_swivel_joint")}
        self.qadr = {n: m.jnt_qposadr[jid(n)] for n in self.dof}
        self.base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.carrier_lo = float(m.jnt_range[jid("left_carrier_joint")][0])
        self.carrier_hi = float(m.jnt_range[jid("left_carrier_joint")][1])
        self.cmd = [0.0, 0.0]                           # wheel speed targets, rad/s (left, right)
        self.applied = [0.0, 0.0]                       # after the acceleration limit (the mux does this on the robot)
        self.accel_limit = 1.2 / WHEEL_R                # rad/s^2  = 1.2 m/s^2 (cmd_vel_mux max_accel)
        self.t = 0.0
        self.log = []

    # ------------------------------------------------------------------ one control + physics step
    def step(self):
        d, dof = self.data, self.dof
        d.qfrc_applied[:] = 0
        for side, w in (("left", 0), ("right", 1)):
            wd = dof[f"{side}_wheel_joint"]
            if self.accel_limit:
                self.applied[w] += float(np.clip(self.cmd[w] - self.applied[w], -self.accel_limit * DT, self.accel_limit * DT))
            else:
                self.applied[w] = self.cmd[w]
            tau = 3.0 * (self.applied[w] - d.qvel[wd])  # ideal velocity servo with the real torque limit
            d.qfrc_applied[wd] = float(np.clip(tau, -MAX_TORQUE, MAX_TORQUE))
            cd = dof[f"{side}_carrier_joint"]
            q = d.qpos[self.qadr[f"{side}_carrier_joint"]]
            d.qfrc_applied[cd] = -self.k * (q - self.free_z)     # spring (the damper is the joint's own <dynamics damping>)
        self.mj.mj_step(self.model, d)
        self.t += DT

    def run(self, seconds, record=True):
        for _ in range(int(seconds / DT)):
            self.step()
            if record:
                self.log.append(self.state())

    def state(self):
        d = self.data
        R = d.xmat[self.base].reshape(3, 3)
        pitch = math.degrees(math.asin(-R[2, 0]))
        roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
        yaw = math.atan2(R[1, 0], R[0, 0])
        return dict(t=self.t, x=d.xpos[self.base][0], y=d.xpos[self.base][1], z=d.xpos[self.base][2],
                    yaw=yaw, pitch=pitch, roll=roll,
                    ql=d.qpos[self.qadr["left_carrier_joint"]], qr=d.qpos[self.qadr["right_carrier_joint"]],
                    wl=d.qvel[self.dof["left_wheel_joint"]], wr=d.qvel[self.dof["right_wheel_joint"]])


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# ---------------------------------------------------------------------- scenarios
def check_rest():
    s = Sim()
    s.run(3.0)
    last = s.log[-1]
    tail = s.log[-500:]
    return dict(z=last["z"], pitch=last["pitch"], roll=last["roll"], sag_l=last["ql"], sag_r=last["qr"],
                jitter=max(r["z"] for r in tail) - min(r["z"] for r in tail),
                lower=s.carrier_lo, mass=s.total_mass)


def check_straight():
    s = Sim()
    s.run(2.0, record=False)
    st0 = s.state()
    w = 0.2 / WHEEL_R
    s.cmd = [w, w]
    s.run(5.0)
    st1 = s.state()
    return dict(dx=st1["x"] - st0["x"], dy=st1["y"] - st0["y"], dyaw=math.degrees(wrap(st1["yaw"] - st0["yaw"])),
                expect=0.2 * 5.0, max_pitch=max(abs(r["pitch"]) for r in s.log), max_roll=max(abs(r["roll"]) for r in s.log))


def check_turn():
    s = Sim()
    s.run(2.0, record=False)
    st0 = s.state()
    w = 0.2 / WHEEL_R
    s.cmd = [-w, w]                                     # right wheel forward, left backward: counter-clockwise (+z)
    s.run(2.0)
    st1 = s.state()
    rate = wrap(st1["yaw"] - st0["yaw"]) / 2.0          # 2 s * 1.14 rad/s = 2.3 rad < pi: no wrap-around
    return dict(rate=rate, expect=2 * 0.2 / TRACK, drift=math.hypot(st1["x"] - st0["x"], st1["y"] - st0["y"]))


def check_accel_and_top_speed():
    s = Sim()
    s.run(2.0, record=False)
    s.cmd = [MAX_SPEED, MAX_SPEED]
    s.run(4.0)
    st = s.log[-1]
    v = (s.log[-1]["x"] - s.log[-251]["x"]) / (250 * DT)
    return dict(v=v, expect=MAX_SPEED * WHEEL_R, max_pitch=max(abs(r["pitch"]) for r in s.log),
                min_q=min(min(r["ql"], r["qr"]) for r in s.log), max_q=max(max(r["ql"], r["qr"]) for r in s.log),
                lo=s.carrier_lo, hi=s.carrier_hi)


def check_ramp():
    s = Sim(ramp_deg=7.0)
    s.run(2.0, record=False)
    w = 0.2 / WHEEL_R
    s.cmd = [w, w]
    s.run(14.0)
    last = s.log[-1]
    on_slope = [abs(r["pitch"]) for r in s.log if r["z"] > 0.08]
    return dict(x=last["x"], z=last["z"], pitch=float(np.median(on_slope)), peak=max(abs(r["pitch"]) for r in s.log),
                dy=last["y"], max_roll=max(abs(r["roll"]) for r in s.log))


def check_estop():
    """Full speed, then the command drops to zero at once (what the e-stop does). The diff_drive controller still limits
    deceleration to 1.8 m/s^2 (my_controllers.yaml). The robot must stop short and must not nose-dive."""
    s = Sim()
    s.run(2.0, record=False)
    s.cmd = [MAX_SPEED, MAX_SPEED]
    s.run(3.0, record=False)
    x0 = s.state()["x"]
    s.cmd = [0.0, 0.0]
    s.accel_limit = 1.8 / WHEEL_R
    s.log = []
    s.run(2.0)
    return dict(dist=s.log[-1]["x"] - x0, max_pitch=max(abs(r["pitch"]) for r in s.log), v_end=s.log[-1]["wl"] * WHEEL_R)


def check_drop():
    """Drop from 5 cm: the damper must settle the body quickly and without bouncing off the floor."""
    s = Sim(z0=0.10)
    s.run(3.0)
    z = np.array([r["z"] for r in s.log])
    final = z[-1]
    settle = next((r["t"] for i, r in enumerate(s.log) if np.all(np.abs(z[i:] - final) < 0.002)), None)
    return dict(settle=settle, overshoot=float(final - z.min()) if z.min() < final else 0.0)


def results():
    rows = []

    def row(name, ok, detail):
        rows.append((bool(ok), name, detail))

    r = check_rest()
    row("rest: ride height", abs(r["z"] - 0.05) < 0.015, f"axle height {r['z']*100:.1f} cm (wheel radius 5.0 cm)")
    row("rest: level", abs(r["pitch"]) < 1.5 and abs(r["roll"]) < 1.0, f"pitch {r['pitch']:.2f} deg, roll {r['roll']:.2f} deg")
    row("rest: suspension not on a stop", r["lower"] + 0.003 < min(r["sag_l"], r["sag_r"]) < 0.019,
        f"carrier at {r['sag_l']*1000:.1f} / {r['sag_r']*1000:.1f} mm, travel {r['lower']*1000:.1f}..20 mm")
    row("rest: no jitter", r["jitter"] < 0.0005, f"height wobble {r['jitter']*1000:.2f} mm over the last second")

    r = check_straight()
    row("drive: moves forward (axis signs)", r["dx"] > 0.8 * r["expect"], f"moved {r['dx']:.2f} m, commanded {r['expect']:.2f} m")
    row("drive: goes straight", abs(r["dy"]) < 0.06 and abs(r["dyaw"]) < 3.0, f"sideways {r['dy']*100:.1f} cm, heading drift {r['dyaw']:.1f} deg")
    row("drive: stays level", r["max_pitch"] < 3.0 and r["max_roll"] < 1.5, f"max pitch {r['max_pitch']:.2f} deg, roll {r['max_roll']:.2f} deg")

    r = check_turn()
    row("turn: correct direction", r["rate"] > 0, f"yaw rate {r['rate']:+.2f} rad/s (positive = counter-clockwise, as diff_drive expects)")
    row("turn: matches kinematics", abs(r["rate"] - r["expect"]) < 0.25 * r["expect"], f"measured {r['rate']:.2f}, expected {r['expect']:.2f} rad/s")
    row("turn: pivots in place (caster)", r["drift"] < 0.12, f"centre moved {r['drift']*100:.1f} cm while turning")

    r = check_accel_and_top_speed()
    row("top speed 0.6 m/s reachable", r["v"] > 0.85 * r["expect"], f"{r['v']:.2f} m/s of {r['expect']:.2f}")
    row("full acceleration: body pitch", r["max_pitch"] < 6.0, f"max pitch {r['max_pitch']:.2f} deg")
    row("full acceleration: suspension travel", r["min_q"] > r["lo"] + 0.002 and r["max_q"] < r["hi"] - 0.002,
        f"carrier range {r['min_q']*1000:.1f}..{r['max_q']*1000:.1f} mm")

    r = check_ramp()
    row("ramp 7 deg: climbs with 2 N*m motors", r["x"] > 2.0 and r["z"] > 0.15, f"reached x={r['x']:.2f} m, axle {r['z']*100:.1f} cm high (flat: 5 cm; 7 deg slope gives ~18 cm after 1.5 m)")
    row("ramp 7 deg: body follows the slope", 6.0 < r["pitch"] < 9.0 and r["peak"] < 13.0 and abs(r["dy"]) < 0.25,
        f"steady pitch {r['pitch']:.1f} deg (slope is 7), peak {r['peak']:.1f} deg on contact bumps, sideways {r['dy']*100:.0f} cm")

    r = check_estop()
    row("e-stop from 0.6 m/s: stops short", r["dist"] < 0.25 and abs(r["v_end"]) < 0.02, f"stopped in {r['dist']*100:.0f} cm")
    row("e-stop from 0.6 m/s: no nose-dive", r["max_pitch"] < 6.0, f"max pitch {r['max_pitch']:.1f} deg")

    r = check_drop()
    row("suspension: settles after a 5 cm drop", r["settle"] is not None and r["settle"] < 2.0 and r["overshoot"] < 0.02,
        f"settled in {r['settle'] if r['settle'] is None else round(r['settle'], 2)} s")
    return rows


def main():
    try:
        rows = results()
    except ImportError as e:
        sys.exit(f"missing dependency: {e}. pip install mujoco xacro numpy")
    width = max(len(n) for _, n, _ in rows)
    for ok, name, detail in rows:
        print(("PASS  " if ok else "FAIL  ") + name.ljust(width) + "  " + detail)
    bad = [n for ok, n, _ in rows if not ok]
    print(f"\n{len(rows) - len(bad)}/{len(rows)} checks passed")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
