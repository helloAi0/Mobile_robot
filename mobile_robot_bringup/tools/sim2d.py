#!/usr/bin/env python3
"""2D kinematic simulator of the robot in the house, running the REAL control code of this repository.

   python3 tools/sim2d.py --seed 3 --duration 300          # one run, prints the metrics

What is real (imported, not re-implemented):
   auto_explore.Explorer   the autopilot
   cmd_vel_mux             select / front_distance / guard / limit_step: the safety layer and the limits
   scan_degrader           pillar mask, dropouts, range-bias drift (the same sensor degradation as in Gazebo)
   house_map               the walls, furniture, pillars, props, walking people and the wet patch of house_world.sdf

What is modelled: a lidar by ray casting (noise from the URDF), first-order wheel response (tau 0.08 s), wheel slip on the
wet tile (see SLIP), wheel encoders and an IMU gyro with the noise numbers of the URDF, the planar EKF of
ekf_model.py. There is no contact dynamics: a move into a wall is refused and counted as a contact.

This is a model, not Gazebo: the numbers it produces say how the *control logic* behaves; the Gazebo backend of
tools/benchmark.py measures the full physical simulation.
"""
import math
import os
import random
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'mobile_robot_bringup', 'scripts'))
sys.path.insert(0, os.path.join(REPO, 'mobile_robot_gazebo', 'scripts'))

import yaml                     # noqa: E402

import auto_explore            # noqa: E402
import benchmark_metrics as bm  # noqa: E402
import cmd_vel_mux as mux       # noqa: E402
import ekf_model                # noqa: E402
import house_map                # noqa: E402
import imu_covariance           # noqa: E402
import scan_degrader            # noqa: E402

CFG = os.path.join(REPO, 'mobile_robot_bringup', 'config')

DT = 0.05                       # control tick of the mux and the autopilot (20 Hz)
SCAN_EVERY = 2                  # lidar at 10 Hz
N_BEAMS = 720
ANGLE_MIN = -3.14159
INC = 2 * 3.14159 / (N_BEAMS - 1)
TAU = 0.08                      # wheel speed response, s

# Mux parameters = the defaults of cmd_vel_mux.py / autonomy.launch.py
MUX = {'stop_dist': 0.38, 'slow_dist': 1.0, 'require_scan': True, 'max_linear': 0.6, 'max_angular': 2.0,
       'max_accel': 1.2, 'max_ang_accel': 4.0, 'rear_stop_dist': 0.3}
CONE_DEG = 28.0

# Wheel slip on the wet tile (mu 0.15). Drawn again at every visit of the patch. The wheels turn as commanded (so the
# encoders report the full speed) but the body moves less and drifts sideways: that is what makes odometry wrong.
SLIP = {'lin': (0.15, 0.45),      # fraction of the wheel speed that is lost (uniform)
        'ang': (0.0, 0.30),       # fraction of the commanded turn rate that is lost
        'yaw_per_m': 0.15}        # rad of unwanted yaw per metre driven (normal, std)

# sensor noise = the numbers in mobile_robot_description/urdf/imu.xacro and the encoder resolution
GYRO_STD, GYRO_BIAS_STD = 0.0009 * math.sqrt(100.0), 0.0008       # rad/s per sample at the EKF rate (100 Hz noise density -> 20 Hz)
ENC_V_STD, ENC_W_STD, ENC_SCALE_STD = 0.004, 0.012, 0.01


class Run:
    def __init__(self, seed=0, duration=300.0, spawn=None, people=True, slip=True, house=None, degrade=True):
        self.h = house or house_map.load()
        self.rng = random.Random(seed)
        self.np = np.random.default_rng(seed)
        self.duration = duration
        self.people_on = people
        self.slip_on = slip
        self.degrade = degrade
        self.spawn = spawn or self.random_spawn()
        x, y, yaw = self.spawn
        self.x, self.y, self.yaw = x, y, yaw
        self.v = self.w = 0.0                # true body velocity
        self.wv = self.ww = 0.0              # wheel-implied velocity (what the encoders see)
        self.cur = (0.0, 0.0)                # mux output state
        self.ex = auto_explore.Explorer(rng=random.Random(seed + 1))
        self.scan = None
        self.front = self.rear = 12.0
        self.scan_t = -10.0
        self.bias = 0.0                      # lidar range bias random walk
        self.gyro_bias = self.np.normal(0.0, GYRO_BIAS_STD)
        self.enc_scale = 1.0 + self.np.normal(0.0, ENC_SCALE_STD)
        # estimators (start aligned with the truth, like a robot that has just been switched on)
        self.odom = [0.0, 0.0, 0.0]          # wheel odometry in its own frame (starts at 0)
        self.ekf = ekf_model.PlanarEKF.from_yaml(os.path.join(CFG, 'ekf.yaml'))
        self.ekf.reset(0.0, 0.0, 0.0)
        self.slip_state = None
        self.on_patch = False
        self.patch_visits = 0
        self.err = {'odom': [], 'ekf': []}   # (pos_err, yaw_err) per sample
        self.acc = bm.RunAccumulator()
        self.reason = ''

    # ------------------------------------------------------------------ setup
    def random_spawn(self):
        free = self.h.reachable((0.0, 0.0), inflate=0.45)
        ii, jj = np.nonzero(free)
        k = self.rng.randrange(len(ii))
        x = self.h.x0 + (ii[k] + 0.5) * self.h.res
        y = self.h.y0 + (jj[k] + 0.5) * self.h.res
        return (float(x), float(y), self.rng.uniform(-math.pi, math.pi))

    # ------------------------------------------------------------------ pieces of one tick
    def people(self, t):
        return self.h.person_positions(t) if self.people_on else []

    def sense(self, t):
        ppl = self.people(t)
        r = self.h.scan(self.x, self.y, self.yaw, N_BEAMS, people=ppl)
        r = r + self.np.normal(0.0, 0.012, r.shape) * np.isfinite(r)          # Gaussian noise, std 0.012 m (lidar.xacro)
        r = [float(v) for v in r]
        if self.degrade:
            self.bias = scan_degrader.step_bias(self.bias, 0.0005, DT * SCAN_EVERY, 0.03, self.rng)
            p = {'sectors': [(137.0, 17.0), (-137.0, 17.0)], 'dropout_prob': 0.01}
            r = scan_degrader.degrade(r, ANGLE_MIN, INC, 0.15, 12.0, self.bias, p, self.rng)
        else:
            r = scan_degrader.mask_sectors(r, ANGLE_MIN, INC, [(137.0, 17.0), (-137.0, 17.0)])
        self.scan = r
        self.front = mux.front_distance(r, ANGLE_MIN, INC, CONE_DEG, 12.0)
        self.rear = mux.front_distance(r, ANGLE_MIN, INC, CONE_DEG, 12.0, center_deg=180.0)
        self.scan_t = t

    def drive(self, t):
        """Autopilot -> mux safety layer -> limits. Returns True if the safety layer is holding the robot back."""
        lin, ang = 0.0, 0.0
        if self.scan is not None and t - self.scan_t < 1.0:
            lin, ang = self.ex.step(self.scan, ANGLE_MIN, INC, t, (self.odom[0], self.odom[1]))
        lin, ang, reason = mux.guard(lin, ang, self.front, t - self.scan_t < 1.0, MUX, self.rear)
        brake = reason in ('obstacle', 'obstacle-behind', 'no-scan')
        self.cur = mux.limit_step(self.cur, (lin, ang), DT, MUX, brake)
        self.reason = reason
        return reason in ('obstacle', 'obstacle-behind') or self.ex.state in ('STOP',)

    def plant(self, t):
        """Wheel response, slip, motion with contact. Returns the true yaw rate."""
        a = DT / (TAU + DT)
        self.wv += (self.cur[0] - self.wv) * a
        self.ww += (self.cur[1] - self.ww) * a
        v, w = self.wv, self.ww
        mu = self.h.slip_factor(self.x, self.y) if self.slip_on else None
        slipping = mu is not None and mu < 0.5
        if slipping and not self.on_patch:
            self.patch_visits += 1
            self.slip_state = (self.rng.uniform(*SLIP['lin']), self.rng.uniform(*SLIP['ang']), self.np.normal(0.0, SLIP['yaw_per_m']))
        self.on_patch = slipping
        if slipping:
            sl, sa, yaw_m = self.slip_state
            v, w = v * (1 - sl), w * (1 - sa) + yaw_m * v
        self.v, self.w = v, w
        yaw = self.yaw + w * DT
        x = self.x + v * math.cos(self.yaw + w * DT / 2) * DT
        y = self.y + v * math.sin(self.yaw + w * DT / 2) * DT
        blocked = self.h.clearance(x, y, yaw) <= 0.0
        if blocked:                       # no penetration: the wall stops the robot
            self.v = 0.0
            self.wv = 0.0
            x, y, yaw = self.x, self.y, self.yaw + (w * DT if self.h.clearance(self.x, self.y, self.yaw + w * DT) > 0 else 0.0)
        self.x, self.y, self.yaw = x, y, yaw
        return blocked

    def estimate(self, w_true):
        """Encoders and gyro -> wheel odometry and EKF."""
        v_enc = self.wv * self.enc_scale + self.np.normal(0.0, ENC_V_STD)
        w_enc = self.ww * self.enc_scale + self.np.normal(0.0, ENC_W_STD)
        ox, oy, oth = self.odom
        self.odom = [ox + v_enc * math.cos(oth + w_enc * DT / 2) * DT, oy + v_enc * math.sin(oth + w_enc * DT / 2) * DT, oth + w_enc * DT]
        gyro = w_true + self.gyro_bias + self.np.normal(0.0, GYRO_STD)
        self.ekf.predict(DT)
        self.ekf.update_wheel(v_enc, w_enc, ENC_V_VAR, ENC_W_VAR)
        self.ekf.update_gyro(gyro, GYRO_VAR)

    def score_estimators(self):
        """Pose error of each estimator against the truth, after aligning its start frame with the spawn pose."""
        sx, sy, sa = self.spawn
        c, s = math.cos(sa), math.sin(sa)
        for name, p in (('odom', self.odom), ('ekf', self.ekf.pose)):
            wx, wy = sx + c * p[0] - s * p[1], sy + s * p[0] + c * p[1]
            self.err[name].append((math.hypot(wx - self.x, wy - self.y), abs(ekf_model.angle_diff(p[2] + sa, self.yaw))))

    # ------------------------------------------------------------------ main loop
    def run(self):
        steps = int(self.duration / DT)
        for k in range(steps):
            t = k * DT
            if k % SCAN_EVERY == 0:
                self.sense(t)
            stopped = self.drive(t)
            blocked = self.plant(t)
            self.estimate(self.w)
            if k % 4 == 0:                                     # trace sample every 0.2 s
                gap = 0.0 if blocked else self.h.clearance(self.x, self.y, self.yaw)
                pgap = self.h.person_gap(self.x, self.y, self.yaw, self.people(t))
                self.acc.update(t, self.x, self.y, self.yaw, gap, stopped=stopped, pgap=pgap)
                self.score_estimators()
        return self.acc.trace(self.spawn, {'backend': 'sim2d', 'seed': None})

    def estimation_summary(self):
        out = {}
        for name, e in self.err.items():
            a = np.array(e)
            out[name] = {'pos_rmse_m': round(float(np.sqrt((a[:, 0] ** 2).mean())), 3), 'pos_final_m': round(float(a[-1, 0]), 3),
                         'yaw_rmse_deg': round(math.degrees(float(np.sqrt((a[:, 1] ** 2).mean()))), 2),
                         'yaw_final_deg': round(math.degrees(float(a[-1, 1])), 2)}
        out['patch_visits'] = self.patch_visits
        return out


# noise variances the EKF is told about = what the real nodes publish: the twist covariance of /diff_cont/odom
# (my_controllers.yaml) and the gyro covariance of /imu/data (imu_covariance.py)
_TWIST = yaml.safe_load(open(os.path.join(CFG, 'my_controllers.yaml')))['diff_cont']['ros__parameters']['twist_covariance_diagonal']
ENC_V_VAR, ENC_W_VAR = _TWIST[0], _TWIST[5]
GYRO_VAR = imu_covariance.GYRO_VAR


if __name__ == '__main__':
    import argparse
    import json
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--duration', type=float, default=300.0)
    a = ap.parse_args()
    r = Run(a.seed, a.duration)
    trace = r.run()
    print(json.dumps({'spawn': trace['spawn'], 'samples': len(trace['samples']), 'collisions_acc': r.acc.collisions,
                      'min_gap': round(r.acc.min_gap, 3), 'distance': round(r.acc.distance, 1), 'estimation': r.estimation_summary()}, indent=1))
