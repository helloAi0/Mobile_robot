#!/usr/bin/env python3
"""Benchmark: N autonomous runs in the house, scored on coverage, collisions, minimum clearance and time.

   # fast, reproducible, no ROS and no GPU: the repository's control code in the 2D model of the house (see tools/sim2d.py)
   python3 mobile_robot_bringup/tools/benchmark.py --backend sim2d --runs 20 --duration 300 --seed 0 --write

   # the real thing: Gazebo + SLAM + the whole ROS stack, one fresh simulation per run (needs the workspace built and sourced)
   python3 mobile_robot_bringup/tools/benchmark.py --backend gazebo --runs 5 --duration 300 --out docs/benchmark_gazebo.json

Each run starts at a random free place with a random heading (seeded: run k always starts at the same place), drives with
the autopilot behind the safety layer for `--duration` simulated seconds, and is scored by scripts/benchmark_metrics.py.
`--write` stores docs/benchmark_results.json, the figure docs/figures/benchmark.png and the table in README.md.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'mobile_robot_bringup', 'scripts'))

import benchmark_metrics as bm  # noqa: E402
import house_map                # noqa: E402

START, END = '<!-- benchmark:start -->', '<!-- benchmark:end -->'
GRID = 5                         # coverage grid = 5 x 2 cm = 10 cm cells


class Coverage:
    """Share of the reachable floor within COVER_RADIUS of the robot path, as a function of time."""

    def __init__(self, house, start=(0.0, 0.0), radius=bm.COVER_RADIUS):
        reach = house.reachable(start)
        self.house = house
        self.free = reach[::GRID, ::GRID]
        self.cell = house.res * GRID
        self.total = int(self.free.sum())
        k = int(np.ceil(radius / self.cell))
        di, dj = np.meshgrid(np.arange(-k, k + 1), np.arange(-k, k + 1), indexing='ij')
        m = di ** 2 + dj ** 2 <= (radius / self.cell) ** 2
        self.off = (di[m], dj[m])
        self.area_m2 = self.total * self.cell ** 2

    def curve(self, samples):
        covered = np.zeros_like(self.free)
        out = []
        h = self.house
        nx, ny = self.free.shape
        for t, x, y, *_ in samples:
            ci = int((x - h.x0) / self.cell)
            cj = int((y - h.y0) / self.cell)
            ii = np.clip(self.off[0] + ci, 0, nx - 1)
            jj = np.clip(self.off[1] + cj, 0, ny - 1)
            covered[ii, jj] = True
            out.append((t, float((covered & self.free).sum()) / self.total))
        return out, covered & self.free


def sim2d_run(seed, duration):
    import sim2d
    r = sim2d.Run(seed=seed, duration=duration)
    trace = r.run()
    trace['stops'] = r.acc.stops
    return trace, r.estimation_summary()


def gazebo_run(seed, duration, workdir, gui=False):
    import sim2d
    spawn = sim2d.Run(seed=seed, duration=1).spawn            # same start as the 2D model for this seed
    out = os.path.join(workdir, f'run_{seed}.json')
    subprocess.run('pkill -f "gz sim"; pkill -f ruby; true', shell=True)
    time.sleep(2)
    launch = subprocess.Popen(['ros2', 'launch', 'mobile_robot_bringup', 'launch_sim.launch.py', 'auto:=true',
                               f'gui:={"true" if gui else "false"}', 'rviz:=false', 'cloud:=false', 'detector:=false',
                               f'x:={spawn[0]:.3f}', f'y:={spawn[1]:.3f}', f'yaw:={spawn[2]:.3f}'],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        subprocess.run(['ros2', 'run', 'mobile_robot_bringup', 'benchmark_monitor.py', '--ros-args', '-p', 'use_sim_time:=true',
                        '-p', f'duration:={duration}', '-p', f'out:={out}', '-p', f'spawn:=[{spawn[0]},{spawn[1]},{spawn[2]}]'],
                       timeout=duration * 4 + 180, check=True)
    finally:
        os.killpg(launch.pid, signal.SIGINT)
        time.sleep(3)
        subprocess.run('pkill -f "gz sim"; pkill -f ruby; true', shell=True)
    return json.load(open(out)), None


def split_gazebo_gap(trace, house):
    """Gazebo gives one lidar gap (nearest thing of any kind). Walking people are known (actor trajectories in the world file,
    simulation time is stored in the trace), so a gap measured while a person is within 0.9 m of the robot is attributed to the
    person, else to the static world."""
    t0 = trace.get('meta', {}).get('sim_t0', 0.0)
    out = []
    for t, x, y, yaw, gap, *rest in trace['samples']:
        ppl = house.person_positions(t + t0)
        near = [p for p in ppl if np.hypot(p[0] - x, p[1] - y) < 0.9]
        if near and gap < 0.5:
            out.append([t, x, y, yaw, house.clearance(x, y, yaw), house.person_gap(x, y, yaw, near) if gap > 0 else min(gap, 0.0)])
        else:
            out.append([t, x, y, yaw, gap, float('inf')])
    trace['samples'] = out
    return trace


def summarise_estimation(items):
    items = [i for i in items if i]
    if not items:
        return None
    out = {}
    for est in ('odom', 'ekf'):
        out[est] = {k: round(float(np.mean([i[est][k] for i in items])), 3) for k in items[0][est]}
    out['patch_visits_per_run'] = round(float(np.mean([i['patch_visits'] for i in items])), 1)
    return out


def figure(house, cov, traces, curves, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, (a, b) = plt.subplots(1, 2, figsize=(14, 5.2), gridspec_kw={'width_ratios': [1.5, 1]})
    img = np.ones(house.solid.shape + (3,))
    img[house.solid] = (0.22, 0.22, 0.25)
    reach = house.reachable()
    img[reach & ~house.solid] = (0.93, 0.96, 0.93)
    a.imshow(np.transpose(img, (1, 0, 2)), origin='lower', extent=(house.x0, house.x1, house.y0, house.y1))
    for (_, x0, x1, y0, y1, mu) in house.patches:
        if mu < 0.5:
            a.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, color='#4a90d9', alpha=0.45, lw=0))
            a.text((x0 + x1) / 2, y1 + 0.1, 'wet tile', ha='center', fontsize=8, color='#2a5a99')
    cmap = plt.get_cmap('tab10')
    for k, tr in enumerate(traces[:10]):
        s = np.array(tr['samples'])
        a.plot(s[:, 1], s[:, 2], color=cmap(k % 10), lw=1.0, alpha=0.85)
        a.plot(s[0, 1], s[0, 2], 'o', color=cmap(k % 10), ms=4)
    a.set_aspect('equal')
    a.set_title(f'Autonomous runs in the house (first {min(10, len(traces))} of {len(traces)}; dots = start)')
    a.set_xlabel('x [m]')
    a.set_ylabel('y [m]')
    T = np.array([[c[1] for c in cv] for cv in curves])
    t = np.array([c[0] for c in curves[0]])
    b.fill_between(t, T.min(0) * 100, T.max(0) * 100, color='#4a90d9', alpha=0.2, label='min to max')
    b.plot(t, T.mean(0) * 100, color='#1f4e8c', lw=2, label='mean')
    b.set_xlabel('time [s]')
    b.set_ylabel('floor covered [%]')
    b.set_ylim(0, 100)
    b.grid(alpha=0.3)
    b.legend(loc='upper left')
    b.set_title(f'Coverage of {cov.area_m2:.0f} m² reachable floor')
    fig.tight_layout()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)


def update_readme(path, table, fig_rel, note):
    text = open(path).read()
    if START not in text or END not in text:
        raise SystemExit(f'{path}: markers {START} / {END} not found')
    block = f'{START}\n{note}\n\n{table}\n\n![benchmark]({fig_rel})\n{END}'
    pre, rest = text.split(START, 1)
    _, post = rest.split(END, 1)
    open(path, 'w').write(pre + block + post)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--backend', choices=['sim2d', 'gazebo'], default='sim2d')
    ap.add_argument('--runs', type=int, default=20)
    ap.add_argument('--duration', type=float, default=300.0, help='simulated seconds per run')
    ap.add_argument('--seed', type=int, default=0, help='run k uses seed (seed + k)')
    ap.add_argument('--out', default=None, help='results JSON (default: docs/benchmark_results.json with --write)')
    ap.add_argument('--write', action='store_true', help='write docs/benchmark_results.json, the figure and the README table')
    ap.add_argument('--gui', action='store_true', help='gazebo backend: show the Gazebo window')
    ap.add_argument('--workdir', default='/tmp/mobile_robot_benchmark')
    a = ap.parse_args()

    house = house_map.load()
    cov = Coverage(house)
    os.makedirs(a.workdir, exist_ok=True)
    runs, traces, curves, ests = [], [], [], []
    for k in range(a.runs):
        seed = a.seed + k
        t0 = time.time()
        trace, est = sim2d_run(seed, a.duration) if a.backend == 'sim2d' else gazebo_run(seed, a.duration, a.workdir, a.gui)
        if a.backend == 'gazebo':
            trace = split_gazebo_gap(trace, house)
        curve, _ = cov.curve(trace['samples'])
        m = bm.run_metrics(trace, curve)
        m.update(seed=seed, spawn=[round(v, 3) for v in trace['spawn']])
        runs.append(m)
        traces.append(trace)
        curves.append(curve)
        ests.append(est)
        print(f'run {k + 1:>2}/{a.runs} seed {seed}: coverage {m["coverage_pct"]:5.1f} %  collisions {m["collisions"]}  '
              f'person contacts {m["person_contacts"]}  '
              f'min clearance {m["min_clearance_m"]} m  distance {m["distance_m"]} m  ({time.time() - t0:.0f} s wall)', flush=True)
    agg = bm.aggregate(runs)
    result = {'backend': a.backend, 'world': 'house_world.sdf', 'runs_config': {'runs': a.runs, 'duration_s': a.duration, 'first_seed': a.seed},
              'reachable_floor_m2': round(cov.area_m2, 1), 'aggregate': agg, 'estimation': summarise_estimation(ests), 'runs': runs}
    table = bm.markdown_table(agg)
    print('\n' + table)
    if a.write or a.out:
        out = a.out or os.path.join(REPO, 'docs', 'benchmark_results.json')
        json.dump(result, open(out, 'w'), indent=1)
        print('wrote', out)
    if a.write:
        fig = os.path.join(REPO, 'docs', 'figures', 'benchmark.png')
        figure(house, cov, traces, curves, fig)
        note = (f'Measured with `tools/benchmark.py --backend {a.backend}`: {a.runs} runs of {a.duration:.0f} simulated seconds, '
                f'random start and heading (seeds {a.seed}..{a.seed + a.runs - 1}), 4 walking people, wet tile in the corridor. '
                f'Reachable floor: {cov.area_m2:.0f} m².')
        update_readme(os.path.join(REPO, 'README.md'), table, 'docs/figures/benchmark.png', note)
        print('wrote', fig, 'and the README table')


if __name__ == '__main__':
    main()
