#!/usr/bin/env python3
"""Latency of one mux cycle, Python vs C++: the numbers for docs/PERFORMANCE.md.

   python3 mobile_robot_bringup/tools/mux_latency.py                 # builds test/cpp/mux_bench with g++ -O2, runs both
   python3 mobile_robot_bringup/tools/mux_latency.py --json out.json

Measures the pure computation of the two things the mux does (no ROS, no DDS, no scheduling):
  scan work   front + rear distance over a 720-beam scan (runs on every lidar scan, 10 Hz)
  tick work   priority select + obstacle guard + speed/acceleration limits (runs every 50 ms)
Same inputs on both sides, 20000 iterations each, single thread. This is computation only. End-to-end latency through ROS
(DDS, executor scheduling, serialisation) is NOT measured here: it needs a machine with ROS, and is expected to be dominated by
the middleware, not by these microseconds. The C++ numbers use ConeFinder (the beams inside the cone are cached while the
scan geometry is unchanged); docs/PERFORMANCE.md also lists the uncached C++ port, so the language and the algorithm are
credited separately.
"""
import argparse
import json
import os
import random
import statistics
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
import cmd_vel_mux as mux  # noqa: E402


def build_cpp(out_dir):
    exe = os.path.join(out_dir, 'mux_bench')
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-I', os.path.join(ROOT, 'include'),
                    os.path.join(ROOT, 'test', 'cpp', 'mux_bench.cpp'), '-o', exe], check=True)
    return exe


def run_cpp(exe, iters):
    return json.loads(subprocess.run([exe, str(iters)], capture_output=True, text=True, check=True).stdout)


def pct(v, q):
    v = sorted(v)
    return v[int(q * (len(v) - 1))]


def run_python(iters):
    n = 720
    amin, inc = -3.14159, 2 * 3.14159 / (n - 1)
    rng = random.Random(1)
    ranges = [rng.uniform(0.2, 9.0) for _ in range(n)]
    p = {'stop_dist': 0.38, 'slow_dist': 1.0, 'require_scan': True, 'max_linear': 0.6, 'max_angular': 2.0,
         'max_accel': 1.2, 'max_ang_accel': 4.0, 'rear_stop_dist': 0.3}
    last = {'teleop': (10.0, (0.4, 0.2))}
    to = {'teleop': 0.6, 'nav': 0.5, 'auto': 0.5}
    cur = (0.0, 0.0)
    scan_us, tick_us = [], []
    clock = time.perf_counter
    for i in range(iters):
        ranges[i % n] = rng.uniform(0.2, 9.0)
        t0 = clock()
        front = mux.front_distance(ranges, amin, inc, 28.0, 12.0)
        rear = mux.front_distance(ranges, amin, inc, 28.0, 12.0, center_deg=180.0)
        t1 = clock()
        _, (lin, ang) = mux.select(last, 10.2, to)
        lin, ang, reason = mux.guard(lin, ang, front, True, p, rear)
        cur = mux.limit_step(cur, (lin, ang), 0.05, p, reason in ('obstacle', 'obstacle-behind', 'no-scan'))
        t2 = clock()
        scan_us.append((t1 - t0) * 1e6)
        tick_us.append((t2 - t1) * 1e6)
    return {'impl': 'python', 'iterations': iters,
            'scan_front_rear_us': {'median': round(statistics.median(scan_us), 2), 'p99': round(pct(scan_us, 0.99), 2), 'max': round(max(scan_us), 2)},
            'tick_us': {'median': round(statistics.median(tick_us), 3), 'p99': round(pct(tick_us, 0.99), 3), 'max': round(max(tick_us), 2)}}


def table(py, cpp):
    rows = [('Scan work: front + rear distance over 720 beams (per scan, 10 Hz)', 'scan_front_rear_us'),
            ('Tick work: select + guard + limits (per 50 ms tick)', 'tick_us')]
    out = ['| Per call | Python median | Python p99 | C++ median | C++ p99 | Speed-up (median) |', '|---|---|---|---|---|---|']
    for label, k in rows:
        a, b = py[k], cpp[k]
        out.append(f"| {label} | {a['median']} µs | {a['p99']} µs | {b['median']} µs | {b['p99']} µs | {a['median'] / max(b['median'], 1e-9):.0f}x |")
    return '\n'.join(out)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--iterations', type=int, default=20000)
    ap.add_argument('--json', default=None)
    a = ap.parse_args()
    with tempfile.TemporaryDirectory() as d:
        cpp = run_cpp(build_cpp(d), a.iterations)
    py = run_python(a.iterations)
    print(table(py, cpp))
    if a.json:
        json.dump({'python': py, 'cpp': cpp}, open(a.json, 'w'), indent=1)
