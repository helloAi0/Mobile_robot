#!/usr/bin/env python3
"""Does gyro fusion fix the wheel-slip error? Drives across the wet tile (mu 0.15) many times and compares wheel odometry with
the planar EKF (the model of config/ekf.yaml, see tools/ekf_model.py). Prints a table; --write puts it into docs/LOCALIZATION.md.

   python3 mobile_robot_bringup/tools/ekf_eval.py --runs 40 --write

Each run: the robot starts at (-3.2, 0) heading +x, drives straight at 0.25 m/s through the wet tile at x = -1.7..-0.3, and on to x = +1.5.
The wheels slip (random loss of forward speed and turn rate, random yaw drift: tools/sim2d.py SLIP), the encoders still report the commanded
motion, the gyro reports the true turn rate. Errors are measured against the true pose at the end of the run.
"""
import argparse
import math
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sim2d  # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))
START, END = '<!-- ekf-eval:start -->', '<!-- ekf-eval:end -->'


def one(seed, slip):
    r = sim2d.Run(seed=seed, duration=22.0, spawn=(-3.2, 0.0, 0.0), people=False, slip=slip)
    r.drive = lambda t: (setattr(r, 'cur', (0.25, 0.0)), False)[1]
    r.run()
    return r.err['odom'][-1], r.err['ekf'][-1], r.patch_visits


def stats(vals):
    return statistics.fmean(vals), statistics.pstdev(vals)


def evaluate(runs):
    out = {}
    for label, slip in (('with wheel slip (wet tile)', True), ('no slip (control)', False)):
        res = [one(s, slip) for s in range(runs)]
        out[label] = {'odom_pos': stats([o[0] for o, _, _ in res]), 'ekf_pos': stats([e[0] for _, e, _ in res]),
                      'odom_yaw': stats([math.degrees(o[1]) for o, _, _ in res]), 'ekf_yaw': stats([math.degrees(e[1]) for _, e, _ in res]),
                      'patch': statistics.fmean(v for _, _, v in res)}
    return out


def table(res, runs):
    lines = [f'| Scenario ({runs} runs each) | Heading error, wheels only | Heading error, wheels + gyro (EKF) | Position error, wheels only | Position error, EKF |',
             '|---|---|---|---|---|']
    for label, r in res.items():
        f = lambda t, u: f'{t[0]:.2f} ± {t[1]:.2f} {u}'  # noqa: E731
        lines.append(f"| {label} | {f(r['odom_yaw'], '°')} | {f(r['ekf_yaw'], '°')} | {f(r['odom_pos'], 'm')} | {f(r['ekf_pos'], 'm')} |")
    return '\n'.join(lines)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', type=int, default=40)
    ap.add_argument('--write', action='store_true')
    a = ap.parse_args()
    res = evaluate(a.runs)
    t = table(res, a.runs)
    print(t)
    if a.write:
        p = os.path.join(REPO, 'docs', 'LOCALIZATION.md')
        text = open(p).read()
        pre, rest = text.split(START, 1)
        _, post = rest.split(END, 1)
        open(p, 'w').write(pre + START + '\n' + t + '\n' + END + post)
        print('wrote', p)
