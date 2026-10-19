#!/usr/bin/env python3
"""Golden vectors for the C++ mux: random inputs, outputs computed by the PYTHON mux (scripts/cmd_vel_mux.py).

   python3 mobile_robot_bringup/tools/gen_mux_golden.py <out.txt>   # test/test_mux_cpp.py does this into a temp folder

Line format (numbers as repr(), 'inf' / 'nan' allowed):
   FD  n angle_min inc cone rmax center expected  r_0 ... r_{n-1}
   GU  lin ang front scan_ok require_scan stop slow rear_stop rear expected_lin expected_ang expected_reason
   LS  cur_lin cur_ang tgt_lin tgt_ang dt brake max_lin max_ang max_acc max_ang_acc expected_lin expected_ang
   SE  now t_teleop t_nav t_auto to_teleop to_nav to_auto  expected_source expected_lin expected_ang   (t < 0: never received)
The seed is fixed, so the file is reproducible; a test regenerates it and compares.
"""
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'scripts'))
import cmd_vel_mux as mux  # noqa: E402



def num(x):
    return repr(float(x)) if isinstance(x, float) else str(x)


def build(seed=12345, n_each=1500, n_scan=300):
    rng = random.Random(seed)
    lines = []
    for _ in range(n_scan):
        n = rng.choice([0, 1, 16, 90, 180, 360, 720])
        amin = rng.choice([-math.pi, -3.14159, -2.0, 0.0])
        inc = (2 * math.pi / n if n else 0.0) * rng.choice([1.0, 1.0, 0.5, -1.0])
        r = []
        for _ in range(n):
            k = rng.random()
            r.append(float('inf') if k < 0.08 else float('nan') if k < 0.12 else 0.0 if k < 0.14 else rng.uniform(0.0, 13.0))
        r = [float(__import__('struct').unpack('f', __import__('struct').pack('f', v))[0]) for v in r]     # LaserScan holds float32
        cone, rmax, center = rng.uniform(1, 90), rng.choice([12.0, 8.0, 3.5]), rng.choice([0.0, 180.0, 90.0, -90.0, rng.uniform(-180, 180)])
        exp = mux.front_distance(r, amin, inc, cone, rmax, center)
        lines.append(' '.join(['FD', str(n), num(amin), num(inc), num(cone), num(rmax), num(center), num(exp)] + [num(v) for v in r]))
    for _ in range(n_each):
        p = {'stop_dist': rng.uniform(0.1, 0.6), 'slow_dist': 0.0, 'require_scan': rng.random() < 0.8, 'max_linear': 0.6,
             'max_angular': 2.0, 'max_accel': 1.2, 'max_ang_accel': 4.0, 'rear_stop_dist': rng.uniform(0.1, 0.5)}
        p['slow_dist'] = p['stop_dist'] + rng.choice([0.0, 0.2, 0.6, 1.0])
        lin = rng.choice([0.0, rng.uniform(-0.6, 0.6), rng.uniform(0.0, 0.6), -rng.uniform(0.0, 0.6)])
        ang, front, rear = rng.uniform(-2, 2), rng.choice([12.0, rng.uniform(0.0, 3.0), p['stop_dist'], p['slow_dist']]), rng.uniform(0.0, 3.0)
        ok = rng.random() < 0.85
        l2, a2, reason = mux.guard(lin, ang, front, ok, p, rear)
        lines.append(' '.join(['GU', num(lin), num(ang), num(front), str(int(ok)), str(int(p['require_scan'])), num(p['stop_dist']),
                               num(p['slow_dist']), num(p['rear_stop_dist']), num(rear), num(l2), num(a2), reason or '-']))
    for _ in range(n_each):
        p = {'max_linear': rng.choice([0.6, 0.3]), 'max_angular': rng.choice([2.0, 1.0]), 'max_accel': rng.choice([1.2, 0.5]),
             'max_ang_accel': rng.choice([4.0, 2.0])}
        cur = (rng.uniform(-0.6, 0.6), rng.uniform(-2, 2))
        tgt = (rng.uniform(-1.0, 1.0), rng.uniform(-3, 3))
        brake = rng.random() < 0.3
        dt = rng.choice([0.05, 0.02, 0.1])
        l2, a2 = mux.limit_step(cur, tgt, dt, p, brake)
        lines.append(' '.join(['LS', num(cur[0]), num(cur[1]), num(tgt[0]), num(tgt[1]), num(dt), str(int(brake)), num(p['max_linear']),
                               num(p['max_angular']), num(p['max_accel']), num(p['max_ang_accel']), num(l2), num(a2)]))
    for _ in range(n_each):
        now = rng.uniform(10, 20)
        ts = [rng.choice([-1.0, now - rng.uniform(0.0, 1.0)]) for _ in range(3)]
        to = [0.6, 0.5, 0.5]
        last = {s: (t, (rng.uniform(-0.6, 0.6), rng.uniform(-2, 2))) for s, t in zip(mux.PRIORITY, ts) if t >= 0}
        src, (lin, ang) = mux.select(last, now, dict(zip(mux.PRIORITY, to)))
        # the C++ receives the command value of each source separately; encode them in the line
        lines.append(' '.join(['SE', num(now)] + [num(t) for t in ts] + [num(v) for t in mux.PRIORITY for v in (last[t][1] if t in last else (0.0, 0.0))]
                              + [num(v) for v in to] + [src or 'none', num(lin), num(ang)]))
    return '\n'.join(lines) + '\n'


if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else 'mux_golden.txt'
    open(out, 'w').write(build())
    print('wrote', out)
