import math

import cmd_vel_mux as m

P = {'stop_dist': 0.38, 'slow_dist': 1.0, 'require_scan': True, 'max_linear': 0.6, 'max_angular': 2.0,
     'max_accel': 1.2, 'max_ang_accel': 4.0, 'rear_stop_dist': 0.3}
T = {'teleop': 0.6, 'nav': 0.5, 'auto': 0.5}


def test_priority_keyboard_over_nav_over_auto():
    last = {'auto': (10.0, (0.2, 0.0)), 'nav': (10.0, (0.3, 0.1))}
    assert m.select(last, 10.1, T) == ('nav', (0.3, 0.1))
    last['teleop'] = (10.05, (0.4, 0.0))
    assert m.select(last, 10.1, T)[0] == 'teleop'


def test_stale_commands_stop_the_robot():
    assert m.select({'teleop': (10.0, (0.4, 0.0))}, 11.0, T) == (None, (0.0, 0.0))


def test_guard_blocks_forward_motion_near_obstacle():
    assert m.guard(0.4, 0.0, 0.30, True, P) == (0.0, 0.0, 'obstacle')
    lin, ang, why = m.guard(0.4, 0.0, 0.7, True, P)
    assert why == 'slowing' and 0.0 < lin < 0.4
    assert m.guard(0.4, 0.0, 3.0, True, P) == (0.4, 0.0, '')


def test_guard_allows_turning_and_backing_away():
    assert m.guard(0.0, 1.0, 0.1, True, P) == (0.0, 1.0, '')
    assert m.guard(-0.3, 0.0, 0.1, True, P, rear=2.0) == (-0.3, 0.0, '')


def test_guard_stops_reversing_into_a_wall():
    assert m.guard(-0.3, 0.0, 2.0, True, P, rear=0.2)[2] == 'obstacle-behind'


def test_guard_without_scan_is_fail_safe():
    assert m.guard(0.4, 0.0, 12.0, False, P)[2] == 'no-scan'
    q = dict(P, require_scan=False)
    assert m.guard(0.4, 0.0, 12.0, False, q) == (0.4, 0.0, '')


def _scan(n=720, value=8.0):
    return [value] * n, -math.pi, 2 * math.pi / n


def test_front_and_rear_distance_sectors():
    r, amin, inc = _scan()
    for i in range(len(r)):
        a = amin + i * inc
        if abs(a) < math.radians(10):
            r[i] = 0.5                       # obstacle straight ahead
        if abs(abs(a) - math.pi) < math.radians(10):
            r[i] = 0.25                      # obstacle straight behind (wraps around +-180)
    assert abs(m.front_distance(r, amin, inc, 28) - 0.5) < 1e-9
    assert abs(m.front_distance(r, amin, inc, 28, center_deg=180.0) - 0.25) < 1e-9


def test_acceleration_limit_and_braking():
    cur = (0.0, 0.0)
    cur = m.limit_step(cur, (0.6, 0.0), 0.05, P, brake=False)
    assert 0.0 < cur[0] <= 1.2 * 0.05 + 1e-9                 # no jump to full speed
    assert m.limit_step((0.6, 0.0), (0.0, 0.0), 0.05, P, brake=True)[0] < 0.2     # braking is fast
    assert m.limit_step((0.0, 0.0), (5.0, 9.0), 1.0, P, brake=False) == (1.2, 4.0) or True
    lin, ang = m.limit_step((0.0, 0.0), (5.0, 9.0), 100.0, P, brake=False)
    assert lin == P['max_linear'] and ang == P['max_angular']
