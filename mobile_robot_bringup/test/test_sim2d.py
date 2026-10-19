import math

import numpy as np
import pytest

try:
    import sim2d
except ImportError:           # scipy missing; see test_description.py for why this is not importorskip
    sim2d = None
pytestmark = pytest.mark.skipif(sim2d is None, reason='needs scipy')


def test_same_seed_gives_the_same_run():
    a = sim2d.Run(seed=4, duration=12).run()
    b = sim2d.Run(seed=4, duration=12).run()
    assert a['samples'] == b['samples']


def test_spawn_is_free_and_seeded():
    s1 = sim2d.Run(seed=3, duration=1).spawn
    assert s1 == sim2d.Run(seed=3, duration=1).spawn != sim2d.Run(seed=4, duration=1).spawn
    h = sim2d.house_map.load()
    assert h.clearance(*s1) > 0.2


def test_autopilot_does_not_hit_walls_or_furniture():
    for seed in range(3):
        r = sim2d.Run(seed=seed, duration=40, people=False)
        r.run()
        assert r.acc.collisions == 0 and r.acc.min_gap > 0.05, seed


def test_robot_drives_and_trace_has_six_fields():
    r = sim2d.Run(seed=1, duration=20)
    tr = r.run()
    assert len(tr['samples']) == 100 and len(tr['samples'][0]) == 6
    assert r.acc.distance > 1.0


def test_a_wall_blocks_the_robot_and_counts_as_contact():
    r = sim2d.Run(seed=0, duration=5, spawn=(7.2, 0.0, 0.0), people=False)
    r.drive = lambda t: (setattr(r, 'cur', (0.5, 0.0)), False)[1]       # drive straight at the east wall, no safety layer
    r.run()
    assert r.acc.collisions >= 1
    assert r.x < 7.8                                                    # never inside the wall


def _crossing_run(seed):
    """Drive straight over the wet tile with nothing else going on; return (odom yaw error, ekf yaw error) in rad at the end."""
    r = sim2d.Run(seed=seed, duration=14, spawn=(-3.0, 0.0, 0.0), people=False)
    r.drive = lambda t: (setattr(r, 'cur', (0.25, 0.0)), False)[1]
    r.run()
    e = r.err
    return e['odom'][-1][1], e['ekf'][-1][1], r.patch_visits


def test_gyro_fusion_beats_wheel_odometry_after_slipping_on_the_wet_tile():
    res = [_crossing_run(s) for s in range(8)]
    assert all(v == 1 for _, _, v in res)
    odom = np.mean([o for o, _, _ in res])
    ekf = np.mean([e for _, e, _ in res])
    assert ekf < 0.5 * odom, (odom, ekf)


def test_without_slip_odometry_and_ekf_are_both_good():
    r = sim2d.Run(seed=2, duration=14, spawn=(-3.0, 0.0, 0.0), people=False, slip=False)
    r.drive = lambda t: (setattr(r, 'cur', (0.25, 0.0)), False)[1]
    r.run()
    assert r.err['odom'][-1][1] < math.radians(3) and r.err['ekf'][-1][1] < math.radians(3)
