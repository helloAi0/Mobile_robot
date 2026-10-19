import math
import os

import numpy as np
import pytest

try:
    import house_map
    H = house_map.load()
except ImportError:           # scipy missing; see test_description.py for why this is not importorskip
    house_map = H = None
pytestmark = pytest.mark.skipif(house_map is None, reason='needs scipy')


def test_world_is_read_from_the_sdf():
    assert len(H.shapes) >= 60
    names = {s['name'] for s in H.shapes}
    assert {'outer_north', 'outer_south', 'sofa', 'pillar_1', 'dining_table'} <= names
    assert not any(n.startswith('door_') or n.startswith('ramp') for n in names)
    assert len(H.people) == 4


def test_every_static_wall_of_the_world_is_solid_in_the_grid():
    for s in H.shapes:
        if s['name'].startswith(('outer_', 'hall_', 'div_')):
            i, j = H.cell(s['x'], s['y'])
            assert H.solid[i, j] and H.seen[i, j], s['name']


def test_start_pose_is_free_and_clear():
    assert H.clearance(0, 0, 0) > 0.6
    assert not H.solid[H.cell(0, 0)]


def test_robot_inside_a_wall_collides():
    assert H.clearance(-3.0, 1.0, 0) <= 0.0       # the wall between corridor and rooms at y = 1
    assert H.clearance(-3.0, 0.8, 0) < 0.15


def test_reachable_floor_covers_every_room_through_the_doorways():
    reach = H.reachable()
    area = reach.sum() * H.res ** 2
    assert 80 < area < 115
    for room_point in [(-5.35, 2.0), (0.0, 2.0), (5.35, 2.0), (-4.0, -3.0), (4.0, -3.0)]:
        assert reach[H.cell(*room_point)], room_point


def test_start_in_a_wall_is_rejected():
    with pytest.raises(ValueError):
        H.reachable(start=(-3.0, 1.0))


def test_lidar_ranges_match_the_geometry():
    r = H.scan(0.0, 0.0, 0.0, n=720)
    front = r[360]                        # beam straight ahead
    assert abs(front - (7.925 - 0.022)) < 0.06          # inner face of the east wall at x = 7.925, lidar 0.022 in front of base_link
    assert abs(r[540] - 0.925) < 0.15 or r[540] > 0.9    # beam at +90 deg: north wall face of the corridor (y = 0.925) or a doorway
    assert np.isfinite(r).sum() > 300


def test_lidar_sees_a_person_but_not_through_a_wall():
    ppl = [(2.0, 0.0)]
    r = H.scan(0.0, 0.0, 0.0, n=720, people=ppl)
    assert abs(r[360] - (2.0 - 0.022 - house_map.PERSON_RADIUS)) < 0.05
    behind_wall = H.scan(0.0, -3.0, 0.0, n=720, people=[(0.0, 2.5)])
    assert behind_wall[540] > 1.0 or not np.isfinite(behind_wall[540])        # south room, person in the north room


def test_person_gap_and_clearance_with_people():
    assert H.person_gap(0, 0, 0, []) == math.inf
    g = H.person_gap(0, 0, 0, [(1.0, 0.0)])
    assert abs(g - (1.0 - 0.26 - house_map.PERSON_RADIUS)) < 0.03
    assert H.person_gap(0, 0, 0, [(0.1, 0.0)]) < 0


def test_walking_people_follow_the_actor_script_and_loop():
    a = H.person_positions(0.0)
    b = H.person_positions(3.0)
    assert len(a) == 4 and a != b
    T = H.people[0]['times'][-1]
    assert np.allclose(H.person_positions(1.0)[0], H.person_positions(1.0 + T)[0], atol=1e-6)


def test_wet_tile_patch_is_found():
    assert H.slip_factor(-1.0, 0.0) == 0.15
    assert H.slip_factor(2.2, 0.0) == 1.6
    assert H.slip_factor(5.0, 4.0) is None


def test_astar_between_rooms_and_blocked_target():
    d = H.astar((0, 0), (-5.35, 2.0))
    assert d is not None and 6.0 < d < 8.5
    assert H.astar((0, 0), (0.0, 3.3)) is None            # the middle of the dining table
