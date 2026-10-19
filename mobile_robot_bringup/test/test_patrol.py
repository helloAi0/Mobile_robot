import math
import os

import pytest

import patrol

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROUTE = os.path.join(ROOT, 'config', 'patrol_house.yaml')
R = patrol.load_route(ROUTE)
try:
    import house_map as hm
    H = hm.load()
except ImportError:           # scipy missing; see test_description.py for why this is not importorskip
    hm = H = None
pytestmark = pytest.mark.skipif(hm is None, reason='needs scipy')
ROBOT_RADIUS = 0.30


def test_route_loads_and_names_are_unique():
    assert len(R['waypoints']) == 9 and R['retries'] >= 1 and R['goal_timeout_s'] > 10
    with pytest.raises(ValueError):
        import tempfile
        f = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
        f.write("patrol:\n  waypoints:\n    - {name: a, x: 0, y: 0}\n    - {name: a, x: 1, y: 0}\n")
        f.close()
        patrol.load_route(f.name)


@pytest.mark.parametrize('wp', R['waypoints'], ids=lambda w: w['name'])
def test_every_waypoint_is_free_and_reachable_with_margin(wp):
    reach = H.reachable((0.0, 0.0), inflate=ROBOT_RADIUS + 0.1)
    assert reach[H.cell(wp['x'], wp['y'])], f"{wp['name']} is inside an obstacle's inflation zone or unreachable"
    assert H.clearance(wp['x'], wp['y'], wp['yaw']) > 0.25


def test_route_visits_all_five_rooms_and_the_corridor():
    rooms = {'living': (-8, -2.7, 1, 5.5), 'dining': (-2.7, 2.7, 1, 5.5), 'bedroom': (2.7, 8, 1, 5.5),
             'office': (-8, 0, -5.5, -1), 'workshop': (0, 8, -5.5, -1), 'corridor': (-8, 8, -1, 1)}
    for room, (x0, x1, y0, y1) in rooms.items():
        assert any(x0 < w['x'] < x1 and y0 < w['y'] < y1 for w in R['waypoints']), room


def test_every_leg_is_plannable_and_not_absurdly_long():
    pts = [(0.0, 0.0)] + [(w['x'], w['y']) for w in R['waypoints']] + [(0.0, 0.0)]
    total = 0.0
    for a, b in zip(pts, pts[1:]):
        d = H.astar(a, b, inflate=ROBOT_RADIUS + 0.05)
        assert d is not None, f'no path {a} -> {b}'
        assert d < 1.7 * math.dist(a, b) + 3.0, f'{a} -> {b}: {d:.1f} m for a straight {math.dist(a, b):.1f} m'
        total += d
    assert 30 < total < 90, total              # one lap is about a minute at 0.35 m/s... a few minutes at worst


def test_waypoints_are_not_on_the_wet_tile_or_in_the_walking_peoples_lanes_by_accident():
    for w in R['waypoints']:
        mu = H.slip_factor(w['x'], w['y'])
        assert mu is None or mu > 0.5, f"{w['name']} stands on the wet tile"


def test_yaw_to_quaternion_is_unit_and_correct():
    x, y, z, w = patrol.yaw_to_quaternion(math.pi / 2)
    assert x == y == 0 and abs(z - math.sqrt(0.5)) < 1e-12 and abs(z * z + w * w - 1) < 1e-12


def test_mission_walks_the_route_and_counts_laps():
    m = patrol.Mission({'waypoints': [{'name': n} for n in 'abc'], 'loops': 2, 'retries': 1})
    seq = []
    while not m.finished:
        seq.append(m.current['name'])
        assert m.report(True) == 'next'
    assert seq == list('abcabc') and m.laps == 2 and m.current is None


def test_mission_retries_then_skips_and_carries_on():
    m = patrol.Mission({'waypoints': [{'name': 'a'}, {'name': 'b'}], 'loops': 1, 'retries': 2})
    assert [m.report(False) for _ in range(3)] == ['retry', 'retry', 'skip']
    assert m.current['name'] == 'b' and m.skipped == ['a']
    m.report(True)
    assert m.finished and m.reached == ['b']


def test_mission_with_zero_loops_never_finishes():
    m = patrol.Mission({'waypoints': [{'name': 'a'}], 'loops': 0, 'retries': 1})
    for _ in range(50):
        m.report(True)
    assert not m.finished and m.laps == 50
