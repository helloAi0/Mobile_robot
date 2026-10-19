import math

import benchmark_metrics as bm

N = 720
AMIN, INC = -3.14159, 2 * 3.14159 / 719


def scan_with(**beams):
    """720 empty beams, plus {angle_deg: range}."""
    r = [math.inf] * N
    for deg, rng in beams.items():
        i = round((math.radians(float(deg.replace('m', '-').replace('p', '.'))) - AMIN) / INC)
        r[i] = rng
    return r


def test_outline_in_front_and_beside():
    tab = bm.outline_table(N)
    assert abs(tab[N // 2] - (0.26 - bm.LIDAR_X)) < 0.01          # straight ahead: nose at 0.26 from base_link
    assert abs(tab[N // 4 * 3] - 0.15) < 0.04 or tab[N // 4 * 3] > 0.14  # beside: about the half width
    assert all(v > 0.1 for v in tab)


def test_scan_gap_is_range_minus_outline():
    g = bm.scan_gap(scan_with(**{'0': 0.238 + 0.30}), AMIN, INC)
    assert abs(g - 0.30) < 0.02


def test_scan_gap_zero_when_touching_and_negative_inside():
    assert abs(bm.scan_gap(scan_with(**{'0': 0.238}), AMIN, INC)) < 0.02
    assert bm.scan_gap(scan_with(**{'0': 0.20}), AMIN, INC) < -0.03


def test_scan_gap_ignores_empty_and_invisible_beams():
    assert bm.scan_gap([math.inf] * N, AMIN, INC) == math.inf
    assert bm.scan_gap(scan_with(**{'0': 0.1}), AMIN, INC) == math.inf      # closer than the lidar's minimum range: no data


def test_accumulator_counts_contacts_as_rising_edges():
    a = bm.RunAccumulator()
    for k, gap in enumerate([0.5, 0.2, 0.0, -0.01, 0.1, 0.0, 0.3]):
        a.update(k * 0.2, k * 0.1, 0.0, 0.0, gap)
    assert a.collisions == 2
    assert a.min_gap == -0.01
    assert abs(a.distance - 0.6) < 1e-9


def test_person_contacts_are_separate_from_wall_collisions():
    a = bm.RunAccumulator()
    for k, (gap, pgap) in enumerate([(0.5, 0.4), (0.5, 0.0), (0.5, -0.1), (0.5, 0.3), (0.5, -0.02)]):
        a.update(k, 0, 0, 0, gap, pgap=pgap)
    assert a.collisions == 0 and a.person_contacts == 2
    assert a.min_pgap == -0.1


def test_stops_count_rising_edges():
    a = bm.RunAccumulator()
    for k, s in enumerate([False, True, True, False, True]):
        a.update(k, 0, 0, 0, 1.0, stopped=s)
    assert a.stops == 2


def test_time_to_level():
    c = [(0, 0.0), (10, 0.2), (20, 0.5), (30, 0.6)]
    assert bm.time_to(c, 0.5) == 20 and bm.time_to(c, 0.9) is None


def trace(gaps, pgaps=None):
    return {'spawn': [0, 0, 0], 'stops': 1,
            'samples': [[k * 0.2, k * 0.1, 0, 0, g, (pgaps[k] if pgaps else math.inf)] for k, g in enumerate(gaps)]}


def test_run_metrics_and_old_five_field_samples():
    t = trace([0.5, 0.3, 0.0, 0.4])
    t['samples'] = [s[:5] for s in t['samples']]                  # older traces have no person gap
    m = bm.run_metrics(t, [(0, 0.0), (0.6, 0.3)])
    assert m['collisions'] == 1 and m['person_contacts'] == 0 and m['min_clearance_m'] == 0.0
    assert m['coverage_pct'] == 30.0 and m['t25_s'] == 0.6 and m['t50_s'] is None and m['stops'] == 1


def test_aggregate_handles_runs_that_never_reach_a_level():
    runs = [{'duration_s': 100, 'coverage_pct': 40, 't25_s': 50, 't50_s': None, 'collisions': 0, 'person_contacts': 2,
             'min_clearance_m': 0.2, 'min_person_gap_m': 0.0, 'distance_m': 30, 'stops': 3},
            {'duration_s': 100, 'coverage_pct': 60, 't25_s': 30, 't50_s': 80, 'collisions': 1, 'person_contacts': 0,
             'min_clearance_m': -0.01, 'min_person_gap_m': 0.3, 'distance_m': 50, 'stops': 5}]
    a = bm.aggregate(runs)
    assert a['runs'] == 2 and a['coverage_pct']['mean'] == 50 and a['t50_s']['n'] == 1 and a['t50_s']['missing'] == 1
    assert a['runs_with_collision'] == 1 and a['runs_with_person_contact'] == 1
    assert a['min_clearance_m']['min'] == -0.01


def test_markdown_table_is_complete():
    runs = [{'duration_s': 100, 'coverage_pct': 40, 't25_s': 50, 't50_s': None, 'collisions': 0, 'person_contacts': 1,
             'min_clearance_m': 0.2, 'min_person_gap_m': 0.1, 'distance_m': 30, 'stops': 3}]
    t = bm.markdown_table(bm.aggregate(runs))
    for label in ('Coverage', 'Time to 50 %', 'Collisions with walls', 'person', 'clearance', 'Distance driven'):
        assert label in t
    assert 'not reached' in t
    assert len({line.count('|') for line in t.splitlines()}) == 1       # every row has the same number of columns
