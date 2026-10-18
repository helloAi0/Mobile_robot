import math

from metrics_logger import path_length, known_area, min_clearance, count_stops, summary


def test_path_length():
    assert path_length([(0, 0), (3, 4), (3, 8)]) == 9
    assert path_length([]) == 0


def test_known_area():
    assert known_area([-1, 0, 100, 50], 0.5) == 0.75


def test_min_clearance_ignores_bad_readings():
    assert min_clearance([float("inf"), float("nan"), 0.0, 1.2, 0.8]) == 0.8
    assert min_clearance([float("inf")]) == math.inf


def test_count_stops_counts_rising_edges():
    assert count_stops(["OK", "STOP front", "STOP front", "OK", "stop rear"]) == 2


def test_summary_zero_duration():
    assert summary(1, 2, 0.5, 0, 0)["avg_speed_mps"] == 0.0
