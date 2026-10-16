import math

import numpy as np

from cloud_filter import filter_cloud

PITCH, H = 0.18, 0.231


def to_cam(lx, lz):
    c, s = math.cos(PITCH), math.sin(PITCH)
    return lx * c - lz * s, 0.0, lx * s + lz * c


def make(points):
    raw = np.zeros((len(points), 32), dtype=np.uint8)
    for i, p in enumerate(points):
        raw[i, 0:12] = np.frombuffer(np.array(p, dtype='<f4').tobytes(), dtype=np.uint8)
    return raw


def test_floor_removed_objects_kept_garbage_dropped():
    floor = [to_cam(d, -H) for d in np.arange(0.5, 6.0, 0.5)]
    objects = [to_cam(3.0, h - H) for h in (0.3, 0.8, 1.5)]
    bad = [(float('inf'), 0, 0), (float('nan'), 0, 0)]
    keep = filter_cloud(make(floor + objects + bad), 32, (0, 4, 8))
    assert not keep[:len(floor)].any()
    assert keep[len(floor):len(floor) + 3].all()
    assert not keep[-2:].any()
