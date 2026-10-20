"""Config files must load in the ROS parameter parser, which is stricter than PyYAML."""
import glob
import os

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GAZEBO = os.path.join(os.path.dirname(ROOT), 'mobile_robot_gazebo')


def _mixed_lists(obj, path=''):
    if isinstance(obj, dict):
        for k, v in obj.items():
            yield from _mixed_lists(v, path + '/' + str(k))
    elif isinstance(obj, list):
        kinds = {type(x).__name__ for x in obj if not isinstance(x, (list, dict))}
        if len(kinds) > 1:
            yield path, kinds
        for x in obj:
            yield from _mixed_lists(x, path)


def test_no_yaml_list_mixes_integers_and_floats():
    """rcl refuses 'Sequence should be of same type' (an int 0 inside a list of doubles): ekf.yaml crashed ekf_node this way."""
    bad = []
    for f in glob.glob(os.path.join(ROOT, 'config', '*.yaml')):
        bad += [(os.path.basename(f), p, k) for p, k in _mixed_lists(yaml.safe_load(open(f)))]
    assert not bad, bad


def test_every_world_has_the_imu_system():
    """Without gz-sim-imu-system the IMU sensor publishes nothing (selftest: 'IMU: NO DATA', EKF starved)."""
    for f in glob.glob(os.path.join(GAZEBO, 'worlds', '*.sdf')):
        s = open(f).read()
        if 'gz-sim-sensors-system' in s:
            assert 'gz-sim-imu-system' in s, f
    for f in glob.glob(os.path.join(GAZEBO, 'tools', 'make_*_world.py')):
        s = open(f).read()
        if 'gz-sim-sensors-system' in s:
            assert 'gz-sim-imu-system' in s, f
