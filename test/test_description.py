"""The robot description must build in every hardware mode (sim / mock / real) and have the same joints."""
import os
import sys
import types
import xml.dom.minidom as minidom

import pytest

xacro = pytest.importorskip('xacro')
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

try:
    import ament_index_python.packages  # noqa: F401  (real ROS installation)
except ImportError:                       # plain python (CI without ROS): fake the one function xacro needs
    pkg = types.ModuleType('ament_index_python')
    sub = types.ModuleType('ament_index_python.packages')
    sub.get_package_share_directory = lambda name: ROOT
    pkg.packages = sub
    sys.modules['ament_index_python'] = pkg
    sys.modules['ament_index_python.packages'] = sub


def build(**kw):
    maps = {'hardware': 'sim', 'hw_plugin': 'mock_components/GenericSystem', 'hw_device': '/dev/ttyUSB0',
            'hw_baud': '57600', 'hw_counts_per_rev': '3436'}
    maps.update(kw)
    text = xacro.process_file(os.path.join(ROOT, 'description', 'robot.urdf.xacro'), mappings=maps).toxml()
    return text, minidom.parseString(text)


def joints(dom):
    return sorted(j.getAttribute('name') for j in dom.getElementsByTagName('joint'))


def test_sim_uses_gazebo_plugin():
    text, dom = build(hardware='sim')
    assert 'gz_ros2_control/GazeboSimSystem' in text and 'rgbd_camera' in text and 'gpu_lidar' in text


def test_mock_uses_mock_plugin_and_same_joints():
    sim_text, sim = build(hardware='sim')
    text, dom = build(hardware='mock')
    assert 'mock_components/GenericSystem' in text and 'GazeboSimSystem' not in text
    assert joints(dom) == joints(sim)


def test_real_passes_driver_parameters():
    text, _ = build(hardware='real', hw_plugin='my_driver/MySystem', hw_device='/dev/ttyACM0', hw_counts_per_rev='1200')
    assert 'my_driver/MySystem' in text and '/dev/ttyACM0' in text and '1200' in text
