"""The robot description must build in every hardware mode (sim / mock / real) and have the same joints."""
import os
import sys
import types
import xml.dom.minidom as minidom

import pytest

xacro = pytest.importorskip('xacro')
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))     # mobile_robot_bringup
REPO = os.path.dirname(ROOT)
DESC = os.path.join(REPO, 'mobile_robot_description')

try:
    import ament_index_python.packages  # noqa: F401  (real ROS installation)
except ImportError:                       # plain python (CI without ROS): fake the one function xacro needs
    pkg = types.ModuleType('ament_index_python')
    sub = types.ModuleType('ament_index_python.packages')
    sub.get_package_share_directory = lambda name: os.path.join(REPO, name)
    pkg.packages = sub
    sys.modules['ament_index_python'] = pkg
    sys.modules['ament_index_python.packages'] = sub


def build(**kw):
    maps = {'hardware': 'sim', 'hw_plugin': 'mock_components/GenericSystem', 'hw_device': '/dev/ttyUSB0',
            'hw_baud': '57600', 'hw_counts_per_rev': '3436',
            'controllers': os.path.join(ROOT, 'config', 'my_controllers.yaml')}
    maps.update(kw)
    text = xacro.process_file(os.path.join(DESC, 'urdf', 'robot.urdf.xacro'), mappings=maps).toxml()
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


# ---------------------------------------------------------------- physical plausibility of the mechanical model
def _links(dom):
    out = {}
    for link in dom.getElementsByTagName('link'):
        inert = link.getElementsByTagName('inertial')
        if not inert:
            continue
        i = inert[0]
        tag = i.getElementsByTagName('inertia')[0]
        g = lambda a: float(tag.getAttribute(a))  # noqa: E731
        out[link.getAttribute('name')] = (float(i.getElementsByTagName('mass')[0].getAttribute('value')),
                                          g('ixx'), g('iyy'), g('izz'), g('ixy'), g('ixz'), g('iyz'))
    return out


def test_inertia_tensors_are_physical():
    """Positive mass, positive-definite tensor, principal moments obey the triangle inequality (a real rigid body)."""
    import numpy as np
    _, dom = build(hardware='mock')
    links = _links(dom)
    assert {'chassis', 'left_wheel', 'right_wheel', 'caster_wheel', 'caster_fork', 'left_carrier', 'imu_link'} <= set(links)
    for name, (m, ixx, iyy, izz, ixy, ixz, iyz) in links.items():
        assert m > 0, name
        eig = np.linalg.eigvalsh(np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]))
        assert eig.min() > 0, name + ' tensor is not positive definite'
        a, b, c = sorted(eig)
        assert a + b >= c * (1 - 1e-6), name + ' violates the triangle inequality: not a real body'


def test_total_mass_matches_budget():
    import re
    text = open(os.path.join(DESC, 'urdf', 'mass_properties.xacro')).read()
    total = float(re.search(r'name="total_mass" value="([\d.]+)"', text).group(1))
    _, dom = build(hardware='mock')
    assert abs(sum(v[0] for v in _links(dom).values()) - total) < 0.01


def test_visual_and_collision_are_split():
    """Detailed meshes only on visuals; collisions are primitives (box / cylinder), never meshes."""
    _, dom = build(hardware='mock')
    for col in dom.getElementsByTagName('collision'):
        assert not col.getElementsByTagName('mesh'), 'collision uses a mesh: slow and jittery'
    assert any(v.getElementsByTagName('mesh') for v in dom.getElementsByTagName('visual'))


def test_actuator_limits_and_suspension_present():
    _, dom = build(hardware='mock')
    byname = {j.getAttribute('name'): j for j in dom.documentElement.childNodes
              if j.nodeName == 'joint'}          # URDF joints only (ros2_control has same-named <joint> tags)
    for w in ('left_wheel_joint', 'right_wheel_joint'):
        lim = byname[w].getElementsByTagName('limit')[0]
        assert float(lim.getAttribute('effort')) > 0 and float(lim.getAttribute('velocity')) > 0
    assert byname['left_carrier_joint'].getAttribute('type') == 'prismatic'
    assert byname['caster_swivel_joint'].getAttribute('type') == 'continuous'


def test_sensors_have_noise_models():
    text, _ = build(hardware='sim')
    assert text.count('<noise') >= 8, 'lidar, camera and 6 IMU axes must all carry a noise model'
    assert 'bias_stddev' in text and '<distortion>' in text
