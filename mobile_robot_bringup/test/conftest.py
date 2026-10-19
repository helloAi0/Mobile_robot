import importlib.util
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(ROOT)
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
sys.path.insert(0, os.path.join(REPO, 'mobile_robot_gazebo', 'scripts'))


def _missing(*modules):
    return [m for m in modules if importlib.util.find_spec(m) is None]


# Test modules that need optional packages are left out of collection when the package is missing. (A module-level
# pytest.importorskip would also skip them, but with pytest 7.4 inside the ROS container it aborted the whole run:
# CI showed "1 skipped, no tests collected" and the colcon test step failed. Not collecting is the same on every pytest version.)
collect_ignore = []
if _missing('mujoco', 'xacro'):
    collect_ignore.append('test_physics.py')
if _missing('xacro'):
    collect_ignore.append('test_description.py')
if _missing('scipy'):
    collect_ignore += ['test_house_map.py', 'test_sim2d.py', 'test_patrol.py']
