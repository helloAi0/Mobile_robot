import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(ROOT)
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
sys.path.insert(0, os.path.join(REPO, 'mobile_robot_gazebo', 'scripts'))

# Test modules that need optional packages (mujoco, scipy, xacro) skip their tests with pytest.mark.skipif and must NEVER raise
# Skipped at import time (pytest.importorskip at module level): inside the ROS container the launch_testing pytest plugin imports every
# test file itself, and one module-level skip aborted the whole run ("1 skipped, no tests collected"), which made `colcon test` fail.
