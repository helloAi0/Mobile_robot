"""Independent physics check (MuJoCo) of the robot description: ride height, level, straight driving, turning direction, speed,
ramp, e-stop distance, suspension settling. Skipped when mujoco is not installed (pip install mujoco)."""
import os
import sys

import pytest

pytest.importorskip('mujoco')
pytest.importorskip('xacro')
TOOLS = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'mobile_robot_description', 'tools')
sys.path.insert(0, TOOLS)
import physics_check  # noqa: E402


def test_robot_physics_checks_pass():
    rows = physics_check.results()
    failed = [f'{name}: {detail}' for ok, name, detail in rows if not ok]
    assert not failed, 'physics checks failed:\n' + '\n'.join(failed)
    assert len(rows) >= 15


def test_physics_check_detects_a_wrong_wheel_axis():
    """The check must be able to fail: with the left wheel axis flipped the robot must NOT pass 'moves forward / goes straight'."""
    import re
    import tempfile
    import shutil
    src = os.path.join(os.path.dirname(TOOLS), 'urdf')
    with tempfile.TemporaryDirectory() as tmp:
        shutil.copytree(src, os.path.join(tmp, 'urdf'))
        shutil.copytree(os.path.join(os.path.dirname(TOOLS), 'meshes'), os.path.join(tmp, 'meshes'))
        core = os.path.join(tmp, 'urdf', 'robot_core.xacro')
        text = open(core).read()
        broken = text.replace('<axis xyz="0 0 ${sign}"/>', '<axis xyz="0 0 ${-sign if side == \'left\' else sign}"/>')
        assert broken != text, 'wheel axis line not found: update this test'
        open(core, 'w').write(broken)
        old = physics_check.DESC
        physics_check.DESC = tmp
        try:
            r = physics_check.check_straight()
        finally:
            physics_check.DESC = old
    assert not (r['dx'] > 0.8 * r['expect'] and abs(r['dy']) < 0.06 and abs(r['dyaw']) < 3.0)
