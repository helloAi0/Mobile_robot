import glob
import os
import re
import shutil
import subprocess
import sys
import xml.dom.minidom as minidom

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))     # mobile_robot_bringup
REPO = os.path.dirname(ROOT)
DESC = os.path.join(REPO, 'mobile_robot_description')
GZ = os.path.join(REPO, 'mobile_robot_gazebo')
PKGS = [ROOT, DESC, GZ]


def files(pattern, *bases):
    return sorted(f for b in (bases or PKGS) for f in glob.glob(os.path.join(b, pattern)))


@pytest.mark.parametrize('path', files('worlds/*.sdf') + files('urdf/*.xacro') + files('package.xml'))
def test_xml_is_well_formed(path):
    minidom.parse(path)


@pytest.mark.parametrize('path', files('config/*.yaml') + files('rviz/*.rviz'))
def test_yaml_parses(path):
    yaml.safe_load(open(path))


@pytest.mark.parametrize('path', files('scripts/*.py') + files('launch/*.py') + files('tools/*.py'))
def test_python_compiles(path):
    compile(open(path).read(), path, 'exec')


@pytest.mark.parametrize('path', files('scripts/*.sh') + files('deploy/*.sh') + [os.path.join(REPO, 'build.sh')])
def test_shell_syntax(path):
    assert subprocess.run(['bash', '-n', path]).returncode == 0


@pytest.mark.parametrize('world', ['house_world', 'street_world', 'actor_height_test'])
def test_world_is_valid(world):
    dom = minidom.parse(os.path.join(GZ, 'worlds', world + '.sdf'))
    names = [e.getAttribute('name') for e in dom.getElementsByTagName('model') + dom.getElementsByTagName('actor')]
    names += [i.getElementsByTagName('name')[0].firstChild.data for i in dom.getElementsByTagName('include')]
    assert len(names) == len(set(names)), 'duplicate model/actor names'
    text = open(os.path.join(GZ, 'worlds', world + '.sdf')).read()
    assert 'gz-sim-sensors-system' in text, 'Sensors system missing: camera and lidar would not render'
    for model in dom.getElementsByTagName('model'):
        static = model.getElementsByTagName('static')
        if static and static[0].firstChild.data == 'false':
            assert model.getElementsByTagName('mass'), model.getAttribute('name') + ' is dynamic but has no mass'
            assert model.getElementsByTagName('collision'), model.getAttribute('name') + ' is dynamic but has no collision'


def test_controller_config_is_complete():
    cfg = yaml.safe_load(open(os.path.join(ROOT, 'config', 'my_controllers.yaml')))
    assert cfg['controller_manager']['ros__parameters']['diff_cont']['type'] == 'diff_drive_controller/DiffDriveController'
    d = cfg['diff_cont']['ros__parameters']
    assert d['left_wheel_names'] == ['left_wheel_joint'] and d['right_wheel_names'] == ['right_wheel_joint']
    assert d['open_loop'] is False and d['cmd_vel_timeout'] <= 1.0


@pytest.mark.parametrize('pkg', [ROOT, GZ])
def test_every_script_is_installed_and_executable(pkg):
    cmake = open(os.path.join(pkg, 'CMakeLists.txt')).read()
    listed = re.search(r'PROGRAMS(.*?)DESTINATION', cmake, re.S).group(1).split()
    for rel in listed:
        path = os.path.join(pkg, rel)
        assert os.path.exists(path), rel + ' is installed by CMake but missing'
        assert os.access(path, os.X_OK), rel + ' is not executable'
    for path in files('scripts/*.py', pkg) + files('scripts/*.sh', pkg):
        assert os.path.relpath(path, pkg) in listed, path + ' is not installed by CMakeLists.txt'


def test_actor_height_tuner(tmp_path):
    shutil.copytree(os.path.join(GZ, 'scripts'), tmp_path / 'scripts')
    shutil.copytree(os.path.join(GZ, 'worlds'), tmp_path / 'worlds')
    out = subprocess.run([sys.executable, '-I', str(tmp_path / 'scripts' / 'tune_actor_height.py'), '0.25'],
                         capture_output=True, text=True)
    assert out.returncode == 0 and 'waypoints set to z=0.25' in out.stdout
    assert '0.25 0 0' in (tmp_path / 'worlds' / 'house_world.sdf').read_text()


# ---------------------------------------------------------------- imported textured models (mobile_robot_gazebo/models)
MODELS = os.path.join(GZ, 'models')
MODEL_NAMES = sorted(d for d in os.listdir(MODELS) if os.path.isdir(os.path.join(MODELS, d)))


@pytest.mark.parametrize('world', ['house_world', 'street_world', 'actor_height_test', 'camera_world'])
def test_world_includes_resolve_to_local_models(world):
    dom = minidom.parse(os.path.join(GZ, 'worlds', world + '.sdf'))
    for inc in dom.getElementsByTagName('include'):
        uri = inc.getElementsByTagName('uri')[0].firstChild.data
        if uri.startswith('model://'):
            assert uri[len('model://'):] in MODEL_NAMES, uri + ' is not in mobile_robot_gazebo/models'


@pytest.mark.parametrize('model', MODEL_NAMES)
def test_model_visual_mesh_collision_primitive_textures_exist(model):
    d = os.path.join(MODELS, model)
    dom = minidom.parse(os.path.join(d, 'model.sdf'))
    assert dom.getElementsByTagName('collision'), model + ' has no collision'
    for col in dom.getElementsByTagName('collision'):
        assert not col.getElementsByTagName('mesh'), model + ': collision must be a primitive, not a mesh'
    for vis in dom.getElementsByTagName('visual'):
        uri = vis.getElementsByTagName('uri')[0].firstChild.data
        mesh = os.path.join(MODELS, uri[len('model://'):])
        assert os.path.exists(mesh), uri
        text = open(mesh, encoding='utf-8', errors='replace').read()
        assert 'meter="1"' in text, 'unit must be metres (sizes are baked into the SDF scale)'
        for tex in re.findall(r'<init_from>([^<]*\.(?:png|jpg|jpeg))</init_from>', text):
            assert os.path.exists(os.path.normpath(os.path.join(os.path.dirname(mesh), tex))), model + ': missing texture ' + tex
    if dom.getElementsByTagName('static')[0].firstChild.data == 'false':
        assert dom.getElementsByTagName('mass') and dom.getElementsByTagName('inertia'), model + ' is dynamic without inertia'


def test_third_party_models_are_attributed():
    text = open(os.path.join(MODELS, 'ATTRIBUTION.md')).read()
    assert 'Creative Commons Attribution 3.0' in text
    for m in MODEL_NAMES:
        assert '`' + m + '`' in text, m + ' missing from ATTRIBUTION.md'


def test_ci_workflow_is_valid_yaml_with_runnable_steps():
    wf = yaml.safe_load(open(os.path.join(REPO, '.github', 'workflows', 'ci.yml')))
    assert wf['jobs']
    for job in wf['jobs'].values():
        for step in job['steps']:
            assert 'uses' in step or 'run' in step, step


def test_no_module_level_skip_in_test_files():
    """pytest.importorskip / pytest.skip at module level aborted the whole colcon test run in the ROS container
    (the launch_testing plugin imports every test file). Use pytestmark = skipif(...) instead."""
    for path in files('test/test_*.py', ROOT):
        for line in open(path).read().splitlines():
            if line.startswith(('pytest.importorskip', 'pytest.skip')) or re.match(r'^\w+ = pytest\.importorskip', line):
                pytest.fail(f'{os.path.basename(path)}: module-level skip: {line}')
