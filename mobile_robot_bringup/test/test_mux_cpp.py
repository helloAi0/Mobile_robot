"""The C++ mux core must behave exactly like the Python mux. Needs g++ (skipped without it)."""
import os
import re
import shutil
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))

pytestmark = pytest.mark.skipif(shutil.which('g++') is None, reason='g++ not installed')
INC = os.path.join(ROOT, 'include')


def build(src, out, include=INC, extra=()):
    subprocess.run(['g++', '-std=c++17', '-O1', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-I', include, src, '-o', out, *extra],
                   check=True, capture_output=True, text=True)
    return out


@pytest.fixture(scope='module')
def golden(tmp_path_factory):
    import gen_mux_golden
    d = tmp_path_factory.mktemp('golden')
    path = d / 'golden.txt'
    path.write_text(gen_mux_golden.build(n_each=600, n_scan=120))
    return str(path)


def run_golden(exe, golden):
    return subprocess.run([exe, golden], capture_output=True, text=True)


def test_cpp_core_matches_python_on_random_inputs(tmp_path, golden):
    exe = build(os.path.join(ROOT, 'test', 'cpp', 'test_mux_golden.cpp'), str(tmp_path / 'tmg'))
    r = run_golden(exe, golden)
    assert r.returncode == 0, r.stdout + r.stderr
    assert '0 mismatches' in r.stdout and r.stdout.count('0 mismatches') == 4


def test_the_golden_test_catches_a_broken_port(tmp_path, golden):
    """Mutation check: break the guard in a copy of the header; the golden test must fail."""
    inc = tmp_path / 'include' / 'mobile_robot_bringup'
    inc.mkdir(parents=True)
    src = open(os.path.join(INC, 'mobile_robot_bringup', 'mux_core.hpp')).read()
    assert 'if (front <= p.stop_dist)' in src
    (inc / 'mux_core.hpp').write_text(src.replace('if (front <= p.stop_dist)', 'if (front < p.stop_dist)'))
    exe = build(os.path.join(ROOT, 'test', 'cpp', 'test_mux_golden.cpp'), str(tmp_path / 'tmg_bad'), include=str(tmp_path / 'include'))
    assert run_golden(exe, golden).returncode != 0


def test_a_second_mutation_in_the_acceleration_limit_is_also_caught(tmp_path, golden):
    inc = tmp_path / 'include' / 'mobile_robot_bringup'
    inc.mkdir(parents=True)
    src = open(os.path.join(INC, 'mobile_robot_bringup', 'mux_core.hpp')).read()
    assert '2.0 * p.max_accel * dt' in src
    (inc / 'mux_core.hpp').write_text(src.replace('2.0 * p.max_accel * dt', '1.5 * p.max_accel * dt'))
    exe = build(os.path.join(ROOT, 'test', 'cpp', 'test_mux_golden.cpp'), str(tmp_path / 'tmg_bad2'), include=str(tmp_path / 'include'))
    assert run_golden(exe, golden).returncode != 0


def test_benchmark_builds_and_prints_json(tmp_path):
    import json
    exe = build(os.path.join(ROOT, 'test', 'cpp', 'mux_bench.cpp'), str(tmp_path / 'bench'))
    out = json.loads(subprocess.run([exe, '2000'], capture_output=True, text=True, check=True).stdout)
    assert out['impl'] == 'cpp' and out['tick_us']['median'] < 50 and out['scan_front_rear_us']['median'] < 1000


def test_node_parameters_and_topics_match_the_python_mux():
    py = open(os.path.join(ROOT, 'scripts', 'cmd_vel_mux.py')).read()
    cpp = open(os.path.join(ROOT, 'src', 'cmd_vel_mux_node.cpp')).read()
    py_params = dict(re.findall(r"d\('(\w+)', ([^)]+)\)", py))
    cpp_params = {m[0]: m[1] for m in re.findall(r'declare_parameter<[\w:]+>\("(\w+)", ([^)]+)\)', cpp)}
    assert set(py_params) == set(cpp_params), set(py_params) ^ set(cpp_params)
    for k, v in py_params.items():
        a, b = v.strip().strip("'").lower(), cpp_params[k].strip().strip('"').lower()
        if a not in ('true', 'false') and not a.startswith('/'):
            assert float(a) == float(b), k
        else:
            assert a == b, k
    for topic in ('/cmd_vel_teleop', '/cmd_vel', '/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel_auto', '/scan', '/emergency_stop', '/mux_status'):
        assert f"'{topic}'" in py and f'"{topic}"' in cpp, topic
    assert 'transient_local' in cpp and 'SensorDataQoS' in cpp
    assert 'steady_clock' in cpp


def test_cmake_builds_the_node_optionally_and_installs_it_for_the_launch_file():
    cm = open(os.path.join(ROOT, 'CMakeLists.txt')).read()
    assert 'add_executable(cmd_vel_mux_node' in cm and 'rclcpp_FOUND' in cm and 'install(TARGETS cmd_vel_mux_node' in cm
    launch = open(os.path.join(ROOT, 'launch', 'autonomy.launch.py')).read()
    assert "'cmd_vel_mux_node' if cpp else 'cmd_vel_mux.py'" in launch
