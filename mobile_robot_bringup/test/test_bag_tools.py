import math
import os
import re
import subprocess

import pytest
import yaml

import bag_report as br
import benchmark_metrics as bm

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(ROOT)
TOPICS = yaml.safe_load(open(os.path.join(ROOT, 'config', 'bag_topics.yaml')))
QOS = yaml.safe_load(open(os.path.join(ROOT, 'config', 'bag_qos.yaml')))
N = 720
INC = 2 * 3.14159 / (N - 1)


def scan(front):
    r = [math.inf] * N
    for i in range(N):
        if abs(-3.14159 + i * INC) < 0.05:
            r[i] = front
    return r, -3.14159, INC, 0.15


def test_topic_groups_cover_what_replay_and_the_report_need():
    core = set(TOPICS['core'])
    assert {'/clock', '/tf', '/tf_static', '/robot_description', '/scan', '/map', '/diff_cont/odom', '/odometry/filtered',
            '/imu/data', '/mux_status', '/diff_cont/cmd_vel'} <= core
    assert '/camera/image_raw' in TOPICS['camera'] and not core & set(TOPICS['camera'])
    assert all(t.startswith('/') for g in TOPICS.values() for t in g)


def test_latched_topics_are_recorded_transient_local():
    for t in ('/tf_static', '/robot_description', '/map'):
        assert QOS[t]['durability'] == 'transient_local'


def test_every_recorded_topic_is_produced_by_something_in_the_repo():
    text = ''
    for sub in ('mobile_robot_bringup', 'mobile_robot_gazebo', 'mobile_robot_description'):
        for d, _, fs in os.walk(os.path.join(REPO, sub)):
            if 'models' in d:
                continue
            for f in fs:
                if f.endswith(('.py', '.yaml', '.xacro')):
                    text += open(os.path.join(d, f), errors='ignore').read()
    external = {'/clock', '/tf', '/tf_static', '/robot_description', '/map', '/plan', '/goal_pose', '/odom', '/imu', '/diff_cont/odom',
                '/camera/camera_info', '/odometry/filtered', '/cmd_vel_smoothed', '/cmd_vel_nav'}      # produced by stacks we launch
    for t in TOPICS['core'] + TOPICS['camera']:
        assert t in text or t in external, t


def test_recorder_script_syntax_and_flags():
    path = os.path.join(ROOT, 'scripts', 'record_bag.sh')
    assert subprocess.run(['bash', '-n', path]).returncode == 0
    src = open(path).read()
    assert 'bag_topics.yaml' in src and 'bag_qos.yaml' in src and 'zstd' in src and '--camera' in src


def test_replay_command_line():
    import importlib.util
    spec = importlib.util.spec_from_file_location('replay_launch', os.path.join(ROOT, 'launch', 'replay.launch.py'))
    src = open(os.path.join(ROOT, 'launch', 'replay.launch.py')).read()
    ns = {}
    exec(compile(re.search(r'def play_command.*?return cmd\n', src, re.S).group(0), 'replay', 'exec'), ns)
    cmd = ns['play_command']('b', 2.0, 5, 'true', 'q.yaml')
    assert cmd[:4] == ['ros2', 'bag', 'play', 'b'] and '--clock' in cmd and '--loop' in cmd and cmd[cmd.index('--rate') + 1] == '2.0'
    assert '--loop' not in ns['play_command']('b', 1.0, 0, 'false', 'q.yaml')


def test_replay_rviz_compares_raw_and_fused_odometry_on_a_map_frame():
    d = yaml.safe_load(open(os.path.join(ROOT, 'rviz', 'replay.rviz')))['Visualization Manager']
    topics = {x.get('Topic', {}).get('Value') for x in d['Displays']}
    assert {'/map', '/scan', '/diff_cont/odom', '/odometry/filtered', '/plan'} <= topics
    assert d['Global Options']['Fixed Frame'] == 'map'


def test_every_replay_display_topic_is_recorded():
    d = yaml.safe_load(open(os.path.join(ROOT, 'rviz', 'replay.rviz')))['Visualization Manager']['Displays']
    recorded = set(TOPICS['core'] + TOPICS['camera'])
    for x in d:
        t = x.get('Topic', {}).get('Value')
        if t and x['Enabled']:
            assert t in recorded, t + ' is shown but not recorded'


def odom(x, y, yaw=0.0):
    return (x, y, yaw)


def test_report_distance_clearance_and_contacts():
    msgs = [('/odom', 0.0, odom(0, 0)), ('/scan', 0.1, scan(2.0)), ('/odom', 1.0, odom(1, 0)), ('/scan', 1.1, scan(0.7)),
            ('/odom', 2.0, odom(2, 0)), ('/scan', 2.1, scan(0.2))]
    r = br.analyse(msgs)
    assert r['distance_m'] == 2.0 and r['distance_source'] == '/odom' and r['scans'] == 3
    assert r['contacts'] == 1 and r['min_clearance_m'] < 0
    assert abs(r['duration_s'] - 2.1) < 0.05


def test_report_prefers_the_fused_odometry_and_shows_the_slip_error():
    msgs = []
    for k in range(11):
        t = k * 0.1
        msgs.append(('/diff_cont/odom', t, odom(0.2 * t, 0.0, 0.0 + 0.3 * t)))      # wheels claim a turn that never happened
        msgs.append(('/odometry/filtered', t, odom(0.2 * t, 0.0, 0.0)))             # fused: the gyro says straight
    r = br.analyse(msgs)
    assert r['distance_source'] == '/odometry/filtered'
    assert r['raw_vs_fused']['final_heading_difference_deg'] == pytest.approx(math.degrees(0.3), abs=0.01)
    assert 'limit events' in br.format_report(r)


def test_report_counts_mux_limits_once_per_change():
    msgs = [('/mux_status', t, s) for t, s in enumerate(['auto', 'auto slowing', 'auto slowing', 'auto', 'auto obstacle', 'auto obstacle', 'auto slowing'])]
    r = br.analyse(msgs)
    assert r['mux_limit_events'] == {'slowing': 2, 'obstacle': 1}


def test_report_of_an_empty_bag_does_not_crash():
    r = br.analyse([])
    assert r['duration_s'] == 0.0 and r['distance_m'] is None and r['min_clearance_m'] is None


def test_yaw_of_quaternion():
    assert br.yaw_of((0, 0, math.sin(0.4), math.cos(0.4))) == pytest.approx(0.8)
