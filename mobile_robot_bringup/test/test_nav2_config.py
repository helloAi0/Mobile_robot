import ast
import math
import os

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(ROOT)
N2 = yaml.safe_load(open(os.path.join(ROOT, 'config', 'nav2_params.yaml')))
CTL = yaml.safe_load(open(os.path.join(ROOT, 'config', 'my_controllers.yaml')))['diff_cont']['ros__parameters']
MUX = open(os.path.join(ROOT, 'scripts', 'cmd_vel_mux.py')).read()
FP = ast.literal_eval(N2['local_costmap']['local_costmap']['ros__parameters']['footprint'])
DWB = N2['controller_server']['ros__parameters']['FollowPath']
INFL = N2['local_costmap']['local_costmap']['ros__parameters']['inflation_layer']
WORLD_GLOBAL = N2['global_costmap']['global_costmap']['ros__parameters']


def mux_default(name):
    import re
    return float(re.search(r"d\('%s', ([\d.]+)\)" % name, MUX).group(1))


def test_footprint_is_the_robots_and_identical_in_both_costmaps():
    assert FP == ast.literal_eval(WORLD_GLOBAL['footprint'])
    try:
        import house_map as hm
    except ImportError:
        pytest.skip('needs scipy')        # inside a test function a skip is fine
    assert [tuple(p) for p in FP] == [tuple(p) for p in hm.FOOTPRINT]
    import benchmark_metrics as bm
    assert [tuple(p) for p in FP] == [tuple(p) for p in bm.FOOTPRINT]


def test_footprint_matches_the_urdf_geometry():
    urdf = open(os.path.join(REPO, 'mobile_robot_description', 'urdf', 'robot_core.xacro')).read()
    assert 'name="track_half" value="0.175"' in urdf
    half_width_axle = max(abs(p[1]) for p in FP)
    assert 0.175 < half_width_axle <= 0.175 + 0.04              # wheel centre plus at most one tyre width
    assert max(p[0] for p in FP) >= 0.26 - 0.01                 # nose: chassis origin -0.10 + caster/box extent


def test_inflation_leaves_a_free_lane_in_a_doorway():
    radius = max(math.hypot(*p) for p in FP)
    inscribed = min(abs(p[1]) for p in FP)
    assert 0.25 < radius < 0.31
    assert INFL['inflation_radius'] > radius                    # costs start outside the circumscribed circle
    door, width = 1.2, 2 * max(abs(p[1]) for p in FP)
    # centre of the doorway: distance to each jamb = 0.6. Cost there must stay below the lethal/inscribed band
    d_jamb = door / 2
    assert d_jamb > INFL['inflation_radius'] * 0.95 or INFL['inflation_radius'] < d_jamb + 0.1
    cost_at_centre = 252 * math.exp(-INFL['cost_scaling_factor'] * (d_jamb - radius))
    assert cost_at_centre < 253 and d_jamb > radius + 0.2, 'doorway must stay passable'
    assert width < door - 0.5


def test_planner_speeds_do_not_exceed_what_the_mux_and_controller_allow():
    assert DWB['max_vel_x'] <= mux_default('max_linear') <= CTL['linear.x.max_velocity']
    assert DWB['max_vel_theta'] <= mux_default('max_angular') <= CTL['angular.z.max_velocity']
    assert DWB['acc_lim_x'] <= mux_default('max_accel') + 1e-9
    assert DWB['acc_lim_theta'] <= mux_default('max_ang_accel') + 1e-9
    vs = N2['velocity_smoother']['ros__parameters']
    assert vs['max_velocity'][0] == DWB['max_vel_x'] and vs['max_velocity'][2] == DWB['max_vel_theta']
    assert vs['max_accel'][0] <= mux_default('max_accel') and abs(vs['max_decel'][0]) <= abs(CTL['linear.x.min_acceleration'])
    assert vs['max_velocity'][1] == 0.0                         # differential drive: no sideways speed


def test_reverse_speed_is_limited_but_possible():
    vs = N2['velocity_smoother']['ros__parameters']
    assert -0.3 <= vs['min_velocity'][0] < 0                    # the tail is blind behind +-137 degrees: back up slowly


def test_navigation_is_planned_for_a_slower_speed_than_the_mux_guard_needs_to_stop_from():
    # stopping distance from cruise speed with the controller deceleration must fit inside the guard's slowing band
    v, a = DWB['max_vel_x'], abs(DWB['decel_lim_x'])
    stop = v * v / (2 * a)
    assert stop < mux_default('safety_slow_distance') - mux_default('safety_stop_distance')


def test_goal_tolerances_are_looser_than_odometry_noise_and_tighter_than_a_doorway():
    t = N2['controller_server']['ros__parameters']['general_goal_checker']
    assert 0.05 <= t['xy_goal_tolerance'] <= 0.3 and 0.1 <= t['yaw_goal_tolerance'] <= 0.5
    assert DWB['xy_goal_tolerance'] == t['xy_goal_tolerance']


def test_frames_and_topics_line_up_with_the_rest_of_the_stack():
    assert N2['bt_navigator']['ros__parameters']['global_frame'] == 'map'
    assert N2['bt_navigator']['ros__parameters']['robot_base_frame'] == CTL['base_frame_id']
    assert N2['bt_navigator']['ros__parameters']['odom_topic'] == '/odom'
    assert N2['local_costmap']['local_costmap']['ros__parameters']['global_frame'] == CTL['odom_frame_id']
    for cm in (N2['local_costmap']['local_costmap'], N2['global_costmap']['global_costmap']):
        layers = [k for k in cm['ros__parameters'] if isinstance(cm['ros__parameters'][k], dict) and 'scan' in cm['ros__parameters'][k]]
        assert layers and cm['ros__parameters'][layers[0]]['scan']['topic'] == '/scan'
    assert '/cmd_vel_nav' in MUX and '/cmd_vel_smoothed' in MUX         # both are inputs of the mux


def test_every_node_uses_sim_time():
    for node, body in N2.items():
        params = body.get('ros__parameters') or next(iter(body.values()))['ros__parameters']
        assert params['use_sim_time'] is True, node


def test_costmap_sensor_ranges_match_the_lidar():
    for cm in (N2['local_costmap']['local_costmap'], N2['global_costmap']['global_costmap']):
        p = cm['ros__parameters']
        scan = next(v['scan'] for v in p.values() if isinstance(v, dict) and 'scan' in v)
        assert scan['obstacle_max_range'] < 12.0 and scan['raytrace_max_range'] <= 12.0 and scan['clearing'] and scan['marking']


def test_launch_passes_the_tuned_params_to_nav2():
    a = open(os.path.join(ROOT, 'launch', 'autonomy.launch.py')).read()
    assert 'nav2_params.yaml' in a and 'patrol.py' in a


def test_global_planner_can_plan_through_the_unexplored_map():
    p = N2['planner_server']['ros__parameters']['GridBased']
    assert p['allow_unknown'] is True and p['use_astar'] is True and p['tolerance'] > 0


def test_nav2_nodes_are_started_one_by_one_not_with_navigation_launch():
    """navigation_launch.py of newer Nav2 releases also starts collision_monitor / route_server / docking, which need their
    own parameters and abort the whole bring-up when missing (seen on a real Jazzy install)."""
    a = open(os.path.join(ROOT, 'launch', 'autonomy.launch.py')).read()
    code = [l for l in a.splitlines() if not l.strip().startswith('#')]
    assert not any("'navigation_launch.py'" in l for l in code)
    for exe in ('controller_server', 'planner_server', 'bt_navigator', 'velocity_smoother', 'lifecycle_manager'):
        assert exe in a
    assert 'collision_monitor' not in a.split('node_names')[1]
