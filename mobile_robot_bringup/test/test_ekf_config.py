import math
import os

import pytest
import yaml

import imu_covariance

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(ROOT)
CFG = os.path.join(ROOT, 'config')
EKF = yaml.safe_load(open(os.path.join(CFG, 'ekf.yaml')))['ekf_filter_node']['ros__parameters']
CTL = yaml.safe_load(open(os.path.join(CFG, 'my_controllers.yaml')))
FIELDS = ['x', 'y', 'z', 'roll', 'pitch', 'yaw', 'vx', 'vy', 'vz', 'vroll', 'vpitch', 'vyaw', 'ax', 'ay', 'az']


def used(cfg):
    return {f for f, u in zip(FIELDS, cfg) if u}


def test_sensor_vectors_have_15_entries_and_fuse_the_intended_axes():
    assert len(EKF['odom0_config']) == 15 and len(EKF['imu0_config']) == 15
    assert used(EKF['odom0_config']) == {'vx', 'vyaw'}          # wheel velocities, not the slip-corrupted pose
    assert used(EKF['imu0_config']) == {'vyaw'}                 # gyro only: accelerometer bias would integrate to a runaway speed


def test_noise_matrices_are_15x15_diagonal_and_positive():
    for key in ('process_noise_covariance', 'initial_estimate_covariance'):
        m = EKF[key]
        assert len(m) == 225
        for i in range(15):
            for j in range(15):
                v = m[i * 15 + j]
                assert (v > 0) if i == j else v == 0, (key, i, j)


def test_filter_is_planar_and_owns_the_odom_frame():
    assert EKF['two_d_mode'] is True and EKF['publish_tf'] is True
    assert EKF['world_frame'] == EKF['odom_frame'] == 'odom' and EKF['base_link_frame'] == 'base_link'
    assert CTL['diff_cont']['ros__parameters']['odom_frame_id'] == EKF['odom_frame']
    assert CTL['diff_cont']['ros__parameters']['base_frame_id'] == EKF['base_link_frame']


def test_topics_match_the_producers():
    assert EKF['odom0'] == '/diff_cont/odom'
    assert EKF['imu0'] == '/imu/data'
    src = open(os.path.join(ROOT, 'scripts', 'imu_covariance.py')).read()
    assert "'/imu/data'" in src and "'/imu'" in src
    gz = open(os.path.join(REPO, 'mobile_robot_gazebo', 'launch', 'gazebo.launch.py')).read()
    assert '/imu@sensor_msgs/msg/Imu' in gz, 'the IMU must be bridged from Gazebo'


def test_filter_runs_faster_than_the_controller_and_sensors():
    assert EKF['frequency'] >= CTL['diff_cont']['ros__parameters']['publish_rate']
    assert EKF['sensor_timeout'] <= 0.5


def test_gyro_dominates_yaw_rate_by_design():
    wheel_yaw_var = CTL['diff_cont']['ros__parameters']['twist_covariance_diagonal'][5]
    assert imu_covariance.GYRO_VAR * 100 < wheel_yaw_var          # at least 100x more trust in the gyro than in slipping wheels


def test_gyro_covariance_covers_the_urdf_noise_and_bias():
    urdf = open(os.path.join(REPO, 'mobile_robot_description', 'urdf', 'imu.xacro')).read()
    import re
    std = float(re.search(r'angular_velocity>.*?<stddev>([\d.]+)</stddev>', urdf, re.S).group(1))
    bias = float(re.search(r'angular_velocity>.*?<bias_stddev>([\d.]+)</bias_stddev>', urdf, re.S).group(1))
    assert imu_covariance.GYRO_VAR >= std ** 2 + bias ** 2
    acc_std = float(re.search(r'linear_acceleration>.*?<stddev>([\d.]+)</stddev>', urdf, re.S).group(1))
    assert imu_covariance.ACCEL_VAR >= acc_std ** 2


def test_imu_covariance_function():
    o, g, a = imu_covariance.covariances()
    assert o[0] == -1.0                                          # REP 145: no orientation
    assert g[0] == g[4] == g[8] == imu_covariance.GYRO_VAR and g[1] == 0.0
    assert a[0] == a[4] == a[8] == imu_covariance.ACCEL_VAR and len(a) == 9


def test_controller_publishes_covariances_for_the_axes_the_ekf_uses():
    d = CTL['diff_cont']['ros__parameters']
    assert len(d['twist_covariance_diagonal']) == 6 and len(d['pose_covariance_diagonal']) == 6
    assert 0 < d['twist_covariance_diagonal'][0] < 0.01 and 0 < d['twist_covariance_diagonal'][5] < 0.1


def test_imu_link_exists_in_the_robot():
    urdf = open(os.path.join(REPO, 'mobile_robot_description', 'urdf', 'imu.xacro')).read()
    assert '<link name="imu_link">' in urdf and '<gz_frame_id>imu_link</gz_frame_id>' in urdf


def test_no_two_publishers_of_odom_to_base_link():
    ov = yaml.safe_load(open(os.path.join(REPO, 'mobile_robot_gazebo', 'config', 'diff_cont_no_tf.yaml')))
    assert ov['diff_cont']['ros__parameters']['enable_odom_tf'] is False
    gz = open(os.path.join(REPO, 'mobile_robot_gazebo', 'launch', 'gazebo.launch.py')).read()
    assert 'diff_cont_no_tf.yaml' in gz and 'odom_tf' in gz
    sim = open(os.path.join(ROOT, 'launch', 'launch_sim.launch.py')).read()
    assert "'odom_tf'" in sim and "ekf" in sim
    hw = open(os.path.join(ROOT, 'launch', 'hardware.launch.py')).read()
    assert "enable_odom_tf'] = False" in hw
    assert CTL['diff_cont']['ros__parameters']['enable_odom_tf'] is True      # default stays: no EKF -> the controller publishes it


def test_mux_feeds_the_filtered_odometry_to_nav2_and_slam():
    a = open(os.path.join(ROOT, 'launch', 'autonomy.launch.py')).read()
    assert "'/odometry/filtered'" in a and "executable='ekf_node'" in a and 'ekf.yaml' in a


# ---------------------------------------------------------------- the planar model that mirrors the configuration
ekf_model = pytest.importorskip('ekf_model')


def test_model_takes_its_process_noise_from_the_yaml():
    m = ekf_model.PlanarEKF.from_yaml(os.path.join(CFG, 'ekf.yaml'))
    pn = EKF['process_noise_covariance']
    assert m.Q[0, 0] == pn[0] and m.Q[2, 2] == pn[5 * 15 + 5] and m.Q[3, 3] == pn[6 * 15 + 6] and m.Q[4, 4] == pn[11 * 15 + 11]


def test_model_ignores_wheel_slip_in_yaw_rate_because_the_gyro_is_trusted():
    f = ekf_model.PlanarEKF.from_yaml(os.path.join(CFG, 'ekf.yaml'))
    wheel_var = CTL['diff_cont']['ros__parameters']['twist_covariance_diagonal']
    odom_yaw = 0.0
    for _ in range(500):                                # 10 s at 50 Hz, wheels report 0.3 rad/s, body (and gyro) turn at 0
        f.predict(0.02)
        f.update_wheel(0.2, 0.3, wheel_var[0], wheel_var[5])
        f.update_gyro(0.0, imu_covariance.GYRO_VAR)
        odom_yaw += 0.3 * 0.02
    assert abs(odom_yaw) > 2.9 and abs(f.pose[2]) < 0.1


def test_model_without_a_gyro_follows_the_wheels():
    f = ekf_model.PlanarEKF.from_yaml(os.path.join(CFG, 'ekf.yaml'))
    for _ in range(500):
        f.predict(0.02)
        f.update_wheel(0.2, 0.3, 4e-4, 2.5e-3)
    assert abs(f.pose[2] - 3.0) < 0.3


def test_model_tracks_a_circle():
    f = ekf_model.PlanarEKF.from_yaml(os.path.join(CFG, 'ekf.yaml'))
    for _ in range(1000):
        f.predict(0.02)
        f.update_wheel(0.3, 0.5, 4e-4, 2.5e-3)
        f.update_gyro(0.5, imu_covariance.GYRO_VAR)
    x, y, th = f.pose
    assert abs(th - 10.0) < 0.1
    assert math.hypot(x, y) < 2 * 0.3 / 0.5 + 0.2          # stays on a circle of radius v / w = 0.6
