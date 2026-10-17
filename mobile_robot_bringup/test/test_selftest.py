import math

from selftest import angle_diff, diagnose, yaw_of

ALIVE = {'lidar (/scan)': True, 'IMU (/imu)': True}


def test_yaw_and_angle_wrap():
    assert abs(yaw_of((0, 0, math.sin(0.5), math.cos(0.5))) - 1.0) < 1e-9
    assert abs(angle_diff(-3.1, 3.1)) < 0.1


def test_healthy_robot_passes():
    assert all(ok for ok, _ in diagnose(ALIVE, 'teleop', 0.55, 1.1, 0.6, 1.2))


def test_mux_missing_is_named():
    msgs = [m for ok, m in diagnose(ALIVE, None, 0, 0, 0.6, 1.2) if not ok]
    assert any('cmd_vel_mux' in m for m in msgs)


def test_estop_is_named():
    msgs = [m for ok, m in diagnose(ALIVE, 'ESTOP', 0, 0, 0.6, 1.2) if not ok]
    assert any('estop.sh release' in m for m in msgs)


def test_safety_block_vs_dead_wheels():
    blocked = [m for ok, m in diagnose(ALIVE, 'teleop front obstacle', 0.0, 0, 0.6, 1.2) if not ok]
    assert any('SAFETY' in m for m in blocked)
    dead = [m for ok, m in diagnose(ALIVE, 'teleop', 0.0, 0, 0.6, 1.2) if not ok]
    assert any('list_controllers' in m for m in dead)


def test_dead_sensor_is_reported():
    msgs = [m for ok, m in diagnose({'lidar (/scan)': False}, 'teleop', 0.6, 1.2, 0.6, 1.2) if not ok]
    assert any('lidar' in m for m in msgs)
