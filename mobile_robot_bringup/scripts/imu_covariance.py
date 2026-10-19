#!/usr/bin/env python3
"""/imu (Gazebo or a real driver, covariance unknown) -> /imu/data with the covariances the EKF needs.

robot_localization weights each sensor by the covariance in its message. Gazebo (and many drivers) publish zeros
("unknown"); the filter would then trust the IMU absolutely. This node fills in honest numbers:

  angular velocity   variance GYRO_VAR = 4e-6 (rad/s)^2   = 2 mrad/s: datasheet noise 0.9 mrad/s plus the turn-on bias 0.8 mrad/s
  linear acceleration variance ACCEL_VAR = 9e-4 (m/s^2)^2 = 0.03 m/s^2: noise 0.011 plus bias 0.02 (not fused, kept for completeness)
  orientation        not provided: covariance[0] = -1 (REP 145), so nobody uses the (world-referenced) simulator quaternion

   ros2 run mobile_robot_bringup imu_covariance.py
"""
GYRO_VAR = 4.0e-6
ACCEL_VAR = 9.0e-4


def diag(var):
    """3x3 covariance, row-major, as the 9 floats of sensor_msgs/Imu."""
    return [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var]


def covariances(gyro_var=GYRO_VAR, accel_var=ACCEL_VAR):
    """(orientation, angular_velocity, linear_acceleration) covariance arrays."""
    return [-1.0] + [0.0] * 8, diag(gyro_var), diag(accel_var)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu

    class ImuCov(Node):
        def __init__(self):
            super().__init__('imu_covariance')
            gv = self.declare_parameter('gyro_variance', GYRO_VAR).value
            av = self.declare_parameter('accel_variance', ACCEL_VAR).value
            self.cov = covariances(gv, av)
            self.pub = self.create_publisher(Imu, '/imu/data', 10)
            self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)

        def on_imu(self, m):
            m.orientation_covariance, m.angular_velocity_covariance, m.linear_acceleration_covariance = self.cov
            self.pub.publish(m)

    rclpy.init()
    n = ImuCov()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    try:
        n.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
