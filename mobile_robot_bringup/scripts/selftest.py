#!/usr/bin/env python3
"""Pre-flight check: drives the robot WITHOUT the keyboard and tells you exactly which part is broken.

   ros2 run mobile_robot_bringup selftest.py            # while the simulation is running; the robot moves ~1 m

Checks: sensors alive (lidar, IMU, odometry, camera), command mux alive, then drives forward, turns, reverses through the
real command path (/cmd_vel_teleop -> mux -> diff_drive_controller -> Gazebo) and compares the odometry with the command.
Exit code 0 = everything works. Pure helpers are unit-tested without ROS."""
import math
import sys
import time


def yaw_of(q):
    return math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]))


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def diagnose(topics_alive, mux_status, moved_m, turned_rad, expect_m, expect_rad):
    """-> list of (ok, message). Pure function: easy to test."""
    out = []
    for name, ok in topics_alive.items():
        out.append((ok, f'{name}: {"receiving data" if ok else "NO DATA"}'))
    if mux_status is None:
        out.append((False, 'cmd_vel_mux: not running (no /mux_status). Start the simulation with launch_sim.launch.py'))
    elif mux_status.startswith('ESTOP'):
        out.append((False, 'emergency stop is ENGAGED. Release it: ros2 run mobile_robot_bringup estop.sh release'))
    else:
        out.append((True, f'cmd_vel_mux: running ({mux_status})'))
    fwd_ok = moved_m >= 0.4 * expect_m
    if fwd_ok:
        out.append((True, f'drive forward: moved {moved_m:.2f} m (commanded about {expect_m:.2f} m)'))
    elif mux_status and ('front' in mux_status.lower() or 'stop' in mux_status.lower() or 'scan' in mux_status.lower()):
        out.append((False, f'drive forward: moved only {moved_m:.2f} m, the SAFETY layer blocked it ({mux_status}). '
                           'Something is in front of the robot or /scan is missing: back away (key m) or check the lidar.'))
    else:
        out.append((False, f'drive forward: moved only {moved_m:.2f} m although the mux accepted the command. '
                           'The wheels do not turn: check  ros2 control list_controllers  (diff_cont must be "active") '
                           'and whether the robot is stuck on an object in Gazebo.'))
    turn_ok = abs(turned_rad) >= 0.4 * expect_rad
    out.append((turn_ok, f'turn in place: rotated {math.degrees(turned_rad):.0f} deg (commanded about {math.degrees(expect_rad):.0f})'))
    return out


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Image, Imu, LaserScan
    from std_msgs.msg import String

    class Test(Node):
        def __init__(self):
            super().__init__('selftest')
            self.count = {'lidar (/scan)': 0, 'IMU (/imu)': 0, 'odometry (/diff_cont/odom)': 0, 'camera (/camera/image_raw)': 0}
            self.pose = None
            self.status = None
            be = qos_profile_sensor_data
            self.create_subscription(LaserScan, '/scan', lambda m: self.bump('lidar (/scan)'), be)
            self.create_subscription(Imu, '/imu', lambda m: self.bump('IMU (/imu)'), be)
            self.create_subscription(Image, '/camera/image_raw', lambda m: self.bump('camera (/camera/image_raw)'), be)
            self.create_subscription(Odometry, '/diff_cont/odom', self.on_odom, QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=5))
            self.create_subscription(String, '/mux_status', lambda m: setattr(self, 'status', m.data), 10)
            self.pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)

        def bump(self, k):
            self.count[k] += 1

        def on_odom(self, m):
            p, q = m.pose.pose.position, m.pose.pose.orientation
            self.pose = (p.x, p.y, yaw_of((q.x, q.y, q.z, q.w)))
            self.bump('odometry (/diff_cont/odom)')

        def spin_for(self, sec, lin=None, ang=0.0):
            end = time.time() + sec
            while time.time() < end and rclpy.ok():
                if lin is not None:
                    t = Twist()
                    t.linear.x, t.angular.z = lin, ang
                    self.pub.publish(t)
                rclpy.spin_once(self, timeout_sec=0.05)

    rclpy.init()
    n = Test()
    print('Waiting for the simulation (up to 40 s) ...')
    n.spin_for(4.0)
    t0 = time.time()
    while n.pose is None and time.time() - t0 < 36:
        n.spin_for(0.5)
    alive = {k: v > 0 for k, v in n.count.items()}
    if n.pose is None:
        for ok, msg in diagnose(alive, n.status, 0, 0, 0.6, 1.2):
            print(('PASS  ' if ok else 'FAIL  ') + msg)
        print('\nNo odometry: is  ros2 launch mobile_robot_bringup launch_sim.launch.py  running and fully started?')
        return 1
    p0 = n.pose
    n.spin_for(3.0, lin=0.2)                       # 0.6 m at 0.2 m/s
    p1 = n.pose
    n.spin_for(0.8, lin=0.0)
    mux_during = n.status
    n.spin_for(2.0, lin=0.0, ang=0.6)              # 1.2 rad
    p2 = n.pose
    n.spin_for(1.5, lin=-0.2)                      # back up
    n.spin_for(0.8, lin=0.0)
    moved = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    turned = angle_diff(p2[2], p1[2])
    results = diagnose(alive, mux_during, moved, turned, 0.6, 1.2)
    print()
    for ok, msg in results:
        print(('PASS  ' if ok else 'FAIL  ') + msg)
    good = all(ok for ok, _ in results)
    print('\n' + ('ALL GOOD: the robot drives. Use  ros2 run mobile_robot_bringup teleop_keys.py  and type in THAT terminal.'
                  if good else 'Fix the FAIL lines above, then run this again.'))
    try:
        n.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass
    return 0 if good else 1


if __name__ == '__main__':
    sys.exit(main())
