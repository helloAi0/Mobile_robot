#!/usr/bin/env python3
"""Autonomous wandering for mapping: drive forward, STOP when something (wall, furniture, a walking person) is in
front, pick the side with more free space, turn until the way is clear, continue. slam_toolbox remembers
everything it sees and builds the map while the robot wanders.

Uses only the lidar (/scan), so it works on the real robot as well. Output: /cmd_vel_auto (Twist), only while
auto mode is on. Switch auto mode on/off:
    ros2 launch mobile_robot_bringup launch_sim.launch.py auto:=true         # start in auto mode
    ros2 topic pub --once /auto_mode std_msgs/msg/Bool "{data: true}" # on  (false = off)
    key 'a' in teleop_keys.py                                         # toggle
Any keyboard command (teleop_keys.py) overrides it while you drive; auto continues when you stop pressing keys.
"""
import math
import random

STOP_DIST = 0.55     # m, measured from the lidar: obstacle closer than this in front -> stop
CLEAR_DIST = 1.1     # m, free space needed in front before driving again
SPEED = 0.22         # m/s
TURN = 0.8           # rad/s
SIDE_DIST = 0.45     # m, start steering away when a side is closer than this


def sector_min(ranges, angle_min, inc, a0, a1, rmax=12.0):
    """Smallest valid range between robot-frame angles a0..a1 (rad). Returns rmax if nothing valid."""
    best = rmax
    n = len(ranges)
    i0 = max(0, int(math.floor((a0 - angle_min) / inc)))
    i1 = min(n - 1, int(math.ceil((a1 - angle_min) / inc)))
    for i in range(i0, i1 + 1):
        r = ranges[i]
        if math.isfinite(r) and r > 0.01 and r < best:
            best = r
    return best


def sector_mean(ranges, angle_min, inc, a0, a1, cap=4.0):
    vals = []
    n = len(ranges)
    i0 = max(0, int(math.floor((a0 - angle_min) / inc)))
    i1 = min(n - 1, int(math.ceil((a1 - angle_min) / inc)))
    for i in range(i0, i1 + 1):
        r = ranges[i]
        vals.append(min(r, cap) if math.isfinite(r) else cap)
    return sum(vals) / len(vals) if vals else cap


class Explorer:
    def __init__(self, rng=None):
        self.state = 'FORWARD'
        self.until = 0.0
        self.turn_dir = 1
        self.turn_started = 0.0
        self.extra = 0.0
        self.rng = rng or random.Random()
        self.message = ''
        self.ref = None              # (time, x, y) when progress was last measured

    def rear_clear(self, ranges, angle_min, inc):
        d = math.radians
        back = min(sector_min(ranges, angle_min, inc, d(140), d(180)), sector_min(ranges, angle_min, inc, -d(180), -d(140)))
        return back > 0.4

    def step(self, ranges, angle_min, inc, now, pos=None):
        """Returns (linear, angular). self.message is set when the state changes.
        pos = (x, y) from odometry (optional): used to notice that the robot is stuck although it is told to drive."""
        d = math.radians
        front = sector_min(ranges, angle_min, inc, -d(35), d(35))
        near = sector_min(ranges, angle_min, inc, -d(60), d(60))
        left = sector_min(ranges, angle_min, inc, d(20), d(90))
        right = sector_min(ranges, angle_min, inc, -d(90), -d(20))
        self.message = ''
        if self.state == 'BACKUP':
            if now >= self.until or not self.rear_clear(ranges, angle_min, inc):
                self.state, self.turn_started = 'TURN', now
                return 0.0, 0.0
            return -0.15, 0.0
        if self.state == 'FORWARD' and pos is not None:
            if self.ref is None:
                self.ref = (now, pos[0], pos[1])
            elif now - self.ref[0] > 4.0:
                moved = math.hypot(pos[0] - self.ref[1], pos[1] - self.ref[2])
                self.ref = (now, pos[0], pos[1])
                if moved < 0.06:
                    self.state, self.until = 'BACKUP', now + 1.2
                    self.turn_dir = self.rng.choice([-1, 1])
                    self.extra = self.rng.uniform(0.5, 1.2)
                    self.message = 'no progress for 4 s (stuck?): backing up, then turning'
                    return 0.0, 0.0
        else:
            self.ref = None
        if self.state == 'FORWARD':
            if front < STOP_DIST or near < 0.30:
                self.state, self.until = 'STOP', now + 0.6
                lm = sector_mean(ranges, angle_min, inc, d(30), d(110))
                rm = sector_mean(ranges, angle_min, inc, -d(110), -d(30))
                self.turn_dir = 1 if lm > rm + 0.05 else (-1 if rm > lm + 0.05 else self.rng.choice([-1, 1]))
                self.extra = self.rng.uniform(0.0, 0.8)
                self.message = (f'obstacle {front:.2f} m ahead: STOP, then turn '
                                f'{"left" if self.turn_dir > 0 else "right"}')
                return 0.0, 0.0
            w = 0.0
            if left < SIDE_DIST:
                w -= 0.6
            if right < SIDE_DIST:
                w += 0.6
            return SPEED, w
        if self.state == 'STOP':
            if now >= self.until:
                self.state, self.turn_started = 'TURN', now
            return 0.0, 0.0
        # TURN: rotate on the spot until the way ahead is free (and a little more, for variety)
        if front > CLEAR_DIST and now - self.turn_started > 0.8 + self.extra:
            self.state = 'FORWARD'
            self.message = 'way is clear: driving on'
            return SPEED, 0.0
        if now - self.turn_started > 12.0:      # boxed in: turn the other way
            self.turn_dir = -self.turn_dir
            self.turn_started = now
        return 0.0, TURN * self.turn_dir


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool

    class AutoExplore(Node):
        def __init__(self):
            super().__init__('auto_explore')
            self.declare_parameter('enabled', False)
            self.enabled = self.get_parameter('enabled').value
            self.ex = Explorer()
            self.scan = None
            self.pos = None
            self.scan_time = 0.0
            self.create_subscription(Odometry, '/odom', self.on_odom, 10)
            self.pub = self.create_publisher(Twist, '/cmd_vel_auto', 10)
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
            self.create_subscription(Bool, '/auto_mode', self.on_mode, 10)
            self.create_timer(0.05, self.tick)
            self.get_logger().info(f'auto_explore ready, auto mode is {"ON" if self.enabled else "OFF"} '
                                   '(ros2 topic pub --once /auto_mode std_msgs/msg/Bool "{data: true}")')

        def on_mode(self, msg):
            if msg.data != self.enabled:
                self.enabled = msg.data
                self.ex = Explorer()
                self.get_logger().info(f'auto mode {"ON" if self.enabled else "OFF"}')

        def on_odom(self, msg):
            self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        def on_scan(self, msg):
            self.scan = msg
            self.scan_time = self.get_clock().now().nanoseconds * 1e-9

        def tick(self):
            if not self.enabled or self.scan is None:
                return
            now = self.get_clock().now().nanoseconds * 1e-9
            t = Twist()
            if now - self.scan_time < 1.0:     # no fresh scan -> do not drive blind
                lin, ang = self.ex.step(list(self.scan.ranges), self.scan.angle_min, self.scan.angle_increment, now, self.pos)
                t.linear.x, t.angular.z = float(lin), float(ang)
                if self.ex.message:
                    self.get_logger().info(self.ex.message)
            self.pub.publish(t)

    rclpy.init()
    node = AutoExplore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
