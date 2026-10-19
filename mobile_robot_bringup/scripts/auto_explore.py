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
TURN_CLEAR = 0.34    # m from the lidar: turning on the spot sweeps a 0.30 m circle (nose corner), so a closer obstacle is first backed away from
CREEP = 0.12         # m/s while making room to turn
YIELD_RANGE = 2.5    # m, something this close that keeps coming closer on its own (a walking person) -> back away from it
YIELD_CLOSING = 0.3   # m/s, approach speed that the robot's own motion does not explain
YIELD_SPEED = 0.55   # m/s, how fast the robot gets out of the way
YIELD_TIME = 2.0     # s per yield manoeuvre


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


def nearest(ranges, angle_min, inc, rmax=12.0):
    """(range, bearing) of the closest valid beam, or (rmax, 0.0) if there is none. Bearing is in the robot frame, rad."""
    best, bi = rmax, None
    for i, r in enumerate(ranges):
        if math.isfinite(r) and 0.15 < r < best:
            best, bi = r, i
    return (best, angle_min + bi * inc) if bi is not None else (rmax, 0.0)


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
    def __init__(self, rng=None, yield_enabled=True):
        self.state = 'FORWARD'
        self.until = 0.0
        self.turn_dir = 1
        self.turn_started = 0.0
        self.extra = 0.0
        self.rng = rng or random.Random()
        self.message = ''
        self.ref = None              # (time, x, y) when progress was last measured
        self.track = []              # recent (time, range, bearing) of the nearest object, to see things that approach by themselves
        self.last_lin = 0.0
        self.yield_dir = -1.0
        self.yield_enabled = yield_enabled

    def rear_clear(self, ranges, angle_min, inc):
        d = math.radians
        back = min(sector_min(ranges, angle_min, inc, d(140), d(180)), sector_min(ranges, angle_min, inc, -d(180), -d(140)))
        return back > 0.4

    def approaching(self, ranges, angle_min, inc, now):
        """Bearing (rad) of an object that is close and moving towards the robot by itself, else None.
        Compares the nearest object now with 0.4..0.8 s ago, after subtracting the closing speed caused by the robot's own
        motion (a static wall closes at v * cos(bearing)). A walking person is the case this is made for."""
        r, b = nearest(ranges, angle_min, inc)
        self.track = [t for t in self.track if now - t[0] <= 0.8] + [(now, r, b)]
        if r > YIELD_RANGE:
            return None
        for (t0, r0, b0) in self.track:
            dt = now - t0
            if dt >= 0.4 and abs(math.atan2(math.sin(b - b0), math.cos(b - b0))) < math.radians(30):
                own = self.last_lin * math.cos(b)
                closing = (r0 - r) / dt - own
                if closing > YIELD_CLOSING:
                    return b
                break
        return None

    def step(self, ranges, angle_min, inc, now, pos=None):
        lin, ang = self._step(ranges, angle_min, inc, now, pos)
        self.last_lin = lin
        return lin, ang

    def _step(self, ranges, angle_min, inc, now, pos=None):
        """Returns (linear, angular). self.message is set when the state changes.
        pos = (x, y) from odometry (optional): used to notice that the robot is stuck although it is told to drive."""
        d = math.radians
        front = sector_min(ranges, angle_min, inc, -d(35), d(35))
        near = sector_min(ranges, angle_min, inc, -d(60), d(60))
        left = sector_min(ranges, angle_min, inc, d(20), d(90))
        right = sector_min(ranges, angle_min, inc, -d(90), -d(20))
        self.message = ''
        threat = self.approaching(ranges, angle_min, inc, now) if self.yield_enabled else None
        if self.state == 'YIELD':
            if now >= self.until:
                self.state, self.until = 'STOP', now + 0.3
                return 0.0, 0.0
            direction = self.yield_dir
            ok = (self.rear_clear(ranges, angle_min, inc) if direction < 0 else front > STOP_DIST + 0.15)
            if not ok:
                self.state, self.until = 'STOP', now + 0.2
                return 0.0, 0.0
            return direction * YIELD_SPEED, 0.0
        if threat is not None and self.state in ('FORWARD', 'STOP', 'TURN'):
            away = -1.0 if math.cos(threat) > 0.0 else 1.0           # object ahead -> reverse, object behind -> drive on
            if (away < 0 and self.rear_clear(ranges, angle_min, inc)) or (away > 0 and front > STOP_DIST + 0.15):
                self.state, self.until, self.yield_dir = 'YIELD', now + YIELD_TIME, away
                self.turn_dir = 1 if sector_mean(ranges, angle_min, inc, d(30), d(110)) >= sector_mean(ranges, angle_min, inc, -d(110), -d(30)) else -1
                self.extra = self.rng.uniform(0.0, 0.8)
                self.message = f'something approaches ({math.degrees(threat):.0f} deg): moving out of the way'
                return 0.0, 0.0
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
        # TURN: rotate on the spot until the way ahead is free (and a little more, for variety).
        # First make room: the robot's corners sweep a circle of 0.30 m around the axle, a wall closer than that would be hit.
        near_r, near_b = nearest(ranges, angle_min, inc)
        if near_r < TURN_CLEAR:
            c = math.cos(near_b)
            if c > 0.2:
                away = -1.0 if self.rear_clear(ranges, angle_min, inc) else 0.0
            elif c < -0.2:
                away = 1.0 if front > STOP_DIST + 0.1 else 0.0
            else:                                              # obstacle at the side: go where there is more room
                away = 1.0 if front > STOP_DIST + 0.3 else (-1.0 if self.rear_clear(ranges, angle_min, inc) else 0.0)
            if away != 0.0:
                self.turn_started = max(self.turn_started, now - 0.5)       # do not count the creeping as turning time
                return away * CREEP, 0.0
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
