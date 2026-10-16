#!/usr/bin/env python3
"""Command multiplexer + safety layer between ALL drivers (keyboard, Nav2, autonomy) and the drive controller.
Same node for simulation and real hardware.

  priority 1  /cmd_vel_teleop   keyboard (ros2 run mobile_robot teleop_keys.py)                          Twist
  priority 2  /cmd_vel, /cmd_vel_nav, /cmd_vel_smoothed   Nav2 or any other teleop / joystick             Twist
  priority 3  /cmd_vel_auto     autonomous wandering (auto_explore.py)                                   Twist
  nothing fresh -> zero velocity (the robot stops by itself)

Safety layer applied to whatever wins (this is what makes it safe to demo and to put on hardware):
  * emergency stop    /emergency_stop (std_msgs/Bool, true = stop and stay stopped, false = release)
  * obstacle guard    forward speed is reduced linearly between 'safety_slow_distance' and 'safety_stop_distance'
                      (measured by the lidar in a +-'safety_cone_deg' cone in front) and is zero closer than that.
                      Reversing and turning on the spot are always allowed, so you can always back away.
                      No fresh /scan (older than 1 s) = no forward motion (fail-safe).
  * limits            max_linear / max_angular and acceleration limits (smooth, no wheel-slip jerks); braking by the
                      obstacle guard / e-stop is immediate.
Output: /diff_cont/cmd_vel (TwistStamped, ros2_control diff_drive_controller on ROS 2 Jazzy), 20 times per second.
Parameters 'stamped:=false' + 'output_topic' publish a plain Twist instead, for drivers that want that.
Also copies /diff_cont/odom to /odom, because Nav2 expects /odom.
Status topic /mux_status (std_msgs/String): "<source> <limited_by>" at 2 Hz, shown in the teleop window.
"""
import math
import time

PRIORITY = ['teleop', 'nav', 'auto']


def select(last, now, timeouts):
    """last: {source: (t_received, (lin, ang))}. Returns (source or None, (lin, ang))."""
    for src in PRIORITY:
        if src in last and now - last[src][0] <= timeouts[src]:
            return src, last[src][1]
    return None, (0.0, 0.0)


def front_distance(ranges, angle_min, inc, cone_deg, rmax=12.0, center_deg=0.0):
    """Smallest valid lidar range within +-cone_deg of the direction center_deg (0 = straight ahead, 180 = behind).
    rmax if nothing valid (or no data)."""
    best = rmax
    n = len(ranges)
    if n == 0 or inc == 0:
        return best
    half, c = math.radians(cone_deg), math.radians(center_deg)
    for i in range(n):
        a = angle_min + i * inc
        d = abs(math.atan2(math.sin(a - c), math.cos(a - c)))      # angular distance, handles the +-180 wrap
        if d <= half:
            r = ranges[i]
            if math.isfinite(r) and 0.01 < r < best:
                best = r
    return best


def guard(lin, ang, front, scan_ok, p, rear=12.0):
    """Obstacle guard. Returns (lin, ang, reason). Straight/arc motion towards an obstacle is limited, turning on the
    spot is always allowed. Reversing is checked against the rear distance (stop closer than rear_stop_dist)."""
    if lin == 0.0:
        return lin, ang, ''
    if not scan_ok and p['require_scan']:
        return 0.0, ang, 'no-scan'
    if lin < 0.0:
        if rear <= p.get('rear_stop_dist', 0.3):
            return 0.0, ang, 'obstacle-behind'
        return lin, ang, ''
    if front <= p['stop_dist']:
        return 0.0, ang, 'obstacle'
    if front < p['slow_dist']:
        k = (front - p['stop_dist']) / max(p['slow_dist'] - p['stop_dist'], 1e-6)
        return lin * k, ang, 'slowing'
    return lin, ang, ''


def ramp(current, target, max_step):
    """Move current towards target by at most max_step."""
    if target > current:
        return min(target, current + max_step)
    return max(target, current - max_step)


def limit_step(cur, tgt, dt, p, brake):
    """One control tick of speed limits + acceleration limits. brake=True -> stop quickly."""
    lin = max(-p['max_linear'], min(p['max_linear'], tgt[0]))
    ang = max(-p['max_angular'], min(p['max_angular'], tgt[1]))
    if brake:
        lin_step, ang_step = 10.0 * dt, 10.0 * dt            # braking: practically immediate
    else:
        lin_step, ang_step = p['max_accel'] * dt, p['max_ang_accel'] * dt
        # slowing down is always allowed quickly (never slower than accelerating)
        if abs(lin) < abs(cur[0]):
            lin_step = max(lin_step, 2.0 * p['max_accel'] * dt)
        if abs(ang) < abs(cur[1]):
            ang_step = max(ang_step, 2.0 * p['max_ang_accel'] * dt)
    return ramp(cur[0], lin, lin_step), ramp(cur[1], ang, ang_step)


def main():
    import rclpy
    from geometry_msgs.msg import Twist, TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String

    class Mux(Node):
        def __init__(self):
            super().__init__('cmd_vel_mux')
            d = self.declare_parameter
            d('output_topic', '/diff_cont/cmd_vel')
            d('stamped', True)
            d('teleop_timeout', 0.6)
            d('nav_timeout', 0.5)
            d('auto_timeout', 0.5)
            d('odom_in', '/diff_cont/odom')
            d('odom_out', '/odom')
            d('safety_enabled', True)
            d('safety_stop_distance', 0.38)   # m from the lidar (the nose is ~0.18 m in front of it)
            d('safety_slow_distance', 1.0)
            d('safety_cone_deg', 28.0)
            d('rear_stop_distance', 0.3)    # m from the lidar to an obstacle behind (the tail is ~0.2 m behind it)
            d('require_scan', True)
            d('max_linear', 0.6)
            d('max_angular', 2.0)
            d('max_accel', 1.2)
            d('max_ang_accel', 4.0)
            g = lambda k: self.get_parameter(k).value  # noqa: E731
            self.stamped = g('stamped')
            self.timeouts = {'teleop': g('teleop_timeout'), 'nav': g('nav_timeout'), 'auto': g('auto_timeout')}
            self.p = {'stop_dist': g('safety_stop_distance'), 'slow_dist': g('safety_slow_distance'),
                      'require_scan': g('require_scan'), 'max_linear': g('max_linear'), 'max_angular': g('max_angular'),
                      'max_accel': g('max_accel'), 'max_ang_accel': g('max_ang_accel'),
                      'rear_stop_dist': g('rear_stop_distance')}
            self.cone = g('safety_cone_deg')
            self.safety = g('safety_enabled')
            self.last = {}
            self.current = None
            self.cur = (0.0, 0.0)
            self.estop = False
            self.front = 12.0
            self.rear = 12.0
            self.scan_time = 0.0
            self.reason = ''
            self.last_reason = None
            out = g('output_topic')
            self.pub = self.create_publisher(TwistStamped if self.stamped else Twist, out, 10)
            for topic, src in [('/cmd_vel_teleop', 'teleop'), ('/cmd_vel', 'nav'), ('/cmd_vel_nav', 'nav'),
                               ('/cmd_vel_smoothed', 'nav'), ('/cmd_vel_auto', 'auto')]:
                self.create_subscription(Twist, topic, lambda m, s=src: self.on_cmd(s, m), 10)
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Bool, '/emergency_stop', self.on_estop, latched)
            self.odom_pub = self.create_publisher(Odometry, g('odom_out'), 10)
            self.create_subscription(Odometry, g('odom_in'), self.odom_pub.publish, 10)
            self.status_pub = self.create_publisher(String, '/mux_status', 10)
            self.create_timer(0.05, self.tick)
            self.create_timer(0.5, self.publish_status)
            self.get_logger().info(f'cmd_vel_mux: teleop > nav > auto -> {out} ({"TwistStamped" if self.stamped else "Twist"}), '
                                   f'safety {"ON" if self.safety else "OFF"} (stop {self.p["stop_dist"]} m, slow {self.p["slow_dist"]} m)')

        def now(self):
            return time.monotonic()

        def on_cmd(self, src, msg):
            self.last[src] = (self.now(), (msg.linear.x, msg.angular.z))

        def on_scan(self, msg):
            r = list(msg.ranges)
            rmax = msg.range_max or 12.0
            self.front = front_distance(r, msg.angle_min, msg.angle_increment, self.cone, rmax)
            self.rear = front_distance(r, msg.angle_min, msg.angle_increment, self.cone, rmax, center_deg=180.0)
            self.scan_time = self.now()

        def on_estop(self, msg):
            if msg.data != self.estop:
                self.estop = msg.data
                self.get_logger().warn('EMERGENCY STOP ' + ('ENGAGED: robot stopped until /emergency_stop is set to false' if self.estop else 'released'))

        def tick(self):
            now = self.now()
            src, (lin, ang) = select(self.last, now, self.timeouts)
            if src != self.current:
                self.current = src
                self.get_logger().info(f'driving source: {src or "none (stopped)"}')
            reason = ''
            brake = False
            if self.estop:
                lin, ang, reason, brake = 0.0, 0.0, 'e-stop', True
            elif self.safety:
                lin2, ang2, reason = guard(lin, ang, self.front, now - self.scan_time < 1.0, self.p, self.rear)
                brake = reason in ('obstacle', 'obstacle-behind', 'no-scan')
                lin, ang = lin2, ang2
            self.reason = reason
            if reason != self.last_reason:
                self.last_reason = reason
                if reason in ('obstacle', 'obstacle-behind', 'no-scan'):
                    self.get_logger().info(f'safety: forward motion blocked ({reason}; nearest in front {self.front:.2f} m)')
            self.cur = limit_step(self.cur, (lin, ang), 0.05, self.p, brake)
            if self.stamped:
                m = TwistStamped()             # stamp 0 = "now" for the controller, so no clock mismatch
                m.header.frame_id = 'base_link'
                m.twist.linear.x, m.twist.angular.z = float(self.cur[0]), float(self.cur[1])
            else:
                m = Twist()
                m.linear.x, m.angular.z = float(self.cur[0]), float(self.cur[1])
            self.pub.publish(m)

        def publish_status(self):
            s = String()
            s.data = f'{"ESTOP" if self.estop else (self.current or "idle")} {self.reason}'.strip()
            self.status_pub.publish(s)

    rclpy.init()
    node = Mux()
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
