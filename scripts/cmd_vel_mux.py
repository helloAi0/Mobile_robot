#!/usr/bin/env python3
"""Decides WHO drives the robot and forwards it to the drive controller. Same node for simulation and hardware.

  priority 1  /cmd_vel_teleop   keyboard (ros2 run mobile_robot teleop_keys.py)         Twist
  priority 2  /cmd_vel, /cmd_vel_nav, /cmd_vel_smoothed   Nav2 (click "2D Goal Pose" in RViz) or any other teleop   Twist
  priority 3  /cmd_vel_auto     autonomous wandering (auto_explore.py)                  Twist
  nothing fresh -> zero velocity (the robot stops by itself)

Output: /diff_cont/cmd_vel (TwistStamped, ros2_control diff_drive_controller on ROS 2 Jazzy), 20 times per second.
Parameter 'stamped:=false' + 'output_topic' publish a plain Twist instead, for drivers that want that.
Also copies /diff_cont/odom to /odom, because Nav2 expects /odom.
"""
import time

PRIORITY = ['teleop', 'nav', 'auto']


def select(last, now, timeouts):
    """last: {source: (t_received, (lin, ang))}. Returns (source or None, (lin, ang))."""
    for src in PRIORITY:
        if src in last and now - last[src][0] <= timeouts[src]:
            return src, last[src][1]
    return None, (0.0, 0.0)


def main():
    import rclpy
    from geometry_msgs.msg import Twist, TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node

    class Mux(Node):
        def __init__(self):
            super().__init__('cmd_vel_mux')
            self.declare_parameter('output_topic', '/diff_cont/cmd_vel')
            self.declare_parameter('stamped', True)
            self.declare_parameter('teleop_timeout', 0.6)
            self.declare_parameter('nav_timeout', 0.5)
            self.declare_parameter('auto_timeout', 0.5)
            self.declare_parameter('odom_in', '/diff_cont/odom')
            self.declare_parameter('odom_out', '/odom')
            self.stamped = self.get_parameter('stamped').value
            self.timeouts = {'teleop': self.get_parameter('teleop_timeout').value,
                             'nav': self.get_parameter('nav_timeout').value,
                             'auto': self.get_parameter('auto_timeout').value}
            self.last = {}
            self.current = None
            out = self.get_parameter('output_topic').value
            self.pub = self.create_publisher(TwistStamped if self.stamped else Twist, out, 10)
            for topic, src in [('/cmd_vel_teleop', 'teleop'), ('/cmd_vel', 'nav'), ('/cmd_vel_nav', 'nav'),
                               ('/cmd_vel_smoothed', 'nav'), ('/cmd_vel_auto', 'auto')]:
                self.create_subscription(Twist, topic, lambda m, s=src: self.on_cmd(s, m), 10)
            self.odom_pub = self.create_publisher(Odometry, self.get_parameter('odom_out').value, 10)
            self.create_subscription(Odometry, self.get_parameter('odom_in').value, self.odom_pub.publish, 10)
            self.create_timer(0.05, self.tick)
            self.get_logger().info(f'cmd_vel_mux: teleop > nav > auto  ->  {out} '
                                   f'({"TwistStamped" if self.stamped else "Twist"})')

        def on_cmd(self, src, msg):
            self.last[src] = (time.monotonic(), (msg.linear.x, msg.angular.z))

        def tick(self):
            src, (lin, ang) = select(self.last, time.monotonic(), self.timeouts)
            if src != self.current:
                self.current = src
                self.get_logger().info(f'driving source: {src or "none (stopped)"}')
            if self.stamped:
                m = TwistStamped()             # stamp 0 = "now" for the controller, so no clock mismatch
                m.header.frame_id = 'base_link'
                m.twist.linear.x, m.twist.angular.z = float(lin), float(ang)
            else:
                m = Twist()
                m.linear.x, m.angular.z = float(lin), float(ang)
            self.pub.publish(m)

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
