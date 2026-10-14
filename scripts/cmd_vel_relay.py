#!/usr/bin/env python3
"""Relay a normal /cmd_vel (geometry_msgs/Twist) to the diff-drive controller (TwistStamped).

Why: in ROS 2 Jazzy the controller only listens on /diff_cont/cmd_vel as TwistStamped, but almost every
teleop tool (teleop_twist_keyboard, joystick, Nav2, ...) publishes a plain Twist on /cmd_vel. This node
bridges the two and stamps every message with ITS OWN clock, which runs on simulation time (like the
controller), so the controller never throws the command away as "too old".

Then you can simply run:   ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/diff_cont/cmd_vel')
        self.declare_parameter('frame_id', 'base_link')
        self.frame_id = self.get_parameter('frame_id').value
        out = self.get_parameter('output_topic').value
        inp = self.get_parameter('input_topic').value
        self.pub = self.create_publisher(TwistStamped, out, 10)
        self.sub = self.create_subscription(Twist, inp, self.on_twist, 10)
        self.count = 0
        self.get_logger().info(f'relaying {inp} (Twist) -> {out} (TwistStamped)')

    def on_twist(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist = msg
        self.pub.publish(out)
        self.count += 1
        if self.count == 1:
            self.get_logger().info('first command received and forwarded')


def main():
    rclpy.init()
    node = CmdVelRelay()
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
