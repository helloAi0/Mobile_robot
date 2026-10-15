#!/usr/bin/env python3
"""Keyboard driver for the simulated robot, with live feedback. Run:   ros2 run mobile_robot teleop_keys.py

Unlike teleop_twist_keyboard this node KEEPS SENDING the current command 20x per second, so the robot
does not stop after half a second and no time-stamp / sim-clock problems can make the controller ignore it.

  i forward    , backward    j turn left    l turn right    u / o forward + turn    k or SPACE stop
  q / z : speed up / down      Ctrl-C quit

The status line shows whether the controller is really listening on /diff_cont/cmd_vel.
If it says 'NO CONTROLLER', the robot cannot move: look at the launch terminal for errors.
"""
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

MOVES = {  # key: (linear sign, angular sign)
    'i': (1, 0), ',': (-1, 0), 'j': (0, 1), 'l': (0, -1),
    'u': (1, 1), 'o': (1, -1), 'm': (-1, 1), '.': (-1, -1),
}


class TeleopKeys(Node):
    def __init__(self):
        super().__init__('teleop_keys')
        self.declare_parameter('topic', '/diff_cont/cmd_vel')
        topic = self.get_parameter('topic').value
        self.pub = self.create_publisher(TwistStamped, topic, 10)
        self.topic = topic
        self.lin, self.ang = 0.0, 0.0     # current command
        self.speed, self.turn = 0.4, 1.0  # m/s, rad/s
        self.create_timer(0.05, self.send)

    def send(self):
        m = TwistStamped()          # stamp left at 0: the controller then uses "now" (no clock mismatch)
        m.header.frame_id = 'base_link'
        m.twist.linear.x = self.lin
        m.twist.angular.z = self.ang
        self.pub.publish(m)

    def status(self):
        n = self.pub.get_subscription_count()
        ctl = f'controller listening ({n})' if n else 'NO CONTROLLER on ' + self.topic
        return f'\r\x1b[K[{ctl}]  v={self.lin:+.2f} m/s  w={self.ang:+.2f} rad/s  (speed {self.speed:.2f}/{self.turn:.2f})'


def main():
    if not sys.stdin.isatty():
        print('teleop_keys needs a real terminal (run it directly, not through a pipe).')
        return
    rclpy.init()
    node = TeleopKeys()
    old = termios.tcgetattr(sys.stdin)
    print(__doc__)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if select.select([sys.stdin], [], [], 0)[0]:
                k = sys.stdin.read(1)
                if k in MOVES:
                    ls, as_ = MOVES[k]
                    node.lin, node.ang = ls * node.speed, as_ * node.turn
                elif k in ('k', ' '):
                    node.lin, node.ang = 0.0, 0.0
                elif k == 'q':
                    node.speed *= 1.2; node.turn *= 1.2
                elif k == 'z':
                    node.speed /= 1.2; node.turn /= 1.2
                elif k == '\x03':
                    break
            sys.stdout.write(node.status())
            sys.stdout.flush()
    finally:
        node.lin = node.ang = 0.0
        node.send()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print()


if __name__ == '__main__':
    main()
