#!/usr/bin/env python3
"""Keyboard driver with live feedback.   ros2 run mobile_robot teleop_keys.py

  i forward    , backward    j turn left    l turn right    u / o forward + turn    k or SPACE stop
  q / z  faster / slower        a  switch AUTONOMOUS mode on/off        Ctrl-C quit

The robot keeps going until you press k. While you drive, the keyboard has priority over autonomous mode and
Nav2; a second after you press k they take over again (if they are running).
Sends /cmd_vel_teleop to cmd_vel_mux.py, which feeds the drive controller.
"""
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

MOVES = {'i': (1, 0), ',': (-1, 0), 'j': (0, 1), 'l': (0, -1), 'u': (1, 1), 'o': (1, -1), 'm': (-1, 1), '.': (-1, -1)}


class TeleopKeys(Node):
    def __init__(self):
        super().__init__('teleop_keys')
        self.pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.mode_pub = self.create_publisher(Bool, '/auto_mode', 10)
        self.lin = self.ang = 0.0
        self.speed, self.turn = 0.4, 1.0
        self.send_until = 0.0         # keep sending (also zeros) until this time, then stay silent
        self.auto = False
        self.create_timer(0.05, self.send)

    def send(self):
        if time.monotonic() > self.send_until:
            return                    # silent: lets autonomous mode / Nav2 drive
        m = Twist()
        m.linear.x, m.angular.z = self.lin, self.ang
        self.pub.publish(m)

    def command(self, lin, ang):
        self.lin, self.ang = lin, ang
        self.send_until = time.monotonic() + (1.0 if lin == 0.0 and ang == 0.0 else 1e9)

    def toggle_auto(self):
        self.auto = not self.auto
        self.mode_pub.publish(Bool(data=self.auto))
        self.command(0.0, 0.0)

    def status(self):
        n = self.pub.get_subscription_count()
        link = 'mux listening' if n else 'NO cmd_vel_mux (is the simulation running?)'
        return (f'\r\x1b[K[{link}] [auto: {"ON" if self.auto else "off"}]  v={self.lin:+.2f} m/s  '
                f'w={self.ang:+.2f} rad/s  speed {self.speed:.2f}/{self.turn:.2f}')


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
                    node.command(MOVES[k][0] * node.speed, MOVES[k][1] * node.turn)
                elif k in ('k', ' '):
                    node.command(0.0, 0.0)
                elif k == 'a':
                    node.toggle_auto()
                elif k == 'q':
                    node.speed *= 1.2
                    node.turn *= 1.2
                elif k == 'z':
                    node.speed /= 1.2
                    node.turn /= 1.2
                elif k == '\x03':
                    break
            sys.stdout.write(node.status())
            sys.stdout.flush()
    finally:
        node.command(0.0, 0.0)
        node.send()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print()


if __name__ == '__main__':
    main()
