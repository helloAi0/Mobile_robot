#!/usr/bin/env python3
"""Keyboard driver with live feedback.   ros2 run mobile_robot_bringup teleop_keys.py

   FORWARD      i   or   w   or   UP arrow        (u / o = forward + turn left / right)
   BACKWARD     m   or   ,   or   s   or   DOWN arrow     (always straight back)
   TURN LEFT    j   or   a   or   LEFT arrow            (turns on the spot)
   TURN RIGHT   l   or   d   or   RIGHT arrow
   STOP         k   or   SPACE   or   x

   e / q : slower / faster        p : autopilot on/off (wander + avoid)        Ctrl-C : quit

The robot keeps going until you press
k, SPACE or x. While you drive, the keyboard has priority over autopilot and Nav2; one second after you stop
they take over again (if they are running). The safety layer in cmd_vel_mux.py slows the robot down in front of
obstacles; backing away is always possible.
"""
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

# key -> (linear sign, angular sign)
MOVES = {
    'i': (1, 0), 'u': (1, 1), 'o': (1, -1), 'j': (0, 1), 'l': (0, -1),
    'm': (-1, 0), ',': (-1, 0), '.': (-1, 0),
    'w': (1, 0), 'a': (0, 1), 'd': (0, -1), 's': (-1, 0),
    '\x1b[A': (1, 0), '\x1b[B': (-1, 0), '\x1b[C': (0, -1), '\x1b[D': (0, 1),
}
STOP_KEYS = ('k', ' ', 'x')


def describe(lin, ang):
    """Plain-words description of what the robot is being told to do."""
    if lin == 0 and ang == 0:
        return 'STOPPED'
    if lin == 0:
        return 'TURNING LEFT (on the spot)' if ang > 0 else 'TURNING RIGHT (on the spot)'
    way = 'FORWARD' if lin > 0 else 'BACKWARD'
    if ang == 0:
        return way
    return f'{way} + {"left" if ang > 0 else "right"} arc'


class TeleopKeys(Node):
    def __init__(self):
        super().__init__('teleop_keys')
        self.pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.mode_pub = self.create_publisher(Bool, '/auto_mode', 10)
        self.create_subscription(String, '/mux_status', self.on_status, 10)
        self.mux_status = ''
        self.lin = self.ang = 0.0
        self.speed, self.turn = 0.35, 1.0
        self.send_until = 0.0         # keep sending (also zeros) until this time, then stay silent
        self.auto = False
        self.create_timer(0.05, self.send)

    def on_status(self, msg):
        self.mux_status = msg.data

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
        link = 'mux OK' if self.pub.get_subscription_count() else 'NO cmd_vel_mux: is the simulation running?'
        extra = f' | {self.mux_status}' if self.mux_status else ''
        return (f'\r\x1b[K[{link}{extra}] [autopilot {"ON" if self.auto else "off"}] '
                f'{describe(self.lin, self.ang)}  v={self.lin:+.2f} m/s w={self.ang:+.2f} rad/s  (speed {self.speed:.2f})')


def _byte():
    return os.read(sys.stdin.fileno(), 1).decode(errors='ignore')    # unbuffered: select() on the fd stays correct


def read_key():
    k = _byte()
    if k == '\x1b':                                   # arrow keys arrive as ESC [ A..D
        if select.select([sys.stdin], [], [], 0.02)[0]:
            k += _byte()
            if select.select([sys.stdin], [], [], 0.02)[0]:
                k += _byte()
    return k


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
                k = read_key()
                if k in MOVES:
                    node.command(MOVES[k][0] * node.speed, MOVES[k][1] * node.turn)
                elif k in STOP_KEYS:
                    node.command(0.0, 0.0)
                elif k == 'p':
                    node.toggle_auto()
                elif k == 'q':
                    node.speed = min(node.speed * 1.2, 0.6)
                    node.turn = min(node.turn * 1.2, 2.0)
                elif k == 'e':
                    node.speed /= 1.2
                    node.turn /= 1.2
                elif k == '\x03':
                    break
            sys.stdout.write(node.status())
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.command(0.0, 0.0)
            node.send()                                # last word: stop
        except Exception:                              # context may already be shut down by Ctrl-C
            pass
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        print('\nstopped.')


if __name__ == '__main__':
    main()
