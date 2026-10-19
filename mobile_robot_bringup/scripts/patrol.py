#!/usr/bin/env python3
"""Patrol mission: drive Nav2 goals through the rooms (config/patrol_house.yaml), forever or for N laps.

   ros2 launch mobile_robot_bringup launch_sim.launch.py nav2:=true patrol:=true
   ros2 run mobile_robot_bringup patrol.py --ros-args -p loops:=2          # on its own, Nav2 already running

A goal that fails is retried, then skipped, and the mission carries on: one blocked door does not end the patrol.
The route logic (loading, ordering, retry bookkeeping) is plain Python and unit-tested without ROS.
"""
import math
import os

import yaml


def load_route(path):
    cfg = yaml.safe_load(open(path))['patrol']
    wps = []
    for w in cfg['waypoints']:
        wps.append({'name': w['name'], 'x': float(w['x']), 'y': float(w['y']), 'yaw': math.radians(float(w.get('yaw_deg', 0.0)))})
    names = [w['name'] for w in wps]
    if len(set(names)) != len(names):
        raise ValueError('duplicate waypoint names')
    if not wps:
        raise ValueError('patrol has no waypoints')
    return {'waypoints': wps, 'loops': int(cfg.get('loops', 0)), 'retries': int(cfg.get('retries', 2)),
            'goal_timeout_s': float(cfg.get('goal_timeout_s', 90.0)), 'pause_s': float(cfg.get('pause_s', 1.0))}


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class Mission:
    """Which waypoint is next, how often it was tried, how many laps are done. No ROS in here."""

    def __init__(self, route):
        self.r = route
        self.i = 0
        self.tries = 0
        self.laps = 0
        self.reached, self.skipped = [], []

    @property
    def finished(self):
        return self.r['loops'] > 0 and self.laps >= self.r['loops']

    @property
    def current(self):
        return None if self.finished else self.r['waypoints'][self.i]

    def _advance(self):
        self.tries = 0
        self.i += 1
        if self.i >= len(self.r['waypoints']):
            self.i = 0
            self.laps += 1

    def report(self, success):
        """Call after each attempt. Returns 'next' | 'retry' | 'skip'."""
        wp = self.current
        if success:
            self.reached.append(wp['name'])
            self._advance()
            return 'next'
        self.tries += 1
        if self.tries > self.r['retries']:
            self.skipped.append(wp['name'])
            self._advance()
            return 'skip'
        return 'retry'


def main():
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node

    class Patrol(Node):
        def __init__(self):
            super().__init__('patrol')
            default = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config', 'patrol_house.yaml')
            try:
                from ament_index_python.packages import get_package_share_directory
                default = os.path.join(get_package_share_directory('mobile_robot_bringup'), 'config', 'patrol_house.yaml')
            except Exception:
                pass
            route = load_route(self.declare_parameter('route', default).value)
            loops = self.declare_parameter('loops', -1).value
            if loops >= 0:
                route['loops'] = loops
            self.m = Mission(route)
            self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.timer = self.create_timer(1.0, self.start)
            self.busy = False

        def start(self):
            if self.busy:
                return
            if not self.client.wait_for_server(timeout_sec=0.1):
                self.get_logger().info('waiting for Nav2 (navigate_to_pose)...', throttle_duration_sec=10.0)
                return
            if self.m.finished:
                self.get_logger().info(f'patrol finished: {len(self.m.reached)} waypoints reached, skipped {self.m.skipped}')
                self.timer.cancel()
                return
            wp = self.m.current
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = 'map'
            goal.pose.pose.position.x, goal.pose.pose.position.y = wp['x'], wp['y']
            (goal.pose.pose.orientation.x, goal.pose.pose.orientation.y,
             goal.pose.pose.orientation.z, goal.pose.pose.orientation.w) = yaw_to_quaternion(wp['yaw'])
            self.busy = True
            self.get_logger().info(f'-> {wp["name"]} ({wp["x"]:.1f}, {wp["y"]:.1f}), lap {self.m.laps + 1}, try {self.m.tries + 1}')
            fut = self.client.send_goal_async(goal)
            fut.add_done_callback(self.accepted)
            self.deadline = self.get_clock().now().nanoseconds * 1e-9 + self.m.r['goal_timeout_s']
            self.watchdog = self.create_timer(1.0, self.check_timeout)

        def accepted(self, fut):
            self.handle = fut.result()
            if not self.handle.accepted:
                self.done(False)
                return
            self.handle.get_result_async().add_done_callback(lambda f: self.done(f.result().status == 4))   # 4 = SUCCEEDED

        def check_timeout(self):
            if self.busy and self.get_clock().now().nanoseconds * 1e-9 > self.deadline:
                self.get_logger().warn('goal timed out: cancelling')
                if getattr(self, 'handle', None) is not None:
                    self.handle.cancel_goal_async()
                self.done(False)

        def done(self, ok):
            if not self.busy:
                return
            self.busy = False
            if hasattr(self, 'watchdog'):
                self.watchdog.cancel()
            action = self.m.report(ok)
            self.get_logger().info(('reached' if ok else 'failed') + f' -> {action}')

    rclpy.init()
    n = Patrol()
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
