#!/usr/bin/env python3
"""Run metrics: how well does the robot explore and stay safe? Writes a CSV row set and prints a summary.

   ros2 run mobile_robot metrics_logger.py                 # Ctrl-C prints the report
   ros2 run mobile_robot metrics_logger.py --ros-args -p csv:=/tmp/run.csv

Metrics: distance driven, area explored (/map known cells), min obstacle clearance (/scan), number of stops
(mux status contains STOP), speed statistics. Pure helpers are unit-tested without ROS."""
import csv
import math
import time


def path_length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def known_area(data, resolution):
    """Occupancy-grid data (-1 unknown) -> explored area in m^2."""
    return sum(1 for v in data if v >= 0) * resolution * resolution


def min_clearance(ranges, rmin=0.05):
    good = [r for r in ranges if math.isfinite(r) and r > rmin]
    return min(good) if good else float("inf")


def count_stops(statuses):
    n, prev = 0, False
    for s in statuses:
        cur = "STOP" in s.upper()
        n += cur and not prev
        prev = cur
    return n


def summary(dist, area, clearance, stops, seconds):
    return {"distance_m": round(dist, 2), "explored_m2": round(area, 1), "min_clearance_m": round(clearance, 2),
            "stops": stops, "duration_s": round(seconds, 1), "avg_speed_mps": round(dist / seconds, 3) if seconds > 0 else 0.0}


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    class Logger(Node):
        def __init__(self):
            super().__init__("metrics_logger")
            self.csv_path = self.declare_parameter("csv", "metrics.csv").value
            self.t0, self.pts, self.area, self.clear, self.stat = time.time(), [], 0.0, float("inf"), []
            self.create_subscription(Odometry, "/odom", self.on_odom, 10)
            self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
            self.create_subscription(OccupancyGrid, "/map", self.on_map,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(String, "/mux_status", lambda m: self.stat.append(m.data), 10)
            self.create_timer(5.0, self.tick)
            self.rows = []

        def on_odom(self, m):
            p = (m.pose.pose.position.x, m.pose.pose.position.y)
            if not self.pts or math.dist(p, self.pts[-1]) > 0.02:
                self.pts.append(p)

        def on_scan(self, m):
            self.clear = min(self.clear, min_clearance(m.ranges))

        def on_map(self, m):
            self.area = known_area(m.data, m.info.resolution)

        def report(self):
            return summary(path_length(self.pts), self.area, self.clear, count_stops(self.stat), time.time() - self.t0)

        def tick(self):
            r = self.report()
            self.rows.append(r)
            self.get_logger().info(str(r))

        def save(self):
            r = self.report()
            self.rows.append(r)
            with open(self.csv_path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(r))
                w.writeheader()
                w.writerows(self.rows)
            print("\n=== run summary ===")
            for k, v in r.items():
                print(f"{k:>16}: {v}")
            print("saved", self.csv_path)

    rclpy.init()
    n = Logger()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.save()
    try:
        n.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
