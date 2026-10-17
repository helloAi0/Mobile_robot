#!/usr/bin/env python3
"""Makes the simulated lidar behave like the real one: /scan_raw (Gazebo) -> /scan.

   * masks the sectors that the robot's own rear pillars occupy (what a laser_filters angular filter does on hardware)
   * random dropouts (a real beam sometimes returns nothing: dark, shiny or oblique surfaces)
   * slow range-bias drift (a random walk, clipped), on top of the Gaussian noise Gazebo already adds

   ros2 run mobile_robot_gazebo scan_degrader.py --ros-args -p dropout_prob:=0.02 -p bias_walk:=0.0005
The processing is in plain functions (no ROS needed), so it is unit-tested."""
import math
import random


def mask_sectors(ranges, angle_min, inc, sectors, fill=math.inf):
    """sectors = [(centre_deg, half_width_deg), ...] in the scan frame. Returns a new list."""
    out = list(ranges)
    for centre, half in sectors:
        c = math.radians(centre)
        h = math.radians(half)
        for i in range(len(out)):
            a = angle_min + i * inc
            d = math.atan2(math.sin(a - c), math.cos(a - c))
            if abs(d) <= h:
                out[i] = fill
    return out


def apply_dropouts(ranges, prob, rng, fill=math.inf):
    if prob <= 0:
        return list(ranges)
    return [fill if (math.isfinite(r) and rng.random() < prob) else r for r in ranges]


def step_bias(bias, walk, dt, limit, rng):
    """One step of a clipped random walk (std = walk * sqrt(dt))."""
    b = bias + rng.gauss(0.0, walk * math.sqrt(max(dt, 0.0)))
    return max(-limit, min(limit, b))


def add_bias(ranges, bias, rmin, rmax):
    return [min(rmax, max(rmin, r + bias)) if (math.isfinite(r) and rmin <= r <= rmax) else r for r in ranges]


def degrade(ranges, angle_min, inc, rmin, rmax, bias, p, rng):
    r = mask_sectors(ranges, angle_min, inc, p["sectors"])
    r = add_bias(r, bias, rmin, rmax)
    return apply_dropouts(r, p["dropout_prob"], rng)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    class Degrader(Node):
        def __init__(self):
            super().__init__("scan_degrader")
            self.p = {
                "dropout_prob": self.declare_parameter("dropout_prob", 0.01).value,
                "bias_walk": self.declare_parameter("bias_walk", 0.0005).value,      # m / sqrt(s)
                "bias_limit": self.declare_parameter("bias_limit", 0.03).value,      # m
                "sectors": [(137.0, 17.0), (-137.0, 17.0)],
            }
            if self.declare_parameter("mask_pillars", True).value is False:
                self.p["sectors"] = []
            self.rng = random.Random(self.declare_parameter("seed", 0).value or None)
            self.bias, self.last = 0.0, None
            self.pub = self.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
            self.create_subscription(LaserScan, "scan_raw", self.on_scan, qos_profile_sensor_data)

        def on_scan(self, m):
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            dt = 0.0 if self.last is None else max(0.0, t - self.last)
            self.last = t
            self.bias = step_bias(self.bias, self.p["bias_walk"], dt, self.p["bias_limit"], self.rng)
            m.ranges = [float(x) for x in degrade(m.ranges, m.angle_min, m.angle_increment,
                                                  m.range_min, m.range_max, self.bias, self.p, self.rng)]
            self.pub.publish(m)

    rclpy.init()
    n = Degrader()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    try:
        n.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
