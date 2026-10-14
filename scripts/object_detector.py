#!/usr/bin/env python3
"""Very simple lidar "something is in front of me" detector.

Subscribes to /scan (sensor_msgs/LaserScan), groups neighbouring beams that belong to the same
object, and publishes a red sphere + a text label for every object closer than `detect_range`
metres inside a front sector of +/- `fov_deg` degrees (topic /detections, RViz MarkerArray).

This is NOT person recognition: it only knows "an object is here, this far away". In the street
world the only thing that moves is the pedestrian, so you will see the marker follow her.
"""
import math

MIN_BEAMS = 2          # ignore clusters with fewer beams (noise)
GAP = 0.35             # metres: a range jump bigger than this starts a new object


def find_objects(ranges, angle_min, angle_increment, range_min, range_max,
                 detect_range=4.0, fov_deg=70.0):
    """Return a list of (x, y, distance) in the lidar frame, one per detected object."""
    fov = math.radians(fov_deg)
    clusters, current, prev_r = [], [], None
    for i, r in enumerate(ranges):
        ang = angle_min + i * angle_increment
        ang = math.atan2(math.sin(ang), math.cos(ang))          # wrap to [-pi, pi]
        ok = (math.isfinite(r) and range_min <= r <= min(range_max, detect_range)
              and abs(ang) <= fov)
        if ok and (prev_r is None or abs(r - prev_r) <= GAP):
            current.append((r, ang))
            prev_r = r
            continue
        if current:
            clusters.append(current)
            current, prev_r = [], None
        if ok:                                                    # start a new cluster
            current, prev_r = [(r, ang)], r
    if current:
        clusters.append(current)

    objects = []
    for c in clusters:
        if len(c) < MIN_BEAMS:
            continue
        xs = [r * math.cos(a) for r, a in c]
        ys = [r * math.sin(a) for r, a in c]
        x, y = sum(xs) / len(xs), sum(ys) / len(ys)
        objects.append((x, y, math.hypot(x, y)))
    return objects


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from visualization_msgs.msg import Marker, MarkerArray

    class ObjectDetector(Node):
        def __init__(self):
            super().__init__('object_detector')
            self.declare_parameter('detect_range', 4.0)
            self.declare_parameter('fov_deg', 70.0)
            self.pub = self.create_publisher(MarkerArray, '/detections', 10)
            self.sub = self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
            self.was_detecting = False
            self.get_logger().info('object_detector ready: listening on /scan, publishing /detections')

        def on_scan(self, msg):
            rng = self.get_parameter('detect_range').value
            fov = self.get_parameter('fov_deg').value
            objs = find_objects(msg.ranges, msg.angle_min, msg.angle_increment,
                                msg.range_min, msg.range_max, rng, fov)
            out = MarkerArray()
            wipe = Marker()
            wipe.action = Marker.DELETEALL
            out.markers.append(wipe)
            for i, (x, y, d) in enumerate(objs):
                sphere = Marker()
                sphere.header = msg.header
                sphere.ns, sphere.id = 'object', i
                sphere.type, sphere.action = Marker.SPHERE, Marker.ADD
                sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = x, y, 0.0
                sphere.pose.orientation.w = 1.0
                sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.4
                sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.1, 0.1, 0.8
                sphere.lifetime.nanosec = 500_000_000
                label = Marker()
                label.header = msg.header
                label.ns, label.id = 'label', i
                label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
                label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, 0.5
                label.pose.orientation.w = 1.0
                label.scale.z = 0.25
                label.color.r = label.color.g = label.color.b = label.color.a = 1.0
                label.text = f'object {d:.1f} m'
                label.lifetime.nanosec = 500_000_000
                out.markers += [sphere, label]
            self.pub.publish(out)
            if objs and not self.was_detecting:
                nearest = min(objs, key=lambda o: o[2])
                self.get_logger().info(f'Object detected {nearest[2]:.1f} m ahead')
            elif not objs and self.was_detecting:
                self.get_logger().info('Object lost')
            self.was_detecting = bool(objs)

    rclpy.init()
    node = ObjectDetector()
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
