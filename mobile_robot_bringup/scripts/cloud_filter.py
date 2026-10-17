#!/usr/bin/env python3
"""Remove the floor (and far points) from the camera point cloud so RViz only shows the OBJECTS.

/camera/points (raw, ~300 000 points, mostly floor)  ->  /camera/points_filtered (only things standing on the floor)

The camera sits 'cam_height' m above the floor, pitched down by 'cam_pitch' rad, and the cloud is in the
camera_link frame (x forward, y left, z up). Every point is rotated into a level frame; points whose height
above the floor is below 'floor_margin' are dropped.
"""
import math

import numpy as np


def filter_cloud(raw, point_step, offsets, cam_height=0.231, cam_pitch=0.18, floor_margin=0.04,
                 max_range=7.0, min_height_keep=None):
    """raw: uint8 array (n_points, point_step). offsets: byte offsets of x, y, z (float32).
    Returns a boolean mask of the points to keep."""
    ox, oy, oz = offsets
    x = np.ascontiguousarray(raw[:, ox:ox + 4]).view('<f4').ravel()
    y = np.ascontiguousarray(raw[:, oy:oy + 4]).view('<f4').ravel()
    z = np.ascontiguousarray(raw[:, oz:oz + 4]).view('<f4').ravel()
    c, s = math.cos(cam_pitch), math.sin(cam_pitch)
    height = -x * s + z * c + cam_height          # height above the floor
    forward = x * c + z * s
    ok = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    ok &= height > floor_margin
    ok &= forward < max_range
    return ok


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2

    class CloudFilter(Node):
        def __init__(self):
            super().__init__('cloud_filter')
            self.declare_parameter('cam_height', 0.231)
            self.declare_parameter('cam_pitch', 0.18)
            self.declare_parameter('floor_margin', 0.04)
            self.declare_parameter('max_range', 7.0)
            self.pub = self.create_publisher(PointCloud2, '/camera/points_filtered', qos_profile_sensor_data)
            self.create_subscription(PointCloud2, '/camera/points', self.on_cloud, qos_profile_sensor_data)
            self.get_logger().info('cloud_filter: /camera/points -> /camera/points_filtered (floor removed)')

        def on_cloud(self, msg):
            offs = {f.name: f.offset for f in msg.fields}
            if not all(k in offs for k in ('x', 'y', 'z')):
                return
            n = msg.width * msg.height
            raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)[:n * msg.point_step].reshape(n, msg.point_step)
            keep = filter_cloud(raw, msg.point_step, (offs['x'], offs['y'], offs['z']),
                                self.get_parameter('cam_height').value, self.get_parameter('cam_pitch').value,
                                self.get_parameter('floor_margin').value, self.get_parameter('max_range').value)
            out = PointCloud2()
            out.header = msg.header
            out.fields = msg.fields
            out.is_bigendian = msg.is_bigendian
            out.point_step = msg.point_step
            out.height = 1
            out.width = int(keep.sum())
            out.row_step = out.point_step * out.width
            out.is_dense = True
            out.data = raw[keep].tobytes()
            self.pub.publish(out)

    rclpy.init()
    node = CloudFilter()
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
