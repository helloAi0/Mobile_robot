#!/usr/bin/env python3
"""Score a recorded run offline: no simulator, no ROS graph, just the bag.

   ros2 run mobile_robot_bringup bag_report.py bags/run_20261019_101500            # prints a table
   ros2 run mobile_robot_bringup bag_report.py <bag> --json report.json

Reports the same quantities as the benchmark (scripts/benchmark_metrics.py) plus estimator drift:
  duration, distance driven, minimum clearance and contacts (from /scan and the robot outline), stops and why
  (from /mux_status), and how far the raw wheel odometry ended up from the fused odometry (/odometry/filtered):
  that gap is the error that wheel slip adds, which the EKF removes from the heading.
The analysis (`analyse`) takes plain tuples, so it is tested without ROS; `read_bag` is the only part that needs rosbag2_py.
"""
import argparse
import json
import math
import sys

try:
    import benchmark_metrics as bm
except ImportError:      # run from a source checkout
    import os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import benchmark_metrics as bm


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def analyse(messages):
    """messages: iterable of (topic, t_seconds, payload).
       /scan -> (ranges, angle_min, angle_increment, range_min)
       /odom, /diff_cont/odom, /odometry/filtered -> (x, y, yaw)
       /mux_status -> str"""
    acc = bm.RunAccumulator()
    t0 = t_end = None
    pose = {'/odom': None, '/diff_cont/odom': None, '/odometry/filtered': None}
    first_pose, dist = {}, {k: 0.0 for k in pose}
    reasons = {}
    last_status = None
    for topic, t, p in messages:
        if t0 is None:
            t0 = t
        t_end = t
        if topic in pose:
            prev = pose[topic]
            if prev is not None:
                dist[topic] += math.hypot(p[0] - prev[0], p[1] - prev[1])
            first_pose.setdefault(topic, p)
            pose[topic] = p
        elif topic == '/scan':
            ranges, amin, inc, rmin = p
            gap = bm.scan_gap(ranges, amin, inc, rmin or 0.15)
            ref = pose['/odometry/filtered'] or pose['/odom'] or pose['/diff_cont/odom'] or (0.0, 0.0, 0.0)
            acc.update(t - t0, ref[0], ref[1], ref[2], gap)
        elif topic == '/mux_status':
            parts = p.split()
            if p != last_status and len(parts) > 1:
                reasons[parts[1]] = reasons.get(parts[1], 0) + 1
            last_status = p
    src = next((k for k in ('/odometry/filtered', '/odom', '/diff_cont/odom') if pose[k] is not None), None)
    out = {'duration_s': round((t_end - t0) if t0 is not None else 0.0, 1),
           'distance_m': round(dist[src], 2) if src else None,
           'distance_source': src,
           'scans': len(acc.samples),
           'min_clearance_m': round(acc.min_gap, 3) if acc.samples and math.isfinite(acc.min_gap) else None,
           'contacts': acc.collisions,
           'mux_limit_events': reasons}
    raw, fused = pose['/diff_cont/odom'], pose['/odometry/filtered']
    if raw and fused:
        fr, ff = first_pose['/diff_cont/odom'], first_pose['/odometry/filtered']
        # both start at the same place only if the robot started them together: compare the displacement they report
        dr = (raw[0] - fr[0], raw[1] - fr[1])
        df = (fused[0] - ff[0], fused[1] - ff[1])
        out['raw_vs_fused'] = {'displacement_difference_m': round(math.hypot(dr[0] - df[0], dr[1] - df[1]), 3),
                               'final_heading_difference_deg': round(math.degrees(abs(math.atan2(
                                   math.sin((raw[2] - fr[2]) - (fused[2] - ff[2])), math.cos((raw[2] - fr[2]) - (fused[2] - ff[2]))))), 2)}
    return out


def read_bag(path):
    """Yield (topic, t, payload) for the topics analyse() uses. Needs ROS (rosbag2_py)."""
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = {'/scan': LaserScan, '/odom': Odometry, '/diff_cont/odom': Odometry, '/odometry/filtered': Odometry, '/mux_status': String}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in wanted or topic not in types:
            continue
        m = deserialize_message(data, wanted[topic])
        t = t_ns * 1e-9
        if topic == '/scan':
            yield topic, t, (list(m.ranges), m.angle_min, m.angle_increment, m.range_min)
        elif topic == '/mux_status':
            yield topic, t, m.data
        else:
            q = m.pose.pose.orientation
            yield topic, t, (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of((q.x, q.y, q.z, q.w)))


def format_report(r):
    lines = [f'duration            {r["duration_s"]} s', f'distance            {r["distance_m"]} m  (from {r["distance_source"]})',
             f'minimum clearance   {r["min_clearance_m"]} m', f'contacts            {r["contacts"]}',
             f'limit events        {r["mux_limit_events"] or "none"}']
    if 'raw_vs_fused' in r:
        lines.append(f'raw vs fused        {r["raw_vs_fused"]["displacement_difference_m"]} m displacement, '
                     f'{r["raw_vs_fused"]["final_heading_difference_deg"]} deg heading')
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--json', default=None)
    a = ap.parse_args()
    r = analyse(read_bag(a.bag))
    print(format_report(r))
    if a.json:
        json.dump(r, open(a.json, 'w'), indent=1)


if __name__ == '__main__':
    main()
