#!/usr/bin/env python3
"""Benchmark metrics for autonomous runs: coverage, collisions, minimum clearance, time. Pure Python, no ROS, no numpy.

One run is a *trace*: samples (t, x, y, yaw, gap, pgap) at a few Hz in the WORLD frame, plus the spawn pose.
  gap  = distance in metres between the robot outline and the nearest wall / furniture / prop (<= 0 means contact)
  pgap = the same for the nearest walking person (people are Gazebo actors: no collision, so the robot would drive
         through them; contact is still counted, because that is what a real person would experience)
Both backends produce the same kind of trace, so the same code scores them:
  * 2D simulator (tools/sim2d.py): both gaps from the true geometry
  * Gazebo (benchmark_monitor.py, below): one gap from the lidar, using the robot outline (scan_gap); tools/benchmark.py
    splits it into gap / pgap with the known positions of the walking people

Definitions (stated here once, used in the README table):
  coverage       share of the reachable floor that the robot has come within COVER_RADIUS of (see tools/benchmark.py)
  collisions     separate contacts with walls, furniture or props: gap drops to <= CONTACT_GAP after having been above it
  person contacts  the same for walking people
  min clearance  smallest gap to walls / furniture over the whole run; closest approach to a person likewise
  time           simulated seconds; also the time to reach 25 % and 50 % coverage (None if never reached)
  stops          times the safety layer or the explorer brought the robot to a halt in front of an obstacle

Run in Gazebo (on a machine with the simulator):
  ros2 run mobile_robot_bringup benchmark_monitor.py --ros-args -p duration:=300.0 -p out:=/tmp/run0.json
"""
import json
import math
import statistics

CONTACT_GAP = 0.0          # m: at or below this the outline touches an obstacle
COVER_RADIUS = 0.5         # m: floor within this distance of the robot centre counts as covered

# robot outline in base_link (same numbers as mobile_robot_bringup/tools/house_map.py FOOTPRINT)
FOOTPRINT = [(0.26, 0.15), (0.26, -0.15), (-0.12, -0.195), (-0.12, 0.195)]
LIDAR_X = 0.022            # the lidar sits 0.022 m in front of base_link (chassis origin -0.10, laser +0.122)


def _ray_polygon(ox, oy, theta, poly):
    """Distance from (ox, oy) along direction theta to the polygon outline, or None."""
    dx, dy = math.cos(theta), math.sin(theta)
    best = None
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        ex, ey = bx - ax, by - ay
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        t = ((ax - ox) * ey - (ay - oy) * ex) / den
        u = ((ax - ox) * dy - (ay - oy) * dx) / den
        if t >= 0 and 0 <= u <= 1 and (best is None or t < best):
            best = t
    return best


def outline_table(n=720, poly=FOOTPRINT, lidar_x=LIDAR_X):
    """Distance from the lidar to the robot outline for each of n beams over -pi..pi: the range at which a beam would touch the robot."""
    inc = 2 * math.pi / n
    return [(_ray_polygon(lidar_x, 0.0, -math.pi + i * inc, poly) or 0.0) for i in range(n)]


_TABLE = {}


def scan_gap(ranges, angle_min=-math.pi, inc=None, rmin=0.15):
    """Smallest (range - outline distance) over all beams: how far the nearest obstacle is from the robot outline.
    Beams that return nothing are ignored. Beams closer than the lidar's minimum range are invisible, so contact
    right behind the robot (outline 0.14 m from the lidar) cannot be seen: reported honestly as no data for those beams."""
    n = len(ranges)
    inc = inc or (2 * math.pi / n)
    tab = _TABLE.get(n)
    if tab is None:
        tab = _TABLE[n] = outline_table(n)
    best = math.inf
    for i, r in enumerate(ranges):
        if math.isfinite(r) and r >= rmin:
            # beam i of this scan -> the table is indexed from -pi, so shift by the scan's own start angle
            k = int(round((angle_min + i * inc + math.pi) / (2 * math.pi / n))) % n
            g = r - tab[k]
            if g < best:
                best = g
    return best


class RunAccumulator:
    """Streaming scorer: feed samples, read the numbers at the end."""

    def __init__(self, contact_gap=CONTACT_GAP):
        self.contact_gap = contact_gap
        self.samples = []
        self.collisions = 0
        self.in_contact = False
        self.person_contacts = 0
        self.in_person_contact = False
        self.min_gap = math.inf
        self.min_pgap = math.inf
        self.distance = 0.0
        self.stops = 0
        self._stopped = False

    def update(self, t, x, y, yaw, gap, stopped=False, pgap=math.inf):
        if self.samples:
            px, py = self.samples[-1][1], self.samples[-1][2]
            self.distance += math.hypot(x - px, y - py)
        self.samples.append((t, x, y, yaw, gap, pgap))
        if gap < self.min_gap:
            self.min_gap = gap
        if pgap < self.min_pgap:
            self.min_pgap = pgap
        touching = gap <= self.contact_gap
        if touching and not self.in_contact:
            self.collisions += 1
        self.in_contact = touching
        p_touch = pgap <= self.contact_gap
        if p_touch and not self.in_person_contact:
            self.person_contacts += 1
        self.in_person_contact = p_touch
        if stopped and not self._stopped:
            self.stops += 1
        self._stopped = stopped

    def trace(self, spawn=(0.0, 0.0, 0.0), meta=None):
        return {'spawn': list(spawn), 'samples': [list(s) for s in self.samples], 'stops': self.stops, 'meta': meta or {}}


def time_to(curve, level):
    """curve: [(t, coverage 0..1)] non-decreasing. First time coverage >= level, else None."""
    for t, c in curve:
        if c >= level:
            return t
    return None


def run_metrics(trace, coverage_curve):
    """trace dict (see RunAccumulator.trace) + coverage curve -> flat metrics dict."""
    acc = RunAccumulator()
    for smp in trace['samples']:
        t, x, y, yaw, gap = smp[:5]
        acc.update(t, x, y, yaw, gap, pgap=smp[5] if len(smp) > 5 else math.inf)
    dur = trace['samples'][-1][0] - trace['samples'][0][0] if trace['samples'] else 0.0
    return {
        'duration_s': round(dur, 1),
        'coverage_pct': round(100.0 * (coverage_curve[-1][1] if coverage_curve else 0.0), 1),
        't25_s': time_to(coverage_curve, 0.25),
        't50_s': time_to(coverage_curve, 0.50),
        'collisions': acc.collisions,
        'person_contacts': acc.person_contacts,
        'min_clearance_m': round(acc.min_gap, 3) if math.isfinite(acc.min_gap) else None,
        'min_person_gap_m': round(acc.min_pgap, 3) if math.isfinite(acc.min_pgap) else None,
        'distance_m': round(acc.distance, 1),
        'stops': trace.get('stops', 0),
    }


def _num(vals):
    v = [x for x in vals if x is not None]
    return v


def aggregate(runs):
    """List of run_metrics dicts -> {metric: {mean, std, min, max, n}} (None entries, e.g. 50 % never reached, are counted separately)."""
    out = {}
    for k in ('duration_s', 'coverage_pct', 't25_s', 't50_s', 'collisions', 'person_contacts', 'min_clearance_m',
              'min_person_gap_m', 'distance_m', 'stops'):
        v = _num([r.get(k) for r in runs])
        if not v:
            out[k] = {'n': 0, 'missing': len(runs)}
            continue
        out[k] = {'mean': round(statistics.fmean(v), 2), 'std': round(statistics.pstdev(v), 2) if len(v) > 1 else 0.0,
                  'min': round(min(v), 3), 'max': round(max(v), 3), 'n': len(v), 'missing': len(runs) - len(v)}
    out['runs'] = len(runs)
    out['runs_with_collision'] = sum(1 for r in runs if r.get('collisions', 0) > 0)
    out['runs_with_person_contact'] = sum(1 for r in runs if r.get('person_contacts', 0) > 0)
    return out


def markdown_table(agg):
    """The results table that goes into the README."""
    def ms(k, unit='', fmt='{:.1f}'):
        a = agg[k]
        if not a.get('n'):
            return 'not reached'
        s = (fmt + ' ± ' + fmt).format(a['mean'], a['std']) + unit
        if a.get('missing'):
            s += f" ({a['n']}/{agg['runs']} runs)"
        return s
    rows = [
        ('Runs', f"{agg['runs']}"),
        ('Coverage of reachable floor', ms('coverage_pct', ' %')),
        ('Time to 25 % coverage', ms('t25_s', ' s', '{:.0f}')),
        ('Time to 50 % coverage', ms('t50_s', ' s', '{:.0f}')),
        ('Collisions with walls / furniture per run', ms('collisions', '', '{:.2f}')),
        ('Runs with at least one such collision', f"{agg['runs_with_collision']} of {agg['runs']}"),
        ('Minimum clearance to walls / furniture (worst run)', f"{agg['min_clearance_m']['min']:.2f} m" if agg['min_clearance_m'].get('n') else 'n/a'),
        ('Minimum clearance to walls / furniture (mean over runs)', ms('min_clearance_m', ' m', '{:.2f}')),
        ('Contacts with walking people per run', ms('person_contacts', '', '{:.2f}')),
        ('Runs with at least one person contact', f"{agg['runs_with_person_contact']} of {agg['runs']}"),
        ('Closest approach to a person (worst run)', f"{agg['min_person_gap_m']['min']:.2f} m" if agg['min_person_gap_m'].get('n') else 'n/a'),
        ('Distance driven', ms('distance_m', ' m', '{:.0f}')),
        ('Stops in front of obstacles', ms('stops', '', '{:.0f}')),
        ('Run length', ms('duration_s', ' s', '{:.0f}')),
    ]
    w = max(len(a) for a, _ in rows)
    lines = ['| ' + 'Metric'.ljust(w) + ' | Result |', '|' + '-' * (w + 2) + '|--------|']
    lines += [f'| {a.ljust(w)} | {b} |' for a, b in rows]
    return '\n'.join(lines)


# --------------------------------------------------------------------------------------------- ROS node (Gazebo runs)
def main():
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    class Monitor(Node):
        def __init__(self):
            super().__init__('benchmark_monitor')
            self.duration = self.declare_parameter('duration', 300.0).value
            self.out = self.declare_parameter('out', 'run.json').value
            self.sample_period = self.declare_parameter('sample_period', 0.2).value
            self.spawn = list(self.declare_parameter('spawn', [0.0, 0.0, 0.0]).value)
            self.start_delay = self.declare_parameter('start_delay', 5.0).value   # let the controllers and SLAM come up
            self.acc = RunAccumulator()
            self.gap = math.inf
            self.t0 = None
            self.last_sample = -1e9
            self.status = ''
            self.tf = Buffer()
            TransformListener(self.tf, self)
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
            self.create_subscription(Odometry, '/odom', self.on_odom, 10)
            self.create_subscription(String, '/mux_status', lambda m: setattr(self, 'status', m.data), 10)
            self.pose = None
            self.done = False
            self.create_timer(self.sample_period, self.tick)

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def on_scan(self, m):
            self.gap = min(self.gap, scan_gap(list(m.ranges), m.angle_min, m.angle_increment, m.range_min or 0.15))

        def on_odom(self, m):
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

        def current_pose(self):
            """Pose in the map frame (slam_toolbox corrects the wheel drift); falls back to odometry."""
            try:
                t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
                q = t.transform.rotation
                return (t.transform.translation.x, t.transform.translation.y,
                        math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
            except Exception:
                return self.pose

        def tick(self):
            if self.done or self.pose is None:
                return
            now = self.now()
            if self.t0 is None:
                self.t0 = now + self.start_delay
            if now < self.t0:
                self.gap = math.inf
                return
            p = self.current_pose()
            if p is None:
                return
            # map frame -> world frame (the map origin is the spawn pose)
            sx, sy, sa = self.spawn
            c, s = math.cos(sa), math.sin(sa)
            x, y, yaw = sx + c * p[0] - s * p[1], sy + s * p[0] + c * p[1], p[2] + sa
            self.acc.update(now - self.t0, x, y, yaw, self.gap, stopped='obstacle' in self.status or 'STOP' in self.status.upper())
            self.gap = math.inf
            if now - self.t0 >= self.duration:
                self.done = True
                with open(self.out, 'w') as f:
                    json.dump(self.acc.trace(self.spawn, {'backend': 'gazebo', 'sim_t0': self.t0}), f)
                self.get_logger().info(f'benchmark run finished: {len(self.acc.samples)} samples -> {self.out}')

    rclpy.init()
    n = Monitor()
    try:
        while rclpy.ok() and not n.done:
            rclpy.spin_once(n, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    try:
        n.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
