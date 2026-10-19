#!/usr/bin/env python3
"""The house world as 2D geometry, read straight from mobile_robot_gazebo/worlds/house_world.sdf.

Nothing is hand-copied: walls, furniture, pillars, doors and the textured props are the <collision> boxes and
cylinders of the real world file, so the 2D model cannot drift away from the Gazebo world (a test checks it).

What it provides (used by the 2D benchmark simulator, the patrol-waypoint tests and the README figures):
  * obstacle grid + Euclidean distance field (2 cm cells)  -> collision test, clearance, lidar ray casting
  * walking people (Gazebo actors) as moving circles, taken from the actor trajectories in the SDF
  * the wet-tile friction patch
  * free-space reachability from a start cell (denominator of the coverage metric)

Conventions: metres, world frame of the SDF, robot starts at (0, 0) facing +x.
"""
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
GZ = os.path.join(REPO, 'mobile_robot_gazebo')
DEFAULT_WORLD = os.path.join(GZ, 'worlds', 'house_world.sdf')

# Heights above the floor (m). The lidar plane crosses the scan at this height; the robot body sweeps this band.
LIDAR_Z = 0.26
BODY_Z0, BODY_Z1 = 0.06, 0.20
PERSON_RADIUS = 0.22          # a walking person seen by the lidar, as a circle
RES = 0.02                    # grid resolution, m
# Robot footprint (base_link frame, x forward): chassis box x -0.10..0.20, caster to x 0.26 - 0.025 - 0.035,
# drive wheels at y = +-0.175 (+ half tyre width 0.02). See mobile_robot_description/urdf/robot_core.xacro.
FOOTPRINT = [(0.26, 0.15), (0.26, -0.15), (-0.12, -0.195), (-0.12, 0.195)]


def _floats(text, n):
    v = [float(t) for t in (text or '').split()]
    return (v + [0.0] * n)[:n]


def _pose(elem):
    p = elem.find('pose') if elem is not None else None
    return _floats(p.text if p is not None else '', 6)


def _compose(parent, child):
    """2D composition of (x, y, z, roll, pitch, yaw) poses (roll/pitch must be ~0, see the ramp special case)."""
    px, py, pz, _, _, pyaw = parent
    cx, cy, cz, _, _, cyaw = child
    c, s = math.cos(pyaw), math.sin(pyaw)
    return [px + c * cx - s * cy, py + s * cx + c * cy, pz + cz, 0.0, 0.0, pyaw + cyaw]


def _collisions(model_elem, model_pose, model_dir=None):
    """Yield dicts {kind, x, y, yaw, size/radius, z0, z1} for every collision shape of an SDF <model>."""
    for link in model_elem.findall('link'):
        lp = _compose(model_pose, _pose(link))
        for col in link.findall('collision'):
            cp = _compose(lp, _pose(col))
            g = col.find('geometry')
            if g is None:
                continue
            box, cyl = g.find('box'), g.find('cylinder')
            if box is not None:
                sx, sy, sz = _floats(box.find('size').text, 3)
                yield dict(kind='box', x=cp[0], y=cp[1], yaw=cp[5], sx=sx, sy=sy, z0=cp[2] - sz / 2, z1=cp[2] + sz / 2)
            elif cyl is not None:
                r = float(cyl.find('radius').text)
                ln = float(cyl.find('length').text)
                # an upright cylinder; a lying one (caster wheel style) does not occur in the world
                yield dict(kind='cyl', x=cp[0], y=cp[1], yaw=0.0, r=r, z0=cp[2] - ln / 2, z1=cp[2] + ln / 2)


class HouseMap:
    def __init__(self, world=DEFAULT_WORLD, models_dir=None, res=RES, margin=0.5):
        self.res = res
        self.world_path = world
        models_dir = models_dir or os.path.join(GZ, 'models')
        root = ET.parse(world).getroot()
        w = root.find('world')
        self.shapes = []                 # every collision shape, with names, for tests and plotting
        self.people = []                 # list of dicts(name, times, points)
        self.patches = []                # (name, x0, x1, y0, y1, mu)
        for m in w.findall('model'):
            name = m.get('name')
            if name == 'ground_plane' or name.startswith('ramp') or name.startswith('door_'):
                continue                  # the ramp is drivable (7 degrees); hinged doors are pushed open: modelled open
            mp = _pose(m)
            shapes = list(_collisions(m, mp))
            for s in shapes:
                s['name'] = name
            # friction patches: a flat, low collision with a friction override
            if shapes and all(s['z1'] < 0.01 for s in shapes) and m.find('.//friction') is not None:
                for s in shapes:
                    mu = float(m.find('.//mu').text)
                    self.patches.append((name, s['x'] - s['sx'] / 2, s['x'] + s['sx'] / 2,
                                         s['y'] - s['sy'] / 2, s['y'] + s['sy'] / 2, mu))
                continue
            self.shapes.extend(shapes)
        for inc in w.findall('include'):
            uri = inc.find('uri').text
            if not uri.startswith('model://'):
                continue
            kind = uri[len('model://'):]
            mdl = ET.parse(os.path.join(models_dir, kind, 'model.sdf')).getroot().find('model')
            ip = _floats(inc.find('pose').text, 6)
            for s in _collisions(mdl, ip):
                s['name'] = inc.find('name').text
                self.shapes.append(s)
        for a in w.findall('actor'):
            wps = a.findall('.//waypoint')
            ts = [float(p.find('time').text) for p in wps]
            pts = [tuple(_floats(p.find('pose').text, 6)[:2]) for p in wps]
            self.people.append(dict(name=a.get('name'), times=ts, points=pts))
        # world extent from the outer walls
        xs = [s['x'] for s in self.shapes if s['name'].startswith('outer_')]
        ys = [s['y'] for s in self.shapes if s['name'].startswith('outer_')]
        self.x0, self.x1 = min(xs) - 1.0, max(xs) + 1.0
        self.y0, self.y1 = min(ys) - 1.0, max(ys) + 1.0
        self._rasterise()
        self.margin = margin

    # ------------------------------------------------------------------ grid
    def _rasterise(self):
        nx = int(round((self.x1 - self.x0) / self.res))
        ny = int(round((self.y1 - self.y0) / self.res))
        xs = self.x0 + (np.arange(nx) + 0.5) * self.res
        ys = self.y0 + (np.arange(ny) + 0.5) * self.res
        X, Y = np.meshgrid(xs, ys, indexing='ij')
        solid = np.zeros((nx, ny), bool)       # blocks the body (collision)
        seen = np.zeros((nx, ny), bool)        # blocks the lidar beam
        for s in self.shapes:
            body = s['z0'] < BODY_Z1 and s['z1'] > BODY_Z0
            lid = s['z0'] < LIDAR_Z < s['z1']
            if not (body or lid):
                continue
            if s['kind'] == 'box':
                c, sn = math.cos(-s['yaw']), math.sin(-s['yaw'])
                dx, dy = X - s['x'], Y - s['y']
                u, v = c * dx - sn * dy, sn * dx + c * dy
                inside = (np.abs(u) <= s['sx'] / 2) & (np.abs(v) <= s['sy'] / 2)
            else:
                inside = (X - s['x']) ** 2 + (Y - s['y']) ** 2 <= s['r'] ** 2
            if body:
                solid |= inside
            if lid:
                seen |= inside
        self.nx, self.ny = nx, ny
        self.solid, self.seen = solid, seen
        # distance (m) to the nearest solid / visible cell; 0 inside
        self.dist_solid = ndimage.distance_transform_edt(~solid) * self.res
        self.dist_seen = ndimage.distance_transform_edt(~seen) * self.res

    def cell(self, x, y):
        return (np.clip(((np.asarray(x) - self.x0) / self.res).astype(int), 0, self.nx - 1),
                np.clip(((np.asarray(y) - self.y0) / self.res).astype(int), 0, self.ny - 1))

    def inside_world(self, x, y):
        return (self.x0 <= x) & (x <= self.x1) & (self.y0 <= y) & (y <= self.y1)

    # ------------------------------------------------------------------ people
    def person_positions(self, t):
        """Walking people at time t (s): list of (x, y). Linear interpolation, looped like the Gazebo actors."""
        out = []
        for p in self.people:
            T = p['times'][-1]
            tt = t % T if T > 0 else 0.0
            ts, pts = p['times'], p['points']
            for i in range(len(ts) - 1):
                if ts[i] <= tt <= ts[i + 1]:
                    f = 0.0 if ts[i + 1] == ts[i] else (tt - ts[i]) / (ts[i + 1] - ts[i])
                    out.append((pts[i][0] + f * (pts[i + 1][0] - pts[i][0]), pts[i][1] + f * (pts[i + 1][1] - pts[i][1])))
                    break
        return out

    # ------------------------------------------------------------------ robot footprint
    @staticmethod
    def footprint_points(x, y, yaw, step=0.04):
        """Points along the footprint outline in the world frame (for collision / clearance)."""
        pts = []
        n = len(FOOTPRINT)
        for i in range(n):
            (ax, ay), (bx, by) = FOOTPRINT[i], FOOTPRINT[(i + 1) % n]
            k = max(1, int(math.hypot(bx - ax, by - ay) / step))
            for j in range(k):
                f = j / k
                pts.append((ax + f * (bx - ax), ay + f * (by - ay)))
        p = np.array(pts)
        c, s = math.cos(yaw), math.sin(yaw)
        return np.stack([x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1]], axis=1)

    def clearance(self, x, y, yaw, people=()):
        """Smallest gap (m) between the robot outline and any obstacle or person; <= 0 means contact."""
        p = self.footprint_points(x, y, yaw)
        i, j = self.cell(p[:, 0], p[:, 1])
        best = float(self.dist_solid[i, j].min())
        for (px, py) in people:
            best = min(best, float(np.hypot(p[:, 0] - px, p[:, 1] - py).min()) - PERSON_RADIUS)
        # the outline sampled every 4 cm hides a thin wall between samples only if the wall is < 4 cm thick; walls are 15 cm
        return best

    def person_gap(self, x, y, yaw, people):
        """Gap (m) between the robot outline and the nearest walking person's body (<= 0: touching). inf without people."""
        if not len(people):
            return math.inf
        p = self.footprint_points(x, y, yaw)
        return min(float(np.hypot(p[:, 0] - px, p[:, 1] - py).min()) - PERSON_RADIUS for (px, py) in people)

    def collides(self, x, y, yaw, people=()):
        return self.clearance(x, y, yaw, people) <= 0.0

    # ------------------------------------------------------------------ lidar
    def scan(self, x, y, yaw, n=720, rmin=0.15, rmax=12.0, people=()):
        """Ideal lidar ranges for n beams from -3.14159 to +3.14159 (robot frame, like the Gazebo sensor), by sphere
        tracing the distance field. The lidar sits 0.022 m in front of base_link (chassis origin -0.10, laser +0.122)."""
        lx = x + math.cos(yaw) * 0.022
        ly = y + math.sin(yaw) * 0.022
        ang = yaw + (-3.14159 + np.arange(n) * (2 * 3.14159 / (n - 1)))
        dx, dy = np.cos(ang), np.sin(ang)
        t = np.zeros(n)
        alive = np.ones(n, bool)
        for _ in range(80):
            px, py = lx + dx * t, ly + dy * t
            i, j = self.cell(px, py)
            d = self.dist_seen[i, j]
            outside = ~self.inside_world(px, py)
            hit = d < self.res * 0.9
            alive &= ~(hit | outside | (t >= rmax))
            if not alive.any():
                break
            t = np.where(alive, t + np.maximum(d, self.res * 0.5), t)
        t = np.where(outside | (t >= rmax), np.inf, t)
        for (cx, cy) in people:                       # people: analytic ray-circle intersection
            fx, fy = lx - cx, ly - cy
            b = fx * dx + fy * dy
            c = fx * fx + fy * fy - PERSON_RADIUS ** 2
            disc = b * b - c
            ok = disc >= 0
            r = np.where(ok, -b - np.sqrt(np.where(ok, disc, 0.0)), np.inf)
            r = np.where(r > 0, r, np.inf)
            t = np.minimum(t, r)
        t = np.where((t < rmin) | (t > rmax), np.inf, t)
        return t

    # ------------------------------------------------------------------ reachable free space
    def free_mask(self, inflate=None):
        """Cells where the robot centre can be: farther than `inflate` from every solid cell (default: circumscribed radius)."""
        inflate = 0.26 if inflate is None else inflate
        return (self.dist_solid > inflate) & self.inside_world_grid()

    def inside_world_grid(self):
        m = np.zeros((self.nx, self.ny), bool)
        m[:] = True
        return m

    def reachable(self, start=(0.0, 0.0), inflate=None):
        free = self.free_mask(inflate)
        lab, _ = ndimage.label(free)
        i, j = self.cell(start[0], start[1])
        k = lab[i, j]
        if k == 0:
            raise ValueError(f'start {start} is not in free space')
        return lab == k

    def slip_factor(self, x, y):
        """Friction patch the robot is on: returns mu (None outside any patch)."""
        for (_, x0, x1, y0, y1, mu) in self.patches:
            if x0 <= x <= x1 and y0 <= y <= y1:
                return mu
        return None

    def astar(self, a, b, inflate=None):
        """Shortest free path length (m) between two points over the inflated grid (8-connected, coarse 5 cm), or None."""
        step = 3
        free = self.free_mask(inflate)[::step, ::step]
        ia, ja = self.cell(*a)
        ib, jb = self.cell(*b)
        sa, sb = (ia // step, ja // step), (ib // step, jb // step)
        if not (free[sa] and free[sb]):
            return None
        import heapq
        g = {sa: 0.0}
        pq = [(0.0, sa)]
        nbrs = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
        while pq:
            f, cur = heapq.heappop(pq)
            if cur == sb:
                return g[cur] * self.res * step
            if f - 0 > g[cur] + 1e9:
                continue
            for di, dj, c in nbrs:
                n = (cur[0] + di, cur[1] + dj)
                if 0 <= n[0] < free.shape[0] and 0 <= n[1] < free.shape[1] and free[n]:
                    ng = g[cur] + c
                    if ng < g.get(n, 1e18):
                        g[n] = ng
                        h = math.hypot(n[0] - sb[0], n[1] - sb[1])
                        heapq.heappush(pq, (ng + h, n))
        return None


_CACHE = {}


def load(world=DEFAULT_WORLD):
    if world not in _CACHE:
        _CACHE[world] = HouseMap(world)
    return _CACHE[world]


if __name__ == '__main__':
    m = load()
    print(f'{len(m.shapes)} collision shapes, {len(m.people)} people, patches: {[(p[0], p[5]) for p in m.patches]}')
    print(f'grid {m.nx} x {m.ny} cells of {m.res} m, reachable area {m.reachable().sum() * m.res ** 2:.1f} m2')
