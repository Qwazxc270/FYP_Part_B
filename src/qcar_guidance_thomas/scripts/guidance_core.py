#! /usr/bin/env python3
"""
guidance_core -- all guidance logic for qcar_guidance_thomas, with NO ROS.

Keeping the algorithms free of rospy means the exact same code runs in the
ROS node (online_guidance_node.py) and in the offline test harness
(extras/offline_test.py), so what is tested is what is deployed.

Two phases
----------
MAPPING (lap 1)
    Local, online planning from the cones currently visible or recently
    remembered. Pair the two track boundaries, take midpoints, spline them,
    set speed from curvature. Modes: CREEP / TRACK / RECOVERY.
    While driving, the car records its own path.

RACE (lap 2 onwards)
    When the car returns to its start pose, the full cone map (from SLAM) is
    used to compute ONE optimised line for the whole lap:
      1. classify every mapped cone as left/right of the recorded lap-1 path
      2. measure the track edges at every point along the lap
      3. minimum-curvature optimisation: each point may slide sideways
         inside the edges (minus a safety margin); choose the offsets that
         make the line as smooth as possible (bounded least squares)
      4. speed profile from physics: lateral-grip limit in corners, then
         forward/backward passes for acceleration and braking limits, so the
         car slows BEFORE a corner rather than in it
    Each cycle only the next few metres, starting at the car, are published
    (the controller treats index 0 as "at the car").

Frames
------
local  smooth wheel+IMU odometry the controller uses (drifts slowly)
map    EKF-SLAM frame the cone map is in (no drift, but jumps on updates)
A low-pass map->local transform is estimated continuously; the race line
lives in the map frame and every published window is re-expressed in the
local frame, so the controller gets a drift-corrected but smooth target.
"""

import math
from collections import deque

import numpy as np
from scipy.interpolate import interp1d
from scipy import sparse
from scipy.sparse.linalg import spsolve

UNKNOWN, BLUE, YELLOW, ORANGE = 0, 1, 2, 3

DEFAULTS = {
    # ---- mapping-lap speeds (m/s)
    'max_velocity': 0.5,
    'min_velocity': 0.2,
    'creep_velocity': 0.15,
    'recovery_velocity': 0.05,
    'curvature_slowdown': 2.0,
    'speed_lookahead_m': 1.0,
    # ---- mapping-lap planning
    'min_cones_per_side': 2,
    'creep_distance': 1.5,
    'max_planning_range': 6.0,
    'plan_rate_hz': 2.0,
    'cone_memory_seconds': 3.0,
    'cone_merge_dist': 0.5,
    'detections_in_car_frame': True,
    'use_geometric_fallback': True,
    'min_ahead': -0.5,
    'max_lateral': 3.0,
    'max_link': 1.6,
    'max_link_turn_deg': 45.0,
    'max_first_link_turn_deg': 80.0, # first link when only the car heading is known
    'max_seeded_first_turn_deg': 65.0, # first link from the boundary's own direction:
                                     # loose enough for hairpins, rejects ~90 deg crossovers
    'use_edge_following': True,      # follow one visible edge at half-width
    'default_track_width': 1.5,      # until real cone pairs have been measured
    # ---- input validation
    'max_detections_per_frame': 80,  # real frames reach ~45 on dense tracks; glitch was 227
    'min_hits': 2,
    'min_track_width': 0.6,
    'max_track_width': 3.0,
    'width_tolerance': (0.6, 1.6),   # accept pairs within this x measured width
    # ---- recovery / stuck
    'detection_timeout': 2.0,
    'lost_stop_timeout': 6.0,        # no track for this long -> stop safely
    'hold_timeout': 1.5,             # keep last path this long through gaps
    'stuck_window': 3.0,
    'stuck_min_motion': 0.10,
    # ---- lap detection
    'lap_close_radius': 1.0,
    'lap_leave_radius': 3.0,
    'lap_min_length': 10.0,
    'lap_max_heading_err_deg': 60.0,
    'total_laps': 0,                 # 0 = keep racing forever
    # ---- race line
    'race_enabled': True,
    'race_map_source': 'auto',       # auto | slam | own
    'race_allow_own_map': False,     # race from our own map with NO SLAM? Unsafe:
                                     # the line drifts with odometry (hits in testing)
    'race_margin': 0.40,             # keep this far inside each edge (m)
    'race_min_clearance': 0.28,      # car half-width + cone radius + tracking allowance
    'race_max_cone_lateral': 2.5,
    'race_build_ds': 0.20,
    'race_iterations': 5,            # re-solve passes (converges in ~4)
    'race_ds': 0.125,
    'race_max_missing_frac': 0.30,
    'race_window_m': 4.0,
    'race_max_velocity': 0.8,
    'race_min_velocity': 0.2,
    'race_lat_accel': 1.0,           # grip limit v^2*kappa (m/s^2)
    'race_accel': 0.5,
    'race_brake': 1.0,
    'race_max_deviation': 0.6,       # fall back to local planning beyond this
    'race_rejoin_deviation': 0.3,
    'race_live_clearance': 0.20,     # race path this close to a cone the car can
                                     # SEE right now -> line misaligned, plan locally
    'race_live_check_m': 2.5,        # how far along the path to check
    # ---- frames
    'align_tau': 1.0,                # map->local smoothing time constant (s)
    'slam_timeout': 2.0,
}


# ============================================================ small helpers

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class ThrottledLog:
    """log(level, key, msg, period) -- only emits each key every `period` s."""

    def __init__(self, sink):
        self.sink = sink
        self.last = {}

    def __call__(self, level, key, msg, period=0.0, now=None):
        if period > 0 and now is not None:
            if now - self.last.get(key, -1e9) < period:
                return
            self.last[key] = now
        self.sink(level, msg)


def dedupe(path):
    """Drop duplicate consecutive points (zero-length segments break interp1d)."""
    if path.shape[1] == 0:
        return path
    keep = [0]
    for i in range(1, path.shape[1]):
        if math.hypot(path[0, i] - path[0, keep[-1]], path[1, i] - path[1, keep[-1]]) > 1e-6:
            keep.append(i)
    return path[:, keep]


def resample_open(path, points_per_metre=8.0, min_points=3):
    """Quadratic spline through an open polyline, resampled evenly."""
    path = dedupe(path)
    if path.shape[1] < 3:
        return path
    d = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(path[0]), np.diff(path[1])))])
    if d[-1] <= 0:
        return path
    n = max(int(round(points_per_metre * d[-1])), min_points)
    return interp1d(d / d[-1], path, kind='quadratic', axis=1)(np.linspace(0, 1, n))


def resample_closed(xy, ds):
    """Linear resample of a CLOSED loop (Nx2) at spacing ds. Returns Mx2."""
    xy = np.asarray(xy, float)
    loop = np.vstack([xy, xy[:1]])
    seg = np.hypot(np.diff(loop[:, 0]), np.diff(loop[:, 1]))
    keep = np.concatenate([[True], seg > 1e-9])
    loop = loop[keep]
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(loop[:, 0]), np.diff(loop[:, 1])))])
    total = s[-1]
    n = max(int(round(total / ds)), 8)
    si = np.linspace(0, total, n, endpoint=False)
    return np.column_stack([np.interp(si, s, loop[:, 0]), np.interp(si, s, loop[:, 1])])


def smooth_closed(xy, window=7, passes=2):
    out = np.asarray(xy, float).copy()
    k = window // 2
    for _ in range(passes):
        acc = np.zeros_like(out)
        for o in range(-k, k + 1):
            acc += np.roll(out, o, axis=0)
        out = acc / (2 * k + 1)
    return out



def box_qp(H, g, lo, hi, max_iter=None):
    """min 0.5 a'Ha + g'a  s.t. lo <= a <= hi  (H sparse, symmetric, PD).

    Textbook primal active-set method (Nocedal & Wright, Alg. 16.3) with
    direct sparse solves. Starts from a = 0 (the track centre, always
    feasible), steps toward the optimum of the free variables, stops at the
    first bound it hits and fixes that variable, and releases a fixed
    variable only when its Lagrange multiplier has the wrong sign. Exact,
    and fast for the banded systems the race-line problem produces.
    """
    n = len(g)
    H = sparse.csc_matrix(H)
    a = np.clip(np.zeros(n), lo, hi)
    state = np.zeros(n, dtype=int)          # 0 free, -1 at lo, +1 at hi
    state[a <= lo] = -1
    state[a >= hi] = 1
    max_iter = max_iter or 20 * n + 50
    for _ in range(max_iter):
        free = np.where(state == 0)[0]
        grad = H @ a + g
        step = np.zeros(n)
        if len(free):
            step[free] = np.atleast_1d(spsolve(H[free][:, free], -grad[free]))
        if np.max(np.abs(step)) < 1e-12:
            # at the optimum of the current working set: check multipliers
            bad_lo = (state == -1) & (grad < -1e-10)   # wants to increase
            bad_hi = (state == 1) & (grad > 1e-10)     # wants to decrease
            bad = bad_lo | bad_hi
            if not bad.any():
                return np.clip(a, lo, hi)
            worst = int(np.argmax(np.where(bad, np.abs(grad), -1.0)))
            state[worst] = 0
            continue
        # largest feasible step along `step`, stopping at the first bound
        t, block, block_state = 1.0, -1, 0
        for i in free:
            if step[i] < -1e-15:
                ti = (lo[i] - a[i]) / step[i]
                if ti < t:
                    t, block, block_state = ti, i, -1
            elif step[i] > 1e-15:
                ti = (hi[i] - a[i]) / step[i]
                if ti < t:
                    t, block, block_state = ti, i, 1
        a = a + max(t, 0.0) * step
        if block >= 0:
            state[block] = block_state
            a[block] = lo[block] if block_state == -1 else hi[block]
    return np.clip(a, lo, hi)


def closed_normals(xy):
    t = np.roll(xy, -1, axis=0) - np.roll(xy, 1, axis=0)
    t /= np.maximum(np.hypot(t[:, 0], t[:, 1]), 1e-9)[:, None]
    return t, np.column_stack([-t[:, 1], t[:, 0]])      # tangent, left normal


def closed_curvature(xy):
    """Signed curvature of a closed polyline (Menger curvature, per point)."""
    a, b, c = np.roll(xy, 1, axis=0), xy, np.roll(xy, -1, axis=0)
    ab, bc, ca = b - a, c - b, a - c
    cross = ab[:, 0] * bc[:, 1] - ab[:, 1] * bc[:, 0]
    den = (np.hypot(ab[:, 0], ab[:, 1]) * np.hypot(bc[:, 0], bc[:, 1]) *
           np.hypot(ca[:, 0], ca[:, 1]))
    return 2.0 * cross / np.maximum(den, 1e-12)


def open_curvature(path):
    n = path.shape[1]
    curv = np.zeros(n)
    for i in range(1, n - 1):
        v1 = path[:, i] - path[:, i - 1]
        v2 = path[:, i + 1] - path[:, i]
        a1, a2 = math.atan2(v1[1], v1[0]), math.atan2(v2[1], v2[0])
        curv[i] = abs(wrap(a2 - a1)) / (np.linalg.norm(v1) + 1e-6)
    if n > 2:
        curv[0], curv[-1] = curv[1], curv[-2]
    return curv


def make_trajectory(path, speeds, lookahead_m):
    """waypoint_times from the speed profile; velocity = slowest speed in the
    next `lookahead_m` metres (the controller uses one scalar speed target,
    so taking the upcoming minimum makes it brake before corners)."""
    n = path.shape[1]
    speeds = np.maximum(np.asarray(speeds, float), 0.0)
    times, dist = [0.0], [0.0]
    for i in range(1, n):
        d = math.hypot(path[0, i] - path[0, i - 1], path[1, i] - path[1, i - 1])
        v = max((speeds[i - 1] + speeds[i]) / 2.0, 0.05)
        times.append(times[-1] + d / v)
        dist.append(dist[-1] + d)
    ahead = [speeds[i] for i in range(n) if dist[i] <= lookahead_m] or [speeds[0]]
    return {'x': path[0].tolist(), 'y': path[1].tolist(),
            't': times, 'velocity': float(min(ahead))}


def rot(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y, s * x + c * y


# ============================================================ cone memory

class ConeMemory:
    """Short-term cone memory in the LOCAL frame, merged across frames.

    Counts sightings per cone so one-frame phantoms can be excluded, and
    forgets cones not seen for cone_memory_seconds.
    """

    def __init__(self, p):
        self.p = p
        self.cones = []          # dict x y colour hits last_seen
        self.rejected_frames = 0

    def _add(self, x, y, colour, now):
        for c in self.cones:
            if math.hypot(x - c['x'], y - c['y']) >= self.p['cone_merge_dist']:
                continue
            if c['colour'] == colour or UNKNOWN in (c['colour'], colour):
                c['x'], c['y'] = x, y
                c['last_seen'] = now
                c['hits'] += 1
                if c['colour'] == UNKNOWN and colour != UNKNOWN:
                    c['colour'] = colour
                return
        self.cones.append({'x': x, 'y': y, 'colour': colour, 'hits': 1, 'last_seen': now})

    def update(self, dets, capture_pose, now):
        """dets: [(x, y, colour)] in the car frame (or local frame if
        detections_in_car_frame is False). Returns False if rejected."""
        if len(dets) > self.p['max_detections_per_frame']:
            self.rejected_frames += 1
            return False
        cx, cy, cyaw = capture_pose
        for (dx, dy, col) in dets:
            if col == ORANGE:
                continue                # start/finish gate, not a boundary
            if self.p['detections_in_car_frame']:
                rx, ry = rot(dx, dy, cyaw)
                x, y = cx + rx, cy + ry
            else:
                x, y = dx, dy
            self._add(x, y, col, now)
        return True

    def expire(self, now):
        self.cones = [c for c in self.cones
                      if now - c['last_seen'] < self.p['cone_memory_seconds']]

    def trusted(self):
        return [c for c in self.cones if c['hits'] >= self.p['min_hits']]

    @staticmethod
    def _in_car(c, pose):
        x, y, yaw = pose
        dx, dy = c['x'] - x, c['y'] - y
        cy, sy = math.cos(yaw), math.sin(yaw)
        return cy * dx + sy * dy, -sy * dx + cy * dy

    def _grow(self, chains, pts, used, dirs, strict=None):
        """Grow one or more boundary chains along the track: each step adds
        the nearest unused cone within max_link that continues roughly
        forward. Keeps outer cones on bends on the correct side."""
        cos_lim = math.cos(math.radians(self.p['max_link_turn_deg']))
        cos_first = math.cos(math.radians(self.p['max_first_link_turn_deg']))
        cos_seeded = math.cos(math.radians(self.p['max_seeded_first_turn_deg']))
        strict = strict or [False] * len(chains)
        active = [True] * len(chains)
        while any(active):
            for k in range(len(chains)):
                if not active[k]:
                    continue
                tail, d = chains[k][-1], dirs[k]
                best, best_s = None, float('inf')
                for q in pts:
                    if q in used:
                        continue
                    vx, vy = q[0] - tail[0], q[1] - tail[1]
                    dist = math.hypot(vx, vy)
                    if dist < 1e-6 or dist > self.p['max_link']:
                        continue
                    ct = (vx * d[0] + vy * d[1]) / dist
                    if len(chains[k]) == 1:
                        lim_k = cos_seeded if strict[k] else cos_first
                    else:
                        lim_k = cos_lim
                    if ct < lim_k:
                        continue
                    score = dist + 0.5 * math.acos(max(-1.0, min(1.0, ct)))
                    if score < best_s:
                        best, best_s = q, score
                if best is None:
                    active[k] = False
                    continue
                used.add(best)
                n = math.hypot(best[0] - tail[0], best[1] - tail[1])
                dirs[k] = ((best[0] - tail[0]) / n, (best[1] - tail[1]) / n)
                chains[k].append(best)
        return chains

    def _seed_dir(self, seed, same_side, heading):
        """Direction the boundary was already travelling at `seed`, from the
        nearest same-side cone just behind it. Returns (dir, found).

        Starting a chain from the boundary's own direction (rather than the
        car's heading) is what keeps the car going STRAIGHT through a
        figure-8 crossover: the crossing corridor is ~90 deg off the
        boundary direction and gets rejected, whereas in a hairpin the
        boundary itself curves so the chain still follows it."""
        best, bd = None, float('inf')
        for q in same_side:
            if q == seed:
                continue
            vx, vy = seed[0] - q[0], seed[1] - q[1]
            d = math.hypot(vx, vy)
            if d < 1e-6 or d > self.p['max_link']:
                continue
            if (vx * heading[0] + vy * heading[1]) / d < 0.3:
                continue                    # not behind the seed
            if d < bd:
                best, bd = (vx / d, vy / d), d
        return (best, True) if best else (heading, False)

    def sides_by_colour(self, pose):
        heading = (math.cos(pose[2]), math.sin(pose[2]))
        result = []
        for colour in (BLUE, YELLOW):
            pts, seeds = [], []
            for c in self.trusted():
                if c['colour'] != colour:
                    continue
                if math.hypot(c['x'] - pose[0], c['y'] - pose[1]) > self.p['max_planning_range']:
                    continue
                pt = (c['x'], c['y'])
                pts.append(pt)
                a, l = self._in_car(c, pose)
                if a >= self.p['min_ahead'] and abs(l) <= self.p['max_lateral']:
                    seeds.append(pt)
            if not seeds:
                result.append([])
                continue
            seed = min(seeds, key=lambda q: math.hypot(q[0] - pose[0], q[1] - pose[1]))
            same = [(c['x'], c['y']) for c in self.trusted() if c['colour'] == colour]
            d0, found = self._seed_dir(seed, same, heading)
            result.append(self._grow([[seed]], pts, {seed}, [d0], [found])[0])
        return result[0], result[1]

    def sides_by_geometry(self, pose):
        pool = []
        for c in self.trusted():
            a, l = self._in_car(c, pose)
            if a < self.p['min_ahead']:
                continue
            if math.hypot(a, l) > self.p['max_planning_range']:
                continue
            pool.append(((c['x'], c['y']), a, l))
        lefts = [t for t in pool if 0 < t[2] <= self.p['max_lateral']]
        rights = [t for t in pool if -self.p['max_lateral'] <= t[2] < 0]
        if not lefts or not rights:
            return [], []
        sl = min(lefts, key=lambda t: math.hypot(t[1], t[2]))[0]
        sr = min(rights, key=lambda t: math.hypot(t[1], t[2]))[0]
        heading = (math.cos(pose[2]), math.sin(pose[2]))
        allc = [((c['x'], c['y']),) + self._in_car(c, pose) for c in self.trusted()]
        same_l = [t[0] for t in allc if t[2] > 0]
        same_r = [t[0] for t in allc if t[2] < 0]
        dl, fl = self._seed_dir(sl, same_l, heading)
        dr, fr = self._seed_dir(sr, same_r, heading)
        chains = self._grow([[sl], [sr]], [t[0] for t in pool], {sl, sr}, [dl, dr], [fl, fr])
        return chains[0], chains[1]


class PersistentMap:
    """Long-term cone map built from our own trusted detections (local frame).
    Only used for the race line when no SLAM map is available."""

    def __init__(self, merge_dist):
        self.merge = merge_dist
        self.cones = []          # dict x y colour n

    def add(self, x, y, colour):
        for c in self.cones:
            if math.hypot(x - c['x'], y - c['y']) < self.merge:
                n = c['n']
                c['x'] = (c['x'] * n + x) / (n + 1)
                c['y'] = (c['y'] * n + y) / (n + 1)
                c['n'] = n + 1
                if c['colour'] == UNKNOWN:
                    c['colour'] = colour
                return
        self.cones.append({'x': x, 'y': y, 'colour': colour, 'n': 1})

    def as_list(self):
        return [(c['x'], c['y'], c['colour']) for c in self.cones]


# ============================================================ local planning

def build_midpoint_path(side_a, side_b, pose, p, expected_width=None):
    """Pair each cone on the shorter side with its NEAREST cone on the other,
    reject implausible widths, skip midpoints behind the car, spline the rest.

    Once the real track width has been measured, pairs far from it are
    rejected: on Track3 a 4 m "corridor" between two separate sections of
    track across the infield was otherwise accepted, and the car cut
    straight across open ground. Returns (path, accepted pairs, widths)."""
    lo_w, hi_w = p['min_track_width'], p['max_track_width']
    if expected_width:
        lo_w = max(lo_w, p['width_tolerance'][0] * expected_width)
        hi_w = min(hi_w, p['width_tolerance'][1] * expected_width)
    short, other = (side_a, side_b) if len(side_a) <= len(side_b) else (side_b, side_a)
    x, y, yaw = pose
    cy, sy = math.cos(yaw), math.sin(yaw)
    pts = [[x], [y]]
    started, pairs, widths = False, 0, []
    for q in short:
        o = min(other, key=lambda r: math.hypot(q[0] - r[0], q[1] - r[1]))
        w = math.hypot(q[0] - o[0], q[1] - o[1])
        if not (lo_w <= w <= hi_w):
            continue
        mx, my = (q[0] + o[0]) / 2.0, (q[1] + o[1]) / 2.0
        if not started:
            if cy * (mx - x) + sy * (my - y) <= 0.2:
                continue
            started = True
        pts[0].append(mx)
        pts[1].append(my)
        pairs += 1
        widths.append(w)
    return resample_open(np.array(pts)), pairs, widths


def build_edge_path(chain, pose, half_width):
    """Single visible boundary (typical on the inside of a tight corner, or
    at the start): follow it at the measured half track width, offset
    toward the side the car is on."""
    x, y, yaw = pose
    c0 = chain[0]
    lat0 = -math.sin(yaw) * (c0[0] - x) + math.cos(yaw) * (c0[1] - y)
    side = 1.0 if lat0 > 0 else -1.0          # +1: boundary is on car's left
    pts = [[x], [y]]
    for i in range(len(chain)):
        a = chain[max(i - 1, 0)]
        b = chain[min(i + 1, len(chain) - 1)]
        tx, ty = b[0] - a[0], b[1] - a[1]
        n = math.hypot(tx, ty)
        if n < 1e-6:
            continue
        lx, ly = -ty / n, tx / n                  # left normal of the chain
        ox, oy = chain[i][0] - side * lx * half_width, chain[i][1] - side * ly * half_width
        if math.cos(yaw) * (ox - x) + math.sin(yaw) * (oy - y) <= 0.2:
            continue                              # behind the car
        pts[0].append(ox)
        pts[1].append(oy)
    return resample_open(np.array(pts))


def build_creep_path(pose, distance):
    """Straight ahead, 6 waypoints (controllers need >= 3-4 to fit a curve).
    Direction-agnostic on purpose: steering off one visible boundary assumes
    a colour convention and curved the car off the track in testing."""
    x, y, yaw = pose
    tx, ty = x + math.cos(yaw) * distance, y + math.sin(yaw) * distance
    return dedupe(np.array([np.linspace(x, tx, 6), np.linspace(y, ty, 6)]))


class StuckDetector:
    def __init__(self, p):
        self.p = p
        self.hist = deque()

    def update(self, now, x, y):
        self.hist.append((now, x, y))
        while self.hist and now - self.hist[0][0] > self.p['stuck_window']:
            self.hist.popleft()

    def is_stuck(self):
        if len(self.hist) < 2:
            return False
        t0, x0, y0 = self.hist[0]
        t1, x1, y1 = self.hist[-1]
        if t1 - t0 < 0.8 * self.p['stuck_window']:
            return False
        return math.hypot(x1 - x0, y1 - y0) < self.p['stuck_min_motion']

    def reset(self):
        self.hist.clear()


# ============================================================ laps & frames

class LapCounter:
    """Counts laps by the car returning to its start pose.

    A lap completes when the car has (a) travelled at least lap_min_length,
    (b) been at least lap_leave_radius away from the start, (c) come back
    within lap_close_radius, and (d) is heading roughly the same way it
    started -- (d) stops a figure-8 crossing counting as a lap.
    """

    def __init__(self, p):
        self.p = p
        self.start = None
        self.laps = 0
        self.travel = 0.0
        self.max_away = 0.0
        self.last = None
        self.lap_start_time = None
        self.lap_times = []

    def set_start(self, pose, now):
        self.start = pose
        self.lap_start_time = now
        self.last = pose

    def update(self, pose, now):
        """pose in the lap frame. Returns True on the step a lap completes."""
        if self.start is None:
            self.set_start(pose, now)
            return False
        self.travel += math.hypot(pose[0] - self.last[0], pose[1] - self.last[1])
        self.last = pose
        d = math.hypot(pose[0] - self.start[0], pose[1] - self.start[1])
        self.max_away = max(self.max_away, d)
        herr = abs(wrap(pose[2] - self.start[2]))
        if (self.travel >= self.p['lap_min_length'] and
                self.max_away >= self.p['lap_leave_radius'] and
                d <= self.p['lap_close_radius'] and
                herr <= math.radians(self.p['lap_max_heading_err_deg'])):
            self.laps += 1
            self.lap_times.append(now - self.lap_start_time)
            self.lap_start_time = now
            self.travel = 0.0
            self.max_away = 0.0
            return True
        return False


class FrameAligner:
    """Estimates the map->local transform: p_local = R(yaw) p_map + t.

    Updated from simultaneous (slam pose, local pose) pairs and low-pass
    filtered with time constant align_tau, so slow drift of the local
    odometry is corrected while the SLAM jumps are smoothed away.
    """

    def __init__(self, p):
        self.p = p
        self.valid = False
        self.yaw = 0.0
        self.tx = 0.0
        self.ty = 0.0
        self.t_last = None

    def update(self, slam_pose, local_pose, now):
        sx, sy, syaw = slam_pose
        lx, ly, lyaw = local_pose
        yaw_raw = wrap(lyaw - syaw)
        if not self.valid:
            self.yaw = yaw_raw
            rx, ry = rot(sx, sy, self.yaw)
            self.tx, self.ty = lx - rx, ly - ry
            self.valid = True
            self.t_last = now
            return
        dt = max(now - self.t_last, 1e-3)
        self.t_last = now
        a = 1.0 - math.exp(-dt / self.p['align_tau'])
        self.yaw = wrap(self.yaw + a * wrap(yaw_raw - self.yaw))
        rx, ry = rot(sx, sy, self.yaw)
        self.tx += a * (lx - rx - self.tx)
        self.ty += a * (ly - ry - self.ty)

    def stale(self, now):
        return (not self.valid) or (now - self.t_last > self.p['slam_timeout'])

    def map_to_local(self, x, y):
        rx, ry = rot(x, y, self.yaw)
        return rx + self.tx, ry + self.ty

    def local_to_map(self, x, y):
        return rot(x - self.tx, y - self.ty, -self.yaw)

    def pose_local_to_map(self, pose):
        mx, my = self.local_to_map(pose[0], pose[1])
        return mx, my, wrap(pose[2] - self.yaw)


# ============================================================ race line

class RaceLine:
    def __init__(self, xy, v, metrics, p):
        self.xy = xy                 # Mx2, closed, even spacing race_ds
        self.v = v
        self.metrics = metrics
        self.p = p
        self.n = len(xy)
        self.hint = None

    def locate(self, x, y):
        """Nearest index; searched locally around the last hint so the
        figure-8 crossing and close parallel sections can't cause jumps."""
        if self.hint is None:
            d = np.hypot(self.xy[:, 0] - x, self.xy[:, 1] - y)
            i = int(np.argmin(d))
        else:
            idx = (self.hint + np.arange(-8, 48)) % self.n
            d = np.hypot(self.xy[idx, 0] - x, self.xy[idx, 1] - y)
            i = int(idx[int(np.argmin(d))])
        self.hint = i
        return i, float(math.hypot(self.xy[i, 0] - x, self.xy[i, 1] - y))

    def window(self, i, length):
        m = int(round(length / self.p['race_ds'])) + 2
        idx = (i - 1 + np.arange(m)) % self.n
        return self.xy[idx].T.copy(), self.v[idx].copy()


def _speed_profile(xy, p):
    kappa = np.abs(closed_curvature(xy))
    ds = np.hypot(np.diff(np.vstack([xy, xy[:1]])[:, 0]),
                  np.diff(np.vstack([xy, xy[:1]])[:, 1]))
    vmax, vmin = p['race_max_velocity'], p['race_min_velocity']
    v = np.minimum(vmax, np.sqrt(p['race_lat_accel'] / np.maximum(kappa, 1e-6)))
    n = len(v)
    for _ in range(3):                       # closed loop: wrap the passes
        for i in range(n):                   # acceleration limit (forward)
            j = (i + 1) % n
            v[j] = min(v[j], math.sqrt(v[i] ** 2 + 2 * p['race_accel'] * ds[i]))
        for i in range(n - 1, -1, -1):       # braking limit (backward)
            j = (i + 1) % n
            v[i] = min(v[i], math.sqrt(v[j] ** 2 + 2 * p['race_brake'] * ds[i]))
    return np.clip(v, vmin, vmax), kappa, ds


def classify_cones(ref, cones_xy, p):
    """Decide ONCE which side of the recorded lap path each cone is on.

    The recorded path is known to have driven between the cones, so its
    left/right split is trustworthy. The split must NOT be recomputed from
    the optimised line: if an intermediate line strays past an apex cone,
    re-classifying would move that cone to the other side and let the line
    cut through it (this happened on Track3's hairpin in testing).
    """
    _, nrm = closed_normals(ref)
    left, right = [], []
    for (cx, cy) in cones_xy:
        d = np.hypot(ref[:, 0] - cx, ref[:, 1] - cy)
        j = int(np.argmin(d))
        lat = (cx - ref[j, 0]) * nrm[j, 0] + (cy - ref[j, 1]) * nrm[j, 1]
        if abs(lat) > p['race_max_cone_lateral'] or abs(lat) < 0.05:
            continue
        (left if lat > 0 else right).append((cx, cy))
    return left, right


def _boundary_segments(cones, max_link):
    """Join each cone to its two nearest same-side neighbours (within
    max_link) -- this reconstructs the edge as a line between cones."""
    c = np.asarray(cones, float)
    segs = set()
    for i in range(len(c)):
        d = np.hypot(c[:, 0] - c[i, 0], c[:, 1] - c[i, 1])
        d[i] = np.inf
        for j in np.argsort(d)[:2]:
            if d[j] <= max_link:
                segs.add((min(i, j), max(i, j)))
    if not segs:
        return np.zeros((0, 2)), np.zeros((0, 2))
    idx = np.array(sorted(segs))
    return c[idx[:, 0]], c[idx[:, 1]]


def _ray_hits(origins, dirs, a, b):
    """Distance along each ray (origin + t*dir, t > 0) to the nearest segment
    a-b. NaN where a ray misses every segment."""
    n = len(origins)
    if len(a) == 0:
        return np.full(n, np.nan)
    e = b - a                                           # S x 2
    ox, oy = origins[:, 0][:, None], origins[:, 1][:, None]
    dx, dy = dirs[:, 0][:, None], dirs[:, 1][:, None]
    ex, ey = e[:, 0][None, :], e[:, 1][None, :]
    ax, ay = a[:, 0][None, :], a[:, 1][None, :]
    den = dx * ey - dy * ex
    with np.errstate(divide='ignore', invalid='ignore'):
        t = ((ax - ox) * ey - (ay - oy) * ex) / den
        u = ((ax - ox) * dy - (ay - oy) * dx) / den
    ok = (np.abs(den) > 1e-12) & (t > 0) & (u >= -1e-9) & (u <= 1 + 1e-9)
    t = np.where(ok, t, np.inf)
    out = t.min(axis=1)
    out[np.isinf(out)] = np.nan
    return out


def _edges(cur, left, right, p, ds):
    """Distance from each point of `cur` to the left and right edges, by
    casting the point's normal ray onto the edge LINE reconstructed from the
    fixed cone classes.

    (Projecting individual cones onto neighbouring normals under-reads the
    outer edge badly on tight hairpins, because those normals fan outward.)
    """
    _, nrm = closed_normals(cur)
    out = []
    for cones, sign in ((left, 1.0), (right, -1.0)):
        a, b = _boundary_segments(cones, p['max_link'])
        w = _ray_hits(cur, sign * nrm, a, b)
        # the line may already have crossed an edge: look backwards too and
        # report that as a negative distance so the bounds push it back
        back = _ray_hits(cur, -sign * nrm, a, b)
        with np.errstate(invalid='ignore'):
            crossed = np.isnan(w) & ~np.isnan(back) & (back < 0.5)
            w[crossed] = -back[crossed]
            w[w > p['race_max_cone_lateral']] = np.nan
        out.append(w)
    return nrm, out[0], out[1]


def _fill_closed(a):
    a = a.copy()
    ok = ~np.isnan(a)
    if ok.sum() < 3:
        return None
    n = len(a)
    idx = np.arange(n)
    xs = np.concatenate([idx[ok] - n, idx[ok], idx[ok] + n])
    ys = np.concatenate([a[ok], a[ok], a[ok]])
    return np.interp(idx, xs, ys)


def build_race_line(ref_xy, cones, p, log=None):
    """Optimised closed racing line from a recorded lap path and a cone map.

    ref_xy : Nx2 recorded path of the mapping lap (map frame)
    cones  : [(x, y, colour)] full cone map (map frame)
    Returns (RaceLine, None) or (None, reason).
    """
    ref_xy = np.asarray(ref_xy, float)
    if len(ref_xy) < 20:
        return None, "recorded lap path too short (%d points)" % len(ref_xy)
    bnd = [(x, y) for x, y, c in cones if c != ORANGE]
    allc = np.array([(x, y) for x, y, _ in cones]) if cones else np.zeros((0, 2))
    if len(bnd) < 12:
        return None, "cone map too small (%d boundary cones)" % len(bnd)

    ds = p['race_build_ds']
    ref = smooth_closed(resample_closed(ref_xy, ds), window=7, passes=2)
    margin = p['race_margin']

    left, right = classify_cones(ref, bnd, p)
    nl, nr = len(left), len(right)
    if nl < 6 or nr < 6:
        return None, "cones on one side too few (left %d, right %d)" % (nl, nr)

    for attempt in range(3):
        cur = ref.copy()
        for it in range(int(p['race_iterations'])):
            nrm, wl, wr = _edges(cur, left, right, p, ds)
            miss = max(np.isnan(wl).mean(), np.isnan(wr).mean())
            if miss > p['race_max_missing_frac']:
                return None, ("track edges missing for %.0f%% of the lap (left cones %d, "
                              "right cones %d)" % (100 * miss, nl, nr))
            wl, wr = _fill_closed(wl), _fill_closed(wr)
            if wl is None or wr is None:
                return None, "not enough cones on one side"
            # conservative: rolling MINIMUM, not a mean -- averaging would
            # over-estimate the room at an apex where one cone sticks in
            wl = np.min(np.stack([np.roll(wl, o) for o in range(-2, 3)]), axis=0)
            wr = np.min(np.stack([np.roll(wr, o) for o in range(-2, 3)]), axis=0)
            width = wl + wr
            med = float(np.median(width))
            if not (p['min_track_width'] <= med <= p['max_track_width']):
                return None, "implausible track width %.2f m" % med
            centre = cur + nrm * ((wl - wr) / 2.0)[:, None]
            half = width / 2.0
            if it == 0:
                first_centre = centre.copy()

            # --- minimum curvature: bounded least squares on lateral offsets
            n = len(centre)
            i = np.arange(n)
            D = sparse.csr_matrix(
                (np.concatenate([np.ones(n), -2.0 * np.ones(n), np.ones(n)]),
                 (np.concatenate([i, i, i]),
                  np.concatenate([(i - 1) % n, i, (i + 1) % n]))), shape=(n, n))
            A = sparse.vstack([D @ sparse.diags(nrm[:, 0]), D @ sparse.diags(nrm[:, 1])]).tocsr()
            b = np.concatenate([D @ centre[:, 0], D @ centre[:, 1]])
            lim = np.maximum(half - margin, 0.0)
            lo, hi = -lim, lim
            # tiny ridge: when two lines are equally smooth, prefer the centre
            H = (A.T @ A + 1e-6 * sparse.identity(n)).tocsc()
            alpha_sol = box_qp(H, A.T @ b, lo, hi)
            alpha = np.clip(alpha_sol, lo, hi)
            race = centre + nrm * alpha[:, None]
            cur = smooth_closed(resample_closed(race, ds), 3, 1)

        race = resample_closed(cur, p['race_ds'])
        clear = float(np.min(np.hypot(race[:, None, 0] - allc[None, :, 0],
                                      race[:, None, 1] - allc[None, :, 1])))
        if clear >= p['race_min_clearance']:
            break
        if log:
            log('warn', "race line clearance %.2f m < %.2f m, retrying with margin %.2f"
                % (clear, p['race_min_clearance'], margin + 0.1))
        margin += 0.1
    else:
        return None, "could not keep race line %.2f m clear of cones" % p['race_min_clearance']

    v, kappa, seg = _speed_profile(race, p)
    cen = resample_closed(smooth_closed(resample_closed(first_centre, ds), 3, 1), p['race_ds'])
    cv, ckappa, cseg = _speed_profile(cen, p)
    metrics = {
        'lap_length_m': float(seg.sum()),
        'centreline_length_m': float(cseg.sum()),
        # what minimum-curvature optimisation minimises: integral of kappa^2
        'curvature_energy': float(np.sum(kappa ** 2 * seg)),
        'centreline_curvature_energy': float(np.sum(ckappa ** 2 * cseg)),
        'peak_curvature': float(np.max(kappa)),
        'centreline_peak_curvature': float(np.max(ckappa)),
        'est_lap_time_s': float(np.sum(seg / v)),
        'centreline_est_lap_time_s': float(np.sum(cseg / cv)),
        'min_clearance_m': clear,
        'margin_used_m': margin,
        'v_min': float(v.min()), 'v_max': float(v.max()),
        'left_cones': nl, 'right_cones': nr,
    }
    return RaceLine(race, v, metrics, p), None


# ============================================================ planner

class GuidancePlanner:
    """Owns all guidance state. The ROS node (or the offline test) feeds it
    sensor data and calls step() at plan_rate_hz."""

    def __init__(self, params=None, log_sink=None):
        self.p = dict(DEFAULTS)
        if params:
            self.p.update(params)
        self.log = ThrottledLog(log_sink or (lambda lvl, msg: None))
        self.memory = ConeMemory(self.p)
        self.own_map = PersistentMap(self.p['cone_merge_dist'])
        self.stuck = StuckDetector(self.p)
        self.laps = LapCounter(self.p)
        self.align = FrameAligner(self.p)
        self.slam_map = []
        self.slam_map_time = None
        self.last_detection = None
        self.mode = 'CREEP'
        self.phase = 'MAPPING'
        self.race = None
        self.race_frame = None       # 'map' or 'local'
        self.race_fallback = False
        self.lap_path_local = []     # recorded path of the current lap
        self.lap_path_map = []
        self.start_local = None
        self.start_map_set = False
        self.race_attempts = 0
        self.race_conflicts = 0
        self.half_width = self.p['default_track_width'] / 2.0
        self.width_measured = False
        self.hold_since = None
        self.last_status = {}

    # ---------------------------------------------------------- inputs
    def on_detections(self, dets, capture_pose_local, now):
        ok = self.memory.update(dets, capture_pose_local, now)
        if ok and len(dets) > 0:
            self.last_detection = now
        else:
            self.log('warn', 'reject', "rejected detection frame: %d cones (limit %d) "
                     "-- perception glitch" % (len(dets), self.p['max_detections_per_frame']),
                     2.0, now)
        return ok

    def on_slam_pose(self, slam_pose, local_pose_same_time, now):
        first = not self.align.valid
        self.align.update(slam_pose, local_pose_same_time, now)
        if first and self.start_local is not None:
            self.log('info', 'align', "map->local alignment initialised")

    def on_map(self, cones, now):
        self.slam_map = list(cones)
        self.slam_map_time = now

    # ---------------------------------------------------------- helpers
    def _lap_pose(self, local_pose):
        if self.align.valid:
            return self.align.pose_local_to_map(local_pose)
        return local_pose

    def _record(self, local_pose):
        if (not self.lap_path_local or
                math.hypot(local_pose[0] - self.lap_path_local[-1][0],
                           local_pose[1] - self.lap_path_local[-1][1]) > 0.05):
            self.lap_path_local.append((local_pose[0], local_pose[1]))
            if self.align.valid:
                self.lap_path_map.append(self.align.local_to_map(local_pose[0], local_pose[1]))

    def _promote_to_own_map(self):
        for c in self.memory.trusted():
            if c['hits'] >= max(self.p['min_hits'], 3):
                self.own_map.add(c['x'], c['y'], c['colour'])

    def _try_build_race(self, now):
        self.race_attempts += 1
        src = self.p['race_map_source']
        use_slam = (src == 'slam' or (src == 'auto' and len(self.slam_map) >= 12 and
                                      self.align.valid and len(self.lap_path_map) >= 20))
        if use_slam:
            ref, cones, frame = self.lap_path_map, self.slam_map, 'map'
        else:
            if not self.p['race_allow_own_map']:
                self.log('warn', 'noslam', "no SLAM map/pose available -- NOT racing (a line "
                         "fixed in drifting odometry slides off the cones); continuing "
                         "mapping-mode laps for safety")
                return False
            ref, cones, frame = self.lap_path_local, self.own_map.as_list(), 'local'
        race, why = build_race_line(ref, cones, self.p,
                                    log=lambda lvl, m: self.log(lvl, 'rb', m))
        if race is None:
            self.log('warn', 'racefail', "race line not built (%s map): %s -- doing "
                     "another mapping lap" % (frame, why))
            return False
        self.race, self.race_frame = race, frame
        m = race.metrics
        self.log('info', 'raceok',
                 "RACE LINE BUILT (%s frame): %.1f m vs centreline %.1f m, curvature "
                 "energy %.2f -> %.2f, est. lap %.1f s vs %.1f s, min clearance %.2f m, "
                 "speed %.2f-%.2f m/s"
                 % (frame, m['lap_length_m'], m['centreline_length_m'],
                    m['centreline_curvature_energy'], m['curvature_energy'],
                    m['est_lap_time_s'], m['centreline_est_lap_time_s'],
                    m['min_clearance_m'], m['v_min'], m['v_max']))
        return True

    def _to_local(self, path_race):
        if self.race_frame == 'map':
            out = np.array([self.align.map_to_local(x, y) for x, y in path_race.T]).T
            return out
        return path_race

    def _live_conflict(self, path, now):
        """Independent sanity check of the race window against the cones the
        car is seeing RIGHT NOW (local frame, so no map or alignment
        involved). Returns the offending distance, or None if clear.

        Without it, a race line built in a drifting frame slides off the
        real cones and the car cannot tell (deviation looks small because
        line and car drift together): 14 cone hits in the no-SLAM test."""
        fresh = [(c['x'], c['y']) for c in self.memory.trusted()
                 if now - c['last_seen'] < 0.6]
        if not fresh:
            return None
        pts = path.T
        d = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(path[0]), np.diff(path[1])))])
        pts = pts[d <= self.p['race_live_check_m']]
        cz = np.array(fresh)
        dmin = float(np.min(np.hypot(pts[:, None, 0] - cz[None, :, 0],
                                     pts[:, None, 1] - cz[None, :, 1])))
        return dmin if dmin < self.p['race_live_clearance'] else None

    def race_line_local(self):
        """Full race line in the local frame (for visualisation)."""
        if self.race is None:
            return None
        return self._to_local(self.race.xy.T)

    # ---------------------------------------------------------- step
    def step(self, now, local_pose):
        """Returns dict(traj=..., status=...). traj is None when the previous
        trajectory should be kept (degenerate plan)."""
        p = self.p
        if self.start_local is None:
            self.start_local = local_pose
        if self.align.valid and not self.start_map_set:
            # express the start pose in the map frame once alignment exists
            self.laps.set_start(self.align.pose_local_to_map(self.start_local), now
                                if self.laps.lap_start_time is None else self.laps.lap_start_time)
            self.lap_path_map = [self.align.local_to_map(x, y) for x, y in self.lap_path_local]
            self.start_map_set = True

        self.memory.expire(now)
        self._promote_to_own_map()
        self.stuck.update(now, local_pose[0], local_pose[1])
        self._record(local_pose)

        lap_done = self.laps.update(self._lap_pose(local_pose), now)
        if lap_done:
            lt = self.laps.lap_times[-1]
            self.log('info', 'lap', "LAP %d complete in %.1f s (%s)"
                     % (self.laps.laps, lt, self.phase))
            if self.phase == 'MAPPING' and p['race_enabled']:
                if self._try_build_race(now):
                    self.phase = 'RACE'
                    self.race_fallback = False
                    self.stuck.reset()
            self.lap_path_local = [(local_pose[0], local_pose[1])]
            self.lap_path_map = ([self.align.local_to_map(local_pose[0], local_pose[1])]
                                 if self.align.valid else [])

        if p['total_laps'] and self.laps.laps >= p['total_laps']:
            self.phase = 'FINISHED'

        traj = self._plan(now, local_pose)
        self.last_status = {
            'phase': self.phase, 'mode': self.mode, 'lap': self.laps.laps + 1,
            'laps_done': self.laps.laps, 'race_frame': self.race_frame,
            'race_fallback': self.race_fallback,
            'mem': len(self.memory.cones), 'rejected': self.memory.rejected_frames,
            'race_conflicts': self.race_conflicts,
            'deviation': self.last_status.get('deviation') if self.phase == 'RACE' else None,
            'cmd_velocity': traj['velocity'] if traj else None,
        }
        return {'traj': traj, 'status': self.last_status}

    def _plan(self, now, pose):
        p = self.p
        if self.phase == 'FINISHED':
            self.mode = 'STOP'
            path = build_creep_path(pose, 0.5)
            t = make_trajectory(path, np.full(path.shape[1], 0.05), p['speed_lookahead_m'])
            t['velocity'] = 0.0
            return t

        stuck = self.stuck.is_stuck()

        if self.phase == 'RACE':
            usable = not (self.race_frame == 'map' and self.align.stale(now))
            if usable:
                car = self.align.pose_local_to_map(pose) if self.race_frame == 'map' else pose
                i, dev = self.race.locate(car[0], car[1])
                self.last_status['deviation'] = dev
                if not self.race_fallback and dev > p['race_max_deviation']:
                    self.race_fallback = True
                    self.log('warn', 'rfb', "off the race line by %.2f m -- falling back "
                             "to local planning" % dev)
                elif self.race_fallback and dev < p['race_rejoin_deviation']:
                    self.race_fallback = False
                    self.stuck.reset()
                    self.log('info', 'rjn', "back on the race line (%.2f m)" % dev)
                if not self.race_fallback and not stuck:
                    wp, v = self.race.window(i, p['race_window_m'])
                    path = self._to_local(wp)
                    conflict = self._live_conflict(path, now)
                    if conflict is None:
                        self.mode = 'RACE'
                        return make_trajectory(path, v, p['speed_lookahead_m'])
                    self.race_conflicts += 1
                    self.log('warn', 'conflict', "race line passes %.2f m from a cone the car "
                             "can see -- line misaligned, planning locally" % conflict, 1.0, now)
            else:
                self.log('warn', 'slamstale', "SLAM pose stale -- local planning until it "
                         "returns", 2.0, now)
            # otherwise drop through to local planning / recovery

        since = 1e9 if self.last_detection is None else now - self.last_detection
        stale = since > p['detection_timeout']
        if stale and self.last_detection is not None and since > p['lost_stop_timeout']:
            if self.mode != 'LOST':
                self.log('warn', 'lost', "no cones seen for %.0f s -- track lost, STOPPING" % since)
            self.mode = 'LOST'
            path = build_creep_path(pose, 0.3)
            t = make_trajectory(path, np.full(path.shape[1], 0.05), p['speed_lookahead_m'])
            t['velocity'] = 0.0
            return t
        if stale or stuck:
            if self.mode != 'RECOVERY':
                self.log('warn', 'rec', "-> RECOVERY (%s)"
                         % ("detections stale" if stale else "car not moving"))
            self.mode = 'RECOVERY'
            path = build_creep_path(pose, 0.3)
            return make_trajectory(path, np.full(path.shape[1], p['recovery_velocity']),
                                   p['speed_lookahead_m'])

        a, b = self.memory.sides_by_colour(pose)
        source = 'colour'
        if p['use_geometric_fallback']:
            ga, gb = self.memory.sides_by_geometry(pose)
            if min(len(ga), len(gb)) > min(len(a), len(b)):
                a, b, source = ga, gb, 'geometry'

        if len(a) >= p['min_cones_per_side'] and len(b) >= p['min_cones_per_side']:
            path, pairs, widths = build_midpoint_path(
                a, b, pose, p, 2 * self.half_width if self.width_measured else None)
            length = float(np.sum(np.hypot(np.diff(path[0]), np.diff(path[1])))) \
                if path.shape[1] > 1 else 0.0
            if pairs >= p['min_cones_per_side'] and path.shape[1] >= 4 and length >= 1.0:
                w = float(np.median(widths))
                self.half_width = w / 2.0 if not self.width_measured else \
                    0.8 * self.half_width + 0.2 * (w / 2.0)
                self.width_measured = True
                if self.mode != 'TRACK':
                    self.stuck.reset()
                self.mode = 'TRACK'
                self.hold_since = None
                self.last_status['source'] = source
                curv = open_curvature(path)
                speeds = np.clip(p['max_velocity'] / (1.0 + p['curvature_slowdown'] * curv),
                                 p['min_velocity'], p['max_velocity'])
                return make_trajectory(path, speeds, p['speed_lookahead_m'])

        # one boundary visible: follow it at the measured half-width
        if p['use_edge_following'] and self.width_measured:
            longer = a if len(a) >= len(b) else b
            if len(longer) >= 3:
                path = build_edge_path(longer, pose, self.half_width)
                length = float(np.sum(np.hypot(np.diff(path[0]), np.diff(path[1])))) \
                    if path.shape[1] > 1 else 0.0
                if path.shape[1] >= 4 and length >= 0.8:
                    if self.mode != 'EDGE':
                        self.stuck.reset()
                    self.mode = 'EDGE'
                    self.hold_since = None
                    return make_trajectory(path, np.full(path.shape[1], p['min_velocity']),
                                           p['speed_lookahead_m'])

        if self.mode in ('TRACK', 'EDGE'):
            if self.hold_since is None:
                self.hold_since = now
            if now - self.hold_since < p['hold_timeout']:
                return None             # momentary gap -- keep previous trajectory
        self.hold_since = None
        if self.mode != 'CREEP':
            self.stuck.reset()
        self.mode = 'CREEP'
        path = build_creep_path(pose, p['creep_distance'])
        return make_trajectory(path, np.full(path.shape[1], p['creep_velocity']),
                               p['speed_lookahead_m'])
