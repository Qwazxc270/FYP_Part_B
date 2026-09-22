#! /usr/bin/env python3
"""
track_sim -- lightweight QCar track simulator (no ROS, no Gazebo).

Used two ways:
  * extras/offline_test.py        -- runs full mapping + race laps in seconds
  * scripts/fake_track_sim_node.py -- ROS wrapper for the isolated launch test

It is deliberately simple. It is NOT a replacement for Gazebo; it exists so
the guidance logic (lap detection, race-line optimisation, frame handling,
input validation) can be exercised end-to-end quickly and repeatably.

What it models
--------------
  * cone positions parsed straight from the qcar_gazebo .world files
  * kinematic bicycle car (QCar wheelbase, steering limit, accel limit)
  * pure-pursuit path follower standing in for the controller
  * camera: coloured detections inside a horizontal FOV and range
  * lidar:  colourless detections all round, longer range
  * perception glitches: occasional frames of phantom cones
  * two odometry sources, like the real stack:
        local  -- smooth but slowly drifting (wheels + IMU)
        slam   -- drift-free but jumps a few cm on landmark corrections
  * a SLAM-style cone map accumulated from what the car has actually seen,
    with colours only for cones the camera has actually photographed
"""

import math
import random
import re

import numpy as np

UNKNOWN, BLUE, YELLOW, ORANGE = 0, 1, 2, 3
_COLOUR_BY_NAME = {'blue': BLUE, 'yellow': YELLOW, 'orange': ORANGE}

# spawn poses taken from qcar_gazebo/launch/<Track>.launch
SPAWN = {
    'Track1':      (-4.30,  3.00, -3.14),
    'Track2':      (-4.35,  3.50, -3.14),
    'Track3':      ( 4.60, 11.50, -3.14),
    'CircleTrack': ( 6.80, -0.20,  1.57),
    'Figure8':     (-2.50,  3.00, -1.5707963),
}


def load_world_cones(path):
    """Return [(x, y, colour), ...] for every cone in a qcar_gazebo world."""
    cones, seen, cur = [], set(), None
    with open(path) as fh:
        for line in fh:
            m = re.search(r"<model name='(blue|yellow|orange)_cone_(\d+)'", line)
            if m:
                cur = (m.group(1), m.group(2))
                continue
            if cur is None or cur in seen:
                continue
            p = re.search(r'<pose[^>]*>\s*([-\d.eE]+)\s+([-\d.eE]+)\s', line)
            if not p:
                continue
            x, y = float(p.group(1)), float(p.group(2))
            if abs(x) < 1e-9 and abs(y) < 1e-9:
                continue            # link-relative pose, not the model pose
            seen.add(cur)
            cones.append((x, y, _COLOUR_BY_NAME[cur[0]]))
            cur = None
    return cones


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def world_to_car(px, py, pose):
    x, y, yaw = pose
    dx, dy = px - x, py - y
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


class SimCar:
    """Kinematic bicycle with a pure-pursuit tracker on the latest path."""

    L = 0.256                       # QCar wheelbase (m), as in runControl.cpp
    MAX_STEER = math.radians(35.0)
    MAX_ACCEL = 1.5                 # m/s^2

    def __init__(self, pose):
        self.x, self.y, self.yaw = pose
        self.v = 0.0
        self.path = None            # 2xN in the TRUE frame
        self.v_target = 0.0
        self.steer = 0.0

    @property
    def pose(self):
        return (self.x, self.y, self.yaw)

    def set_command(self, path_true, v_target):
        self.path = path_true
        self.v_target = max(0.0, v_target)

    def _pure_pursuit(self):
        if self.path is None or self.path.shape[1] < 2:
            return 0.0
        look = 0.35 + 0.6 * self.v
        pts = self.path.T
        d = np.hypot(pts[:, 0] - self.x, pts[:, 1] - self.y)
        i0 = int(np.argmin(d))
        target = pts[-1]
        for j in range(i0, len(pts)):
            if d[j] >= look:
                target = pts[j]
                break
        lx, ly = world_to_car(target[0], target[1], self.pose)
        ld2 = max(lx * lx + ly * ly, 1e-6)
        return math.atan2(2.0 * self.L * ly, ld2)

    def step(self, dt):
        self.steer = max(-self.MAX_STEER, min(self.MAX_STEER, self._pure_pursuit()))
        dv = self.v_target - self.v
        self.v += max(-self.MAX_ACCEL * dt, min(self.MAX_ACCEL * dt, dv))
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw = wrap(self.yaw + self.v / self.L * math.tan(self.steer) * dt)


class OdometryModel:
    """Produces the two odometry streams of the real stack from the truth.

    local: integrates true motion with a heading-rate bias and a distance
           scale error, so it is smooth but drifts over a lap.
    slam:  truth plus small noise plus an occasional sideways jump, like the
           EKF-SLAM landmark corrections (5-18 cm every ~1.6 s).
    """

    def __init__(self, pose, drift_yaw_rate=math.radians(0.6), scale_err=0.01,
                 jump=0.10, jump_period=1.6, seed=0):
        self.rng = random.Random(seed)
        self.local = list(pose)
        self.prev_true = list(pose)
        self.drift_yaw_rate = drift_yaw_rate
        self.scale_err = scale_err
        self.jump = jump
        self.jump_period = jump_period
        self.jump_offset = [0.0, 0.0]
        self.t_next_jump = jump_period

    def update(self, true_pose, dt, t):
        tx, ty, tyaw = true_pose
        px, py, pyaw = self.prev_true
        ds = math.hypot(tx - px, ty - py) * (1.0 + self.scale_err)
        dyaw = wrap(tyaw - pyaw) + self.drift_yaw_rate * dt * (1 if ds > 1e-4 else 0)
        self.local[2] = wrap(self.local[2] + dyaw)
        self.local[0] += ds * math.cos(self.local[2])
        self.local[1] += ds * math.sin(self.local[2])
        self.prev_true = [tx, ty, tyaw]

        if t >= self.t_next_jump:
            ang = self.rng.uniform(-math.pi, math.pi)
            mag = self.rng.uniform(0.5, 1.0) * self.jump
            self.jump_offset = [mag * math.cos(ang), mag * math.sin(ang)]
            self.t_next_jump = t + self.jump_period
        else:
            # corrections decay back toward the truth between jumps
            self.jump_offset = [0.9 * self.jump_offset[0], 0.9 * self.jump_offset[1]]
        slam = (tx + self.jump_offset[0] + self.rng.gauss(0, 0.005),
                ty + self.jump_offset[1] + self.rng.gauss(0, 0.005),
                wrap(tyaw + self.rng.gauss(0, 0.003)))
        return tuple(self.local), slam


class Sensors:
    """Camera (coloured, limited FOV) and lidar (colourless) cone detections
    in the CAR frame, plus a SLAM-style accumulated map in the TRUE frame."""

    def __init__(self, cones, cam_fov=1.6, cam_range=3.0, lidar_range=4.0,
                 noise=0.02, phantom_period=9.0, phantom_count=120, seed=1):
        self.cones = cones
        self.cam_half = cam_fov / 2.0
        self.cam_range = cam_range
        self.lidar_range = lidar_range
        self.noise = noise
        self.rng = random.Random(seed)
        self.phantom_period = phantom_period
        self.phantom_count = phantom_count
        self.t_next_phantom = phantom_period
        self.map_seen = {}          # cone index -> colour votes seen

    def detect(self, true_pose, t):
        cam, lidar = [], []
        for i, (cx, cy, col) in enumerate(self.cones):
            ax, ly = world_to_car(cx, cy, true_pose)
            r = math.hypot(ax, ly)
            if r < 0.25:
                continue
            nx = ax + self.rng.gauss(0, self.noise)
            ny = ly + self.rng.gauss(0, self.noise)
            if r <= self.lidar_range:
                lidar.append((nx, ny, UNKNOWN))
                self.map_seen.setdefault(i, UNKNOWN)
            if ax > 0 and r <= self.cam_range and abs(math.atan2(ly, ax)) <= self.cam_half:
                cam.append((nx, ny, col))
                self.map_seen[i] = col
        phantom = False
        if self.phantom_period and t >= self.t_next_phantom:
            self.t_next_phantom = t + self.phantom_period
            phantom = True
            lidar = lidar + [(self.rng.uniform(-3, 3), self.rng.uniform(-3, 3), UNKNOWN)
                             for _ in range(self.phantom_count)]
        return cam, lidar, phantom

    def slam_map(self):
        """Cones seen so far, true frame, small noise; colour only if the
        camera ever saw the cone (mirrors EKF-SLAM colour voting)."""
        out = []
        for i, col in self.map_seen.items():
            cx, cy, _ = self.cones[i]
            out.append((cx + self.rng.gauss(0, 0.02), cy + self.rng.gauss(0, 0.02), col))
        return out


def min_cone_distance(x, y, cones):
    return min(math.hypot(cx - x, cy - y) for cx, cy, _ in cones)
