#!/usr/bin/env python3
"""
EKF-SLAM node: joint estimation of vehicle pose and cone landmark map.

Replaces both nav_filter_node (pose-only EKF) and cone_mapper_node
(separate per-landmark Kalman filters) with a single filter whose state
vector is:

    x = [X, Y, psi, v, m1_x, m1_y, m2_x, m2_y, ...]^T

This satisfies FYP Outcome O1: "a joint probability distribution of the
car pose and map of the environment".

Subscriptions (same as nav_filter_node + cone detections):
    /imu                           - IMU yaw-rate
    /odom                          - Gazebo ground-truth (yaw init + yaw update)
    /wheelrl_motor/velocity        - rear-left wheel speed
    /wheelrr_motor/velocity        - rear-right wheel speed
    /qcar/base_fl_controller/cmd   - front-left steering
    /qcar/base_fr_controller/cmd   - front-right steering
    /cone_detections_fused         - fused LiDAR+depth cone detections

Publications:
    /odometry/filtered             - filtered vehicle odometry (for guidance/control)
    /cone_map                      - confirmed landmark positions (PoseArray)
    /cone_map_markers              - RViz markers (spheres + covariance ellipses)
"""

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64, Float32
import tf.transformations as tft
from qcar_navigation.msg import ConeDetection, ConeDetectionArray


# ======================================================================
# Helpers
# ======================================================================

def wrap_to_pi(angle):
    """Wrap an angle to [-pi, pi]."""
    wrapped = np.mod(angle + np.pi, 2 * np.pi)
    if wrapped == 0 and (angle + np.pi) > 0:
        wrapped = 2 * np.pi
    wrapped = wrapped - np.pi
    if wrapped == -np.pi and angle > 0:
        wrapped = np.pi
    return wrapped


# ======================================================================
# EKF-SLAM core
# ======================================================================

class EkfSlam:
    """
    Joint EKF over vehicle state + landmark positions.

    State layout (column vector):
        [0]  X       vehicle x
        [1]  Y       vehicle y
        [2]  psi     vehicle heading
        [3]  v       vehicle speed
        [4]  m0_x    landmark 0 x
        [5]  m0_y    landmark 0 y
        ...
    """

    NV = 4  # vehicle-state dimension

    def __init__(self, x0):
        self.x = np.array(x0, dtype=float).reshape(self.NV, 1)

        sigma_x = 0.2
        sigma_y = 0.2
        sigma_psi = np.deg2rad(5)
        sigma_v = 0.2
        self.P = np.diag([sigma_x ** 2, sigma_y ** 2,
                          sigma_psi ** 2, sigma_v ** 2])

        # Process noise (vehicle only; landmarks are static)
        self.Q_vehicle = np.diag([0.03 ** 2, 0.03 ** 2,
                                  np.deg2rad(1) ** 2, 0.1 ** 2])

        # IMU / wheel-speed measurement noise
        self.R_imu = np.diag([np.deg2rad(0.8) ** 2, 0.03 ** 2])

        # Cached sensor readings
        self.yaw_rate_meas = 0.0
        self.v_meas = 0.0

        self.n_landmarks = 0

    # ------------------------------------------------------------------
    @property
    def n(self):
        """Total state dimension."""
        return self.NV + 2 * self.n_landmarks

    # ------------------------------------------------------------------
    # Motion model
    # ------------------------------------------------------------------
    def predict(self, u, dt, L):
        x = self.x
        P = self.P
        n = self.n

        X = x[0, 0]
        Y = x[1, 0]
        psi = x[2, 0]
        v = x[3, 0]
        delta = u[0, 0]
        a = u[1, 0]

        # Propagate vehicle state
        x[0, 0] = X + v * np.cos(psi) * dt
        x[1, 0] = Y + v * np.sin(psi) * dt
        x[2, 0] = wrap_to_pi(psi + (v / L) * np.tan(delta) * dt)
        x[3, 0] = v + a * dt

        # Full-state Jacobian (landmarks are identity)
        F = np.eye(n)
        F[0, 2] = -v * np.sin(psi) * dt
        F[0, 3] = np.cos(psi) * dt
        F[1, 2] = v * np.cos(psi) * dt
        F[1, 3] = np.sin(psi) * dt
        F[2, 3] = (1.0 / L) * np.tan(delta) * dt

        # Process noise (only vehicle block)
        Q = np.zeros((n, n))
        Q[:self.NV, :self.NV] = self.Q_vehicle

        self.x = x
        self.P = F @ P @ F.T + Q

        for i in range(self.n_landmarks):
            j = self.NV + 2*i
            self.P[j, j] = min(self.P[j, j], 1.0)
            self.P[j+1, j+1] = min(self.P[j+1, j+1], 1.0)
            self.P[:self.NV, j:j+2] = 0.0
            self.P[j:j+2, :self.NV] = 0.0

    # ------------------------------------------------------------------
    # IMU + wheel-speed update
    # ------------------------------------------------------------------
    def update_imu(self, delta, L):
        x = self.x
        P = self.P
        n = self.n

        v = x[3, 0]

        z = np.array([[self.yaw_rate_meas],
                       [self.v_meas]])
        z_hat = np.array([[(v / L) * np.tan(delta)],
                           [v]])

        C = np.zeros((2, n))
        C[0, 3] = np.tan(delta) / L
        C[1, 3] = 1.0

        e = z - z_hat
        R = self.R_imu
        S = C @ P @ C.T + R
        K = P @ C.T @ np.linalg.inv(S)

        x = x + K @ e
        x[2, 0] = wrap_to_pi(x[2, 0])

        I = np.eye(n)
        self.x = x
        self.P = (I - K @ C) @ P @ (I - K @ C).T + K @ R @ K.T

    # ------------------------------------------------------------------
    # Absolute yaw update (from Gazebo ground-truth odometry)
    # ------------------------------------------------------------------
    def update_yaw(self, yaw_meas, R_yaw):
        x = self.x
        P = self.P
        n = self.n

        H = np.zeros((1, n))
        H[0, 2] = 1.0

        innov = np.array([[wrap_to_pi(yaw_meas - x[2, 0])]])
        R = np.array([[R_yaw]])
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)

        x = x + K @ innov
        x[2, 0] = wrap_to_pi(x[2, 0])

        I = np.eye(n)
        self.x = x
        self.P = (I - K @ H) @ P @ (I - K @ H).T + K @ R @ K.T

    # ------------------------------------------------------------------
    # Cone-observation helpers
    # ------------------------------------------------------------------
    def _cone_R(self, rng):
        """Range-dependent measurement covariance (range, bearing)."""
        sigma_r = 0.15 + 0.03 * rng
        sigma_b = np.deg2rad(5.0)
        return np.diag([sigma_r ** 2, sigma_b ** 2])

    def _landmark_jacobian_and_innovation(self, lm_idx, z_rb):
        """
        Compute H, innovation, and S for landmark *lm_idx* given
        observation z_rb = [[range], [bearing]].
        Returns (H, innovation, S, r_exp) or None if degenerate.
        """
        x = self.x
        n = self.n

        vx, vy, vpsi = x[0, 0], x[1, 0], x[2, 0]
        j = self.NV + 2 * lm_idx
        mx, my = x[j, 0], x[j + 1, 0]

        dx = mx - vx
        dy = my - vy
        q = dx ** 2 + dy ** 2
        r_exp = np.sqrt(q)
        if r_exp < 1e-6:
            return None

        z_hat = np.array([
            [r_exp],
            [wrap_to_pi(np.arctan2(dy, dx) - vpsi)]
        ])

        H = np.zeros((2, n))
        # Vehicle columns
        H[0, 0] = -dx / r_exp
        H[0, 1] = -dy / r_exp
        H[1, 0] = dy / q
        H[1, 1] = -dx / q
        H[1, 2] = -1.0
        # Landmark columns
        H[0, j] = dx / r_exp
        H[0, j + 1] = dy / r_exp
        H[1, j] = -dy / q
        H[1, j + 1] = dx / q

        R = self._cone_R(z_rb[0, 0])
        innov = z_rb - z_hat
        innov[1, 0] = wrap_to_pi(innov[1, 0])
        S = H @ self.P @ H.T + R

        return H, innov, S, r_exp

    # ------------------------------------------------------------------
    # Landmark update
    # ------------------------------------------------------------------
    def update_landmark(self, lm_idx, z_rb):
        """EKF update step for re-observing an existing landmark."""
        result = self._landmark_jacobian_and_innovation(lm_idx, z_rb)
        if result is None:
            return
        H, innov, S, _ = result

        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innov
        self.x[2, 0] = wrap_to_pi(self.x[2, 0])

        I = np.eye(self.n)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ self._cone_R(z_rb[0, 0]) @ K.T

    # ------------------------------------------------------------------
    # Data association (Mahalanobis)
    # ------------------------------------------------------------------
    def mahalanobis_distance(self, lm_idx, z_rb):
        result = self._landmark_jacobian_and_innovation(lm_idx, z_rb)
        if result is None:
            return 1e9
        _, innov, S, _ = result
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return 1e9
        return float(innov.T @ S_inv @ innov)

    # ------------------------------------------------------------------
    # Add new landmark (state augmentation)
    # ------------------------------------------------------------------
    def add_landmark(self, z_body):
        """
        Augment the SLAM state with a new confirmed landmark.
        z_body = [[bx], [by]] in base_footprint frame.
        Returns the new landmark index.
        """
        vx = self.x[0, 0]
        vy = self.x[1, 0]
        vpsi = self.x[2, 0]
        bx = z_body[0, 0]
        by = z_body[1, 0]

        c = np.cos(vpsi)
        s = np.sin(vpsi)

        # World-frame landmark position
        mx = vx + c * bx - s * by
        my = vy + s * bx + c * by

        n_old = self.n

        # Augment state vector
        new_x = np.zeros((n_old + 2, 1))
        new_x[:n_old, :] = self.x
        new_x[n_old, 0] = mx
        new_x[n_old + 1, 0] = my

        # Jacobian of g(x_v, z) w.r.t. vehicle state
        Jv = np.zeros((2, self.NV))
        Jv[0, 0] = 1.0
        Jv[0, 2] = -s * bx - c * by
        Jv[1, 1] = 1.0
        Jv[1, 2] = c * bx - s * by

        # Jacobian of g w.r.t. body-frame measurement
        Jz = np.array([[c, -s],
                        [s,  c]])

        # Body-frame measurement noise (via range-bearing)
        rng = np.sqrt(bx ** 2 + by ** 2)
        bearing = np.arctan2(by, bx)
        sigma_r = 0.25 + 0.05 * rng
        sigma_b = np.deg2rad(10.0)
        R_rb = np.diag([sigma_r ** 2, sigma_b ** 2])
        J_rb = np.array([
            [np.cos(bearing), -rng * np.sin(bearing)],
            [np.sin(bearing),  rng * np.cos(bearing)]
        ])
        R_body = J_rb @ R_rb @ J_rb.T

        # Full Jacobian row: G_full = [Jv | 0 ... 0] (2 x n_old)
        G_full = np.zeros((2, n_old))
        G_full[:, :self.NV] = Jv

        # Augment covariance
        new_P = np.zeros((n_old + 2, n_old + 2))
        new_P[:n_old, :n_old] = self.P

        cross = G_full @ self.P               # (2 x n_old)
        new_P[n_old:n_old + 2, :n_old] = cross
        new_P[:n_old, n_old:n_old + 2] = cross.T

        new_P[n_old:n_old + 2,
              n_old:n_old + 2] = (G_full @ self.P @ G_full.T
                                  + Jz @ R_body @ Jz.T)

        self.x = new_x
        self.P = new_P
        self.n_landmarks += 1
        return self.n_landmarks - 1

    def add_landmark_world(self, mu_world, P_world):
        """
        Augment the SLAM state with a landmark at a known world position.
        Used when promoting candidates (whose world-frame position has
        already been Kalman-filtered).
        """
        n_old = self.n

        new_x = np.zeros((n_old + 2, 1))
        new_x[:n_old, :] = self.x
        new_x[n_old, 0] = mu_world[0, 0]
        new_x[n_old + 1, 0] = mu_world[1, 0]

        new_P = np.zeros((n_old + 2, n_old + 2))
        new_P[:n_old, :n_old] = self.P
        # Cross-covariance starts at zero; builds up through observations
        new_P[n_old:n_old + 2, n_old:n_old + 2] = P_world

        self.x = new_x
        self.P = new_P
        self.n_landmarks += 1
        return self.n_landmarks - 1

    def remove_landmark(self, lm_idx):
        j = self.NV + 2*lm_idx
        idx = list(range(self.n))
        del idx[j + 1]
        del idx[j]
        self.x = self.x[idx, :]
        self.P = self.P[np.ix_(idx, idx)]
        self.n_landmarks -= 1

    # ------------------------------------------------------------------
    # Accessors
    # ------------------------------------------------------------------
    def landmark_pos(self, lm_idx):
        j = self.NV + 2 * lm_idx
        return self.x[j, 0], self.x[j + 1, 0]

    def landmark_cov(self, lm_idx):
        j = self.NV + 2 * lm_idx
        return self.P[j:j + 2, j:j + 2].copy()

    def vehicle_pose_cov_6x6(self):
        """Map the top-left 4x4 block into a 6x6 ROS-style pose covariance."""
        P = self.P
        cov6 = np.zeros((6, 6), dtype=float)
        cov6[0, 0] = P[0, 0]
        cov6[0, 1] = P[0, 1]
        cov6[1, 0] = P[1, 0]
        cov6[1, 1] = P[1, 1]
        cov6[0, 5] = P[0, 2]
        cov6[5, 0] = P[2, 0]
        cov6[1, 5] = P[1, 2]
        cov6[5, 1] = P[2, 1]
        cov6[5, 5] = P[2, 2]
        cov6[2, 2] = 1e-6
        cov6[3, 3] = 1e-6
        cov6[4, 4] = 1e-6
        return cov6


# ======================================================================
# ROS node
# ======================================================================

class EkfSlamNode:
    def __init__(self):
        rospy.init_node("ekf_slam_node")

        # ---- vehicle parameters ----
        self.wheel_radius = 0.0325
        self.L = 0.26
        self.R_yaw_abs = np.deg2rad(2.0) ** 2

        # ---- data-association parameters ----
        self.mahal_gate = rospy.get_param("~mahal_gate", 5.99)  # chi2(2) 95 %
        self.euclidean_gate = rospy.get_param("~euclidean_gate", 1.0)
        self.min_range = rospy.get_param("~min_range", 0.1)
        self.max_range = rospy.get_param("~max_range", 4.5)

        # ---- candidate confirmation ----
        self.confirmation_hits = rospy.get_param("~confirmation_hits", 3)
        self.candidate_max_age = rospy.get_param("~candidate_max_age", 12)
        self.candidate_assoc_dist = rospy.get_param("~candidate_assoc_dist", 0.45)
        self.min_landmark_spacing = rospy.get_param("~min_landmark_spacing", 0.75)
        self.min_motion_for_rehit = rospy.get_param("~min_motion_for_rehit", 0.20)
        self.use_gazebo_yaw = rospy.get_param("~use_gazebo_yaw", True)

        # ---- topics ----
        detections_topic = rospy.get_param("~detections_topic",
                                           "/cone_detections_fused")
        odom_topic = rospy.get_param("~odom_topic", "/odometry/filtered")
        map_topic = rospy.get_param("~map_topic", "/cone_map")
        marker_topic = rospy.get_param("~marker_topic", "/cone_map_markers")
        self.world_frame = rospy.get_param("~world_frame", "odom")

        # ---- internal state ----
        self.latest_imu = None
        self.imu_yaw = 0
        self.imu_yaw_ready = False
        self.latest_odom = None
        self.w_rl = 0.0
        self.w_rr = 0.0
        self.delta_fl = 0.0
        self.delta_fr = 0.0
        self.pre_v_meas = None
        self.last_time = None
        self.initialized = False
        self.start_time = None
        self.startup_delay = 5.0

        self.slam = None  # type: EkfSlam

        # Candidates (not yet in the SLAM state)
        self.candidates = []

        # Track which landmarks have been updated and from which pose
        self.landmark_last_update_pose = []

        self.landmark_colour_votes = []
        self.landmark_last_obs_time = []
        self.landmark_hits = []
        self.landmark_max_age = 30.0

        # ---- subscribers ----
        rospy.Subscriber("/imu", Imu, self._imu_cb)
        rospy.Subscriber("/odom", Odometry, self._odom_cb)
        rospy.Subscriber("/wheelrl_motor/velocity", Float32, self._wrl_cb)
        rospy.Subscriber("/wheelrr_motor/velocity", Float32, self._wrr_cb)
        rospy.Subscriber("/qcar/base_fl_controller/command", Float64,
                         self._sfl_cb)
        rospy.Subscriber("/qcar/base_fr_controller/command", Float64,
                         self._sfr_cb)
        rospy.Subscriber(detections_topic, PoseArray,
                         self._detections_cb, queue_size=1)

        rospy.Subscriber("/cone_detections_camera", ConeDetectionArray, self._colour_cb, queue_size=1)

        # ---- publishers ----
        self.odom_pub = rospy.Publisher(odom_topic, Odometry, queue_size=10)
        self.map_pub = rospy.Publisher(map_topic, PoseArray, queue_size=1)
        self.marker_pub = rospy.Publisher(marker_topic, MarkerArray,
                                          queue_size=1)

        rospy.loginfo("ekf_slam_node started")

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------
    def _imu_cb(self, msg):
        self.latest_imu = msg
        # Extract yaw from IMU orientation quaternion if available
        q = msg.orientation
        if q.w != 0.0 or q.x != 0.0 or q.y != 0.0 or q.z != 0.0:
            _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.imu_yaw = yaw
            self.imu_yaw_ready = True

    def _odom_cb(self, msg):
        self.latest_odom = msg

    def _wrl_cb(self, msg):
        self.w_rl = msg.data

    def _wrr_cb(self, msg):
        self.w_rr = msg.data

    def _sfl_cb(self, msg):
        self.delta_fl = msg.data

    def _sfr_cb(self, msg):
        self.delta_fr = msg.data

    def _colour_cb(self, msg):
        if self.slam is None:
            return
        vx = self.slam.x[0, 0]
        vy = self.slam.x[1, 0]
        vpsi = self.slam.x[2, 0]
        c = np.cos(vpsi)
        s = np.sin(vpsi)

        for det in msg.detections:
            if det.colour == ConeDetection.UNKNOWN:
                continue

            # Transform detection from base_footprint to world frame
            bx = det.position.x
            by = det.position.y
            wx = vx + c * bx - s * by
            wy = vy + s * bx + c * by

            z = np.array([wx, wy])
            best_idx, best_dist = -1, float('inf')
            for i in range(self.slam.n_landmarks):
                mx, my = self.slam.landmark_pos(i)
                d = np.linalg.norm(z - np.array([mx, my]))
                if d < best_dist:
                    best_dist, best_idx = d, i

            if best_idx >= 0 and best_dist < 0.8:
                while len(self.landmark_colour_votes) <= best_idx:
                    self.landmark_colour_votes.append({})
                votes = self.landmark_colour_votes[best_idx]
                votes[det.colour] = votes.get(det.colour, 0) + 1

    def _detections_cb(self, msg):
        if not self.initialized or self.slam is None:
            return
        if self.start_time is None:
            self.start_time = rospy.Time.now()
        if (rospy.Time.now() - self.start_time).to_sec() < self.startup_delay:
            return

        current_pose = np.array([self.slam.x[0, 0],
                                 self.slam.x[1, 0]], dtype=float)
        vpsi = self.slam.x[2, 0]

        for pose in msg.poses:
            bx = pose.position.x
            by = pose.position.y
            rng = np.sqrt(bx ** 2 + by ** 2)

            if rng < self.min_range or rng > self.max_range:
                continue

            z_rb = np.array([[rng],
                             [np.arctan2(by, bx)]])
            z_body = np.array([[bx], [by]])

            self._process_observation(z_rb, z_body, current_pose)

        self._age_candidates()

        stamp = (msg.header.stamp
                 if msg.header.stamp != rospy.Time()
                 else rospy.Time.now())
        self._prune_zombies()
        self._publish_map(stamp)

    def _prune_zombies(self):
        now = rospy.Time.now()
        i = 0
        while i < self.slam.n_landmarks:
            if i < len(self.landmark_last_obs_time):
                age = (now - self.landmark_last_obs_time[i]).to_sec()
                hits = self.landmark_hits[i] if i < len(self.landmark_hits) else 0
                if age > self.landmark_max_age and hits < 5:
                    rospy.loginfo("SLAM: pruning zombie landmark %d (hits=%d)", i, hits)
                    self.slam.remove_landmark(i)
                    del self.landmark_last_update_pose[i]
                    del self.landmark_last_obs_time[i]
                    del self.landmark_hits[i]
                    if i < len(self.landmark_colour_votes):
                        del self.landmark_colour_votes[i]
                    continue
            i += 1

    # ------------------------------------------------------------------
    def _process_observation(self, z_rb, z_body, current_pose):
        """
        Try to associate z_rb with an existing SLAM landmark.
        If matched  -> EKF update (corrects BOTH pose AND map).
        If no match -> candidate pipeline -> promote after enough hits.
        """
        # --- try confirmed landmarks ---
        best_idx = -1
        best_md = 1e9
        for i in range(self.slam.n_landmarks):
            md = self.slam.mahalanobis_distance(i, z_rb)
            if md < best_md:
                best_md = md
                best_idx = i

        if best_idx >= 0 and best_md <= self.mahal_gate:
            # Also check Euclidean as a sanity bound
            mx, my = self.slam.landmark_pos(best_idx)
            c = np.cos(self.slam.x[2, 0])
            s = np.sin(self.slam.x[2, 0])
            wx = self.slam.x[0, 0] + c * z_body[0, 0] - s * z_body[1, 0]
            wy = self.slam.x[1, 0] + s * z_body[0, 0] + c * z_body[1, 0]
            euc = np.sqrt((wx - mx) ** 2 + (wy - my) ** 2)
            if euc <= self.euclidean_gate:
                if self._pose_moved_enough_lm(best_idx, current_pose):
                    self.slam.update_landmark(best_idx, z_rb)
                    if best_idx < len(self.landmark_hits):
                        self.landmark_hits[best_idx] += 1
                    self.landmark_last_update_pose[best_idx] = current_pose.copy()
                    if best_idx < len(self.landmark_last_obs_time):
                        self.landmark_last_obs_time[best_idx] = rospy.Time.now()                    
                return

        # --- check if too close to an existing confirmed landmark ---
        if self._too_close_to_confirmed(z_body):
            return

        # --- try candidates ---
        c = np.cos(self.slam.x[2, 0])
        s = np.sin(self.slam.x[2, 0])
        wx = self.slam.x[0, 0] + c * z_body[0, 0] - s * z_body[1, 0]
        wy = self.slam.x[1, 0] + s * z_body[0, 0] + c * z_body[1, 0]
        z_world = np.array([[wx], [wy]])

        cand_idx, cand_dist = self._find_nearest_candidate(z_world)
        if cand_idx >= 0 and cand_dist <= self.candidate_assoc_dist:
            cand = self.candidates[cand_idx]
            if self._pose_moved_enough_cand(cand, current_pose):
                # Simple Kalman update on candidate
                self._update_candidate(cand, z_world)
                cand["age"] = 0
                cand["last_update_pose"] = current_pose.copy()
                cand["last_z_body"] = z_body.copy()

                if (cand["hits"] >= self.confirmation_hits
                        and not self._too_close_to_confirmed(
                            cand["last_z_body"])):
                    self._promote_candidate(cand_idx)
            return

        # --- new candidate ---
        self.candidates.append({
            "mu": z_world.copy(),
            "P": np.eye(2) * 0.5 ** 2,
            "hits": 1,
            "age": 0,
            "last_update_pose": current_pose.copy(),
            "last_z_body": z_body.copy(),
        })

    # ------------------------------------------------------------------
    # Candidate helpers
    # ------------------------------------------------------------------
    def _find_nearest_candidate(self, z_world):
        best_idx = -1
        best_dist = float("inf")
        for i, cand in enumerate(self.candidates):
            d = float(np.linalg.norm(z_world - cand["mu"]))
            if d < best_dist:
                best_dist = d
                best_idx = i
        return best_idx, best_dist

    def _update_candidate(self, cand, z_world):
        """Simple 2D Kalman update on candidate position."""
        H = np.eye(2)
        R = np.eye(2) * 0.3 ** 2
        S = H @ cand["P"] @ H.T + R
        K = cand["P"] @ H.T @ np.linalg.inv(S)
        innov = z_world - cand["mu"]
        cand["mu"] = cand["mu"] + K @ innov
        I = np.eye(2)
        cand["P"] = (I - K @ H) @ cand["P"] @ (I - K @ H).T + K @ R @ K.T
        cand["hits"] += 1

    def _promote_candidate(self, cand_idx):
        """Move a confirmed candidate into the SLAM state vector."""
        cand = self.candidates.pop(cand_idx)
        # Use the candidate's Kalman-filtered world-frame position,
        # NOT body-frame z_body (which would be stale since the car moved).
        lm_idx = self.slam.add_landmark_world(cand["mu"], cand["P"])
        self.landmark_last_update_pose.append(
            cand["last_update_pose"].copy())
        self.landmark_last_obs_time.append(rospy.Time.now())
        self.landmark_hits.append(cand["hits"])
        rospy.loginfo("SLAM: promoted landmark %d at (%.2f, %.2f)",
                      lm_idx, *self.slam.landmark_pos(lm_idx))

    def _age_candidates(self):
        survivors = []
        for c in self.candidates:
            c["age"] += 1
            if c["age"] <= self.candidate_max_age:
                survivors.append(c)
        self.candidates = survivors

    def _too_close_to_confirmed(self, z_body):
        """Check if the observation is too close to an existing landmark."""
        c = np.cos(self.slam.x[2, 0])
        s = np.sin(self.slam.x[2, 0])
        wx = self.slam.x[0, 0] + c * z_body[0, 0] - s * z_body[1, 0]
        wy = self.slam.x[1, 0] + s * z_body[0, 0] + c * z_body[1, 0]

        for i in range(self.slam.n_landmarks):
            mx, my = self.slam.landmark_pos(i)
            dist = np.sqrt((wx - mx) ** 2 + (wy - my) ** 2)
            if dist < self.min_landmark_spacing:
                return True
        return False

    def _pose_moved_enough_lm(self, lm_idx, current_pose):
        if lm_idx >= len(self.landmark_last_update_pose):
            return True
        last = self.landmark_last_update_pose[lm_idx]
        if last is None:
            return True
        return np.linalg.norm(current_pose - last) >= self.min_motion_for_rehit

    @staticmethod
    def _pose_moved_enough_cand(cand, current_pose):
        last = cand.get("last_update_pose")
        if last is None:
            return True
        return np.linalg.norm(current_pose - last) >= 0.20

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------
    def _odom_to_yaw(self, odom_msg):
        q = odom_msg.pose.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def _initialize(self):
        if self.latest_odom is None:
            return False
        x0 = self.latest_odom.pose.pose.position.x
        y0 = self.latest_odom.pose.pose.position.y
        psi0 = self._odom_to_yaw(self.latest_odom)
        v0 = 0.0
        self.slam = EkfSlam([x0, y0, psi0, v0])
        self.initialized = True
        rospy.loginfo("EKF-SLAM initialised: x=%.3f y=%.3f psi=%.3f",
                      x0, y0, psi0)
        return True

    # ------------------------------------------------------------------
    # Publish filtered odometry
    # ------------------------------------------------------------------
    def _publish_odom(self, stamp, delta):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"

        x = self.slam.x
        msg.pose.pose.position.x = x[0, 0]
        msg.pose.pose.position.y = x[1, 0]
        msg.pose.pose.position.z = 0.0

        q = tft.quaternion_from_euler(0.0, 0.0, x[2, 0])
        msg.pose.pose.orientation = Quaternion(*q)

        v = x[3, 0]
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = (v / self.L) * np.tan(delta)

        msg.pose.covariance = self.slam.vehicle_pose_cov_6x6().flatten().tolist()
        self.odom_pub.publish(msg)

    # ------------------------------------------------------------------
    # Publish map + markers
    # ------------------------------------------------------------------
    def _publish_map(self, stamp):
        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = self.world_frame

        markers = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)

        for i in range(self.slam.n_landmarks):
            mx, my = self.slam.landmark_pos(i)
            Plm = self.slam.landmark_cov(i)

            pose = Pose()
            pose.position.x = mx
            pose.position.y = my
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

            # Sphere marker
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self.world_frame
            m.ns = "cones"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = mx
            m.pose.position.y = my
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = 0.18
            m.scale.y = 0.18
            m.scale.z = 0.18
            m.color.a = 1.0
            votes = self.landmark_colour_votes[i] if i < len(self.landmark_colour_votes) else {}
            winner = max(votes, key=votes.get) if votes else 0 
            colour_map = {
                ConeDetection.BLUE: (0.0, 0.3, 1.0),
                ConeDetection.YELLOW: (1.0, 0.9, 0.0),
                ConeDetection.ORANGE: (1.0, 0.4, 0.0),
            }
            m.color.r, m.color.g, m.color.b = colour_map.get(winner, (1.0, 0.5, 0.0))
            markers.markers.append(m)

            # Covariance ellipse
            cm = Marker()
            cm.header.stamp = stamp
            cm.header.frame_id = self.world_frame
            cm.ns = "cone_covariance"
            cm.id = 1000 + i
            cm.type = Marker.CYLINDER
            cm.action = Marker.ADD
            cm.pose.position.x = mx
            cm.pose.position.y = my
            cm.pose.position.z = 0.02
            cm.pose.orientation.w = 1.0
            cm.scale.x = max(0.05, min(2.0, 2.0*np.sqrt(max(Plm[0, 0], 1e-6))))
            cm.scale.y = max(0.05, min(2.0, 2.0*np.sqrt(max(Plm[1, 1], 1e-6))))
            cm.scale.z = 0.02
            cm.color.a = 0.25
            cm.color.r = 0.1
            cm.color.g = 0.8
            cm.color.b = 0.1
            markers.markers.append(cm)

        # Candidate markers (small red spheres)
        for i, cand in enumerate(self.candidates):
            cm = Marker()
            cm.header.stamp = stamp
            cm.header.frame_id = self.world_frame
            cm.ns = "cone_candidates"
            cm.id = 2000 + i
            cm.type = Marker.SPHERE
            cm.action = Marker.ADD
            cm.pose.position.x = cand["mu"][0, 0]
            cm.pose.position.y = cand["mu"][1, 0]
            cm.pose.position.z = 0.05
            cm.pose.orientation.w = 1.0
            cm.scale.x = 0.10
            cm.scale.y = 0.10
            cm.scale.z = 0.10
            cm.color.a = 0.4
            cm.color.r = 1.0
            markers.markers.append(cm)

        self.map_pub.publish(pose_array)
        self.marker_pub.publish(markers)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def run(self):
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            now = rospy.Time.now()

            if self.last_time is None:
                self.last_time = now
                rate.sleep()
                continue

            dt = (now - self.last_time).to_sec()
            self.last_time = now

            if dt <= 0.0:
                rate.sleep()
                continue

            if not self.initialized:
                self._initialize()
                rate.sleep()
                continue

            # Steering & speed
            delta = 0.63 * 0.5 * (self.delta_fl + self.delta_fr)
            delta = np.clip(delta, -0.23, 0.23)
            v_meas = self.wheel_radius * 0.5 * (-self.w_rl + self.w_rr)

            if self.pre_v_meas is None:
                a = 0.0
            else:
                a = (v_meas - self.pre_v_meas) / dt
            self.pre_v_meas = v_meas

            u = np.array([[delta], [a]])

            if self.latest_imu is None:
                rate.sleep()
                continue

            self.slam.yaw_rate_meas = self.latest_imu.angular_velocity.z
            self.slam.v_meas = v_meas

            # --- EKF-SLAM predict ---
            self.slam.predict(u, dt, self.L)

            # --- Proprioceptive updates ---
            self.slam.update_imu(delta, self.L)

            if self.use_gazebo_yaw and self.latest_odom is not None:
                yaw_meas = self._odom_to_yaw(self.latest_odom)
                self.slam.update_yaw(yaw_meas, self.R_yaw_abs)
            elif not self.use_gazebo_yaw and self.imu_yaw_ready:
                self.slam.update_yaw(self.imu_yaw, np.deg2rad(1.0) ** 2)

            # (cone observation updates happen in _detections_cb)

            rospy.loginfo_throttle(
                1.0,
                "SLAM: x=%.2f y=%.2f psi=%.1f v=%.2f landmarks=%d candidates=%d",
                self.slam.x[0, 0], self.slam.x[1, 0],
                np.rad2deg(self.slam.x[2, 0]),
                self.slam.x[3, 0],
                self.slam.n_landmarks,
                len(self.candidates),
            )

            self._publish_odom(now, delta)
            rate.sleep()


if __name__ == "__main__":
    try:
        node = EkfSlamNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
