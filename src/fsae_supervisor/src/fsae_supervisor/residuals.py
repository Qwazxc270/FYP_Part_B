"""Residual generator — port of computeResiduals.m.

Consumes a ``SupervisorInputs`` snapshot (built by the node from
the latest ROS messages) and emits:

    r       : (n,) residual vector
    r_valid : (n,) per-element validity mask — False entries are
              marginalised by the IMM
    faults  : FaultFlagBag — human-readable boolean flags

The math is identical to the MATLAB reference; only the bookkeeping
differs (Python dataclasses instead of MATLAB structs, explicit
RingBuffer instances instead of the sup.buf.* struct-of-arrays).

Indexing note: inside this module r is 0-indexed — r[0]..r[11]
corresponds to the MATLAB r(1)..r(12). Comments keep the 1-indexed
names for traceability with fault_dictionary.md.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import tan
from typing import Dict, List, Optional, Tuple

import numpy as np

from .config_loader import SupervisorConfig
from .ring_buffer   import RingBuffer


# =====================================================================
#  Input / output dataclasses
# =====================================================================

@dataclass
class IMUData:
    w:     Optional[float] = None    # yaw rate [rad/s]
    valid: bool = True

@dataclass
class EncoderData:
    v:     Optional[float] = None    # longitudinal velocity [m/s]
    valid: bool = True

@dataclass
class ConeObs:
    """Any non-empty range array marks the sensor as 'fresh this step'."""
    range_array: np.ndarray = field(default_factory=lambda: np.zeros(0))

@dataclass
class LidarScan:
    ranges: np.ndarray = field(default_factory=lambda: np.zeros(0))  # for obstacle override

@dataclass
class EKFSnapshot:
    x:        Optional[np.ndarray] = None     # at least 4 elements
    P:        Optional[np.ndarray] = None     # full covariance
    last_nis: Optional[float] = None
    updated:  bool = False

@dataclass
class SupervisorInputs:
    """Snapshot of everything the residual generator needs this step."""
    imu:           IMUData     = field(default_factory=IMUData)
    enc:           EncoderData = field(default_factory=EncoderData)
    cam_cones:     ConeObs     = field(default_factory=ConeObs)
    lidar_cones:   ConeObs     = field(default_factory=ConeObs)
    lidar_scan:    LidarScan   = field(default_factory=LidarScan)
    ekf:           EKFSnapshot = field(default_factory=EKFSnapshot)

    # Control layer
    delta_cmd:     float = 0.0           # commanded steering [rad]
    v_ref:         float = 0.0           # reference speed [m/s]
    cte:           float = 0.0           # cross-track error [m]
    mpc_time_ms:   float = 0.0
    mpc_budget_ms: float = 50.0

    # Optional — only used if present
    throttle_cmd:    Optional[float] = None
    steering_meas:   Optional[float] = None     # QCar has none; stays None
    steering_fault_injected: bool = False       # fault-flag surrogate
    drive_fault_injected:    bool = False
    # r13 filter_gap, precomputed by the node from the two odometry
    # histories (team_adapters.filter_gap); None = not computable.
    filter_gap:      Optional[float] = None


@dataclass
class FaultFlagBag:
    camera_timeout:        bool = False
    lidar_timeout:         bool = False
    cones_cam_timeout:     bool = False
    cones_lidar_timeout:   bool = False
    cones_both_timeout:    bool = False
    imu_timeout:           bool = False
    encoder_timeout:       bool = False
    steering_fault:        bool = False
    drive_fault:           bool = False
    ekf_divergence:        bool = False
    cte_exceeded:          bool = False
    speed_deviation:       bool = False
    mpc_overrun:           bool = False
    obstacle_detected:     bool = False
    battery_low:           bool = False
    overcurrent:           bool = False

    def reset(self) -> None:
        for k in self.__dataclass_fields__:
            setattr(self, k, False)


# =====================================================================
#  Residual generator
# =====================================================================

class ResidualGenerator:
    """Stateful residual computer. One instance per supervisor node."""

    def __init__(self, cfg: SupervisorConfig) -> None:
        self.cfg = cfg
        self.n   = cfg.residual.n

        # Rolling buffers
        w = cfg.window
        self.buf_nis   = RingBuffer(w.NIS_W)
        self.buf_mpc   = RingBuffer(w.mpc_W)
        self.buf_imu_w = RingBuffer(w.freeze_W)
        self.buf_enc_v = RingBuffer(w.freeze_W)

        # Freshness timers [s]
        self.t_cam = 0.0
        self.t_lid = 0.0
        self.t_imu = 0.0
        self.t_enc = 0.0

        # Hard-override accumulators [s]
        self.steer_err_time = 0.0
        self.drive_err_time = 0.0

        # Adjust-SLAM request latch (cleared at every step; set when
        # conesCam stays stale > 1.0 s — same semantics as MATLAB).
        self.adjust_slam = False

        # The freeze floor from the MATLAB code (see eps_freeze in
        # computeResiduals.m). Healthy IMU variance is 1e-4..1e-3;
        # frozen is 1e-12.
        self._eps_freeze = 1e-8

    # ------------------------------------------------------------------
    def step(self, inp: SupervisorInputs, dt: float
             ) -> Tuple[np.ndarray, np.ndarray, FaultFlagBag]:
        """Compute r_k, r_valid, and the flag bag for one step."""
        cfg = self.cfg
        n   = self.n

        r       = np.zeros(n)
        r_valid = np.zeros(n, dtype=bool)
        faults  = FaultFlagBag()
        self.adjust_slam = False

        # -------- r[0] sigma_pos --------
        if inp.ekf.P is not None and inp.ekf.P.shape[0] >= 2:
            P_pos = inp.ekf.P[:2, :2]
            sigma_pos = float(np.sqrt(np.trace(P_pos) / 2.0))
            r[0] = sigma_pos
            r_valid[0] = True
            if sigma_pos > cfg.flag.sigma_pos_warn:
                faults.ekf_divergence = True

        # -------- r[1] cte --------
        r[1]       = abs(inp.cte)
        r_valid[1] = True
        if r[1] > cfg.flag.cte_warn:
            faults.cte_exceeded = True

        # -------- r[2] speed_dev --------
        if inp.ekf.x is not None and inp.ekf.x.size >= 4:
            v_hat = float(inp.ekf.x[3])
            r[2]       = abs(v_hat - inp.v_ref)
            r_valid[2] = True
            if r[2] > 0.3:
                faults.speed_deviation = True

        # -------- r[3] NIS_mean --------
        nis = inp.ekf.last_nis
        if nis is not None and np.isfinite(nis):
            self.buf_nis.push(float(nis))
        if self.buf_nis.n >= 3:
            m = self.buf_nis.mean()
            if m is not None:
                r[3] = m
                r_valid[3] = True

        # -------- r[4] mpc_time_norm --------
        self.buf_mpc.push(inp.mpc_time_ms)
        if self.buf_mpc.n >= 1:
            m = self.buf_mpc.mean()
            if m is not None:
                budget = inp.mpc_budget_ms if inp.mpc_budget_ms > 0 else 50.0
                r[4] = m / budget
                r_valid[4] = True
                if r[4] > 1.0:
                    faults.mpc_overrun = True

        # -------- r[5] imu_enc_parity --------
        # omega_pred = (v_enc / L) * tan(delta_cmd)
        imu_ok = inp.imu.valid and (inp.imu.w is not None)
        enc_ok = inp.enc.valid and (inp.enc.v is not None)
        if imu_ok and enc_ok:
            L = max(cfg.L_wheelbase, 1e-3)
            omega_pred = (inp.enc.v / L) * tan(inp.delta_cmd)
            r[5]       = abs(inp.imu.w - omega_pred)
            r_valid[5] = True

        # -------- r[6] steer_parity --------
        # QCar has no servo feedback; fall back to the injected-fault
        # surrogate (binary). If you ever wire servo position into
        # inp.steering_meas, that path takes over automatically.
        if inp.steering_meas is not None:
            r[6]       = abs(inp.delta_cmd - inp.steering_meas)
            r_valid[6] = True
        elif inp.steering_fault_injected:
            r[6]       = cfg.override.steer_err_critical * 1.5
            r_valid[6] = True

        if r_valid[6] and r[6] > cfg.override.steer_err_critical:
            self.steer_err_time += dt
        else:
            self.steer_err_time = 0.0
        if self.steer_err_time >= cfg.override.steer_timeout:
            faults.steering_fault = True

        # -------- r[7] drive_parity (SOFT residual) --------
        if enc_ok:
            r[7]       = abs(inp.v_ref - inp.enc.v)
            r_valid[7] = True
        elif inp.drive_fault_injected:
            r[7]       = cfg.override.drive_err_critical * 2.0
            r_valid[7] = True

        # Drive-stall hard override (true DRV-01 signature)
        throttle_high = (inp.throttle_cmd is not None
                         and np.isfinite(inp.throttle_cmd)
                         and abs(inp.throttle_cmd) > 0.2)
        v_near_zero = enc_ok and abs(inp.enc.v) < 0.05
        stall_now = throttle_high and v_near_zero
        if stall_now or inp.drive_fault_injected:
            self.drive_err_time += dt
        else:
            self.drive_err_time = 0.0
        if self.drive_err_time >= cfg.override.drive_timeout:
            faults.drive_fault = True

        # -------- r[8] cam_age, r[9] lid_age --------
        cam_active = inp.cam_cones.range_array.size > 0
        if cam_active:
            self.t_cam = 0.0
        else:
            self.t_cam += dt
        r[8]       = self.t_cam
        r_valid[8] = True
        if r[8] > 0.5:
            faults.camera_timeout = True
        # conesCam mirrors cam_age (same topic in this pipeline)
        if self.t_cam > 1.0:
            faults.cones_cam_timeout = True
            self.adjust_slam = True

        lid_active = inp.lidar_cones.range_array.size > 0
        if lid_active:
            self.t_lid = 0.0
        else:
            self.t_lid += dt
        r[9]       = self.t_lid
        r_valid[9] = True
        if r[9] > 0.5:
            faults.lidar_timeout = True
        if self.t_lid > 0.5:
            faults.cones_lidar_timeout = True

        if (self.t_cam >= cfg.override.cones_both_timeout
                and self.t_lid >= cfg.override.cones_both_timeout):
            faults.cones_both_timeout = True

        # IMU/encoder freshness timers
        if inp.imu.valid:
            self.t_imu = 0.0
        else:
            self.t_imu += dt
        if self.t_imu > 0.5:
            faults.imu_timeout = True

        if inp.enc.valid:
            self.t_enc = 0.0
        else:
            self.t_enc += dt
        if self.t_enc > 0.5:
            faults.encoder_timeout = True

        # -------- r[10] imu_freeze --------
        # Runs UNCONDITIONALLY on whatever the IMU reports — a
        # silently-stuck sensor still publishes, and the freeze
        # detector is precisely the test that catches it.
        if inp.imu.w is not None and np.isfinite(inp.imu.w):
            self.buf_imu_w.push(float(inp.imu.w))
            if self.buf_imu_w.n >= cfg.window.freeze_W:
                v = self.buf_imu_w.var()
                if v is not None and np.isfinite(v):
                    v = max(v, 1e-15)
                    r[10]       = max(0.0, np.log10(self._eps_freeze / v))
                    r_valid[10] = True

        # -------- r[11] enc_freeze --------
        if inp.enc.v is not None and np.isfinite(inp.enc.v):
            self.buf_enc_v.push(float(inp.enc.v))
            if self.buf_enc_v.n >= cfg.window.freeze_W:
                v = self.buf_enc_v.var()
                if v is not None and np.isfinite(v):
                    v = max(v, 1e-15)
                    moving = abs(inp.v_ref) > 0.1
                    if moving:
                        r[11] = max(0.0, np.log10(self._eps_freeze / v))
                    else:
                        r[11] = 0.0
                    r_valid[11] = True

        # -------- r[12] filter_gap (optional 13th channel) --------
        i_fg = cfg.channel_index("filter_gap") if hasattr(cfg, "channel_index") else -1
        if i_fg >= 0 and inp.filter_gap is not None and np.isfinite(inp.filter_gap):
            r[i_fg] = float(inp.filter_gap)
            r_valid[i_fg] = True

        # -------- Config-level channel mask --------
        # Channels disabled in config (platform can't supply the data yet)
        # are forced invalid so the IMM marginalises them, and the soft
        # flags derived purely from them are suppressed. See
        # INTEGRATION_TODO.md for what must be published to re-enable each.
        en = getattr(cfg, "channels_enabled", None)
        if en is not None:
            off = ~np.asarray(en, dtype=bool)
            r[off] = 0.0
            r_valid[off] = False
            if off[1]:  faults.cte_exceeded    = False
            if off[2]:  faults.speed_deviation = False
            if off[4]:  faults.mpc_overrun     = False
            if off[0]:  faults.ekf_divergence  = False
            if off[6]:
                faults.steering_fault = False
                self.steer_err_time = 0.0

        # -------- Obstacle hard override --------
        if inp.lidar_scan.ranges.size > 0:
            if float(np.min(inp.lidar_scan.ranges)) < cfg.override.obstacle_min_dist:
                faults.obstacle_detected = True

        return r, r_valid, faults

    # ------------------------------------------------------------------
    def reset(self) -> None:
        """Zero all timers, counters, and rolling buffers."""
        self.buf_nis.reset()
        self.buf_mpc.reset()
        self.buf_imu_w.reset()
        self.buf_enc_v.reset()
        self.t_cam = 0.0
        self.t_lid = 0.0
        self.t_imu = 0.0
        self.t_enc = 0.0
        self.steer_err_time = 0.0
        self.drive_err_time = 0.0
        self.adjust_slam = False
