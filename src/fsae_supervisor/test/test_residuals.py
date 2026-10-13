"""Unit tests for the residual generator port."""
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SRC  = os.path.normpath(os.path.join(HERE, "..", "src"))
sys.path.insert(0, SRC)

from fsae_supervisor import (
    SupervisorConfig, ResidualGenerator,
    SupervisorInputs, IMUData, EncoderData, ConeObs, LidarScan, EKFSnapshot,
)

CFG_PATH = os.path.normpath(os.path.join(HERE, "..", "config", "supervisor.yaml"))


def _cfg(): return SupervisorConfig.from_yaml(CFG_PATH)


def test_empty_inputs_give_zero_residual():
    cfg = _cfg()
    rg = ResidualGenerator(cfg)
    r, r_valid, _ = rg.step(SupervisorInputs(), cfg.dt)
    assert r.shape == (cfg.residual.n,)
    # Only cam_age and lid_age are computable without upstream data
    # (they default to 0 on first step because the timers started at 0).
    assert r_valid[8] and r_valid[9]
    assert r[8] >= 0 and r[9] >= 0


def test_cam_age_grows_without_cones():
    cfg = _cfg()
    rg = ResidualGenerator(cfg)
    for k in range(20):
        inp = SupervisorInputs()   # empty cones
        r, _, faults = rg.step(inp, cfg.dt)
    # After 20 steps at dt=0.05s cam_age should be ~1s
    assert r[8] >= 0.95, f"cam_age did not grow: {r[8]}"
    assert faults.camera_timeout     # triggered at > 0.5 s
    assert faults.cones_cam_timeout  # triggered at > 1.0 s


def test_cam_age_resets_on_detections():
    cfg = _cfg()
    rg = ResidualGenerator(cfg)
    for _ in range(10):
        inp = SupervisorInputs()
        rg.step(inp, cfg.dt)
    # Now deliver a cone detection
    inp = SupervisorInputs(cam_cones=ConeObs(range_array=np.array([1.5, 2.0])))
    r, _, _ = rg.step(inp, cfg.dt)
    assert r[8] == 0.0


def test_imu_encoder_parity_zero_under_bicycle_model():
    """With delta=0, omega_pred = 0, so r[5] = |omega_imu - 0| = |omega_imu|."""
    cfg = _cfg()
    rg = ResidualGenerator(cfg)
    inp = SupervisorInputs(
        imu=IMUData(w=0.0, valid=True),
        enc=EncoderData(v=1.0, valid=True),
        delta_cmd=0.0,
    )
    r, r_valid, _ = rg.step(inp, cfg.dt)
    assert r_valid[5]
    assert abs(r[5]) < 1e-12


def test_imu_freeze_detection():
    cfg = _cfg()
    rg = ResidualGenerator(cfg)
    # Pump W samples of the exact same IMU reading → variance ≈ 0 → freeze
    for _ in range(cfg.window.freeze_W + 2):
        inp = SupervisorInputs(imu=IMUData(w=0.123456, valid=True),
                               v_ref=1.0)
        r, r_valid, _ = rg.step(inp, cfg.dt)
    assert r_valid[10]
    assert r[10] > 1.0, f"imu_freeze should be >> 0: {r[10]}"


def test_drive_stall_hard_override():
    """Throttle high + v near zero sustained > drive_timeout → drive_fault."""
    cfg = _cfg()
    rg = ResidualGenerator(cfg)
    steps = int(cfg.override.drive_timeout / cfg.dt) + 2
    faults = None
    for _ in range(steps):
        inp = SupervisorInputs(
            enc=EncoderData(v=0.0, valid=True),
            throttle_cmd=0.5,              # throttle commanded
            v_ref=1.0,
        )
        _, _, faults = rg.step(inp, cfg.dt)
    assert faults is not None and faults.drive_fault
