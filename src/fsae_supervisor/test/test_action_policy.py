"""Unit tests for the Stage-4 action policy port."""
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SRC  = os.path.normpath(os.path.join(HERE, "..", "src"))
sys.path.insert(0, SRC)

from fsae_supervisor import SupervisorConfig, ActionPolicy, FaultFlagInputs

CFG_PATH = os.path.normpath(os.path.join(HERE, "..", "config", "supervisor.yaml"))


def _cfg(): return SupervisorConfig.from_yaml(CFG_PATH)


def _mu(primary: int, p: float = 1.0) -> np.ndarray:
    """Build a 6-vector posterior with mass `p` on fine mode `primary` (1-indexed)."""
    mu = np.full(6, (1.0 - p) / 5.0)
    mu[primary - 1] = p
    mu /= mu.sum()
    return mu


def test_nominal_posterior_stays_nominal():
    cfg = _cfg()
    policy = ActionPolicy(cfg)
    for _ in range(10):
        act = policy.step(_mu(1, 1.0), FaultFlagInputs())
    assert act.mode_coarse == 1 and act.mode_fine == 1
    assert act.speed_scale == 1.0
    assert act.estop is False


def test_dp_posterior_transitions_after_hysteresis():
    cfg = _cfg()
    policy = ActionPolicy(cfg)
    # Not enough steps yet — should stay Nominal
    act = policy.step(_mu(2, 1.0), FaultFlagInputs())
    assert act.mode_coarse == 1
    for _ in range(cfg.action.k_enter_degraded):
        act = policy.step(_mu(2, 1.0), FaultFlagInputs())
    assert act.mode_coarse == 2
    assert act.fault_isolation == "Perception"
    assert act.speed_scale == cfg.action.speed_scale["degraded_perception"]


def test_hard_override_immediate_critical():
    cfg = _cfg()
    policy = ActionPolicy(cfg)
    # Even with mu concentrated on Nominal, a hard-override flag forces Critical
    act = policy.step(_mu(1, 1.0), FaultFlagInputs(cones_both_timeout=True))
    assert act.mode_coarse == 3
    assert act.estop is True
    assert "cones_both_timeout" in act.override_cause


def test_override_latches():
    cfg = _cfg()
    policy = ActionPolicy(cfg)
    policy.step(_mu(1, 1.0), FaultFlagInputs(steering_fault=True))
    # Clear the flag — we should still be in Critical (latched)
    act = policy.step(_mu(1, 1.0), FaultFlagInputs())
    assert act.mode_coarse == 3
    assert "steering_critical" in act.override_cause


def test_critical_coarse_is_absorbing_even_without_latch():
    cfg = _cfg()
    policy = ActionPolicy(cfg)
    # Drive into Critical via IMM (k_enter_critical consecutive steps)
    for _ in range(cfg.action.k_enter_critical):
        act = policy.step(_mu(6, 1.0), FaultFlagInputs())
    # After entering Critical, feeding Nominal posteriors should not exit
    for _ in range(10):
        act = policy.step(_mu(1, 1.0), FaultFlagInputs())
    assert act.mode_coarse == 3


def test_degraded_argmax_over_submodes():
    """When coarse=Degraded, fine_mode is argmax over the 4 Degraded submodes."""
    cfg = _cfg()
    policy = ActionPolicy(cfg)
    for _ in range(cfg.action.k_enter_degraded):
        # Heavy on DPr (index 3)
        act = policy.step(_mu(3, 0.9), FaultFlagInputs())
    assert act.mode_coarse == 2
    assert act.mode_fine == 3
    assert act.fault_isolation == "Proprioception"
