"""Unit tests for the IMM port.

Exercises the three paths that matter most:
 1. The posterior is a valid probability distribution after any update.
 2. Nominal residuals keep the posterior near the Nominal prior.
 3. A single sustained firing channel drives the corresponding
    Degraded mode past 95% probability within a few steps.
"""
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SRC  = os.path.normpath(os.path.join(HERE, "..", "src"))
sys.path.insert(0, SRC)

from fsae_supervisor import SupervisorConfig, IMM


CFG_PATH = os.path.normpath(os.path.join(HERE, "..", "config", "supervisor.yaml"))


def _cfg() -> SupervisorConfig:
    return SupervisorConfig.from_yaml(CFG_PATH)


def _valid_posterior(mu: np.ndarray) -> bool:
    return mu.ndim == 1 and mu.size == 6 \
        and np.all(mu >= 0) and np.isclose(mu.sum(), 1.0, atol=1e-9)


def test_initial_posterior_is_valid():
    cfg = _cfg()
    imm = IMM(cfg)
    assert _valid_posterior(imm.mu)
    # Prior mass concentrated on Nominal
    assert imm.mu[0] > 0.9


def test_nominal_residuals_stay_near_nominal():
    cfg = _cfg()
    imm = IMM(cfg)
    r = np.zeros(cfg.residual.n)
    r_valid = np.ones(cfg.residual.n, dtype=bool)
    for _ in range(100):
        imm.step(r, r_valid)
    assert _valid_posterior(imm.mu)
    assert imm.mu[0] > 0.9, f"Nominal collapsed under zero residuals: mu={imm.mu}"


def test_cam_age_drives_DP():
    """cam_age (residual index 8, 1-indexed 9) is inflated only in DP.
    A sustained firing there should push P(DP) past 0.95 quickly."""
    cfg = _cfg()
    imm = IMM(cfg)
    r = np.zeros(cfg.residual.n)
    r_valid = np.ones(cfg.residual.n, dtype=bool)
    for k in range(60):
        r[8] = 0.5 + 0.05 * k       # beyond sigma_nom[8]=0.2 sharply
        imm.step(r, r_valid)
    assert _valid_posterior(imm.mu)
    assert imm.mu[1] > 0.95, f"P(DP) did not dominate: mu={imm.mu}"


def test_imu_freeze_drives_DPr():
    """imu_freeze (index 10 / 1-indexed 11) is inflated only in DPr."""
    cfg = _cfg()
    imm = IMM(cfg)
    r = np.zeros(cfg.residual.n)
    r_valid = np.ones(cfg.residual.n, dtype=bool)
    for _ in range(60):
        r[10] = 3.0                 # log10(1e-8 / 1e-11) = 3
        imm.step(r, r_valid)
    assert _valid_posterior(imm.mu)
    assert imm.mu[2] > 0.95, f"P(DPr) did not dominate: mu={imm.mu}"


def test_mpc_time_drives_DC():
    """mpc_time_norm (index 4 / 1-indexed 5) is inflated only in DC."""
    cfg = _cfg()
    imm = IMM(cfg)
    r = np.zeros(cfg.residual.n)
    r_valid = np.ones(cfg.residual.n, dtype=bool)
    for _ in range(60):
        r[4] = 2.0                  # 2x budget, sustained
        imm.step(r, r_valid)
    assert _valid_posterior(imm.mu)
    assert imm.mu[3] > 0.9, f"P(DC) did not dominate: mu={imm.mu}"


def test_z_clamp_absorbs_transient():
    """A single huge spike should NOT flip the posterior — Z_CLAMP=5
    bounds the evidence any one channel can contribute in one step."""
    cfg = _cfg()
    imm = IMM(cfg)
    r = np.zeros(cfg.residual.n)
    r_valid = np.ones(cfg.residual.n, dtype=bool)

    # Warm up on zeros
    for _ in range(20):
        imm.step(r, r_valid)
    mu_before = imm.mu.copy()

    # Single massive spike on mpc_time
    r[4] = 100.0
    imm.step(r, r_valid)
    # Posterior should move but NOT collapse onto DC in one step
    assert imm.mu[0] > 0.3, \
        f"Single transient collapsed Nominal: before={mu_before}, after={imm.mu}"


def test_masked_residual_marginalised():
    """A residual with r_valid=False must contribute zero evidence."""
    cfg = _cfg()
    imm_a = IMM(cfg)
    imm_b = IMM(cfg)

    r_valid_all = np.ones(cfg.residual.n, dtype=bool)
    r_valid_mask = r_valid_all.copy()
    r_valid_mask[8] = False     # mask cam_age

    r = np.zeros(cfg.residual.n)
    r[8] = 1.0                  # 5 sigma — within clamp, inflated channel dominates

    for _ in range(20):
        imm_a.step(r, r_valid_all)
        imm_b.step(r, r_valid_mask)

    # With cam_age enabled → strong DP evidence (net evidence positive
    # because Z_CLAMP isn't saturated at 5 sigma for DP's inflated sigma).
    assert imm_a.mu[1] > 0.9, f"unmasked cam_age did not drive DP: {imm_a.mu}"
    # With cam_age masked → stay near Nominal
    assert imm_b.mu[0] > 0.9, f"masked cam_age leaked evidence: {imm_b.mu}"


def test_posterior_recovers_from_floor():
    """Regression: an extreme posterior (one mode at ~1, others at floor)
    must remain a valid distribution after an update, even if the data
    drags probability away from the concentrated mode.

    Note: Critical-as-absorbing is a property of the *action policy*
    (actionPolicy.m: `elif prev_mode == 3: coarse_mode = 3`), not of
    the IMM posterior itself — see the action-policy test suite.
    """
    cfg = _cfg()
    imm = IMM(cfg)
    imm.mu = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    r = np.zeros(cfg.residual.n)
    r_valid = np.ones(cfg.residual.n, dtype=bool)
    for _ in range(5):
        imm.step(r, r_valid)
        assert _valid_posterior(imm.mu), f"posterior invalid: {imm.mu}"
