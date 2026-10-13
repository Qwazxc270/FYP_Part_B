"""Interacting Multiple Model filter on the discrete fault-mode state.

Direct port of ``imm.m``. Same three steps:

    1.  Predict  : mu_bar = TPM' * mu         (Chapman-Kolmogorov)
    2.  Weight   : loglik(j) = sum_i -0.5 * (log(2pi)
                               + 2 log sigma_{j,i}
                               + clamp(z_{j,i}, ±Z_CLAMP)^2)
                   over the subset of residual indices with r_valid[i] = True.
    3.  Update   : mu(j) ∝ exp(loglik(j)) * mu_bar(j); normalise in the
                   log domain for stability.

The Z_CLAMP=5 is a Huber-style bound on per-channel evidence. Without
it a single transient outlier (e.g. an MPC JIT warm-up spike) could
flip the posterior out of Nominal in one step.
"""
from __future__ import annotations

import numpy as np

from .config_loader import SupervisorConfig


_LOG_2PI = float(np.log(2.0 * np.pi))


class IMM:
    """Stateful IMM over ``M`` fault modes, posterior stored on ``self.mu``."""

    def __init__(self, cfg: SupervisorConfig) -> None:
        self.cfg = cfg
        M = cfg.mode.n_modes

        self.mu     = cfg.imm.mu0.copy()           # posterior
        self.mu_bar = cfg.imm.mu0.copy()           # predicted prior
        self.loglik = np.zeros(M, dtype=float)      # diagnostic

        # Precompute things used every step.
        self._TPM_T       = cfg.imm.TPM.T.copy()
        self._sigma       = cfg.mode.sigma_per_mode       # (M, n)
        self._log_sigma   = cfg.mode.log_sigma_per_mode   # (M, n)
        self._mu_per_mode = cfg.mode.mu_per_mode          # (M, n), all 0
        self._mu_floor    = cfg.imm.mu_floor
        self._loglik_flr  = cfg.imm.loglik_floor
        self._z_clamp     = cfg.imm.z_clamp

    # ------------------------------------------------------------------
    def coarse(self) -> np.ndarray:
        """Collapse the fine posterior to [P(Nominal), P(Degraded), P(Critical)]
        using the explicit cfg.mode.coarse_map (no hard-coded indices)."""
        cm = self.cfg.mode.coarse_map
        p = np.zeros(3, dtype=float)
        for j in range(cm.size):
            p[cm[j] - 1] += self.mu[j]
        s = p.sum()
        return p / s if s > 0 else p

    def step(self, r: np.ndarray, r_valid: np.ndarray) -> None:
        """Run one IMM prediction + update.

        Parameters
        ----------
        r        : (n,) residual vector
        r_valid  : (n,) bool mask — True iff the residual was computable
        """
        r       = np.asarray(r, dtype=float).reshape(-1)
        r_valid = np.asarray(r_valid, dtype=bool).reshape(-1)
        if r.size != self._sigma.shape[1]:
            raise ValueError(
                f"residual length {r.size} != n_residuals "
                f"{self._sigma.shape[1]}")

        # ---- 1. Prediction (discrete Chapman-Kolmogorov) ----
        mu_bar = self._TPM_T @ self.mu
        np.maximum(mu_bar, self._mu_floor, out=mu_bar)
        mu_bar /= mu_bar.sum()

        # ---- 2. Per-mode log-likelihood of r ----
        # Diagonal-Gaussian with Huber-style z-clamp.
        # Vectorised version of the MATLAB inner loop.
        if not np.any(r_valid):
            # No valid residuals this step — no evidence, just propagate.
            self.mu_bar = mu_bar
            self.loglik = np.zeros_like(self.mu)
            self.mu     = mu_bar.copy()
            return

        # Work on the masked subset so the cost scales with the number
        # of valid residuals, not with n.
        r_valid_idx = np.where(r_valid)[0]
        r_sub       = r[r_valid_idx]                   # (k,)
        sigma_sub   = self._sigma[:, r_valid_idx]       # (M, k)
        log_sig_sub = self._log_sigma[:, r_valid_idx]   # (M, k)
        mu_sub      = self._mu_per_mode[:, r_valid_idx] # (M, k), zeros

        z      = (r_sub[None, :] - mu_sub) / sigma_sub
        z_clip = np.clip(z, -self._z_clamp, self._z_clamp)

        # -0.5 * (log2pi + 2 log sigma + z^2) summed over valid channels
        per_chan = -0.5 * (_LOG_2PI + 2.0 * log_sig_sub + z_clip * z_clip)
        loglik   = per_chan.sum(axis=1)
        np.maximum(loglik, self._loglik_flr, out=loglik)

        # ---- 3. Bayes update in log domain ----
        log_post = loglik + np.log(mu_bar)
        log_post -= log_post.max()            # numerical stability
        unnorm   = np.exp(log_post)
        mu_new   = unnorm / unnorm.sum()

        # Floor + renormalise (defensive — matches MATLAB)
        np.maximum(mu_new, self._mu_floor, out=mu_new)
        mu_new /= mu_new.sum()

        # Store
        self.mu_bar = mu_bar
        self.loglik = loglik
        self.mu     = mu_new

    # ------------------------------------------------------------------
    def reset(self) -> None:
        self.mu     = self.cfg.imm.mu0.copy()
        self.mu_bar = self.cfg.imm.mu0.copy()
        self.loglik[:] = 0.0
