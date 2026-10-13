"""Stage-4 action policy.

Direct port of ``actionPolicy.m``. Decision pipeline:

    (a) Hard-override layer — FMEA-mandated immediate-Critical.
    (b) Coarse probabilities from fine posterior.
    (c) Hysteresis-gated coarse-mode decision.
    (d) Argmax over Degraded sub-modes for isolation.

The only behavioural change vs the MATLAB is that the hard-override
latch is now exposed through ``override_cause`` so downstream logging
can record *why* the latch fired instead of inferring it from flag
history.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

import numpy as np

from .config_loader import SupervisorConfig


ACTION_POLICY_VERSION = "1.0-ros"


# ---------------------------------------------------------------------------
@dataclass
class SupervisorAction:
    mode_coarse: int = 1            # 1=Nom 2=Deg 3=Crit
    mode_fine:   int = 1            # 1..M  (M=6 in the default config)
    mode_name:        str = "Nominal"
    mode_name_fine:   str = "Nominal"
    speed_scale: float = 1.0
    estop: bool = False
    adjust_slam: bool = False
    fault_isolation: str = "none"
    override_cause:  str = ""
    severity: float = 0.0
    mu_fine:   np.ndarray = field(default_factory=lambda: np.zeros(6))
    mu_coarse: np.ndarray = field(default_factory=lambda: np.zeros(3))


# ---------------------------------------------------------------------------
@dataclass
class FaultFlagInputs:
    """Subset of FaultFlags fields used by the hard-override layer."""
    cones_both_timeout: bool = False
    steering_fault:     bool = False
    drive_fault:        bool = False
    obstacle_detected:  bool = False


# ---------------------------------------------------------------------------
class ActionPolicy:
    """Stateful wrapper around the actionPolicy.m logic.

    Keeps the hysteresis counters and override latch across calls —
    these are exactly the fields ``sup.hyst.*`` and ``sup.override_*``
    in the MATLAB implementation.
    """

    _ISOLATION = [
        "none",                # fine 1 Nominal
        "Perception",          # fine 2
        "Proprioception",      # fine 3
        "Compute",             # fine 4
        "Actuation",           # fine 5
        "Critical (multiple)", # fine 6
    ]

    def __init__(self, cfg: SupervisorConfig) -> None:
        self.cfg = cfg
        # The isolation strings / speed map below are tied to the agreed
        # 6-mode set; fail loudly if the config drifts from it.
        if cfg.mode.n_modes != len(self._ISOLATION):
            raise ValueError(f"ActionPolicy expects {len(self._ISOLATION)} fine modes "
                             f"(Nominal, Perception, Proprioception, Compute, Actuation, Critical); "
                             f"config has {cfg.mode.n_modes}")
        if list(cfg.mode.coarse_map) != [1, 2, 2, 2, 2, 3]:
            raise ValueError(f"coarse mapping must be N,D,D,D,D,C; got {list(cfg.mode.coarse_map)}")

        self.prev_coarse_mode: int = 1

        # Hysteresis counters
        self.k_above_critical: int = 0
        self.k_above_degraded: int = 0
        self.k_below_degraded: int = 0

        # Hard-override latch
        self.override_latch: bool = False
        self.override_cause: str  = ""

        # Pre-compute degraded sub-mode indices
        self._deg_idx = cfg.mode.deg_indices_zero_based()     # (4,) zero-based
        self._crit_idx_0 = cfg.mode.critical_index_zero_based()

    # ------------------------------------------------------------------
    def step(self, mu: np.ndarray, flags) -> SupervisorAction:
        # ``flags`` is accepted structurally — anything with the four
        # attributes below works. In practice it is a FaultFlagBag
        # (from residuals.py) or the local FaultFlagInputs test shim.
        """Evaluate one step and return the action struct."""
        cfg = self.cfg
        mu  = np.asarray(mu, dtype=float).reshape(-1)

        # ---------------- (a) Hard-override layer ---------------------
        hard_critical = (flags.cones_both_timeout
                         or flags.steering_fault
                         or flags.drive_fault
                         or flags.obstacle_detected)

        if hard_critical and not self.override_latch:
            self.override_latch = True
            causes: List[str] = []
            if flags.cones_both_timeout:  causes.append("cones_both_timeout")
            if flags.steering_fault:      causes.append("steering_critical")
            if flags.drive_fault:         causes.append("drive_critical")
            if flags.obstacle_detected:   causes.append("obstacle_imminent")
            self.override_cause = ", ".join(causes)

        # ---------------- (b) Coarse probabilities --------------------
        coarse_map = cfg.mode.coarse_map      # (M,) values in {1,2,3}
        P_coarse = np.zeros(3, dtype=float)
        for j in range(coarse_map.size):
            P_coarse[coarse_map[j] - 1] += mu[j]
        s = P_coarse.sum()
        if s > 0:
            P_coarse /= s

        p_nom, p_deg, p_crit = float(P_coarse[0]), float(P_coarse[1]), float(P_coarse[2])

        # ---------------- (c) Hysteresis-gated coarse decision --------
        if p_crit >= cfg.action.thresh_critical:
            self.k_above_critical += 1
        else:
            self.k_above_critical = 0

        if (p_deg + p_crit) >= cfg.action.thresh_degraded:
            self.k_above_degraded += 1
            self.k_below_degraded = 0
        else:
            self.k_above_degraded = 0
            self.k_below_degraded += 1

        prev = self.prev_coarse_mode
        if self.override_latch:
            coarse_mode = 3
        elif self.k_above_critical >= cfg.action.k_enter_critical:
            coarse_mode = 3
        elif prev == 3:
            coarse_mode = 3                                         # absorbing
        elif self.k_above_degraded >= cfg.action.k_enter_degraded:
            coarse_mode = 2
        elif prev == 2 and self.k_below_degraded < cfg.action.k_exit_degraded:
            coarse_mode = 2
        else:
            coarse_mode = 1

        # ---------------- (d) Fine mode (isolation) -------------------
        if coarse_mode == 1:
            fine_mode = 1
        elif coarse_mode == 3:
            fine_mode = self._crit_idx_0 + 1      # 1-based fine index
        else:
            # Argmax over the Degraded sub-modes. mu is 0-indexed so we
            # index with the 0-based array, then convert to 1-based.
            sub = self._deg_idx[int(np.argmax(mu[self._deg_idx]))]
            fine_mode = int(sub) + 1

        # Speed scale keyed on fine mode (case-mapped to MATLAB)
        speed_map = [
            cfg.action.speed_scale["nominal"],                 # 1
            cfg.action.speed_scale["degraded_perception"],     # 2
            cfg.action.speed_scale["degraded_proprioception"], # 3
            cfg.action.speed_scale["degraded_compute"],        # 4
            cfg.action.speed_scale["degraded_actuation"],      # 5
            cfg.action.speed_scale["critical"],                # 6
        ]
        speed_scale = float(speed_map[fine_mode - 1])

        adjust_slam = (fine_mode == 2)     # matches MATLAB
        estop = (coarse_mode == 3)

        # Severity scalar (legacy compat)
        severity = 10.0 * p_crit + 5.0 * p_deg

        act = SupervisorAction(
            mode_coarse=int(coarse_mode),
            mode_fine=int(fine_mode),
            mode_name=cfg.mode.names_coarse[coarse_mode - 1],
            mode_name_fine=cfg.mode.names_fine[fine_mode - 1],
            speed_scale=speed_scale,
            estop=bool(estop),
            adjust_slam=bool(adjust_slam),
            fault_isolation=self._ISOLATION[fine_mode - 1],
            override_cause=self.override_cause if self.override_latch else "",
            severity=float(severity),
            mu_fine=mu.copy(),
            mu_coarse=P_coarse.copy(),
        )

        self.prev_coarse_mode = int(coarse_mode)
        return act

    # ------------------------------------------------------------------
    def reset(self) -> None:
        self.prev_coarse_mode = 1
        self.k_above_critical = 0
        self.k_above_degraded = 0
        self.k_below_degraded = 0
        self.override_latch   = False
        self.override_cause   = ""
