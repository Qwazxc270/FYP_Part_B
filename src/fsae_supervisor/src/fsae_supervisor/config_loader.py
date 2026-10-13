"""Config loader for the supervisor.

Port of supervisorConfig.m. Tuning values live in supervisor.yaml;
this module materialises them into a SupervisorConfig dataclass.

The key work this file does, beyond YAML parsing, is building the
per-mode sigma vectors from the ``inflate_idx`` spec. The MATLAB
function computes::

    sigma_j = sigma_nom .* (1 + (f-1) * oneHot(idx))

which is equivalent to ``sigma_j[i] = f * sigma_nom[i]`` at the
inflated indices and ``sigma_nom[i]`` elsewhere. We store both the
raw matrix and ``log(sigma_j)`` so the IMM hot loop doesn't have
to recompute them every step.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

import numpy as np


# =====================================================================
#  Sub-specs
# =====================================================================

@dataclass
class ResidualSpec:
    names: List[str]
    units: List[str]
    sigma_nom: np.ndarray                 # (n,)

    @property
    def n(self) -> int:
        return int(self.sigma_nom.size)


@dataclass
class ModeSpec:
    names_fine: List[str]
    names_coarse: List[str]
    coarse_map: np.ndarray                # (M,) ints in {1,2,3}
    inflate_idx: List[List[int]]          # 1-indexed channel lists
    inf_deg: float
    inf_crit: float

    sigma_per_mode: np.ndarray = field(init=False)       # (M, n)
    log_sigma_per_mode: np.ndarray = field(init=False)   # (M, n)
    mu_per_mode: np.ndarray = field(init=False)          # (M, n)  — zeros

    @property
    def n_modes(self) -> int:
        return len(self.names_fine)

    def deg_indices_zero_based(self) -> np.ndarray:
        """Fine-mode indices (0-based) whose coarse mapping is Degraded."""
        return np.where(self.coarse_map == 2)[0]

    def critical_index_zero_based(self) -> int:
        """The single fine-mode index (0-based) whose coarse mapping is Critical."""
        idxs = np.where(self.coarse_map == 3)[0]
        if idxs.size == 0:
            raise ValueError("No Critical coarse mode in coarse_map")
        return int(idxs[0])


@dataclass
class IMMSpec:
    TPM:   np.ndarray
    mu0:   np.ndarray
    mu_floor:     float
    loglik_floor: float
    z_clamp:      float


@dataclass
class ActionSpec:
    thresh_critical:  float
    thresh_degraded:  float
    k_enter_degraded: int
    k_enter_critical: int
    k_exit_degraded:  int
    speed_scale: Dict[str, float]


@dataclass
class OverrideSpec:
    cones_both_timeout: float
    steer_err_critical: float
    steer_timeout:      float
    drive_err_critical: float
    drive_timeout:      float
    obstacle_min_dist:  float


@dataclass
class WindowSpec:
    freeze_W: int
    NIS_W:    int
    parity_W: int
    mpc_W:    int


@dataclass
class FlagSpec:
    sigma_pos_warn: float
    sigma_pos_crit: float
    cte_warn:       float
    cte_crit:       float


class TopicSpec:
    """Topic names, read from the platform profile (config/platforms/*.yaml).

    Kept as a free-form mapping so a platform profile can add/omit topics
    without code changes. An empty string means "not available on this
    platform" and the node does not subscribe.
    """

    def __init__(self, d: Dict[str, str]) -> None:
        self._d = {k: ("" if v is None else str(v)) for k, v in (d or {}).items()}

    def __getattr__(self, k: str) -> str:
        if k.startswith("_"):
            raise AttributeError(k)
        return self._d.get(k, "")

    def get(self, k: str, default: str = "") -> str:
        return self._d.get(k, default)

    def as_dict(self) -> Dict[str, str]:
        return dict(self._d)


@dataclass
class FilterGapSpec:
    """r13 filter_gap parameters (see supervisor.yaml)."""
    window_s:       float = 1.0
    pos_thresh_m:   float = 0.15
    yaw_thresh_rad: float = 0.10
    max_age_s:      float = 0.5
    history_s:      float = 3.0


@dataclass
class PlatformSpec:
    """Platform-specific physical constants + interface options."""
    name:          str   = "gazebo"
    wheel_radius:  float = 0.0325
    rear_left_wheel_sign:  float = -1.0
    rear_right_wheel_sign: float = 1.0
    steer_sign:    float = 1.0
    v_ref_source:  str   = "trajectory"     # trajectory | mpc_stats | none
    scan_range_min: float = 0.0


# =====================================================================
#  Top-level
# =====================================================================

@dataclass
class SupervisorConfig:
    rate_hz:     float
    L_wheelbase: float
    residual:    ResidualSpec
    mode:        ModeSpec
    imm:         IMMSpec
    action:      ActionSpec
    override:    OverrideSpec
    window:      WindowSpec
    flag:        FlagSpec
    topics:      TopicSpec
    channels_enabled: np.ndarray = None      # (n,) bool — config-level channel mask
    platform:    PlatformSpec = field(default_factory=PlatformSpec)
    filter_gap:  FilterGapSpec = field(default_factory=FilterGapSpec)

    def channel_index(self, name: str) -> int:
        """0-based index of a residual channel, or -1 if not configured."""
        try:
            return self.residual.names.index(name)
        except ValueError:
            return -1

    def channel_enabled(self, name: str) -> bool:
        i = self.channel_index(name)
        return i >= 0 and bool(self.channels_enabled[i])

    @property
    def dt(self) -> float:
        return 1.0 / float(self.rate_hz)

    # ------------------------------------------------------------------
    @classmethod
    def from_dict(cls, d: dict) -> "SupervisorConfig":
        # -- residual
        rs = d["residual"]
        residual = ResidualSpec(
            names=list(rs["names"]),
            units=list(rs["units"]),
            sigma_nom=np.asarray(rs["sigma_nom"], dtype=float),
        )
        n = residual.n

        # -- mode: materialise sigma matrices
        ms = d["mode"]
        inflate_idx: List[List[int]] = []
        for entry in ms["inflate_idx"]:
            if entry == "all":
                inflate_idx.append(list(range(1, n + 1)))
            elif entry in (None, []):
                inflate_idx.append([])
            else:
                inflate_idx.append([int(x) for x in entry])

        inf_deg  = float(ms["inf_deg"])
        inf_crit = float(ms["inf_crit"])

        M = len(ms["names_fine"])
        names_coarse = list(ms["names_coarse"])
        if names_coarse != ["Nominal", "Degraded", "Critical"]:
            raise ValueError(f"names_coarse must be [Nominal, Degraded, Critical]; got {names_coarse}")
        # Explicit fine -> coarse mapping (preferred). coarse_map (ints) is
        # derived from it; if both are given they must agree.
        if "coarse_of" in ms:
            co = ms["coarse_of"]
            missing = [n_ for n_ in ms["names_fine"] if n_ not in co]
            if missing:
                raise ValueError(f"coarse_of missing fine modes: {missing}")
            derived = [names_coarse.index(co[n_]) + 1 for n_ in ms["names_fine"]]
            if "coarse_map" in ms and list(ms["coarse_map"]) != derived:
                raise ValueError(f"coarse_map {list(ms['coarse_map'])} disagrees with coarse_of {derived}")
            coarse_map_list = derived
        else:
            coarse_map_list = [int(x) for x in ms["coarse_map"]]
        if len(coarse_map_list) != M:
            raise ValueError("coarse mapping length must equal number of fine modes")
        if coarse_map_list[0] != 1 or coarse_map_list.count(3) != 1 or coarse_map_list[-1] != 3:
            raise ValueError("first fine mode must be Nominal and exactly one (the last) must be Critical")
        sigma_per_mode = np.tile(residual.sigma_nom, (M, 1))   # (M, n)

        # MATLAB:
        #   mode 1 Nominal                -> sigma_nom
        #   mode 2..5 Degraded            -> sigma_nom with INF_DEG at idx
        #   mode M (last) Critical        -> sigma_nom * INF_CRIT everywhere
        sigma_per_mode[M - 1] = residual.sigma_nom * inf_crit
        for j in range(M - 1):
            for ii in inflate_idx[j]:
                sigma_per_mode[j, ii - 1] = residual.sigma_nom[ii - 1] * inf_deg

        mode = ModeSpec(
            names_fine=list(ms["names_fine"]),
            names_coarse=list(ms["names_coarse"]),
            coarse_map=np.asarray(coarse_map_list, dtype=int),
            inflate_idx=inflate_idx,
            inf_deg=inf_deg,
            inf_crit=inf_crit,
        )
        mode.sigma_per_mode     = sigma_per_mode
        mode.log_sigma_per_mode = np.log(sigma_per_mode)
        mode.mu_per_mode        = np.zeros_like(sigma_per_mode)

        # -- IMM
        imm_raw = d["imm"]
        TPM = np.asarray(imm_raw["TPM"], dtype=float)
        if TPM.shape != (M, M):
            raise ValueError(f"TPM must be {(M,M)}; got {TPM.shape}")
        row_sums = TPM.sum(axis=1)
        if not np.allclose(row_sums, 1.0, atol=1e-6):
            raise ValueError(f"TPM rows must sum to 1; got {row_sums}")

        imm = IMMSpec(
            TPM=TPM,
            mu0=np.asarray(imm_raw["mu0"], dtype=float),
            mu_floor=float(imm_raw["mu_floor"]),
            loglik_floor=float(imm_raw["loglik_floor"]),
            z_clamp=float(imm_raw["z_clamp"]),
        )

        ac = d["action"]
        action = ActionSpec(
            thresh_critical=float(ac["thresh_critical"]),
            thresh_degraded=float(ac["thresh_degraded"]),
            k_enter_degraded=int(ac["k_enter_degraded"]),
            k_enter_critical=int(ac["k_enter_critical"]),
            k_exit_degraded=int(ac["k_exit_degraded"]),
            speed_scale={k: float(v) for k, v in ac["speed_scale"].items()},
        )

        ov = d["override"]
        override = OverrideSpec(
            cones_both_timeout=float(ov["cones_both_timeout"]),
            steer_err_critical=float(ov["steer_err_critical"]),
            steer_timeout=float(ov["steer_timeout"]),
            drive_err_critical=float(ov["drive_err_critical"]),
            drive_timeout=float(ov["drive_timeout"]),
            obstacle_min_dist=float(ov["obstacle_min_dist"]),
        )

        w = d["window"]
        window = WindowSpec(
            freeze_W=int(w["freeze_W"]),
            NIS_W=int(w["NIS_W"]),
            parity_W=int(w["parity_W"]),
            mpc_W=int(w["mpc_W"]),
        )

        fg = d["flag"]
        flag = FlagSpec(
            sigma_pos_warn=float(fg["sigma_pos_warn"]),
            sigma_pos_crit=float(fg["sigma_pos_crit"]),
            cte_warn=float(fg["cte_warn"]),
            cte_crit=float(fg["cte_crit"]),
        )

        topics = TopicSpec(d.get("topics", {}))

        en_raw = rs.get("enabled", None)
        enabled = np.ones(n, dtype=bool)
        if en_raw is not None:
            for k_, v_ in en_raw.items():
                if k_ not in residual.names:
                    raise ValueError(f"residual.enabled: unknown channel '{k_}'")
                enabled[residual.names.index(k_)] = bool(v_)

        pl = d.get("platform", {}) or {}
        known = PlatformSpec.__dataclass_fields__.keys()
        platform = PlatformSpec(**{k_: v_ for k_, v_ in pl.items() if k_ in known})

        fg_raw = d.get("filter_gap", {}) or {}
        fgk = FilterGapSpec.__dataclass_fields__.keys()
        filter_gap = FilterGapSpec(**{k_: float(v_) for k_, v_ in fg_raw.items() if k_ in fgk})
        if len(residual.units) != n or len(residual.names) != n:
            raise ValueError("residual names/units/sigma_nom lengths differ")

        return cls(
            rate_hz=float(d["rate_hz"]),
            L_wheelbase=float(d["L_wheelbase"]),
            residual=residual,
            mode=mode,
            imm=imm,
            action=action,
            override=override,
            window=window,
            flag=flag,
            topics=topics,
            channels_enabled=enabled,
            platform=platform,
            filter_gap=filter_gap,
        )

    # ------------------------------------------------------------------
    @classmethod
    def from_rosparam(cls, ns: str = "supervisor") -> "SupervisorConfig":
        """Read from the ROS parameter server (must already be loaded)."""
        import rospy   # lazy — the unit tests don't need rospy
        if not rospy.has_param(ns):
            raise RuntimeError(
                f"rosparam '{ns}' not found — was supervisor.yaml loaded?")
        return cls.from_dict(rospy.get_param(ns))

    # ------------------------------------------------------------------
    @classmethod
    def from_yaml(cls, path: str, *overlays: str) -> "SupervisorConfig":
        """Read from standalone YAML file(s) (for offline tests).

        ``overlays`` (e.g. config/platforms/gazebo.yaml) are deep-merged
        on top, exactly like successive ``rosparam load`` calls do.
        """
        import yaml

        def _load(p_):
            with open(p_, "r") as f:
                d_ = yaml.safe_load(f) or {}
            if "supervisor" in d_ and isinstance(d_["supervisor"], dict):
                d_ = d_["supervisor"]
            return d_

        d = _load(path)
        for ov in overlays:
            d = deep_merge(d, _load(ov))
        return cls.from_dict(d)


def deep_merge(a: dict, b: dict) -> dict:
    """Recursive dict merge (b wins), matching rosparam load semantics."""
    out = dict(a)
    for k, v in (b or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out
