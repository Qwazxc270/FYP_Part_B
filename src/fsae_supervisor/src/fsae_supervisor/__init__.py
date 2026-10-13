"""fsae_supervisor — model-based supervisor for an FSAE-DV vehicle.

Pure-Python (numpy) port of the MATLAB reference implementation in
supervisorConfig.m / computeResiduals.m / imm.m / actionPolicy.m.
ROS-agnostic on purpose: nodes in scripts/ do all I/O.
"""

from .config_loader import SupervisorConfig
from .ring_buffer   import RingBuffer
from .residuals     import (
    ResidualGenerator,
    SupervisorInputs,
    IMUData, EncoderData, ConeObs, LidarScan, EKFSnapshot,
    FaultFlagBag,
)
from .imm           import IMM
from .action_policy import ActionPolicy, SupervisorAction, FaultFlagInputs, ACTION_POLICY_VERSION

__all__ = [
    "SupervisorConfig",
    "RingBuffer",
    "ResidualGenerator",
    "SupervisorInputs", "IMUData", "EncoderData", "ConeObs", "LidarScan", "EKFSnapshot",
    "FaultFlagBag",
    "IMM",
    "ActionPolicy", "SupervisorAction", "FaultFlagInputs",
    "ACTION_POLICY_VERSION",
]
