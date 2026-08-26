"""
ApexDrive Python SDK
High-Bandwidth Compliant Actuator Interface & Hardware-Accelerated Simulation Core
"""

from .client import (
    Actuator, 
    JointState, 
    JointTelemetry, 
    ImpedanceCommand,
    compute_foc,
    compute_mtpa,
    step_joint_physics
)

__version__ = "1.3.0"
__all__ = [
    "Actuator", 
    "JointState", 
    "JointTelemetry", 
    "ImpedanceCommand",
    "compute_foc",
    "compute_mtpa",
    "step_joint_physics"
]
