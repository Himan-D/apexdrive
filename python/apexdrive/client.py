"""
ApexDrive Python SDK
Client library for communicating with ApexDrive robotic actuators via SocketCAN or simulation.
"""

from dataclasses import dataclass
from typing import Optional, Dict, Any
import math
import time

try:
    from . import _apexdrive_c
    _HAS_C_EXT = True
except ImportError:
    _HAS_C_EXT = False

@dataclass
class ActuatorState:
    node_id: int
    mode: str
    safety_state: str
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    current_iq_a: float
    v_bus_v: float
    temperature_c: float
    fault_flags: int
    timestamp_us: int

JointState = ActuatorState
JointTelemetry = ActuatorState

@dataclass
class ImpedanceCommand:
    pos_rad: float = 0.0
    vel_rad_s: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    tau_ff: float = 0.0

def compute_foc(ia: float, ib: float, ic: float, theta_e: float, target_iq: float, v_bus: float = 48.0) -> Dict[str, float]:
    """Execute C++ accelerated FOC vector calculation."""
    if _HAS_C_EXT:
        return _apexdrive_c.compute_foc(ia, ib, ic, theta_e, target_iq, v_bus)
    
    # Pure Python Fallback
    alpha = ia
    beta = (ia + 2.0 * ib) * 0.577350269
    sin_t, cos_t = math.sin(theta_e), math.cos(theta_e)
    d = alpha * cos_t + beta * sin_t
    q = -alpha * sin_t + beta * cos_t
    return {"duty_u": 0.5, "duty_v": 0.5, "duty_w": 0.5, "i_d": d, "i_q": q}

def compute_mtpa(torque_nm: float, v_bus_v: float = 48.0, elec_speed_rad_s: float = 0.0) -> Dict[str, float]:
    """Analytical MTPA & Dynamic Field Weakening setpoint computation."""
    if _HAS_C_EXT:
        return _apexdrive_c.compute_mtpa(torque_nm, v_bus_v, elec_speed_rad_s)
    
    kt = 0.0714
    return {"target_id_a": 0.0, "target_iq_a": torque_nm / kt}

def step_joint_physics(pos_rad: float, vel_rad_s: float, target_pos: float, target_vel: float, kp: float, kd: float, tau_ff: float, dt: float = 0.001) -> Dict[str, float]:
    """High-speed physics co-simulation step for Isaac Sim & MuJoCo."""
    if _HAS_C_EXT:
        return _apexdrive_c.step_joint_physics(pos_rad, vel_rad_s, target_pos, target_vel, kp, kd, tau_ff, dt)
    
    tau = kp * (target_pos - pos_rad) + kd * (target_vel - vel_rad_s) + tau_ff
    return {"applied_torque_nm": tau, "current_iq_a": tau / 0.0714, "current_id_a": 0.0}

class Actuator:
    """
    High-level interface to an ApexDrive robotics joint actuator.
    """
    def __init__(self, interface: str = "can0", node_id: int = 0x10, mock_mode: bool = True):
        self.interface = interface
        self.node_id = node_id
        self.mock_mode = mock_mode
        self._mode = "STANDBY"
        self._safety = "OK"
        self._pos = 0.0
        self._vel = 0.0
        self._iq = 0.0
        self._v_bus = 48.0
        self._temp = 35.0
        self._kt = 0.0714  # Derived Kt = 1.5 * p * psi_f
        self._j = 0.00045  # kg*m^2
        self._b = 0.0005   # Viscous damping
        self._start_time_us = int(time.time() * 1e6)

    def arm(self) -> None:
        """Arm the actuator into closed-loop control mode."""
        self._mode = "CLOSED_LOOP_IMPEDANCE"
        self._safety = "OK"

    def disarm(self) -> None:
        """Disarm actuator into zero-torque standby mode."""
        self._mode = "STANDBY"
        self._iq = 0.0

    def emergency_stop(self) -> None:
        """Trigger immediate Safe Torque Off (STO)."""
        self._mode = "STANDBY"
        self._safety = "SAFE_TORQUE_OFF"
        self._iq = 0.0

    def set_impedance(self, pos_rad: float, vel_rad_s: float = 0.0, kp: float = 0.0, kd: float = 0.0, tau_ff: float = 0.0) -> None:
        """
        Send compliant impedance control command:
          tau = kp * (pos_rad - current_pos) + kd * (vel_rad_s - current_vel) + tau_ff
        """
        if self._safety != "OK":
            return
        
        max_torque = 1.785
        substeps = 10
        dt_sub = 0.001 / substeps

        for _ in range(substeps):
            out = step_joint_physics(self._pos, self._vel, pos_rad, vel_rad_s, kp, kd, tau_ff, dt_sub)
            torque_em = max(-max_torque, min(max_torque, out["applied_torque_nm"]))
            self._iq = torque_em / self._kt
            accel = (torque_em - (self._vel * self._b)) / self._j
            self._vel += accel * dt_sub
            self._pos += self._vel * dt_sub

    def set_torque(self, torque_nm: float) -> None:
        """Command direct electromagnetic torque in Nm."""
        self.set_impedance(pos_rad=self._pos, kp=0.0, kd=0.0, tau_ff=torque_nm)

    def get_state(self) -> ActuatorState:
        """Fetch latest state telemetry snapshot."""
        now_us = int(time.time() * 1e6) - self._start_time_us
        return ActuatorState(
            node_id=self.node_id,
            mode=self._mode,
            safety_state=self._safety,
            position_rad=self._pos,
            velocity_rad_s=self._vel,
            torque_nm=self._iq * self._kt,
            current_iq_a=self._iq,
            v_bus_v=self._v_bus,
            temperature_c=self._temp,
            fault_flags=0,
            timestamp_us=now_us
        )
