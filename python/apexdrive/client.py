"""
ApexDrive Python SDK
Client library for communicating with ApexDrive robotic actuators via SocketCAN or simulation.
"""

from dataclasses import dataclass
from typing import Optional
import math
import time

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
        self._kt = 0.084  # Nm/A
        self._j = 0.00045 # kg*m^2
        self._b = 0.0005  # Viscous damping
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

    def set_impedance(self, pos_rad: float, vel_rad_s: float = 0.0, 
                      kp: float = 40.0, kd: float = 2.0, tau_ff: float = 0.0) -> None:
        """
        Stream 1 kHz compliant impedance control command.
        Tau = Kp * (pos_target - pos) + Kd * (vel_target - vel) + tau_ff
        """
        if not (math.isfinite(pos_rad) and math.isfinite(kp) and kp >= 0.0 and math.isfinite(kd) and kd >= 0.0):
            self._safety = "FAULT_STOP"
            return

        if self._mode != "STANDBY" and self._safety == "OK":
            # Compliant virtual spring-damper
            torque_cmd = kp * (pos_rad - self._pos) + kd * (vel_rad_s - self._vel) + tau_ff
            target_iq = max(min(torque_cmd / self._kt, 40.0), -40.0)
            
            # Step internal physics (1ms step)
            dt = 0.001
            self._iq += (target_iq - self._iq) * (dt / 0.001)
            torque_em = self._iq * self._kt
            accel = (torque_em - self._b * self._vel) / self._j
            self._vel += accel * dt
            self._pos += self._vel * dt

    def get_state(self) -> ActuatorState:
        """Retrieve current telemetry snapshot."""
        now_us = int(time.time() * 1e6) - self._start_time_us
        torque_em = self._iq * self._kt
        return ActuatorState(
            node_id=self.node_id,
            mode=self._mode,
            safety_state=self._safety,
            position_rad=self._pos,
            velocity_rad_s=self._vel,
            torque_nm=torque_em,
            current_iq_a=self._iq,
            v_bus_v=self._v_bus,
            temperature_c=self._temp,
            fault_flags=0,
            timestamp_us=now_us
        )
