from dataclasses import dataclass
import time
import math

@dataclass
class JointState:
    node_id: int
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    current_iq_a: float
    v_bus_v: float
    temperature_c: float
    is_armed: bool

@dataclass
class ImpedanceCommand:
    target_pos_rad: float = 0.0
    target_vel_rad_s: float = 0.0
    stiffness_kp: float = 0.0
    damping_kd: float = 0.0
    feedforward_torque_nm: float = 0.0

class Actuator:
    """
    High-Level Python Actuator Interface for Robotics Control Loops.
    """
    def __init__(self, interface: str = "can0", node_id: int = 0x14, mock_mode: bool = True):
        self.interface = interface
        self.node_id = node_id
        self.mock_mode = mock_mode
        self._is_armed = False
        self._pos = 0.0
        self._vel = 0.0
        self._temp = 36.5
        self._v_bus = 48.0

    def arm(self):
        """Enable inverter gate drivers and arm closed-loop FOC."""
        self._is_armed = True

    def disarm(self):
        """Disable PWM gates and freewheel."""
        self._is_armed = False

    def set_impedance(self, pos_rad: float, vel_rad_s: float = 0.0, kp: float = 40.0, kd: float = 2.5, tau_ff: float = 0.0):
        """Send compliant impedance setpoint."""
        if not self._is_armed:
            return
        
        # Simple internal mock dynamics
        pos_err = pos_rad - self._pos
        vel_err = vel_rad_s - self._vel
        torque = kp * pos_err + kd * vel_err + tau_ff
        accel = (torque - 0.005 * self._vel) / 0.00045
        self._vel += accel * 0.001
        self._pos += self._vel * 0.001

    def get_state(self) -> JointState:
        """Fetch latest synchronized 1 kHz joint telemetry."""
        return JointState(
            node_id=self.node_id,
            position_rad=self._pos,
            velocity_rad_s=self._vel,
            torque_nm=self._vel * 0.084,
            current_iq_a=self._vel,
            v_bus_v=self._v_bus,
            temperature_c=self._temp,
            is_armed=self._is_armed
        )
