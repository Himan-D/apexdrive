#pragma once

#include <cstdint>
#include <string_view>

namespace apexdrive {

/**
 * Actuator Operational State Machine
 */
enum class DriveState : uint8_t {
    UNINITIALIZED = 0,
    STANDBY = 1,
    CALIBRATING = 2,
    ARMED = 3,
    CLOSED_LOOP_TORQUE = 4,
    CLOSED_LOOP_VELOCITY = 5,
    CLOSED_LOOP_POSITION = 6,
    CLOSED_LOOP_IMPEDANCE = 7,
    FAULT_OVERCURRENT = 8,
    FAULT_BROWNOUT = 9,
    FAULT_OVERTEMP = 10,
    FAULT_WATCHDOG_TIMEOUT = 11,
    FAULT_ENCODER_ERROR = 12
};

[[nodiscard]] constexpr std::string_view StateToString(DriveState state) noexcept {
    switch (state) {
        case DriveState::UNINITIALIZED:          return "UNINITIALIZED";
        case DriveState::STANDBY:                return "STANDBY";
        case DriveState::CALIBRATING:            return "CALIBRATING";
        case DriveState::ARMED:                  return "ARMED";
        case DriveState::CLOSED_LOOP_TORQUE:     return "CLOSED_LOOP_TORQUE";
        case DriveState::CLOSED_LOOP_VELOCITY:   return "CLOSED_LOOP_VELOCITY";
        case DriveState::CLOSED_LOOP_POSITION:   return "CLOSED_LOOP_POSITION";
        case DriveState::CLOSED_LOOP_IMPEDANCE:  return "CLOSED_LOOP_IMPEDANCE";
        case DriveState::FAULT_OVERCURRENT:      return "FAULT_OVERCURRENT";
        case DriveState::FAULT_BROWNOUT:         return "FAULT_BROWNOUT";
        case DriveState::FAULT_OVERTEMP:         return "FAULT_OVERTEMP";
        case DriveState::FAULT_WATCHDOG_TIMEOUT: return "FAULT_WATCHDOG_TIMEOUT";
        case DriveState::FAULT_ENCODER_ERROR:    return "FAULT_ENCODER_ERROR";
        default:                                 return "UNKNOWN";
    }
}

/**
 * Raw Hardware ADC & Sensor Samples (25 kHz Interrupt Data)
 */
struct SensorReadings {
    float i_phase_a{0.0f};       // Phase A Current (Amperes)
    float i_phase_b{0.0f};       // Phase B Current (Amperes)
    float i_phase_c{0.0f};       // Phase C Current (Amperes)
    float v_bus{24.0f};          // DC Bus Rail Voltage (Volts)
    float rotor_angle_rad{0.0f}; // Absolute Mechanical Rotor Angle [0, 2pi)
    float rotor_speed_rad_s{0.0f};// Filtered Rotor Velocity (rad/s)
    float mosfet_temp_c{35.0f};  // Inverter MOSFET Temperature (°C)
    float winding_temp_c{38.0f}; // Motor Stator Winding Temperature (°C)
};

/**
 * 1 kHz Synchronized Joint Telemetry Frame
 */
struct JointTelemetry {
    uint8_t node_id{0};
    DriveState state{DriveState::STANDBY};
    float position_rad{0.0f};
    float velocity_rad_s{0.0f};
    float torque_nm{0.0f};
    float current_iq_a{0.0f};
    float v_bus_v{0.0f};
    float temperature_c{0.0f};
    uint32_t fault_flags{0};
    uint32_t timestamp_us{0};
};

/**
 * High-Level Compliant Impedance & Force Command
 */
struct ImpedanceCommand {
    float target_pos_rad{0.0f};     // Desired equilibrium position (rad)
    float target_vel_rad_s{0.0f};   // Desired velocity feedforward (rad/s)
    float stiffness_kp{0.0f};       // Virtual Spring Stiffness (Nm/rad)
    float damping_kd{0.0f};         // Virtual Damper (Nm/(rad/s))
    float feedforward_torque_nm{0.0f}; // Feedforward torque e.g. gravity compensation (Nm)
};

/**
 * Calibrated Motor & Inverter Hardware Profile
 */
struct MotorProfile {
    float phase_resistance_ohm{0.182f};   // Measured Phase Resistance R (Ohms)
    float phase_inductance_h{0.000118f};  // Measured Phase Inductance L (Henries)
    float torque_constant_kt{0.084f};     // Motor Torque Constant Kt (Nm/A)
    float pole_pairs{7.0f};               // Magnetic Pole Pairs
    float max_continuous_current_a{25.0f};// Maximum Continuous Current (A)
    float peak_current_a{45.0f};          // Peak Inrush Trip Current (A)
    float max_voltage_v{52.0f};           // Maximum Operating Voltage (V)
    float min_voltage_uvlo_v{18.0f};      // Under-Voltage Lockout (V)
    float max_winding_temp_c{95.0f};      // Thermal Trip Limit (°C)
    float encoder_offset_rad{0.0f};       // Calibrated zero electrical angle offset
};

} // namespace apexdrive
