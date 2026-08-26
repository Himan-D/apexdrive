#pragma once

#include "motor_model.hpp"
#include <cstdint>
#include <string_view>
#include <cmath>

namespace apexdrive {

using MotorProfile = MotorParameters;

[[nodiscard]] constexpr std::string_view ModeToString(OperatingMode mode) noexcept {
    switch (mode) {
        case OperatingMode::DISARMED:              return "DISARMED";
        case OperatingMode::SELF_TEST:             return "SELF_TEST";
        case OperatingMode::CALIBRATING:           return "CALIBRATING";
        case OperatingMode::CLOSED_LOOP_TORQUE:    return "CLOSED_LOOP_TORQUE";
        case OperatingMode::CLOSED_LOOP_VELOCITY:  return "CLOSED_LOOP_VELOCITY";
        case OperatingMode::CLOSED_LOOP_POSITION:  return "CLOSED_LOOP_POSITION";
        case OperatingMode::CLOSED_LOOP_IMPEDANCE: return "CLOSED_LOOP_IMPEDANCE";
        default:                                   return "UNKNOWN";
    }
}

[[nodiscard]] constexpr std::string_view SafetyStateToString(SafetyState state) noexcept {
    switch (state) {
        case SafetyState::BOOT:            return "BOOT";
        case SafetyState::READY:           return "READY";
        case SafetyState::ACTIVE:          return "ACTIVE";
        case SafetyState::WARNING:         return "WARNING";
        case SafetyState::FAULT_STOP:      return "FAULT_STOP";
        case SafetyState::SAFE_TORQUE_OFF: return "SAFE_TORQUE_OFF";
        default:                           return "UNKNOWN";
    }
}

struct SensorReadings {
    float i_phase_a{0.0f};          // Phase A Current (A_peak)
    float i_phase_b{0.0f};          // Phase B Current (A_peak)
    float i_phase_c{0.0f};          // Phase C Current (A_peak)
    float v_bus{48.0f};             // DC Bus Rail Voltage (V)
    float rotor_angle_rad{0.0f};    // Absolute Mechanical Rotor Angle [0, 2pi)
    float rotor_speed_rad_s{0.0f};  // Filtered Mechanical Rotor Velocity (rad/s)
    float mosfet_temp_c{35.0f};     // Inverter MOSFET Temp (°C)
    float winding_temp_c{38.0f};    // Stator Winding Temp (°C)
};

struct JointTelemetry {
    uint8_t node_id{0};
    OperatingMode mode{OperatingMode::DISARMED};
    SafetyState safety_state{SafetyState::READY};
    float position_rad{0.0f};
    float velocity_rad_s{0.0f};
    float torque_nm{0.0f};
    float current_iq_a{0.0f};
    float current_id_a{0.0f};
    float v_bus_v{48.0f};
    float temperature_c{35.0f};
    uint32_t fault_flags{FaultFlag::NONE};
    uint64_t timestamp_us{0};
    uint16_t sequence_number{0};
};

struct ImpedanceCommand {
    float target_pos_rad{0.0f};        // Target position (rad)
    float target_vel_rad_s{0.0f};      // Target velocity (rad/s)
    float stiffness_kp{0.0f};          // Stiffness (Nm/rad)
    float damping_kd{0.0f};            // Damping (Nm*s/rad)
    float feedforward_torque_nm{0.0f}; // Feedforward Torque (Nm)

    [[nodiscard]] bool IsValid() const noexcept {
        return std::isfinite(target_pos_rad) &&
               std::isfinite(target_vel_rad_s) &&
               std::isfinite(stiffness_kp) && (stiffness_kp >= 0.0f) &&
               std::isfinite(damping_kd) && (damping_kd >= 0.0f) &&
               std::isfinite(feedforward_torque_nm);
    }
};

} // namespace apexdrive
