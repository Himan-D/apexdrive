#pragma once

#include <cstdint>
#include <string_view>
#include <cmath>

namespace apexdrive {

/**
 * Clean Orthogonal Separation:
 * 1. OperatingMode: Commanded motion control mode.
 * 2. SafetyState: Hardware supervisor / safety interlock status.
 * 3. FaultFlag: 32-bit bitmask of active fault conditions.
 */
enum class OperatingMode : uint8_t {
    STANDBY = 0,
    CALIBRATING = 1,
    CLOSED_LOOP_TORQUE = 2,
    CLOSED_LOOP_VELOCITY = 3,
    CLOSED_LOOP_POSITION = 4,
    CLOSED_LOOP_IMPEDANCE = 5
};

enum class SafetyState : uint8_t {
    OK = 0,
    WARNING = 1,
    FAULT_STOP = 2,
    SAFE_TORQUE_OFF = 3
};

// Machine-readable 32-bit Fault Bitmask
namespace FaultFlag {
    inline constexpr uint32_t NONE               = 0;
    inline constexpr uint32_t OVERCURRENT_PHASE  = (1 << 0);
    inline constexpr uint32_t OVERVOLTAGE_BUS    = (1 << 1);
    inline constexpr uint32_t UNDERVOLTAGE_BUS   = (1 << 2);
    inline constexpr uint32_t OVERTEMP_WINDING   = (1 << 3);
    inline constexpr uint32_t OVERTEMP_MOSFET    = (1 << 4);
    inline constexpr uint32_t WATCHDOG_TIMEOUT   = (1 << 5);
    inline constexpr uint32_t ENCODER_CRC_ERROR  = (1 << 6);
    inline constexpr uint32_t COMMAND_OUT_OF_BOUNDS = (1 << 7);
    inline constexpr uint32_t GATE_DRIVER_FAULT  = (1 << 8);
}

[[nodiscard]] constexpr std::string_view ModeToString(OperatingMode mode) noexcept {
    switch (mode) {
        case OperatingMode::STANDBY:               return "STANDBY";
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
        case SafetyState::OK:              return "OK";
        case SafetyState::WARNING:         return "WARNING";
        case SafetyState::FAULT_STOP:      return "FAULT_STOP";
        case SafetyState::SAFE_TORQUE_OFF: return "SAFE_TORQUE_OFF";
        default:                           return "UNKNOWN";
    }
}

/**
 * Sensor Readings from Physical Hardware / ADC Shunts
 */
struct SensorReadings {
    float i_phase_a{0.0f};          // Phase A Current (A)
    float i_phase_b{0.0f};          // Phase B Current (A)
    float i_phase_c{0.0f};          // Phase C Current (A)
    float v_bus{48.0f};             // DC Bus Rail Voltage (V)
    float rotor_angle_rad{0.0f};    // Absolute Mechanical Rotor Angle [0, 2pi)
    float rotor_speed_rad_s{0.0f};  // Rotor Velocity (rad/s)
    float mosfet_temp_c{35.0f};     // Inverter MOSFET Temp (°C)
    float winding_temp_c{38.0f};    // Stator Winding Temp (°C)
};

/**
 * Telemetry Broadcast Frame
 */
struct JointTelemetry {
    uint8_t node_id{0};
    OperatingMode mode{OperatingMode::STANDBY};
    SafetyState safety_state{SafetyState::OK};
    float position_rad{0.0f};
    float velocity_rad_s{0.0f};
    float torque_nm{0.0f};
    float current_iq_a{0.0f};
    float current_id_a{0.0f};
    float v_bus_v{48.0f};
    float temperature_c{35.0f};
    uint32_t fault_flags{FaultFlag::NONE};
    uint64_t timestamp_us{0};
};

/**
 * 1 kHz Compliant Impedance Control Command
 */
struct ImpedanceCommand {
    float target_pos_rad{0.0f};        // Desired position (rad)
    float target_vel_rad_s{0.0f};      // Desired velocity feedforward (rad/s)
    float stiffness_kp{0.0f};          // Virtual Spring Stiffness (Nm/rad)
    float damping_kd{0.0f};            // Virtual Damper (Nm/(rad/s))
    float feedforward_torque_nm{0.0f}; // Feedforward Torque (Nm)

    [[nodiscard]] bool IsValid() const noexcept {
        return std::isfinite(target_pos_rad) &&
               std::isfinite(target_vel_rad_s) &&
               std::isfinite(stiffness_kp) && (stiffness_kp >= 0.0f) &&
               std::isfinite(damping_kd) && (damping_kd >= 0.0f) &&
               std::isfinite(feedforward_torque_nm);
    }
};

/**
 * Motor Electrical & Mechanical Parameters
 */
struct MotorProfile {
    float phase_resistance_ohm{0.182f};      // Phase Resistance Rs (Ohms)
    float inductance_d_h{0.000118f};         // Direct-axis Inductance Ld (H)
    float inductance_q_h{0.000135f};         // Quadrature-axis Inductance Lq (H) [Salient PMSM]
    float flux_linkage_wb{0.0068f};          // Permanent Magnet Flux Linkage (Wb)
    float torque_constant_kt{0.084f};        // Torque Constant Kt (Nm/A)
    float pole_pairs{7.0f};                  // Pole Pairs
    float rotor_inertia_kgm2{0.00045f};      // Rotor Inertia J (kg*m^2)
    float viscous_friction_b{0.0005f};       // Viscous Damping B (Nm*s/rad)
    float coulomb_friction_nm{0.045f};       // Coulomb Friction (Nm)
    float max_continuous_current_a{25.0f};   // Max Continuous RMS Current (A)
    float peak_current_a{45.0f};             // Absolute Peak Trip Current (A)
    float max_voltage_v{54.0f};              // Max Operating Bus Voltage (V)
    float min_voltage_uvlo_v{18.0f};         // Under-Voltage Lockout (V)
    float max_winding_temp_c{105.0f};        // Stator Thermal Cutoff (°C)
    float encoder_offset_rad{0.0f};          // Electrical Angle Zero Offset
    float command_timeout_sec{0.025f};       // Watchdog Timeout (25ms)
};

} // namespace apexdrive
