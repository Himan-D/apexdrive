#pragma once

#include <cmath>
#include <cstdint>
#include <string_view>
#include <algorithm>

namespace apexdrive {

namespace Units {
    static constexpr float SQRT2 = 1.4142135623730951f;
    static constexpr float ONE_BY_SQRT2 = 0.7071067811865475f;
    static constexpr float SQRT3 = 1.7320508075688772f;
    static constexpr float ONE_BY_SQRT3 = 0.5773502691896257f;

    [[nodiscard]] constexpr inline float PhasePeakToRms(float peak_a) noexcept {
        return peak_a * ONE_BY_SQRT2;
    }

    [[nodiscard]] constexpr inline float RmsToPhasePeak(float rms_a) noexcept {
        return rms_a * SQRT2;
    }

    [[nodiscard]] constexpr inline float MechanicalToElectricalAngle(float mech_rad, float pole_pairs) noexcept {
        return mech_rad * pole_pairs;
    }

    [[nodiscard]] constexpr inline float ElectricalToMechanicalAngle(float elec_rad, float pole_pairs) noexcept {
        return (pole_pairs > 0.0f) ? (elec_rad / pole_pairs) : 0.0f;
    }
}

/**
 * Authoritative Single-Source-of-Truth Motor Parameters.
 */
struct MotorParameters {
    // Primary Parameters
    float pole_pairs{7.0f};                 // Magnetic Pole Pairs (p)
    float flux_linkage_wb{0.0068f};          // Permanent Magnet Flux Linkage (psi_f, Wb)
    float rs_ohm{0.182f};                    // Phase Resistance Rs (Ohms)
    float ld_h{0.000118f};                   // Direct-axis Inductance Ld (H)
    float lq_h{0.000135f};                   // Quadrature-axis Inductance Lq (H)
    float rotor_inertia_kgm2{0.00045f};      // Rotor Inertia J (kg*m^2)
    float viscous_friction_b{0.0005f};       // Viscous Damping B (Nm*s/rad)
    float coulomb_friction_nm{0.045f};       // Coulomb Friction (Nm)

    // Aliases for full compatibility across modules
    float phase_resistance_ohm{0.182f};
    float phase_inductance_h{0.000118f};
    float inductance_d_h{0.000118f};
    float inductance_q_h{0.000135f};
    float torque_constant_kt{0.084f};

    // Hardware Inverter & Safe Limits
    float max_continuous_current_a{25.0f};  // Max continuous current (A_peak)
    float peak_current_a{45.0f};            // Absolute peak trip current (A_peak)
    float max_voltage_v{54.0f};             // Max bus voltage (V)
    float max_bus_voltage_v{54.0f};         // Max bus voltage alias (V)
    float min_voltage_uvlo_v{18.0f};        // Under-Voltage Lockout (V)
    float max_winding_temp_c{105.0f};       // Stator thermal cutoff (°C)
    float max_mechanical_speed_rad_s{250.0f}; // Overspeed cutoff (rad/s)
    float encoder_offset_rad{0.0f};         // Electrical zero offset (rad)
    float command_timeout_sec{0.025f};      // Watchdog timeout (25ms)

    [[nodiscard]] constexpr float GetDerivedKt() const noexcept {
        return 1.5f * pole_pairs * flux_linkage_wb;
    }

    [[nodiscard]] constexpr float GetDerivedKe() const noexcept {
        return (Units::SQRT3 * 0.5f) * pole_pairs * flux_linkage_wb;
    }

    [[nodiscard]] constexpr float GetBaseSpeed(float v_bus) const noexcept {
        const float denom = Units::SQRT3 * pole_pairs * flux_linkage_wb;
        return (denom > 1e-6f) ? (v_bus / denom) : 0.0f;
    }
};

enum class OperatingMode : uint8_t {
    STANDBY = 0,
    DISARMED = 0,
    SELF_TEST = 1,
    CALIBRATING = 2,
    CLOSED_LOOP_TORQUE = 3,
    CLOSED_LOOP_VELOCITY = 4,
    CLOSED_LOOP_POSITION = 5,
    CLOSED_LOOP_IMPEDANCE = 6
};

enum class SafetyState : uint8_t {
    BOOT = 0,
    OK = 1,
    READY = 1,
    ACTIVE = 2,
    WARNING = 3,
    FAULT_STOP = 4,
    SAFE_TORQUE_OFF = 5
};

namespace FaultFlag {
    inline constexpr uint32_t NONE                  = 0;
    inline constexpr uint32_t OVERCURRENT_PHASE     = (1 << 0);
    inline constexpr uint32_t OVERVOLTAGE_BUS       = (1 << 1);
    inline constexpr uint32_t UNDERVOLTAGE_BUS      = (1 << 2);
    inline constexpr uint32_t OVERTEMP_WINDING      = (1 << 3);
    inline constexpr uint32_t OVERTEMP_MOSFET       = (1 << 4);
    inline constexpr uint32_t WATCHDOG_TIMEOUT      = (1 << 5);
    inline constexpr uint32_t ENCODER_CRC_ERROR     = (1 << 6);
    inline constexpr uint32_t COMMAND_OUT_OF_BOUNDS = (1 << 7);
    inline constexpr uint32_t GATE_DRIVER_FAULT     = (1 << 8);
    inline constexpr uint32_t VOLTAGE_SATURATION    = (1 << 9);
}

} // namespace apexdrive
