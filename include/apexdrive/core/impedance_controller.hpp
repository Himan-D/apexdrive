#pragma once

#include "types.hpp"
#include <cmath>
#include <algorithm>

namespace apexdrive {

/**
 * Compliant Impedance & Force Controller.
 * Enables robots (humanoids/quadrupeds/arms) to behave like virtual springs and dampers.
 */
class ImpedanceController {
public:
    constexpr ImpedanceController() = default;

    /**
     * Computes the commanded quadrature current Iq based on impedance parameters.
     * @param cmd High-level impedance command (position, velocity, Kp, Kd, feedforward)
     * @param current_pos_rad Current measured joint angle in radians
     * @param current_vel_rad_s Current measured joint velocity in rad/s
     * @param kt Motor torque constant (Nm/A)
     * @param max_current_a Hardware current clamp limit
     * @return Commanded target current Iq in Amperes
     */
    [[nodiscard]] static inline float ComputeTorqueCurrent(
        const ImpedanceCommand& cmd,
        float current_pos_rad,
        float current_vel_rad_s,
        float kt,
        float max_current_a
    ) noexcept {
        if (kt <= 1e-6f) return 0.0f;

        // 1. Virtual Spring-Damper Impedance Law:
        // Tau = Kp * (pos_target - pos_actual) + Kd * (vel_target - vel_actual) + Tau_feedforward
        const float pos_error = cmd.target_pos_rad - current_pos_rad;
        const float vel_error = cmd.target_vel_rad_s - current_vel_rad_s;

        const float spring_torque = cmd.stiffness_kp * pos_error;
        const float damper_torque = cmd.damping_kd * vel_error;
        const float total_torque_nm = spring_torque + damper_torque + cmd.feedforward_torque_nm;

        // 2. Torque to Quadrature Current: Iq = Tau / Kt
        const float target_iq = total_torque_nm / kt;

        // 3. Hardware Saturation Clamping
        return std::clamp(target_iq, -max_current_a, max_current_a);
    }
};

} // namespace apexdrive
