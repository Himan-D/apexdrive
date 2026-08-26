#pragma once

#include "types.hpp"
#include <cmath>
#include <numbers>
#include <algorithm>
#include <cstdint>

namespace apexdrive {

/**
 * Enterprise FOC Vector Core:
 * - Forward/Inverse Clarke and Park Transforms.
 * - Back-Calculation Anti-Windup Current PI Regulators.
 * - Salient PMSM Reluctance + Permanent Magnet Torque Calculation.
 * - Decoupled Cross-Coupling Voltage Feedforward.
 * - Dynamic Field Weakening (FW) closed-loop voltage controller.
 * - Space Vector PWM (SVPWM) with center-aligned common-mode injection.
 */
class FocMath {
public:
    static constexpr float SQRT3 = 1.7320508075688772f;
    static constexpr float ONE_BY_SQRT3 = 0.5773502691896257f;
    static constexpr float TWO_BY_THREE = 0.6666666666666666f;
    static constexpr float SQRT3_BY_TWO = 0.8660254037844386f;

    struct AlphaBeta {
        float alpha{0.0f};
        float beta{0.0f};
    };

    struct DirectQuadrature {
        float d{0.0f};
        float q{0.0f};
    };

    struct InverterDutyCycles {
        float u{0.5f};
        float v{0.5f};
        float w{0.5f};
    };

    // Forward Clarke: 3-Phase currents -> Stationary Alpha-Beta Frame
    [[nodiscard]] static constexpr inline AlphaBeta Clarke(float ia, float ib, float ic) noexcept {
        (void)ic; // Assumes balanced system ia + ib + ic = 0
        return AlphaBeta{
            .alpha = ia,
            .beta  = (ia + 2.0f * ib) * ONE_BY_SQRT3
        };
    }

    // Forward Park: Stationary Alpha-Beta -> Rotating Rotor DQ Frame
    [[nodiscard]] static inline DirectQuadrature Park(AlphaBeta ab, float electrical_angle_rad) noexcept {
        const float sin_t = std::sin(electrical_angle_rad);
        const float cos_t = std::cos(electrical_angle_rad);
        return DirectQuadrature{
            .d = ab.alpha * cos_t + ab.beta * sin_t,
            .q = -ab.alpha * sin_t + ab.beta * cos_t
        };
    }

    // Inverse Park: Rotating DQ Frame -> Stationary Alpha-Beta Frame
    [[nodiscard]] static inline AlphaBeta InversePark(DirectQuadrature dq, float electrical_angle_rad) noexcept {
        const float sin_t = std::sin(electrical_angle_rad);
        const float cos_t = std::cos(electrical_angle_rad);
        return AlphaBeta{
            .alpha = dq.d * cos_t - dq.q * sin_t,
            .beta  = dq.d * sin_t + dq.q * cos_t
        };
    }

    /**
     * Cross-Coupling Voltage Decoupling Feedforward:
     *   V_d_ff = -omega_e * L_q * I_q
     *   V_q_ff = +omega_e * (L_d * I_d + psi_f)
     */
    [[nodiscard]] static inline DirectQuadrature DecoupleCrossCoupling(
        DirectQuadrature v_pi_cmd,
        DirectQuadrature i_meas,
        float omega_e_rad_s,
        float L_d_h,
        float L_q_h,
        float flux_linkage_wb
    ) noexcept {
        const float v_d_decouple = -omega_e_rad_s * L_q_h * i_meas.q;
        const float v_q_decouple = omega_e_rad_s * (L_d_h * i_meas.d + flux_linkage_wb);

        return DirectQuadrature{
            .d = v_pi_cmd.d + v_d_decouple,
            .q = v_pi_cmd.q + v_q_decouple
        };
    }

    /**
     * PMSM Electromagnetic Torque Equation (Permanent Magnet + Reluctance Torque):
     *   Te = 1.5 * p * [ psi_f * i_q + (L_d - L_q) * i_d * i_q ]
     */
    [[nodiscard]] static inline float ComputeElectromagneticTorque(
        float id, float iq, float pole_pairs, float flux_linkage_wb, float ld_h, float lq_h
    ) noexcept {
        const float pm_torque = flux_linkage_wb * iq;
        const float reluctance_torque = (ld_h - lq_h) * id * iq;
        return 1.5f * pole_pairs * (pm_torque + reluctance_torque);
    }

    /**
     * Closed-Loop Field Weakening Controller:
     * Injects demagnetizing negative Id current when voltage vector magnitude exceeds bus limit.
     */
    [[nodiscard]] static inline float ComputeFieldWeakeningId(
        float v_cmd_mag,
        float v_bus,
        float max_fw_current_a,
        float fw_gain = 0.5f
    ) noexcept {
        const float max_voltage = v_bus * ONE_BY_SQRT3 * 0.95f; // 95% modulation index ceiling
        if (v_cmd_mag > max_voltage) {
            const float v_error = v_cmd_mag - max_voltage;
            const float id_demag = -fw_gain * v_error;
            return std::clamp(id_demag, -max_fw_current_a, 0.0f);
        }
        return 0.0f;
    }

    /**
     * Center-Aligned Space Vector PWM (SVPWM) with Min/Max Common-Mode Injection:
     * Yields +15.4% greater bus voltage utilization over pure sinusoidal PWM.
     */
    [[nodiscard]] static inline InverterDutyCycles Svpwm(AlphaBeta v_ab, float v_bus) noexcept {
        if (v_bus < 1.0f) {
            return InverterDutyCycles{0.5f, 0.5f, 0.5f};
        }

        const float inv_vbus = 1.0f / v_bus;
        const float v_a = v_ab.alpha * inv_vbus;
        const float v_b = (-0.5f * v_ab.alpha + SQRT3_BY_TWO * v_ab.beta) * inv_vbus;
        const float v_c = (-0.5f * v_ab.alpha - SQRT3_BY_TWO * v_ab.beta) * inv_vbus;

        const float v_max = std::max({v_a, v_b, v_c});
        const float v_min = std::min({v_a, v_b, v_c});
        const float v_com = 0.5f * (v_max + v_min);

        return InverterDutyCycles{
            .u = std::clamp(0.5f + (v_a - v_com), 0.0f, 1.0f),
            .v = std::clamp(0.5f + (v_b - v_com), 0.0f, 1.0f),
            .w = std::clamp(0.5f + (v_c - v_com), 0.0f, 1.0f)
        };
    }
};

/**
 * Industrial PI Controller with Back-Calculation Anti-Windup and Output Saturation.
 */
class PiController {
public:
    constexpr PiController() = default;
    constexpr PiController(float kp, float ki, float limit, float kaw = -1.0f) noexcept
        : kp_(kp), ki_(ki), limit_(limit), 
          kaw_(kaw > 0.0f ? kaw : (kp > 1e-4f ? 1.0f / kp : 1.0f)), 
          integrator_(0.0f) {}

    void Reset() noexcept {
        integrator_ = 0.0f;
    }

    void SetGains(float kp, float ki, float limit, float kaw = -1.0f) noexcept {
        kp_ = kp;
        ki_ = ki;
        limit_ = limit;
        kaw_ = (kaw > 0.0f ? kaw : (kp > 1e-4f ? 1.0f / kp : 1.0f));
    }

    [[nodiscard]] float Update(float error, float dt) noexcept {
        const float p_term = kp_ * error;
        const float u_unsat = p_term + integrator_;
        const float u_sat = std::clamp(u_unsat, -limit_, limit_);

        // Back-calculation anti-windup:
        // dI/dt = Ki * error + Kaw * (u_sat - u_unsat)
        const float windup_diff = u_sat - u_unsat;
        integrator_ += (ki_ * error + kaw_ * windup_diff) * dt;
        integrator_ = std::clamp(integrator_, -limit_, limit_);

        return u_sat;
    }

    [[nodiscard]] float GetIntegrator() const noexcept { return integrator_; }
    [[nodiscard]] float GetKp() const noexcept { return kp_; }
    [[nodiscard]] float GetKi() const noexcept { return ki_; }

private:
    float kp_{0.0f};
    float ki_{0.0f};
    float limit_{0.0f};
    float kaw_{1.0f};
    float integrator_{0.0f};
};

} // namespace apexdrive
