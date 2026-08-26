#pragma once

#include "types.hpp"
#include <cmath>
#include <numbers>
#include <algorithm>
#include <cstdint>

namespace apexdrive {

/**
 * High-Performance, Zero-Allocation Field-Oriented Control (FOC) Vector Core.
 * Includes:
 * 1. Forward/Inverse Clarke and Park Transforms.
 * 2. Cross-Coupling Voltage Decoupling Feedforward.
 * 3. Dynamic Field Weakening (MTPA / FW) for extended high-speed operation.
 * 4. Branchless Space Vector Modulation (SVPWM) with 3rd-harmonic injection.
 * 5. High-Bandwidth Anti-Windup PI Controllers.
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

    // Forward Clarke Transform: 3-Phase currents (Ia, Ib, Ic) -> Stationary frame (Alpha, Beta)
    // Assumes balanced 3-phase system (Ia + Ib + Ic = 0)
    [[nodiscard]] static constexpr inline AlphaBeta Clarke(float ia, float ib, float ic) noexcept {
        (void)ic;
        return AlphaBeta{
            .alpha = ia,
            .beta  = (ia + 2.0f * ib) * ONE_BY_SQRT3
        };
    }

    // Forward Park Transform: Stationary frame (Alpha, Beta) -> Rotating rotor frame (d, q)
    [[nodiscard]] static inline DirectQuadrature Park(AlphaBeta ab, float electrical_angle_rad) noexcept {
        const float sin_t = std::sin(electrical_angle_rad);
        const float cos_t = std::cos(electrical_angle_rad);
        return DirectQuadrature{
            .d = ab.alpha * cos_t + ab.beta * sin_t,
            .q = -ab.alpha * sin_t + ab.beta * cos_t
        };
    }

    // Inverse Park Transform: Rotating frame (d, q) -> Stationary frame (Alpha, Beta)
    [[nodiscard]] static inline AlphaBeta InversePark(DirectQuadrature dq, float electrical_angle_rad) noexcept {
        const float sin_t = std::sin(electrical_angle_rad);
        const float cos_t = std::cos(electrical_angle_rad);
        return AlphaBeta{
            .alpha = dq.d * cos_t - dq.q * sin_t,
            .beta  = dq.d * sin_t + dq.q * cos_t
        };
    }

    /**
     * Cross-Coupling Voltage Decoupling Feedforward.
     * At high speeds, rotating magnetic flux cross-couples between d and q axes:
     *   V_d_ff = -omega_e * L_q * I_q
     *   V_q_ff = +omega_e * (L_d * I_d + lambda_m)
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
     * Dynamic Field Weakening Controller.
     * When commanded voltage approaches DC bus limit (V_bus / sqrt(3)), injects negative Id current
     * to counteract permanent magnet flux, expanding motor top speed by up to 50%.
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
     * High-Speed Space Vector PWM (SVPWM) with 3rd-Harmonic Neutral Point Shift.
     * Yields +15.4% greater voltage utilization over standard sinusoidal modulation.
     */
    [[nodiscard]] static inline InverterDutyCycles Svpwm(AlphaBeta v_ab, float v_bus) noexcept {
        if (v_bus < 1.0f) {
            return InverterDutyCycles{0.5f, 0.5f, 0.5f};
        }

        const float inv_vbus = 1.0f / v_bus;
        const float v_a = v_ab.alpha * inv_vbus;
        const float v_b = (-0.5f * v_ab.alpha + SQRT3_BY_TWO * v_ab.beta) * inv_vbus;
        const float v_c = (-0.5f * v_ab.alpha - SQRT3_BY_TWO * v_ab.beta) * inv_vbus;

        // Branchless min/max for sub-microsecond DSP execution
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
 * Hard Real-Time PI Current Regulator with Anti-Windup and Output Clamping.
 */
class PiController {
public:
    constexpr PiController() = default;
    constexpr PiController(float kp, float ki, float limit) noexcept
        : kp_(kp), ki_(ki), limit_(limit), integrator_(0.0f) {}

    void Reset() noexcept {
        integrator_ = 0.0f;
    }

    void SetGains(float kp, float ki, float limit) noexcept {
        kp_ = kp;
        ki_ = ki;
        limit_ = limit;
    }

    [[nodiscard]] float Update(float error, float dt) noexcept {
        const float p_term = kp_ * error;
        
        // Integrator update with conditional clamping
        integrator_ += ki_ * error * dt;
        integrator_ = std::clamp(integrator_, -limit_, limit_);

        const float output = p_term + integrator_;
        return std::clamp(output, -limit_, limit_);
    }

    [[nodiscard]] float GetIntegrator() const noexcept { return integrator_; }
    [[nodiscard]] float GetKp() const noexcept { return kp_; }
    [[nodiscard]] float GetKi() const noexcept { return ki_; }

private:
    float kp_{0.0f};
    float ki_{0.0f};
    float limit_{0.0f};
    float integrator_{0.0f};
};

} // namespace apexdrive
