#pragma once

#include "../core/types.hpp"
#include <cmath>
#include <algorithm>

namespace apexdrive {

/**
 * Salient Maximum Torque Per Ampere (MTPA) & Dynamic Field Weakening (FW) Optimizer.
 * 
 * Mathematical Formulation:
 * 1. For Interior Permanent Magnet Synchronous Motors (IPMSM) where L_q > L_d:
 *    I_d^* = \frac{\psi_f}{2(L_q - L_d)} - \sqrt{\frac{\psi_f^2}{4(L_q - L_d)^2} + (I_q^*)^2}
 * 2. For Surface PMSM (SPMSM) where L_d \approx L_q:
 *    I_d^* = 0
 * 3. Closed-Loop Field Weakening:
 *    Dynamically injects negative d-axis current when back-EMF approaches voltage limit:
 *    V_mag = \sqrt{V_d^2 + V_q^2} > V_max
 */
class MtpaOptimizer {
public:
    explicit MtpaOptimizer(const MotorParameters& params) noexcept
        : params_(params) {}

    struct OptimalCurrentVector {
        float target_id_a{0.0f};
        float target_iq_a{0.0f};
    };

    /**
     * Computes optimal (Id, Iq) current setpoints for commanded torque demand.
     */
    [[nodiscard]] OptimalCurrentVector ComputeOptimalCurrents(
        float commanded_torque_nm, 
        float v_bus_v, 
        float electrical_speed_rad_s,
        float dt_seconds
    ) noexcept {
        const float kt = params_.GetDerivedKt();
        if (kt <= 0.0f) return OptimalCurrentVector{};

        // 1. Initial Iq command from torque demand
        float iq_raw = commanded_torque_nm / kt;
        float iq_clamped = std::clamp(iq_raw, -params_.peak_current_a, params_.peak_current_a);

        // 2. Analytical MTPA Computation
        float id_mtpa = 0.0f;
        const float delta_l = params_.inductance_q_h - params_.inductance_d_h;

        // Check for machine saliency (Lq > Ld)
        if (delta_l > 1e-6f && params_.flux_linkage_wb > 0.0f) {
            float ratio = params_.flux_linkage_wb / (2.0f * delta_l);
            float sqrt_term = std::sqrt(ratio * ratio + (iq_clamped * iq_clamped));
            id_mtpa = ratio - sqrt_term; // Always negative d-axis reluctance current
        }

        // 3. Dynamic Closed-Loop Field Weakening
        float v_max = (v_bus_v * FocMath::ONE_BY_SQRT3) * 0.98f;
        float v_est_d = -electrical_speed_rad_s * params_.inductance_q_h * iq_clamped;
        float v_est_q = electrical_speed_rad_s * (params_.inductance_d_h * id_mtpa + params_.flux_linkage_wb);
        float v_mag_est = std::sqrt(v_est_d * v_est_d + v_est_q * v_est_q);

        if (v_mag_est > v_max) {
            float v_error = v_mag_est - v_max;
            fw_integral_ += fw_ki_ * v_error * dt_seconds;
            fw_integral_ = std::clamp(fw_integral_, 0.0f, params_.peak_current_a);
        } else {
            fw_integral_ -= fw_ki_ * (v_max - v_mag_est) * 0.5f * dt_seconds;
            fw_integral_ = std::max(fw_integral_, 0.0f);
        }

        float total_id = id_mtpa - fw_integral_;

        // 4. Stator Current Vector Magnitude Limiting: sqrt(Id^2 + Iq^2) <= I_max
        float i_mag = std::sqrt(total_id * total_id + iq_clamped * iq_clamped);
        if (i_mag > params_.peak_current_a && i_mag > 0.0f) {
            float scale = params_.peak_current_a / i_mag;
            total_id *= scale;
            iq_clamped *= scale;
        }

        return OptimalCurrentVector{
            .target_id_a = total_id,
            .target_iq_a = iq_clamped
        };
    }

    void Reset() noexcept {
        fw_integral_ = 0.0f;
    }

private:
    MotorParameters params_;
    float fw_ki_{45.0f};       // Field weakening integration gain
    float fw_integral_{0.0f};  // Accumulated negative d-axis demagnetizing current
};

} // namespace apexdrive
