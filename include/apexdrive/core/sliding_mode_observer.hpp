#pragma once

#include "types.hpp"
#include <cmath>
#include <algorithm>

namespace apexdrive {

/**
 * Sensorless Sliding Mode Observer (SMO) & Flux Estimator.
 * Acts as a redundant safety observer to cross-validate magnetic encoder readings
 * and provide emergency sensorless commutation if an encoder wire snaps.
 */
class SlidingModeObserver {
public:
    SlidingModeObserver(float r_phase_ohm, float l_phase_h, float k_slide = 25.0f) noexcept
        : r_(r_phase_ohm), l_(l_phase_h), k_slide_(k_slide),
          i_alpha_est_(0.0f), i_beta_est_(0.0f),
          e_alpha_est_(0.0f), e_beta_est_(0.0f),
          theta_est_rad_(0.0f), speed_est_rad_s_(0.0f) {}

    /**
     * Updates sliding mode current and back-EMF observer step.
     * @param v_alpha Commanded alpha voltage (V)
     * @param v_beta Commanded beta voltage (V)
     * @param i_alpha Measured alpha current (A)
     * @param i_beta Measured beta current (A)
     * @param dt Sampling period in seconds (e.g. 0.00004 for 25kHz)
     */
    void Update(float v_alpha, float v_beta, float i_alpha, float i_beta, float dt) noexcept {
        if (l_ <= 1e-7f) return;

        // Current estimation errors
        const float err_alpha = i_alpha_est_ - i_alpha;
        const float err_beta  = i_beta_est_ - i_beta;

        // Sliding surface switching function (Hyperbolic tangent approximation for chattering reduction)
        const float z_alpha = k_slide_ * std::tanh(err_alpha * 2.5f);
        const float z_beta  = k_slide_ * std::tanh(err_beta * 2.5f);

        // State derivatives for estimated currents:
        // dI_est/dt = -R/L * I_est + 1/L * (V - z)
        const float inv_l = 1.0f / l_;
        const float di_alpha = (-r_ * inv_l * i_alpha_est_) + inv_l * (v_alpha - z_alpha);
        const float di_beta  = (-r_ * inv_l * i_beta_est_)  + inv_l * (v_beta  - z_beta);

        i_alpha_est_ += di_alpha * dt;
        i_beta_est_  += di_beta * dt;

        // Low-pass filter the switching functions to extract continuous back-EMF:
        const float cutoff_w = 2.0f * 3.14159265f * 800.0f; // 800 Hz filter
        e_alpha_est_ += cutoff_w * (z_alpha - e_alpha_est_) * dt;
        e_beta_est_  += cutoff_w * (z_beta  - e_beta_est_)  * dt;

        // Angle estimation from back-EMF: theta_e = -atan2(e_alpha, e_beta)
        if (std::abs(e_alpha_est_) > 0.1f || std::abs(e_beta_est_) > 0.1f) {
            theta_est_rad_ = std::atan2(-e_alpha_est_, e_beta_est_);
            if (theta_est_rad_ < 0.0f) theta_est_rad_ += 6.2831853f;
        }
    }

    [[nodiscard]] float GetEstimatedAngle() const noexcept { return theta_est_rad_; }
    [[nodiscard]] float GetBackEmfAlpha() const noexcept { return e_alpha_est_; }
    [[nodiscard]] float GetBackEmfBeta() const noexcept { return e_beta_est_; }
    [[nodiscard]] float GetEstimatedSpeed() const noexcept { return speed_est_rad_s_; }

private:
    float r_;
    float l_;
    float k_slide_;
    float i_alpha_est_;
    float i_beta_est_;
    float e_alpha_est_;
    float e_beta_est_;
    float theta_est_rad_;
    [[maybe_unused]] float speed_est_rad_s_;
};

} // namespace apexdrive
