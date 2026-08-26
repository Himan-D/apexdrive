#pragma once

#include "types.hpp"
#include <cmath>
#include <algorithm>

namespace apexdrive {

/**
 * Sensorless Sliding Mode Observer (SMO) & Phase-Locked Loop (PLL) Flux Estimator.
 * Continuously reconstructs back-EMF (e_alpha, e_beta) and tracks estimated rotor velocity and angle.
 */
class SlidingModeObserver {
public:
    SlidingModeObserver(float r_phase_ohm, float l_phase_h, float k_slide = 25.0f) noexcept
        : r_(r_phase_ohm), l_(l_phase_h), k_slide_(k_slide),
          i_alpha_est_(0.0f), i_beta_est_(0.0f),
          e_alpha_est_(0.0f), e_beta_est_(0.0f),
          theta_est_rad_(0.0f), speed_est_rad_s_(0.0f),
          pll_integrator_(0.0f) {}

    /**
     * Updates sliding mode observer and PLL tracking.
     * @param v_alpha Modulated stator alpha voltage (V)
     * @param v_beta  Modulated stator beta voltage (V)
     * @param i_alpha Measured stator alpha current (A)
     * @param i_beta  Measured stator beta current (A)
     * @param dt      Sample time (e.g. 0.00004s for 25kHz)
     */
    void Update(float v_alpha, float v_beta, float i_alpha, float i_beta, float dt) noexcept {
        if (l_ <= 1e-7f || dt <= 0.0f) return;

        // 1. Current Estimation Errors
        const float err_alpha = i_alpha_est_ - i_alpha;
        const float err_beta  = i_beta_est_ - i_beta;

        // 2. Continuous Sliding Surface Switching Function (Chatter-Free Hyperbolic Tangent)
        const float z_alpha = k_slide_ * std::tanh(err_alpha * 2.5f);
        const float z_beta  = k_slide_ * std::tanh(err_beta * 2.5f);

        // 3. Stator Current State Derivatives: dI/dt = -R/L * I + 1/L * (V - z)
        const float inv_l = 1.0f / l_;
        const float di_alpha = (-r_ * inv_l * i_alpha_est_) + inv_l * (v_alpha - z_alpha);
        const float di_beta  = (-r_ * inv_l * i_beta_est_)  + inv_l * (v_beta  - z_beta);

        i_alpha_est_ += di_alpha * dt;
        i_beta_est_  += di_beta * dt;

        // 4. Low-Pass Filter to extract continuous Back-EMF
        const float cutoff_w = 2.0f * 3.14159265f * 800.0f; // 800 Hz cutoff
        e_alpha_est_ += cutoff_w * (z_alpha - e_alpha_est_) * dt;
        e_beta_est_  += cutoff_w * (z_beta  - e_beta_est_)  * dt;

        // 5. Discrete Tracking PLL for Angle and Velocity Estimation
        if (std::abs(e_alpha_est_) > 0.05f || std::abs(e_beta_est_) > 0.05f) {
            // Raw back-EMF angle
            float raw_angle = std::atan2(-e_alpha_est_, e_beta_est_);
            if (raw_angle < 0.0f) raw_angle += 6.2831853f;

            // PLL Phase Detector Error
            float phase_err = raw_angle - theta_est_rad_;
            while (phase_err > 3.14159265f)  phase_err -= 6.2831853f;
            while (phase_err < -3.14159265f) phase_err += 6.2831853f;

            // PLL PI loop
            const float pll_kp = 180.0f;
            const float pll_ki = 4500.0f;
            pll_integrator_ += pll_ki * phase_err * dt;
            speed_est_rad_s_ = pll_kp * phase_err + pll_integrator_;
            theta_est_rad_ += speed_est_rad_s_ * dt;

            while (theta_est_rad_ >= 6.2831853f) theta_est_rad_ -= 6.2831853f;
            while (theta_est_rad_ < 0.0f)        theta_est_rad_ += 6.2831853f;
        }
    }

    [[nodiscard]] float GetEstimatedAngle() const noexcept { return theta_est_rad_; }
    [[nodiscard]] float GetEstimatedSpeed() const noexcept { return speed_est_rad_s_; }
    [[nodiscard]] float GetBackEmfAlpha() const noexcept { return e_alpha_est_; }
    [[nodiscard]] float GetBackEmfBeta() const noexcept { return e_beta_est_; }

private:
    float r_;
    float l_;
    float k_slide_;
    float i_alpha_est_;
    float i_beta_est_;
    float e_alpha_est_;
    float e_beta_est_;
    float theta_est_rad_;
    float speed_est_rad_s_;
    float pll_integrator_;
};

} // namespace apexdrive
