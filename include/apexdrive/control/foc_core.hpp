#pragma once

#include "../core/types.hpp"
#include "../core/foc_math.hpp"
#include <cmath>
#include <algorithm>

namespace apexdrive {

/**
 * Unified Hard Real-Time FOC Execution Engine.
 * Shared between host simulation, HIL testbench, and bare-metal STM32 firmware.
 */
class FocEngine {
public:
    struct FocInputs {
        float i_phase_a{0.0f};          // Phase A Current (A_peak)
        float i_phase_b{0.0f};          // Phase B Current (A_peak)
        float i_phase_c{0.0f};          // Phase C Current (A_peak)
        float electrical_angle_rad{0.0f};// Electrical Angle [0, 2pi)
        float electrical_speed_rad_s{0.0f};// Electrical Speed (rad/s)
        float target_id_a{0.0f};        // Target Direct Current (A)
        float target_iq_a{0.0f};        // Target Quadrature Current (A)
        float v_bus{48.0f};             // Measured DC Bus Voltage (V)
    };

    struct FocOutputs {
        float v_d_cmd{0.0f};            // Commanded Vd before saturation (V)
        float v_q_cmd{0.0f};            // Commanded Vq before saturation (V)
        float v_d_sat{0.0f};            // Saturated Vd applied to SVPWM (V)
        float v_q_sat{0.0f};            // Saturated Vq applied to SVPWM (V)
        float v_mag{0.0f};              // Voltage Vector Magnitude (V)
        float v_max_available{0.0f};    // Max Available Voltage Vector (V)
        bool is_saturated{false};       // True if voltage vector was clamped
        FocMath::AlphaBeta v_ab{};      // Stator Alpha-Beta Modulated Voltages
        FocMath::InverterDutyCycles duty_cycles{}; // SVPWM Compare Values [0.0, 1.0]
        FocMath::DirectQuadrature i_dq{}; // Measured Id, Iq currents
    };

    explicit FocEngine(const MotorParameters& params) noexcept
        : params_(params),
          pi_d_(params.rs_ohm * 9424.0f, params.ld_h * 9424.0f * 200.0f, params.max_bus_voltage_v),
          pi_q_(params.rs_ohm * 9424.0f, params.lq_h * 9424.0f * 200.0f, params.max_bus_voltage_v) {}

    void Reset() noexcept {
        pi_d_.Reset();
        pi_q_.Reset();
    }

    void SetCurrentLoopBandwidth(float bandwidth_hz) noexcept {
        const float omega_bw = 2.0f * 3.14159265f * bandwidth_hz;
        const float kp_d = params_.ld_h * omega_bw;
        const float ki_d = params_.rs_ohm * omega_bw;
        const float kp_q = params_.lq_h * omega_bw;
        const float ki_q = params_.rs_ohm * omega_bw;

        pi_d_.SetGains(kp_d, ki_d, params_.max_bus_voltage_v);
        pi_q_.SetGains(kp_q, ki_q, params_.max_bus_voltage_v);
    }

    /**
     * Executes single hard real-time 25 kHz FOC cycle.
     * Execution time: < 3.0 microseconds on Cortex-M4 / < 40ns on x86/ARM64.
     */
    [[nodiscard]] FocOutputs Step(const FocInputs& in, float dt) noexcept {
        FocOutputs out;

        // 1. Forward Clarke Transform (3-Phase -> Stationary Alpha/Beta)
        auto i_ab = FocMath::Clarke(in.i_phase_a, in.i_phase_b, in.i_phase_c);

        // 2. Forward Park Transform (Stationary Alpha/Beta -> Rotating DQ Frame)
        out.i_dq = FocMath::Park(i_ab, in.electrical_angle_rad);

        // 3. Current Error Computation
        float err_d = in.target_id_a - out.i_dq.d;
        float err_q = in.target_iq_a - out.i_dq.q;

        // 4. PI Controllers (Unsaturated command)
        float v_d_pi = pi_d_.Update(err_d, dt);
        float v_q_pi = pi_q_.Update(err_q, dt);

        // 5. Cross-Coupling Decoupling Feedforward
        auto v_dq_decoupled = FocMath::DecoupleCrossCoupling(
            {v_d_pi, v_q_pi}, out.i_dq, in.electrical_speed_rad_s,
            params_.ld_h, params_.lq_h, params_.flux_linkage_wb
        );

        out.v_d_cmd = v_dq_decoupled.d;
        out.v_q_cmd = v_dq_decoupled.q;

        // 6. Vector-Space Voltage Limiter (SVPWM Inscription Circle)
        // Vmax = Vbus / sqrt(3) * 0.98 (Modulation index ceiling)
        out.v_max_available = in.v_bus * FocMath::ONE_BY_SQRT3 * 0.98f;
        out.v_mag = std::sqrt(out.v_d_cmd * out.v_d_cmd + out.v_q_cmd * out.v_q_cmd);

        if (out.v_mag > out.v_max_available && out.v_mag > 1e-4f) {
            out.is_saturated = true;
            float scale = out.v_max_available / out.v_mag;
            out.v_d_sat = out.v_d_cmd * scale;
            out.v_q_sat = out.v_q_cmd * scale;
        } else {
            out.is_saturated = false;
            out.v_d_sat = out.v_d_cmd;
            out.v_q_sat = out.v_q_cmd;
        }

        // 7. Inverse Park Transform (DQ -> Alpha/Beta)
        out.v_ab = FocMath::InversePark({out.v_d_sat, out.v_q_sat}, in.electrical_angle_rad);

        // 8. Space Vector PWM Duty Generation
        out.duty_cycles = FocMath::Svpwm(out.v_ab, in.v_bus);

        return out;
    }

    [[nodiscard]] const MotorParameters& GetParameters() const noexcept { return params_; }

private:
    MotorParameters params_;
    PiController pi_d_;
    PiController pi_q_;
};

} // namespace apexdrive
