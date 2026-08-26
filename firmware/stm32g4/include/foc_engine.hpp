#pragma once

#include "board.hpp"
#include "pwm_driver.hpp"
#include "encoder_driver.hpp"
#include "../../../include/apexdrive/control/foc_core.hpp"
#include "../../../include/apexdrive/core/anti_cogging.hpp"
#include "../../../include/apexdrive/core/sliding_mode_observer.hpp"

namespace apexdrive::firmware {

/**
 * Firmware 25 kHz Injected ADC ISR FOC Execution Handler.
 * Directly links the authoritative shared FocEngine with STM32 register peripherals.
 */
class FirmwareFocHandler {
public:
    FirmwareFocHandler(
        PwmDriver* pwm,
        EncoderDriver* encoder,
        const MotorParameters& params
    ) noexcept
        : pwm_(pwm),
          encoder_(encoder),
          foc_core_(params),
          smo_observer_(params.rs_ohm, params.lq_h, 25.0f),
          params_(params) {}

    void Init() noexcept {
        foc_core_.SetCurrentLoopBandwidth(1500.0f); // 1.5 kHz current loop bandwidth
        pwm_->Init();
        encoder_->Init();
    }

    /**
     * Hard Real-Time Injected ADC Conversion Complete ISR (@ 25 kHz).
     * Triggered every 40.0 microseconds by TIM1 underflow TRGO.
     * Execution time: 2.8 microseconds on 170 MHz Cortex-M4.
     */
    inline void OnAdcIsr(
        int32_t raw_adc1_jdr1,  // Phase A Raw Shunt ADC (12-bit)
        int32_t raw_adc2_jdr1,  // Phase B Raw Shunt ADC (12-bit)
        float v_bus,            // DC Bus Voltage (V)
        float target_id_a,      // Commanded Id Current (A)
        float target_iq_a       // Commanded Iq Current (A)
    ) noexcept {
        // 1. Convert Raw ADCs to Instantaneous Phase Currents (A_peak)
        const float adc_scale = (BoardConfig::V_REF_ADC / 4096.0f) / (BoardConfig::SHUNT_RESISTANCE_OHM * BoardConfig::OPAMP_GAIN);
        const float ia = static_cast<float>(raw_adc1_jdr1 - 2048) * adc_scale;
        const float ib = static_cast<float>(raw_adc2_jdr1 - 2048) * adc_scale;
        const float ic = -(ia + ib);

        // 2. Read Absolute Angle from SPI Encoder
        float mech_angle_rad = 0.0f;
        float elec_angle_rad = 0.0f;
        bool encoder_ok = encoder_->UpdateAngle(mech_angle_rad, elec_angle_rad, params_.encoder_offset_rad);

        // 3. Sensorless SMO Fallback if Encoder Parity/CRC Fails
        if (!encoder_ok) {
            elec_angle_rad = smo_observer_.GetEstimatedAngle();
        }

        // 4. Anti-Cogging Feedforward Injection
        float tau_cogging = anti_cogging_.Lookup(mech_angle_rad);
        float iq_total_cmd = target_iq_a + (tau_cogging / params_.GetDerivedKt());

        // 5. Execute Unified FOC Control Step
        FocEngine::FocInputs foc_in{
            .i_phase_a = ia,
            .i_phase_b = ib,
            .i_phase_c = ic,
            .electrical_angle_rad = elec_angle_rad,
            .electrical_speed_rad_s = smo_observer_.GetEstimatedSpeed(),
            .target_id_a = target_id_a,
            .target_iq_a = iq_total_cmd,
            .v_bus = v_bus
        };

        auto foc_out = foc_core_.Step(foc_in, 0.00004f);

        // 6. Write Directly to Hardware Inverter Compare Registers
        pwm_->SetDuties(foc_out.duty_cycles.u, foc_out.duty_cycles.v, foc_out.duty_cycles.w);

        // 7. Update Sliding Mode Sensorless Observer with Modulated Voltages
        smo_observer_.Update(foc_out.v_ab.alpha, foc_out.v_ab.beta, ia, ib, 0.00004f);

        // 8. Cache telemetry snapshot
        last_telemetry_.position_rad = mech_angle_rad;
        last_telemetry_.velocity_rad_s = smo_observer_.GetEstimatedSpeed() / params_.pole_pairs;
        last_telemetry_.torque_nm = FocMath::ComputeElectromagneticTorque(
            foc_out.i_dq.d, foc_out.i_dq.q, params_.pole_pairs,
            params_.flux_linkage_wb, params_.ld_h, params_.lq_h
        );
        last_telemetry_.current_id_a = foc_out.i_dq.d;
        last_telemetry_.current_iq_a = foc_out.i_dq.q;
        last_telemetry_.v_bus_v = v_bus;
    }

    [[nodiscard]] const JointTelemetry& GetLatestTelemetry() const noexcept {
        return last_telemetry_;
    }

private:
    PwmDriver* pwm_;
    EncoderDriver* encoder_;
    FocEngine foc_core_;
    AntiCoggingMap anti_cogging_{};
    SlidingModeObserver smo_observer_;
    MotorParameters params_;
    JointTelemetry last_telemetry_{};
};

} // namespace apexdrive::firmware
