#pragma once

#include "board.hpp"
#include <cstdint>

namespace apexdrive::firmware {

/**
 * STM32G4 Injected Dual-ADC Current Sense Driver.
 * Performs startup zero-current offset calibration by averaging 1024 samples.
 */
class AdcDriver {
public:
    struct AdcRegisters {
        volatile uint32_t ISR;
        volatile uint32_t IER;
        volatile uint32_t CR;
        volatile uint32_t CFGR;
        volatile uint32_t CFGR2;
        volatile uint32_t SMPR1;
        volatile uint32_t SMPR2;
        volatile uint32_t JSQR;
        volatile uint32_t JDR1; // Injected Channel 1 Result (Phase Current)
        volatile uint32_t JDR2; // Injected Channel 2 Result (Bus Voltage)
    };

    AdcDriver(AdcRegisters* adc1, AdcRegisters* adc2) noexcept
        : adc1_(adc1), adc2_(adc2) {}

    void Init() noexcept {
        if (!adc1_ || !adc2_) return;
        // Injected trigger from TIM1 TRGO (Update/Underflow Event)
        // Auto-zero offset will be measured during CalibrateOffsets()
    }

    /**
     * Measures zero-current ADC offsets before inverter arming.
     * Takes 1024 raw samples and computes exact fractional mean.
     */
    void CalibrateOffsets(uint32_t num_samples = 1024) noexcept {
        if (!adc1_ || !adc2_) return;

        uint32_t sum_ia = 0;
        uint32_t sum_ib = 0;

        for (uint32_t i = 0; i < num_samples; ++i) {
            sum_ia += (adc1_->JDR1 & 0x0FFF);
            sum_ib += (adc2_->JDR1 & 0x0FFF);
        }

        offset_ia_ = static_cast<float>(sum_ia) / static_cast<float>(num_samples);
        offset_ib_ = static_cast<float>(sum_ib) / static_cast<float>(num_samples);
        is_calibrated_ = true;
    }

    /**
     * Converts injected ADC readings into physical Phase Currents (A_peak).
     */
    inline void ReadPhaseCurrents(float& out_ia, float& out_ib, float& out_ic) const noexcept {
        if (!adc1_ || !adc2_) {
            out_ia = out_ib = out_ic = 0.0f;
            return;
        }

        const float lsb_scale = BoardConfig::GetCurrentLsbScale();
        const float raw_ia = static_cast<float>(adc1_->JDR1 & 0x0FFF);
        const float raw_ib = static_cast<float>(adc2_->JDR1 & 0x0FFF);

        out_ia = (raw_ia - offset_ia_) * lsb_scale;
        out_ib = (raw_ib - offset_ib_) * lsb_scale;
        out_ic = -(out_ia + out_ib); // Kirchhoff's Current Law: Ia + Ib + Ic = 0
    }

    [[nodiscard]] float GetOffsetIa() const noexcept { return offset_ia_; }
    [[nodiscard]] float GetOffsetIb() const noexcept { return offset_ib_; }
    [[nodiscard]] bool IsCalibrated() const noexcept { return is_calibrated_; }

private:
    AdcRegisters* adc1_;
    AdcRegisters* adc2_;
    float offset_ia_{2048.0f};
    float offset_ib_{2048.0f};
    bool is_calibrated_{false};
};

} // namespace apexdrive::firmware
