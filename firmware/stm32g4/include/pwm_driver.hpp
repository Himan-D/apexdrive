#pragma once

#include "board.hpp"
#include <cstdint>

namespace apexdrive::firmware {

/**
 * Register-Level TIM1 Advanced Timer PWM Driver with Hardware STO Break Input.
 */
class PwmDriver {
public:
    struct Registers {
        volatile uint32_t CR1;
        volatile uint32_t CR2;
        volatile uint32_t SMCR;
        volatile uint32_t DIER;
        volatile uint32_t SR;
        volatile uint32_t EGR;
        volatile uint32_t CCMR1;
        volatile uint32_t CCMR2;
        volatile uint32_t CCER;
        volatile uint32_t CNT;
        volatile uint32_t PSC;
        volatile uint32_t ARR;
        volatile uint32_t RCR;
        volatile uint32_t CCR1;
        volatile uint32_t CCR2;
        volatile uint32_t CCR3;
        volatile uint32_t CCR4;
        volatile uint32_t BDTR;
    };

    explicit PwmDriver(Registers* tim1) noexcept : tim1_(tim1) {}

    void Init() noexcept {
        if (!tim1_) return;

        // 1. Center-aligned Mode 1 (Counts up and down, output compare interrupt on underflow)
        tim1_->CR1 = (1 << 5); // CMS = 01 (Center-aligned mode 1)

        // 2. Set Period (3400 ticks for 25 kHz at 170 MHz)
        tim1_->ARR = BoardConfig::TIM1_ARR_TICKS;
        tim1_->PSC = 0; // 170 MHz clock directly

        // 3. Set TRGO on Update (Underflow) to trigger Injected ADC conversion
        tim1_->CR2 = (2 << 4); // MMS = 010 (Update event selected as TRGO)

        // 4. Configure PWM Mode 1 on Channels 1, 2, 3 with Preload enabled
        tim1_->CCMR1 = (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);
        tim1_->CCMR2 = (6 << 4) | (1 << 3);

        // 5. Enable Complementary Outputs on CH1, CH1N, CH2, CH2N, CH3, CH3N
        tim1_->CCER = (1 << 0) | (1 << 2) | (1 << 4) | (1 << 6) | (1 << 8) | (1 << 10);

        // 6. Break & Dead-Time Register (TIM1_BDTR):
        // Dead-time = 120 ns (DTG = 0x24 at 170MHz)
        // BKE = 1 (Break Enable - Hardware STO from analog comparator)
        // MOE = 1 (Main Output Enable)
        tim1_->BDTR = (1 << 15) | (1 << 12) | 0x24;

        // Initial 50% Idle Duty Cycles
        tim1_->CCR1 = BoardConfig::TIM1_ARR_TICKS / 2;
        tim1_->CCR2 = BoardConfig::TIM1_ARR_TICKS / 2;
        tim1_->CCR3 = BoardConfig::TIM1_ARR_TICKS / 2;

        // Enable Counter
        tim1_->CR1 |= (1 << 0);
    }

    /**
     * Updates 3-phase PWM compare values directly from normalized duty cycles [0.0, 1.0].
     * Zero branching / register-direct.
     */
    inline void SetDuties(float u, float v, float w) noexcept {
        if (!tim1_) return;
        tim1_->CCR1 = static_cast<uint32_t>(u * static_cast<float>(BoardConfig::TIM1_ARR_TICKS));
        tim1_->CCR2 = static_cast<uint32_t>(v * static_cast<float>(BoardConfig::TIM1_ARR_TICKS));
        tim1_->CCR3 = static_cast<uint32_t>(w * static_cast<float>(BoardConfig::TIM1_ARR_TICKS));
    }

    /**
     * Hardware Safe Torque Off: Immediately clears MOE bit in silicon, tri-stating all 6 MOSFET gates in < 40ns.
     */
    void SafeTorqueOff() noexcept {
        if (!tim1_) return;
        tim1_->BDTR &= ~(1 << 15); // Clear MOE bit
    }

    [[nodiscard]] bool IsHardwareStoTripped() const noexcept {
        if (!tim1_) return true;
        return (tim1_->SR & (1 << 7)) != 0; // BIF (Break Interrupt Flag)
    }

private:
    Registers* tim1_;
};

} // namespace apexdrive::firmware
