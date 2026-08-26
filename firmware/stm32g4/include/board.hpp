#pragma once

#include <cstdint>
#include <algorithm>

namespace apexdrive::firmware {

/**
 * STM32G474RE Physical Board & Peripheral Register Map
 * 
 * Hardware Architecture:
 * - MCU: STM32G474RET6 (ARM Cortex-M4 with FPU @ 170 MHz)
 * - Inverter: 3-Phase Gate Driver (DRV8353RS / DRV8323RS) with 6x Power MOSFETs
 * - Shunts: 3x Low-side 5 mOhm current shunts with 20x internal differential Op-Amps
 * - Sensing: Dual Simultaneous Injected ADCs triggered by TIM1 underflow (TRGO)
 * - Encoder: 14-Bit Magnetic SPI Absolute Angle Sensor (AS5047P / MA730)
 * - Transceiver: 3.3V CAN-FD Transceiver (TCAN334 / SN65HVD230) on FDCAN1
 */
struct BoardConfig {
    static constexpr uint32_t SYSCLK_HZ = 170000000;       // 170 MHz System Clock
    static constexpr uint32_t PWM_FREQ_HZ = 25000;         // 25 kHz Center-Aligned PWM
    static constexpr uint32_t TIM1_ARR_TICKS = 3400;        // 170MHz / (2 * 25kHz) = 3400 ticks
    static constexpr float SHUNT_RESISTANCE_OHM = 0.005f;   // 5 mOhm low-side current shunts
    static constexpr float OPAMP_GAIN = 20.0f;              // 20x differential amplifier gain
    static constexpr float V_REF_ADC = 3.3f;                // 3.3V ADC reference

    /**
     * Analytical Current Scaling:
     *   A / LSB = V_ref / (4096 * R_shunt * OpAmp_Gain)
     *   For 3.3V, 5mOhm, 20x -> 3.3 / (4096 * 0.1) = 0.00805664 A/count
     */
    [[nodiscard]] static constexpr float GetCurrentLsbScale() noexcept {
        return V_REF_ADC / (4096.0f * SHUNT_RESISTANCE_OHM * OPAMP_GAIN);
    }

    /**
     * Exact STM32 TIM1 BDTR Dead-Time Register Generator:
     * Converts desired nanoseconds into hardware DTG bitfield based on 170 MHz clock tree.
     */
    [[nodiscard]] static constexpr uint8_t EncodeDeadTime(uint32_t dead_time_ns, uint32_t timer_clk_hz = SYSCLK_HZ) noexcept {
        float t_dts_ns = (1.0f / static_cast<float>(timer_clk_hz)) * 1e9f; // ~5.88 ns at 170 MHz
        uint32_t counts = static_cast<uint32_t>(static_cast<float>(dead_time_ns) / t_dts_ns);

        if (counts <= 127) {
            return static_cast<uint8_t>(counts & 0x7F);
        } else if (counts <= 254) {
            return static_cast<uint8_t>(0x80 | ((counts / 2 - 64) & 0x3F));
        } else if (counts <= 508) {
            return static_cast<uint8_t>(0xC0 | ((counts / 8 - 32) & 0x1F));
        } else {
            return static_cast<uint8_t>(0xE0 | ((counts / 16 - 32) & 0x1F));
        }
    }
};

} // namespace apexdrive::firmware
