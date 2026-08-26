#pragma once

#include <cstdint>

namespace apexdrive::firmware {

/**
 * STM32G474RE Pinout & Hardware Pin Mappings
 * 
 * Clocks:
 * - SYSCLK: 170 MHz (PLL from 24 MHz HSE crystal)
 * - APB1/APB2 Timers: 170 MHz
 * 
 * PWM Inverter Outputs (TIM1):
 * - PA8:  TIM1_CH1  (Phase U High-Side)
 * - PB13: TIM1_CH1N (Phase U Low-Side)
 * - PA9:  TIM1_CH2  (Phase V High-Side)
 * - PB14: TIM1_CH2N (Phase V Low-Side)
 * - PA10: TIM1_CH3  (Phase W High-Side)
 * - PB15: TIM1_CH3N (Phase W Low-Side)
 * - PA6:  TIM1_BKIN (Hardware Safe Torque Off / Analog Comparator Trip)
 * 
 * Injected Fast ADCs:
 * - PA0: ADC1_IN1 (Phase A Low-Side Shunt Current)
 * - PA1: ADC2_IN2 (Phase B Low-Side Shunt Current)
 * - PA2: ADC1_IN3 (DC Bus Voltage Sense - 1:20 Resistor Divider)
 * - PA3: ADC2_IN4 (Inverter MOSFET NTC Temperature)
 * 
 * SPI Absolute Encoder (SPI1):
 * - PA4: SPI1_NSS  (Encoder Chip Select)
 * - PA5: SPI1_SCK  (10 MHz SPI Clock)
 * - PB4: SPI1_MISO (14-Bit Position Data)
 * 
 * CAN-FD Interface (FDCAN1):
 * - PA11: FDCAN1_RX
 * - PA12: FDCAN1_TX
 */
struct BoardConfig {
    static constexpr uint32_t SYSCLK_HZ = 170000000;       // 170 MHz
    static constexpr uint32_t PWM_FREQ_HZ = 25000;         // 25 kHz
    static constexpr uint32_t TIM1_ARR_TICKS = 3400;        // 170MHz / (2 * 25kHz) = 3400 center-aligned
    static constexpr uint32_t DEAD_TIME_NS = 120;           // 120 ns complementary dead-time
    static constexpr float SHUNT_RESISTANCE_OHM = 0.005f;   // 5 mOhm low-side current shunts
    static constexpr float OPAMP_GAIN = 20.0f;              // 20x onboard current sense amplifier
    static constexpr float V_REF_ADC = 3.3f;                // 3.3V ADC reference
};

} // namespace apexdrive::firmware
