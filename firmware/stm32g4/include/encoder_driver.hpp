#pragma once

#include <cstdint>
#include <cmath>

namespace apexdrive::firmware {

/**
 * 14-Bit SPI Magnetic Encoder Driver (AS5047P / MA730 Standard).
 * Includes hardware parity validation, angle unwrapping, and electrical angle derivation.
 */
class EncoderDriver {
public:
    struct SpiRegisters {
        volatile uint32_t CR1;
        volatile uint32_t CR2;
        volatile uint32_t SR;
        volatile uint32_t DR;
    };

    explicit EncoderDriver(SpiRegisters* spi1, float pole_pairs) noexcept
        : spi1_(spi1), pole_pairs_(pole_pairs) {}

    void Init() noexcept {
        if (!spi1_) return;
        // SPI Master, 10.6 MHz (170 MHz / 16), Mode 1 (CPOL=0, CPHA=1), 16-bit Data Size
        spi1_->CR1 = (1 << 2) | (3 << 3) | (1 << 0); // MSTR=1, BR=011 (div 16), CPHA=1
        spi1_->CR2 = (15 << 8) | (1 << 12);          // DS=1111 (16-bit), FRXTH=1
        spi1_->CR1 |= (1 << 6);                      // SPE=1 (Enable SPI)
    }

    /**
     * Reads angle directly from SPI bus in < 1.5 microseconds.
     * @return true if parity check passed, false on corrupted transmission.
     */
    [[nodiscard]] bool UpdateAngle(float& out_mechanical_rad, float& out_electrical_rad, float encoder_offset_rad) noexcept {
        if (!spi1_) return false;

        // Transmit read position command (0x3FFF for AS5047P / 0xFFFF with parity)
        spi1_->DR = 0xFFFF;

        // Wait for TX/RX completion (at 10.6 MHz, takes ~1.5us)
        while (!(spi1_->SR & (1 << 0))); // Wait RXNE
        uint16_t raw_data = static_cast<uint16_t>(spi1_->DR);

        // Parity Validation (Bit 15 is even parity of bits 0..14)
        uint16_t parity_calc = 0;
        for (int i = 0; i < 15; ++i) {
            parity_calc ^= (raw_data >> i) & 1;
        }
        bool parity_ok = (parity_calc == ((raw_data >> 15) & 1));
        if (!parity_ok) {
            ++error_count_;
            return false;
        }

        // 14-bit angle [0..16383]
        uint16_t raw_pos = raw_data & 0x3FFF;
        out_mechanical_rad = (static_cast<float>(raw_pos) / 16384.0f) * 6.2831853f;
        out_electrical_rad = (out_mechanical_rad * pole_pairs_) - encoder_offset_rad;

        while (out_electrical_rad >= 6.2831853f) out_electrical_rad -= 6.2831853f;
        while (out_electrical_rad < 0.0f)        out_electrical_rad += 6.2831853f;

        last_mechanical_angle_ = out_mechanical_rad;
        return true;
    }

    [[nodiscard]] uint32_t GetErrorCount() const noexcept { return error_count_; }
    [[nodiscard]] float GetLastMechanicalAngle() const noexcept { return last_mechanical_angle_; }

private:
    SpiRegisters* spi1_;
    float pole_pairs_;
    float last_mechanical_angle_{0.0f};
    uint32_t error_count_{0};
};

} // namespace apexdrive::firmware
