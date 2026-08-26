#pragma once

#include "types.hpp"
#include <array>
#include <cmath>
#include <algorithm>

namespace apexdrive {

/**
 * 256-Point High-Resolution Anti-Cogging Feedforward Lookup Table.
 * Eliminates stator slotting torque ripple, enabling glassy-smooth motion at low velocities.
 */
class AntiCoggingMap {
public:
    static constexpr size_t TABLE_SIZE = 256;
    static constexpr float TWO_PI = 6.283185307179586f;

    AntiCoggingMap() noexcept {
        table_.fill(0.0f);
    }

    /**
     * Set calibrated torque ripple compensation value at specific bucket index
     */
    void SetBucket(size_t index, float torque_comp_nm) noexcept {
        if (index < TABLE_SIZE) {
            table_[index] = torque_comp_nm;
        }
    }

    /**
     * Fast Linear-Interpolated Anti-Cogging Torque Compensation.
     * @param mechanical_angle_rad Current mechanical angle of rotor [0, 2pi)
     * @return Feedforward torque in Nm to add to current loop
     */
    [[nodiscard]] inline float Lookup(float mechanical_angle_rad) const noexcept {
        // Normalize angle to [0, 2pi)
        float norm_angle = std::fmod(mechanical_angle_rad, TWO_PI);
        if (norm_angle < 0.0f) norm_angle += TWO_PI;

        const float index_float = (norm_angle / TWO_PI) * static_cast<float>(TABLE_SIZE);
        const size_t idx_low = static_cast<size_t>(index_float) % TABLE_SIZE;
        const size_t idx_high = (idx_low + 1) % TABLE_SIZE;
        const float frac = index_float - static_cast<float>(idx_low);

        // Linear interpolation between adjacent buckets
        return table_[idx_low] + frac * (table_[idx_high] - table_[idx_low]);
    }

    void Clear() noexcept {
        table_.fill(0.0f);
    }

    [[nodiscard]] const std::array<float, TABLE_SIZE>& GetRawTable() const noexcept {
        return table_;
    }

private:
    std::array<float, TABLE_SIZE> table_;
};

} // namespace apexdrive
