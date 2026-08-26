#pragma once

#include "types.hpp"
#include <array>
#include <cstdint>
#include <string>
#include <sstream>
#include <iomanip>

namespace apexdrive {

/**
 * 25 kHz In-Memory Circular Flight Recorder.
 * Stores continuous telemetry snapshots in a zero-allocation ring buffer and freezes on fault.
 */
class FlightRecorder {
public:
    static constexpr size_t BUFFER_CAPACITY = 250; // 250 samples @ 25kHz = 10ms pre-fault buffer

    struct SnapshotFrame {
        uint64_t timestamp_us{0};
        float i_a{0.0f};
        float i_b{0.0f};
        float i_c{0.0f};
        float v_bus{0.0f};
        float temp_c{0.0f};
        OperatingMode mode{OperatingMode::STANDBY};
        SafetyState safety{SafetyState::OK};
        uint32_t faults{0};
    };

    FlightRecorder() noexcept {
        buffer_.fill(SnapshotFrame{});
    }

    void RecordSample(uint64_t timestamp_us, const SensorReadings& sensors, OperatingMode mode, SafetyState safety, uint32_t faults) noexcept {
        if (is_frozen_) return;

        buffer_[write_idx_] = SnapshotFrame{
            .timestamp_us = timestamp_us,
            .i_a = sensors.i_phase_a,
            .i_b = sensors.i_phase_b,
            .i_c = sensors.i_phase_c,
            .v_bus = sensors.v_bus,
            .temp_c = sensors.winding_temp_c,
            .mode = mode,
            .safety = safety,
            .faults = faults
        };

        write_idx_ = (write_idx_ + 1) % BUFFER_CAPACITY;
        if (count_ < BUFFER_CAPACITY) ++count_;
    }

    void FreezeOnFault() noexcept {
        is_frozen_ = true;
    }

    void Reset() noexcept {
        is_frozen_ = false;
        write_idx_ = 0;
        count_ = 0;
        buffer_.fill(SnapshotFrame{});
    }

    [[nodiscard]] bool IsFrozen() const noexcept { return is_frozen_; }
    [[nodiscard]] size_t GetCount() const noexcept { return count_; }

    [[nodiscard]] std::string DumpForensicReport() const {
        std::ostringstream oss;
        oss << "======================================================================\n";
        oss << " APEXDRIVE FORENSIC BLACK-BOX INCIDENT REPORT                         \n";
        oss << " Status: " << (is_frozen_ ? "FROZEN ON FAULT" : "ACTIVE RECORDING") << "\n";
        oss << " Recorded Frames: " << count_ << " samples in ring buffer\n";
        oss << "======================================================================\n";
        oss << "  TIME(us)  |  I_A (A)  |  I_B (A)  |  I_C (A)  | V_BUS(V) | TEMP(C) | MODE\n";
        oss << "----------------------------------------------------------------------\n";

        size_t start_idx = (count_ < BUFFER_CAPACITY) ? 0 : write_idx_;
        for (size_t i = 0; i < count_; ++i) {
            size_t idx = (start_idx + i) % BUFFER_CAPACITY;
            const auto& f = buffer_[idx];
            oss << std::setw(10) << f.timestamp_us << " | "
                << std::fixed << std::setprecision(2)
                << std::setw(9) << f.i_a << " | "
                << std::setw(9) << f.i_b << " | "
                << std::setw(9) << f.i_c << " | "
                << std::setw(8) << f.v_bus << " | "
                << std::setw(7) << f.temp_c << " | "
                << ModeToString(f.mode) << "\n";
        }
        oss << "======================================================================\n";
        return oss.str();
    }

private:
    std::array<SnapshotFrame, BUFFER_CAPACITY> buffer_;
    size_t write_idx_{0};
    size_t count_{0};
    bool is_frozen_{false};
};

} // namespace apexdrive
