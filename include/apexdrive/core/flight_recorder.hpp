#pragma once

#include "types.hpp"
#include <array>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

namespace apexdrive {

/**
 * High-Frequency In-Memory Circular Flight Recorder ("The Robotics Black Box")
 * Continuously records 25 kHz sensor telemetry without disk I/O or LTE bandwidth.
 * Freezes a forensic snapshot upon any hardware fault or emergency stop.
 */
class FlightRecorder {
public:
    static constexpr size_t BUFFER_CAPACITY = 250; // Rolling sliding window of snapshots

    struct SnapshotFrame {
        uint32_t timestamp_us{0};
        float i_phase_a{0.0f};
        float i_phase_b{0.0f};
        float i_phase_c{0.0f};
        float v_bus{0.0f};
        float rotor_angle{0.0f};
        float mosfet_temp{0.0f};
        DriveState state{DriveState::STANDBY};
    };

    FlightRecorder() noexcept : write_index_(0), count_(0), is_frozen_(false) {}

    // Record high-speed frame into sliding ring buffer
    void RecordSample(uint32_t timestamp_us, const SensorReadings& s, DriveState state) noexcept {
        if (is_frozen_) return; // Do not overwrite post-fault freeze

        buffer_[write_index_] = SnapshotFrame{
            .timestamp_us = timestamp_us,
            .i_phase_a = s.i_phase_a,
            .i_phase_b = s.i_phase_b,
            .i_phase_c = s.i_phase_c,
            .v_bus = s.v_bus,
            .rotor_angle = s.rotor_angle_rad,
            .mosfet_temp = s.mosfet_temp_c,
            .state = state
        };

        write_index_ = (write_index_ + 1) % BUFFER_CAPACITY;
        if (count_ < BUFFER_CAPACITY) count_++;
    }

    void FreezeOnFault() noexcept {
        is_frozen_ = true;
    }

    void Reset() noexcept {
        write_index_ = 0;
        count_ = 0;
        is_frozen_ = false;
    }

    [[nodiscard]] bool IsFrozen() const noexcept { return is_frozen_; }
    [[nodiscard]] size_t GetCount() const noexcept { return count_; }

    // Dumps formatted forensic package for remote triage
    [[nodiscard]] std::string DumpForensicReport() const {
        std::stringstream ss;
        ss << "======================================================================\n";
        ss << " APEXDRIVE FORENSIC BLACK-BOX INCIDENT REPORT                         \n";
        ss << " Status: " << (is_frozen_ ? "FROZEN ON HARDWARE FAULT" : "HEALTHY BUFFER") << "\n";
        ss << " Recorded Frames: " << count_ << " samples in ring buffer\n";
        ss << "======================================================================\n";
        ss << "  TIME(us)  |  I_A (A)  |  I_B (A)  |  I_C (A)  | V_BUS(V) | TEMP(C) | STATE\n";
        ss << "----------------------------------------------------------------------\n";

        size_t start = (count_ < BUFFER_CAPACITY) ? 0 : write_index_;
        for (size_t i = 0; i < count_; ++i) {
            size_t idx = (start + i) % BUFFER_CAPACITY;
            const auto& f = buffer_[idx];
            ss << std::setw(10) << f.timestamp_us << " | "
               << std::setw(9) << std::fixed << std::setprecision(2) << f.i_phase_a << " | "
               << std::setw(9) << f.i_phase_b << " | "
               << std::setw(9) << f.i_phase_c << " | "
               << std::setw(8) << f.v_bus << " | "
               << std::setw(7) << f.mosfet_temp << " | "
               << StateToString(f.state) << "\n";
        }
        ss << "======================================================================\n";
        return ss.str();
    }

private:
    std::array<SnapshotFrame, BUFFER_CAPACITY> buffer_{};
    size_t write_index_{0};
    size_t count_{0};
    bool is_frozen_{false};
};

} // namespace apexdrive
