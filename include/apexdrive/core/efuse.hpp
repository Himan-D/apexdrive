#pragma once

#include "types.hpp"
#include <cmath>
#include <algorithm>

namespace apexdrive {

/**
 * Software Safety Supervisor & Health Guard.
 * Validates operational envelope, voltage bounds, thermal budgets, and watchdog heartbeat.
 */
class HardwareSafetySupervisor {
public:
    explicit HardwareSafetySupervisor(const MotorProfile& profile) noexcept
        : profile_(profile), i2t_accumulator_(0.0f), time_since_last_command_sec_(0.0f) {}

    void Reset() noexcept {
        i2t_accumulator_ = 0.0f;
        time_since_last_command_sec_ = 0.0f;
    }

    void FeedWatchdog() noexcept {
        time_since_last_command_sec_ = 0.0f;
    }

    /**
     * Safety evaluation loop.
     * @return true if all signals are within normal limits; false if a safety fault occurred.
     */
    [[nodiscard]] bool CheckHealth(
        const SensorReadings& sensors, 
        float dt, 
        SafetyState& out_safety, 
        uint32_t& out_faults
    ) noexcept {
        out_faults = FaultFlag::NONE;
        out_safety = SafetyState::OK;

        // 1. Watchdog Heartbeat Check
        time_since_last_command_sec_ += dt;
        if (time_since_last_command_sec_ > profile_.command_timeout_sec) {
            out_faults |= FaultFlag::WATCHDOG_TIMEOUT;
        }

        // 2. Peak Phase Current Protection
        const float max_current = std::max({std::abs(sensors.i_phase_a), 
                                            std::abs(sensors.i_phase_b), 
                                            std::abs(sensors.i_phase_c)});
        if (max_current > profile_.peak_current_a) {
            out_faults |= FaultFlag::OVERCURRENT_PHASE;
        }

        // 3. DC Bus Overvoltage Clamp
        if (sensors.v_bus > profile_.max_voltage_v) {
            out_faults |= FaultFlag::OVERVOLTAGE_BUS;
        }

        // 4. DC Bus Under-Voltage Lockout (UVLO)
        if (sensors.v_bus < profile_.min_voltage_uvlo_v) {
            out_faults |= FaultFlag::UNDERVOLTAGE_BUS;
        }

        // 5. Thermal Limits
        if (sensors.winding_temp_c > profile_.max_winding_temp_c) {
            out_faults |= FaultFlag::OVERTEMP_WINDING;
        }
        if (sensors.mosfet_temp_c > 105.0f) {
            out_faults |= FaultFlag::OVERTEMP_MOSFET;
        }

        // 6. I^2t Thermal Accumulator
        const float i_sq = max_current * max_current;
        const float i_cont_sq = profile_.max_continuous_current_a * profile_.max_continuous_current_a;
        if (i_sq > i_cont_sq) {
            const float delta_heat = (i_sq - i_cont_sq) * dt;
            i2t_accumulator_ += delta_heat;
            const float max_i2t_budget = (profile_.peak_current_a * profile_.peak_current_a - i_cont_sq) * 2.0f;
            if (i2t_accumulator_ > max_i2t_budget) {
                out_faults |= FaultFlag::OVERTEMP_WINDING;
            }
        } else {
            i2t_accumulator_ = std::max(0.0f, i2t_accumulator_ - (i_cont_sq * 0.1f * dt));
        }

        if (out_faults != FaultFlag::NONE) {
            out_safety = SafetyState::FAULT_STOP;
            return false;
        }

        return true;
    }

    [[nodiscard]] float GetThermalEnergyAccumulator() const noexcept {
        return i2t_accumulator_;
    }

private:
    MotorProfile profile_;
    float i2t_accumulator_;
    float time_since_last_command_sec_;
};

} // namespace apexdrive
