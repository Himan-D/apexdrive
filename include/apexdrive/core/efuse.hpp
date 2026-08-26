#pragma once

#include "types.hpp"
#include <cmath>
#include <algorithm>

namespace apexdrive {

/**
 * Software Safety Supervisor & Health Guard.
 * Monitors operational limits, under-voltage lockout, overvoltage, overtemperature,
 * and software I^2t thermal accumulation.
 * Note: On physical hardware, this works in tandem with the hardware analog comparator (BKIN / STO).
 */
class HardwareSafetySupervisor {
public:
    explicit HardwareSafetySupervisor(const MotorProfile& profile) noexcept
        : profile_(profile), i2t_accumulator_(0.0f) {}

    void Reset() noexcept {
        i2t_accumulator_ = 0.0f;
    }

    /**
     * Real-time safety validation loop.
     * @return true if all sensor values are within safe envelope; false if fault triggered.
     */
    [[nodiscard]] bool CheckHealth(const SensorReadings& sensors, float dt, DriveState& out_fault) noexcept {
        // 1. Peak Instantaneous Overcurrent Protection
        const float max_current = std::max({std::abs(sensors.i_phase_a), 
                                            std::abs(sensors.i_phase_b), 
                                            std::abs(sensors.i_phase_c)});
        if (max_current > profile_.peak_current_a) {
            out_fault = DriveState::FAULT_OVERCURRENT;
            return false;
        }

        // 2. DC Bus Overvoltage Clamping (e.g. during heavy deceleration without braking resistor)
        if (sensors.v_bus > profile_.max_voltage_v) {
            out_fault = DriveState::FAULT_OVERVOLTAGE;
            return false;
        }

        // 3. DC Bus Under-Voltage Lockout (UVLO Brownout)
        if (sensors.v_bus < profile_.min_voltage_uvlo_v) {
            out_fault = DriveState::FAULT_BROWNOUT;
            return false;
        }

        // 4. Inverter & Motor Overtemperature Cutoff
        if (sensors.winding_temp_c > profile_.max_winding_temp_c || sensors.mosfet_temp_c > 105.0f) {
            out_fault = DriveState::FAULT_OVERTEMP;
            return false;
        }

        // 5. I^2t Continuous Thermal Energy Accumulator
        const float i_sq = max_current * max_current;
        const float i_cont_sq = profile_.max_continuous_current_a * profile_.max_continuous_current_a;
        if (i_sq > i_cont_sq) {
            const float delta_heat = (i_sq - i_cont_sq) * dt;
            i2t_accumulator_ += delta_heat;
            // Thermal budget: 2.0 seconds at peak current before trip
            const float max_i2t_budget = (profile_.peak_current_a * profile_.peak_current_a - i_cont_sq) * 2.0f;
            if (i2t_accumulator_ > max_i2t_budget) {
                out_fault = DriveState::FAULT_OVERTEMP;
                return false;
            }
        } else {
            // Passive thermal dissipation cooldown
            i2t_accumulator_ = std::max(0.0f, i2t_accumulator_ - (i_cont_sq * 0.1f * dt));
        }

        return true;
    }

    [[nodiscard]] float GetThermalEnergyAccumulator() const noexcept {
        return i2t_accumulator_;
    }

private:
    MotorProfile profile_;
    float i2t_accumulator_;
};

} // namespace apexdrive
