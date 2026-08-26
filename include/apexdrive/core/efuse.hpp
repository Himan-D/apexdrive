#pragma once

#include "types.hpp"
#include <cmath>
#include <algorithm>

namespace apexdrive {

/**
 * High-Speed Electronic Fuse (eFuse) & Power Electronics Safety Supervisor.
 * Evaluated on every single 25kHz ADC interrupt in sub-microsecond time.
 */
class HardwareSafetySupervisor {
public:
    explicit HardwareSafetySupervisor(const MotorProfile& profile) noexcept
        : profile_(profile), i2t_accumulator_(0.0f), i2t_threshold_(profile.max_continuous_current_a * profile.max_continuous_current_a * 2.0f) {}

    /**
     * Level-0 Fast Hardware Fault Check (Executes in < 50 nanoseconds)
     * @return True if system is healthy, False if a hardware safety fault tripped.
     */
    [[nodiscard]] bool CheckHealth(const SensorReadings& sensors, float dt, DriveState& out_fault) noexcept {
        // 1. Instantaneous Peak Overcurrent (Short-circuit / locked rotor breaker)
        const float max_current = std::max({std::abs(sensors.i_phase_a), 
                                            std::abs(sensors.i_phase_b), 
                                            std::abs(sensors.i_phase_c)});
        if (max_current >= profile_.peak_current_a) {
            out_fault = DriveState::FAULT_OVERCURRENT;
            return false;
        }

        // 2. DC Bus Brownout / Under-Voltage Lockout (UVLO)
        // Catches battery voltage collapse before the 5V/3.3V logic regulator fails
        if (sensors.v_bus < profile_.min_voltage_uvlo_v) {
            out_fault = DriveState::FAULT_BROWNOUT;
            return false;
        }

        // 3. DC Bus Over-Voltage Clamp (Regenerative braking back-EMF surge)
        if (sensors.v_bus > profile_.max_voltage_v) {
            out_fault = DriveState::FAULT_BROWNOUT;
            return false;
        }

        // 4. Over-Temperature Cutoff (MOSFET / Stator winding limit)
        if (sensors.mosfet_temp_c >= profile_.max_winding_temp_c ||
            sensors.winding_temp_c >= profile_.max_winding_temp_c) {
            out_fault = DriveState::FAULT_OVERTEMP;
            return false;
        }

        // 5. I²t Thermal Energy Accumulator (Continuous Overload Integrator)
        // Accumulates Joule heating when I > I_continuous, cools down when I < I_continuous
        const float i_sq = max_current * max_current;
        const float i_cont_sq = profile_.max_continuous_current_a * profile_.max_continuous_current_a;
        const float delta_heat = (i_sq - i_cont_sq) * dt;

        i2t_accumulator_ += delta_heat;
        if (i2t_accumulator_ < 0.0f) i2t_accumulator_ = 0.0f;

        if (i2t_accumulator_ > i2t_threshold_) {
            out_fault = DriveState::FAULT_OVERCURRENT;
            return false;
        }

        return true;
    }

    void Reset() noexcept {
        i2t_accumulator_ = 0.0f;
    }

    [[nodiscard]] float GetI2tNormalized() const noexcept {
        return (i2t_threshold_ > 0.0f) ? std::clamp(i2t_accumulator_ / i2t_threshold_, 0.0f, 1.0f) : 0.0f;
    }

private:
    MotorProfile profile_;
    float i2t_accumulator_{0.0f};
    float i2t_threshold_{1000.0f};
};

} // namespace apexdrive
