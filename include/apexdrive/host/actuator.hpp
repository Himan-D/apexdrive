#pragma once

#include "../core/types.hpp"
#include "../core/foc_math.hpp"
#include "../core/efuse.hpp"
#include "../core/impedance_controller.hpp"
#include "../core/auto_tuner.hpp"
#include "../core/anti_cogging.hpp"
#include "../core/sliding_mode_observer.hpp"
#include "../core/flight_recorder.hpp"
#include "../protocol/can_frame.hpp"

#include <string>
#include <memory>
#include <chrono>

namespace apexdrive {

/**
 * Host Actuator Client & Simulation Engine.
 * Implements real-time closed-loop FOC with Clarke/Park transforms, PI regulators,
 * voltage decoupling, back-calculation anti-windup, and continuous PMSM dynamics.
 */
class Actuator {
public:
    Actuator(std::string interface_name, uint8_t node_id, bool mock_mode = true);
    ~Actuator() = default;

    // Operational Commands
    void Arm();
    void Disarm();
    void EmergencyStop();

    // Motion Control Modes
    void SetTorque(float torque_nm);
    void SetPosition(float target_pos_rad, float max_vel_rad_s = 20.0f);
    void SetImpedance(float pos_rad, float vel_rad_s, float kp, float kd, float tau_ff = 0.0f);

    // Diagnostics & State
    [[nodiscard]] JointTelemetry GetState() const;
    [[nodiscard]] AutoTuner::TuningResult Calibrate();
    [[nodiscard]] std::string DumpBlackBox();

    // Closed-Loop Physics & Inverter Simulation Step
    void StepPhysics(float dt_s = 0.00004f);

    [[nodiscard]] uint8_t GetNodeId() const noexcept { return node_id_; }
    [[nodiscard]] OperatingMode GetCurrentMode() const noexcept { return mode_; }
    [[nodiscard]] SafetyState GetSafetyState() const noexcept { return safety_state_; }
    [[nodiscard]] float GetSmoEstimatedAngle() const noexcept { return smo_observer_.GetEstimatedAngle(); }
    [[nodiscard]] float GetSmoEstimatedSpeed() const noexcept { return smo_observer_.GetEstimatedSpeed(); }

private:
    [[maybe_unused]] std::string interface_;
    uint8_t node_id_;
    [[maybe_unused]] bool mock_mode_;
    
    OperatingMode mode_{OperatingMode::STANDBY};
    SafetyState safety_state_{SafetyState::OK};
    uint32_t fault_flags_{FaultFlag::NONE};
    
    MotorProfile profile_{};
    ImpedanceCommand last_command_{};
    JointTelemetry last_telemetry_{};
    
    // Core Embedded FOC Subsystems
    PiController current_pid_d_{0.25f, 150.0f, 24.0f};
    PiController current_pid_q_{0.25f, 150.0f, 24.0f};
    HardwareSafetySupervisor safety_guard_;
    AntiCoggingMap anti_cogging_map_{};
    SlidingModeObserver smo_observer_;
    FlightRecorder flight_recorder_;

    // Continuous PMSM Physical State
    float sim_pos_rad_{0.0f};
    float sim_vel_rad_s_{0.0f};
    float sim_current_id_{0.0f};
    float sim_current_iq_{0.0f};
    float sim_winding_temp_c_{35.0f};
    float sim_mosfet_temp_c_{35.0f};
    float sim_v_bus_{48.0f};
    uint64_t current_timestamp_us_{0};
};

} // namespace apexdrive
