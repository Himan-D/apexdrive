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
 * High-Level Host Actuator Client.
 * Connects to a physical or simulated joint actuator with real-time FOC,
 * anti-cogging feedforward, and sensorless sliding mode observer backup.
 */
class Actuator {
public:
    Actuator(std::string interface_name, uint8_t node_id, bool mock_mode = true);
    ~Actuator() = default;

    // Operational Commands
    void Arm();
    void Disarm();
    void EmergencyStop();

    // Motion Control Modes (1 kHz streaming API)
    void SetTorque(float torque_nm);
    void SetPosition(float target_pos_rad, float max_vel_rad_s = 20.0f);
    void SetImpedance(float pos_rad, float vel_rad_s, float kp, float kd, float tau_ff = 0.0f);

    // Telemetry & Diagnostics
    [[nodiscard]] JointTelemetry GetState() const;
    [[nodiscard]] AutoTuner::TuningResult Calibrate();
    [[nodiscard]] std::string DumpBlackBox();

    // Internal 25kHz Mock Simulation Step (Used for testing & verification)
    void StepPhysics(float dt_s = 0.001f);

    [[nodiscard]] uint8_t GetNodeId() const noexcept { return node_id_; }
    [[nodiscard]] DriveState GetCurrentState() const noexcept { return state_; }
    [[nodiscard]] float GetSmoEstimatedAngle() const noexcept { return smo_observer_.GetEstimatedAngle(); }

private:
    [[maybe_unused]] std::string interface_;
    uint8_t node_id_;
    [[maybe_unused]] bool mock_mode_;
    
    DriveState state_{DriveState::STANDBY};
    MotorProfile profile_{};
    ImpedanceCommand last_command_{};
    JointTelemetry last_telemetry_{};
    
    // Core Embedded Subsystems
    PiController current_pid_d_{0.25f, 150.0f, 24.0f};
    PiController current_pid_q_{0.25f, 150.0f, 24.0f};
    HardwareSafetySupervisor safety_guard_;
    AntiCoggingMap anti_cogging_map_{};
    SlidingModeObserver smo_observer_;
    FlightRecorder flight_recorder_;

    // Simulated Physical Dynamics
    float sim_pos_rad_{0.0f};
    float sim_vel_rad_s_{0.0f};
    float sim_current_id_{0.0f};
    float sim_current_iq_{0.0f};
    float sim_winding_temp_c_{35.0f};
    float sim_v_bus_{48.0f};
};

} // namespace apexdrive
