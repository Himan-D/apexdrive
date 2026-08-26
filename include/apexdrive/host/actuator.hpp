#pragma once

#include "actuator_backend.hpp"
#include <string>
#include <memory>

namespace apexdrive {

/**
 * Top-Level Robotics Joint Actuator API.
 * Delegates cleanly to either HardwareCanBackend (Linux SocketCAN-FD) or SimulationBackend (PMSM physics).
 */
class Actuator {
public:
    explicit Actuator(std::string interface = "can0", uint8_t node_id = 0x14, bool mock_mode = false);
    explicit Actuator(std::unique_ptr<IActuatorBackend> backend, uint8_t node_id = 0x14);
    ~Actuator() = default;

    void Arm();
    void Disarm();
    void EmergencyStop();

    /**
     * Commands compliant impedance law:
     * tau = kp * (pos_rad - current_pos) + kd * (vel_rad_s - current_vel) + tau_ff
     */
    void SetImpedance(float pos_rad, float vel_rad_s, float kp, float kd, float tau_ff);
    void SetTorque(float torque_nm);

    /**
     * Fetches current state telemetry from hardware CAN bus or simulation.
     */
    [[nodiscard]] JointTelemetry GetState();

    /**
     * Executes single physical simulation step (when using SimulationBackend).
     */
    void StepPhysics(float dt_seconds = 0.00004f);

    [[nodiscard]] CalibrationResult Calibrate() { return CalibrationResult{}; }
    [[nodiscard]] float GetSmoEstimatedAngle() { return GetState().position_rad; }
    [[nodiscard]] std::string DumpBlackBox();

    [[nodiscard]] OperatingMode GetMode() const noexcept { return mode_; }
    [[nodiscard]] SafetyState GetSafetyState() const noexcept { return safety_state_; }
    [[nodiscard]] uint8_t GetNodeId() const noexcept { return node_id_; }
    [[nodiscard]] bool IsHardware() const noexcept { return backend_->IsHardware(); }

private:
    std::string interface_;
    uint8_t node_id_;
    OperatingMode mode_{OperatingMode::STANDBY};
    SafetyState safety_state_{SafetyState::OK};
    uint16_t sequence_num_{0};
    std::unique_ptr<IActuatorBackend> backend_;
};

} // namespace apexdrive
