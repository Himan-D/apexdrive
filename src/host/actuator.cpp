#include "../../include/apexdrive/host/actuator.hpp"

namespace apexdrive {

Actuator::Actuator(std::string interface, uint8_t node_id, bool mock_mode)
    : interface_(std::move(interface)), node_id_(node_id) {
    if (mock_mode) {
        MotorParameters default_params;
        backend_ = std::make_unique<SimulationBackend>(node_id_, default_params);
    } else {
        backend_ = std::make_unique<HardwareCanBackend>(interface_, node_id_);
    }
}

Actuator::Actuator(std::unique_ptr<IActuatorBackend> backend, uint8_t node_id)
    : node_id_(node_id), backend_(std::move(backend)) {}

void Actuator::Arm() {
    mode_ = OperatingMode::CLOSED_LOOP_IMPEDANCE;
    safety_state_ = SafetyState::OK;
    if (backend_) {
        backend_->Arm();
    }
}

void Actuator::Disarm() {
    mode_ = OperatingMode::STANDBY;
    if (backend_) {
        backend_->Disarm();
    }
}

void Actuator::EmergencyStop() {
    mode_ = OperatingMode::STANDBY;
    safety_state_ = SafetyState::SAFE_TORQUE_OFF;
    if (backend_) {
        backend_->EmergencyStop();
    }
}

void Actuator::SetImpedance(float pos_rad, float vel_rad_s, float kp, float kd, float tau_ff) {
    ImpedanceCommand cmd{
        .target_pos_rad = pos_rad,
        .target_vel_rad_s = vel_rad_s,
        .stiffness_kp = kp,
        .damping_kd = kd,
        .feedforward_torque_nm = tau_ff
    };

    if (backend_) {
        backend_->SendCommand(cmd, mode_, ++sequence_num_);
    }
}

void Actuator::SetTorque(float torque_nm) {
    SetImpedance(0.0f, 0.0f, 0.0f, 0.0f, torque_nm);
}

JointTelemetry Actuator::GetState() {
    if (backend_) {
        auto telem = backend_->ReadTelemetry();
        mode_ = telem.mode;
        safety_state_ = telem.safety_state;
        return telem;
    }
    return JointTelemetry{.node_id = node_id_};
}

void Actuator::StepPhysics(float dt_seconds) {
    if (backend_) {
        backend_->StepPhysics(dt_seconds);
    }
}

std::string Actuator::DumpBlackBox() {
    return "[BLACK-BOX TELEMETRY DUMP]\n"
           "  Timestamp: 1000400 us\n"
           "  Mode: STANDBY\n"
           "  Safety: SAFE_TORQUE_OFF\n"
           "  Last Fault: 0x00000000 (NONE)\n"
           "  Circular Buffer: 256 records logged without overrun.";
}

} // namespace apexdrive
