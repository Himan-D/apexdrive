#pragma once

#include "../core/types.hpp"
#include "../control/foc_core.hpp"
#include "../core/anti_cogging.hpp"
#include "../core/sliding_mode_observer.hpp"
#include "can_socket.hpp"
#include <memory>

namespace apexdrive {

/**
 * Abstract Actuator Backend Interface.
 * Completely decouples physical SocketCAN-FD hardware communication from simulation testbenches.
 */
class IActuatorBackend {
public:
    virtual ~IActuatorBackend() = default;

    virtual void Arm() = 0;
    virtual void Disarm() = 0;
    virtual void EmergencyStop() = 0;
    virtual bool SendCommand(const ImpedanceCommand& cmd, OperatingMode mode, uint16_t sequence_num) = 0;
    virtual JointTelemetry ReadTelemetry() = 0;
    virtual void StepPhysics(float dt_seconds) = 0;
    [[nodiscard]] virtual bool IsHardware() const noexcept = 0;
};

/**
 * Physical Hardware Backend: Transmits over SocketCAN-FD to physical STM32 Inverter.
 */
class HardwareCanBackend : public IActuatorBackend {
public:
    HardwareCanBackend(std::string interface_name, uint8_t node_id)
        : node_id_(node_id), transport_(std::move(interface_name), false) {
        last_telemetry_.node_id = node_id_;
    }

    void Arm() override {
        mode_ = OperatingMode::CLOSED_LOOP_IMPEDANCE;
        safety_state_ = SafetyState::OK;
    }

    void Disarm() override {
        mode_ = OperatingMode::STANDBY;
    }

    void EmergencyStop() override {
        mode_ = OperatingMode::STANDBY;
        safety_state_ = SafetyState::SAFE_TORQUE_OFF;
    }

    bool SendCommand(const ImpedanceCommand& cmd, OperatingMode mode, uint16_t sequence_num) override {
        mode_ = mode;
        return transport_.SendCommand(node_id_, cmd, mode_, sequence_num);
    }

    JointTelemetry ReadTelemetry() override {
        auto telem = transport_.ReceiveTelemetry(1);
        if (telem.has_value() && telem->node_id == node_id_) {
            last_telemetry_ = *telem;
        }
        return last_telemetry_;
    }

    void StepPhysics(float /*dt_seconds*/) override {
        // Physical hardware runs its own 25 kHz FOC loop on the MCU
    }

    [[nodiscard]] bool IsHardware() const noexcept override { return true; }

private:
    uint8_t node_id_;
    OperatingMode mode_{OperatingMode::STANDBY};
    SafetyState safety_state_{SafetyState::OK};
    CanTransport transport_;
    JointTelemetry last_telemetry_{};
};

/**
 * High-Precision Continuous Simulation Backend: Runs 25 kHz closed-loop PMSM physics & FocEngine.
 */
class SimulationBackend : public IActuatorBackend {
public:
    SimulationBackend(uint8_t node_id, const MotorParameters& profile)
        : node_id_(node_id), profile_(profile), foc_engine_(profile),
          smo_observer_(profile.phase_resistance_ohm, profile.inductance_q_h, 25.0f) {
        foc_engine_.SetCurrentLoopBandwidth(1500.0f);
        last_telemetry_.node_id = node_id_;
    }

    void Arm() override {
        mode_ = OperatingMode::CLOSED_LOOP_IMPEDANCE;
        safety_state_ = SafetyState::OK;
    }

    void Disarm() override {
        mode_ = OperatingMode::STANDBY;
    }

    void EmergencyStop() override {
        mode_ = OperatingMode::STANDBY;
        safety_state_ = SafetyState::SAFE_TORQUE_OFF;
    }

    bool SendCommand(const ImpedanceCommand& cmd, OperatingMode mode, uint16_t sequence_num) override {
        last_cmd_ = cmd;
        mode_ = mode;
        sequence_num_ = sequence_num;
        time_since_last_cmd_ = 0.0f;
        return true;
    }

    JointTelemetry ReadTelemetry() override {
        last_telemetry_.node_id = node_id_;
        last_telemetry_.mode = mode_;
        last_telemetry_.safety_state = safety_state_;
        last_telemetry_.position_rad = sim_pos_rad_;
        last_telemetry_.velocity_rad_s = sim_vel_rad_s_;
        last_telemetry_.torque_nm = FocMath::ComputeElectromagneticTorque(
            sim_current_id_, sim_current_iq_, profile_.pole_pairs,
            profile_.flux_linkage_wb, profile_.inductance_d_h, profile_.inductance_q_h
        );
        last_telemetry_.current_iq_a = sim_current_iq_;
        last_telemetry_.current_id_a = sim_current_id_;
        last_telemetry_.v_bus_v = 48.0f;
        last_telemetry_.temperature_c = 35.0f;
        last_telemetry_.fault_flags = fault_flags_;
        last_telemetry_.sequence_number = sequence_num_;
        return last_telemetry_;
    }

    void StepPhysics(float dt_seconds) override {
        time_since_last_cmd_ += dt_seconds;
        if (time_since_last_cmd_ > profile_.command_timeout_sec) {
            safety_state_ = SafetyState::FAULT_STOP;
            fault_flags_ |= FaultFlag::WATCHDOG_TIMEOUT;
        }

        if (safety_state_ == SafetyState::OK && mode_ != OperatingMode::STANDBY) {
            const float kt = profile_.GetDerivedKt();
            float tau_impedance = last_cmd_.stiffness_kp * (last_cmd_.target_pos_rad - sim_pos_rad_) +
                                  last_cmd_.damping_kd * (last_cmd_.target_vel_rad_s - sim_vel_rad_s_) +
                                  last_cmd_.feedforward_torque_nm;
            float iq_cmd = std::clamp(tau_impedance / kt, -profile_.peak_current_a, profile_.peak_current_a);

            float theta_e = sim_pos_rad_ * static_cast<float>(profile_.pole_pairs);
            FocMath::DirectQuadrature dq_meas{.d = sim_current_id_, .q = sim_current_iq_};
            FocMath::AlphaBeta ab_meas = FocMath::InversePark(dq_meas, theta_e);
            auto i_meas = FocMath::InverseClarke(ab_meas);

            FocEngine::FocInputs foc_in{
                .i_phase_a = i_meas.u,
                .i_phase_b = i_meas.v,
                .i_phase_c = i_meas.w,
                .electrical_angle_rad = theta_e,
                .electrical_speed_rad_s = sim_vel_rad_s_ * static_cast<float>(profile_.pole_pairs),
                .target_id_a = 0.0f,
                .target_iq_a = iq_cmd,
                .v_bus = 48.0f
            };

            auto out = foc_engine_.Step(foc_in, dt_seconds);

            // Step continuous differential PMSM dynamics
            float d_id = (out.v_d_sat - profile_.phase_resistance_ohm * sim_current_id_ + foc_in.electrical_speed_rad_s * profile_.inductance_q_h * sim_current_iq_) / profile_.inductance_d_h;
            float d_iq = (out.v_q_sat - profile_.phase_resistance_ohm * sim_current_iq_ - foc_in.electrical_speed_rad_s * profile_.inductance_d_h * sim_current_id_ - foc_in.electrical_speed_rad_s * profile_.flux_linkage_wb) / profile_.inductance_q_h;

            sim_current_id_ += d_id * dt_seconds;
            sim_current_iq_ += d_iq * dt_seconds;

            float tau_em = FocMath::ComputeElectromagneticTorque(
                sim_current_id_, sim_current_iq_, profile_.pole_pairs,
                profile_.flux_linkage_wb, profile_.inductance_d_h, profile_.inductance_q_h
            );

            float friction = (sim_vel_rad_s_ > 0.001f) ? profile_.coulomb_friction_nm : (sim_vel_rad_s_ < -0.001f ? -profile_.coulomb_friction_nm : 0.0f);
            float accel = (tau_em - (sim_vel_rad_s_ * profile_.viscous_friction_b) - friction) / profile_.rotor_inertia_kgm2;

            sim_vel_rad_s_ += accel * dt_seconds;
            sim_pos_rad_ += sim_vel_rad_s_ * dt_seconds;
        } else {
            sim_current_id_ = 0.0f;
            sim_current_iq_ = 0.0f;
        }
    }

    [[nodiscard]] bool IsHardware() const noexcept override { return false; }

private:
    uint8_t node_id_;
    MotorParameters profile_;
    FocEngine foc_engine_;
    SlidingModeObserver smo_observer_;
    OperatingMode mode_{OperatingMode::STANDBY};
    SafetyState safety_state_{SafetyState::OK};
    uint32_t fault_flags_{FaultFlag::NONE};
    ImpedanceCommand last_cmd_{};
    uint16_t sequence_num_{0};
    float time_since_last_cmd_{0.0f};

    float sim_pos_rad_{0.0f};
    float sim_vel_rad_s_{0.0f};
    float sim_current_id_{0.0f};
    float sim_current_iq_{0.0f};
    JointTelemetry last_telemetry_{};
};

} // namespace apexdrive
