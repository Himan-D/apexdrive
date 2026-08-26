#include "../../include/apexdrive/host/actuator.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

namespace apexdrive {

Actuator::Actuator(std::string interface_name, uint8_t node_id, bool mock_mode)
    : interface_(std::move(interface_name)),
      node_id_(node_id),
      mock_mode_(mock_mode),
      safety_guard_(profile_),
      smo_observer_(profile_.phase_resistance_ohm, profile_.inductance_q_h, 25.0f) {
    last_telemetry_.node_id = node_id_;
    last_telemetry_.mode = mode_;
    last_telemetry_.safety_state = safety_state_;
    last_telemetry_.v_bus_v = sim_v_bus_;
}

void Actuator::Arm() {
    if (safety_state_ == SafetyState::OK) {
        mode_ = OperatingMode::CLOSED_LOOP_IMPEDANCE;
        current_pid_d_.Reset();
        current_pid_q_.Reset();
        safety_guard_.Reset();
        safety_guard_.FeedWatchdog();
    }
}

void Actuator::Disarm() {
    mode_ = OperatingMode::STANDBY;
    last_command_ = ImpedanceCommand{};
}

void Actuator::EmergencyStop() {
    safety_state_ = SafetyState::SAFE_TORQUE_OFF;
    mode_ = OperatingMode::STANDBY;
    fault_flags_ |= FaultFlag::WATCHDOG_TIMEOUT;
    last_command_ = ImpedanceCommand{};
    flight_recorder_.FreezeOnFault();
}

void Actuator::SetTorque(float torque_nm) {
    SetImpedance(0.0f, 0.0f, 0.0f, 0.0f, torque_nm);
    if (safety_state_ == SafetyState::OK) mode_ = OperatingMode::CLOSED_LOOP_TORQUE;
}

void Actuator::SetPosition(float target_pos_rad, float max_vel_rad_s) {
    SetImpedance(target_pos_rad, max_vel_rad_s, 60.0f, 3.5f, 0.0f);
    if (safety_state_ == SafetyState::OK) mode_ = OperatingMode::CLOSED_LOOP_POSITION;
}

void Actuator::SetImpedance(float pos_rad, float vel_rad_s, float kp, float kd, float tau_ff) {
    ImpedanceCommand cmd{
        .target_pos_rad = pos_rad,
        .target_vel_rad_s = vel_rad_s,
        .stiffness_kp = kp,
        .damping_kd = kd,
        .feedforward_torque_nm = tau_ff
    };

    if (!cmd.IsValid()) {
        fault_flags_ |= FaultFlag::COMMAND_OUT_OF_BOUNDS;
        safety_state_ = SafetyState::FAULT_STOP;
        return;
    }

    last_command_ = cmd;
    safety_guard_.FeedWatchdog();

    if (safety_state_ == SafetyState::OK && mode_ == OperatingMode::STANDBY) {
        mode_ = OperatingMode::CLOSED_LOOP_IMPEDANCE;
    }
}

JointTelemetry Actuator::GetState() const {
    return last_telemetry_;
}

AutoTuner::TuningResult Actuator::Calibrate() {
    OperatingMode prev_mode = mode_;
    mode_ = OperatingMode::CALIBRATING;
    auto result = AutoTuner::RunAutoCalibration(profile_.phase_resistance_ohm, profile_.inductance_q_h, profile_.torque_constant_kt);
    
    // Apply tuned current loop gains & anti-cogging LUT
    current_pid_d_.SetGains(result.optimal_current_kp, result.optimal_current_ki, profile_.max_voltage_v);
    current_pid_q_.SetGains(result.optimal_current_kp, result.optimal_current_ki, profile_.max_voltage_v);
    anti_cogging_map_ = result.anti_cogging_lut;
    profile_.encoder_offset_rad = result.encoder_offset_rad;

    mode_ = prev_mode;
    return result;
}

std::string Actuator::DumpBlackBox() {
    return flight_recorder_.DumpForensicReport();
}

void Actuator::StepPhysics(float dt_s) {
    current_timestamp_us_ += static_cast<uint64_t>(dt_s * 1e6f);

    // 1. Calculate Commanded Currents (Iq, Id)
    float target_iq = 0.0f;
    float target_id = 0.0f;

    if (safety_state_ == SafetyState::OK && mode_ != OperatingMode::STANDBY && mode_ != OperatingMode::CALIBRATING) {
        // Base impedance torque
        const float kt = profile_.GetDerivedKt();
        float base_iq = ImpedanceController::ComputeTorqueCurrent(
            last_command_, sim_pos_rad_, sim_vel_rad_s_, 
            kt, profile_.peak_current_a
        );

        // Anti-cogging feedforward torque compensation
        float tau_cogging_comp = anti_cogging_map_.Lookup(sim_pos_rad_);
        float iq_cogging = tau_cogging_comp / kt;
        target_iq = std::clamp(base_iq + iq_cogging, -profile_.peak_current_a, profile_.peak_current_a);

        // Dynamic Field Weakening at high speed
        float omega_e = sim_vel_rad_s_ * profile_.pole_pairs;
        float v_mag_est = std::sqrt(std::pow(omega_e * profile_.flux_linkage_wb, 2.0f) + 
                                    std::pow(target_iq * profile_.phase_resistance_ohm, 2.0f));
        target_id = FocMath::ComputeFieldWeakeningId(v_mag_est, sim_v_bus_, 15.0f);
    }

    // 2. Closed-Loop FOC Inverter Execution:
    // Rotor electrical angle
    float theta_e = (sim_pos_rad_ * profile_.pole_pairs) - profile_.encoder_offset_rad;
    float omega_e = sim_vel_rad_s_ * profile_.pole_pairs;

    // Current PI Regulators with Back-Calculation Anti-Windup
    float err_d = target_id - sim_current_id_;
    float err_q = target_iq - sim_current_iq_;
    float v_d_pi = current_pid_d_.Update(err_d, dt_s);
    float v_q_pi = current_pid_q_.Update(err_q, dt_s);

    // Cross-Coupling Voltage Decoupling Feedforward
    auto v_dq_decoupled = FocMath::DecoupleCrossCoupling(
        {v_d_pi, v_q_pi}, {sim_current_id_, sim_current_iq_},
        omega_e, profile_.inductance_d_h, profile_.inductance_q_h, profile_.flux_linkage_wb
    );

    // Inverse Park -> Modulated Alpha/Beta Voltages
    auto v_ab_mod = FocMath::InversePark(v_dq_decoupled, theta_e);

    // Space Vector PWM Modulation
    auto duties = FocMath::Svpwm(v_ab_mod, sim_v_bus_);
    (void)duties;

    // 3. Continuous Salient PMSM Differential Equations:
    // dId/dt = (Vd - Rs*Id + omega_e*Lq*Iq) / Ld
    // dIq/dt = (Vq - Rs*Iq - omega_e*Ld*Id - omega_e*psi_f) / Lq
    float d_id_dt = (v_dq_decoupled.d - profile_.phase_resistance_ohm * sim_current_id_ + omega_e * profile_.inductance_q_h * sim_current_iq_) / profile_.inductance_d_h;
    float d_iq_dt = (v_dq_decoupled.q - profile_.phase_resistance_ohm * sim_current_iq_ - omega_e * profile_.inductance_d_h * sim_current_id_ - omega_e * profile_.flux_linkage_wb) / profile_.inductance_q_h;

    sim_current_id_ += d_id_dt * dt_s;
    sim_current_iq_ += d_iq_dt * dt_s;

    // 4. Electromagnetic Torque with Saliency (Reluctance + PM Torque)
    float torque_em = FocMath::ComputeElectromagneticTorque(
        sim_current_id_, sim_current_iq_, profile_.pole_pairs,
        profile_.flux_linkage_wb, profile_.inductance_d_h, profile_.inductance_q_h
    );

    // 5. Mechanical Dynamics with Coulomb + Viscous Friction
    float sign_vel = (sim_vel_rad_s_ > 1e-3f) ? 1.0f : ((sim_vel_rad_s_ < -1e-3f) ? -1.0f : 0.0f);
    float friction_torque = (profile_.viscous_friction_b * sim_vel_rad_s_) + (profile_.coulomb_friction_nm * sign_vel);
    float accel = (torque_em - friction_torque) / profile_.rotor_inertia_kgm2;

    sim_vel_rad_s_ += accel * dt_s;
    sim_pos_rad_ += sim_vel_rad_s_ * dt_s;

    // Thermal dissipation and Joule heating
    float total_current_sq = (sim_current_iq_ * sim_current_iq_) + (sim_current_id_ * sim_current_id_);
    float joule_power = total_current_sq * profile_.phase_resistance_ohm;
    sim_winding_temp_c_ += (joule_power * 0.04f - (sim_winding_temp_c_ - 25.0f) * 0.01f) * dt_s;
    sim_mosfet_temp_c_ += (joule_power * 0.02f - (sim_mosfet_temp_c_ - 25.0f) * 0.015f) * dt_s;

    // DC Bus Inrush Voltage Drop
    sim_v_bus_ = 48.0f - (std::sqrt(total_current_sq) * 0.035f);

    // 6. Synthesize Phase Currents for Sensor Interface
    SensorReadings s;
    s.i_phase_a = sim_current_iq_ * std::sin(theta_e) + sim_current_id_ * std::cos(theta_e);
    s.i_phase_b = sim_current_iq_ * std::sin(theta_e - 2.094395f) + sim_current_id_ * std::cos(theta_e - 2.094395f);
    s.i_phase_c = -(s.i_phase_a + s.i_phase_b);
    s.v_bus = sim_v_bus_;
    s.rotor_angle_rad = sim_pos_rad_;
    s.rotor_speed_rad_s = sim_vel_rad_s_;
    s.mosfet_temp_c = sim_mosfet_temp_c_;
    s.winding_temp_c = sim_winding_temp_c_;

    // 7. Update Sliding Mode Sensorless Observer with ACTUAL Modulated Voltages
    auto ab_currents = FocMath::Clarke(s.i_phase_a, s.i_phase_b, s.i_phase_c);
    smo_observer_.Update(v_ab_mod.alpha, v_ab_mod.beta, ab_currents.alpha, ab_currents.beta, dt_s);

    // 8. Hardware Safety Supervisor
    if (!safety_guard_.CheckHealth(s, dt_s, safety_state_, fault_flags_)) {
        mode_ = OperatingMode::STANDBY;
        flight_recorder_.FreezeOnFault();
    }

    // 9. Flight Black-Box Recording
    flight_recorder_.RecordSample(current_timestamp_us_, s, mode_, safety_state_, fault_flags_);

    // 10. Update Telemetry
    last_telemetry_.node_id = node_id_;
    last_telemetry_.mode = mode_;
    last_telemetry_.safety_state = safety_state_;
    last_telemetry_.position_rad = sim_pos_rad_;
    last_telemetry_.velocity_rad_s = sim_vel_rad_s_;
    last_telemetry_.torque_nm = torque_em;
    last_telemetry_.current_iq_a = sim_current_iq_;
    last_telemetry_.current_id_a = sim_current_id_;
    last_telemetry_.v_bus_v = sim_v_bus_;
    last_telemetry_.temperature_c = sim_winding_temp_c_;
    last_telemetry_.fault_flags = fault_flags_;
    last_telemetry_.timestamp_us = current_timestamp_us_;
}

} // namespace apexdrive
