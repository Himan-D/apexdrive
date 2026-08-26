#include "../../include/apexdrive/host/actuator.hpp"
#include <iostream>
#include <cmath>

namespace apexdrive {

Actuator::Actuator(std::string interface_name, uint8_t node_id, bool mock_mode)
    : interface_(std::move(interface_name)),
      node_id_(node_id),
      mock_mode_(mock_mode),
      safety_guard_(profile_),
      smo_observer_(profile_.phase_resistance_ohm, profile_.phase_inductance_h, 25.0f) {
    last_telemetry_.node_id = node_id_;
    last_telemetry_.state = state_;
    last_telemetry_.v_bus_v = sim_v_bus_;
}

void Actuator::Arm() {
    if (state_ == DriveState::STANDBY || state_ == DriveState::UNINITIALIZED) {
        state_ = DriveState::ARMED;
        current_pid_d_.Reset();
        current_pid_q_.Reset();
        safety_guard_.Reset();
    }
}

void Actuator::Disarm() {
    state_ = DriveState::STANDBY;
    last_command_ = ImpedanceCommand{};
}

void Actuator::EmergencyStop() {
    state_ = DriveState::FAULT_WATCHDOG_TIMEOUT;
    last_command_ = ImpedanceCommand{};
    flight_recorder_.FreezeOnFault();
}

void Actuator::SetTorque(float torque_nm) {
    SetImpedance(0.0f, 0.0f, 0.0f, 0.0f, torque_nm);
    if (state_ == DriveState::ARMED) state_ = DriveState::CLOSED_LOOP_TORQUE;
}

void Actuator::SetPosition(float target_pos_rad, float max_vel_rad_s) {
    SetImpedance(target_pos_rad, max_vel_rad_s, 60.0f, 3.5f, 0.0f);
    if (state_ == DriveState::ARMED) state_ = DriveState::CLOSED_LOOP_POSITION;
}

void Actuator::SetImpedance(float pos_rad, float vel_rad_s, float kp, float kd, float tau_ff) {
    last_command_ = ImpedanceCommand{
        .target_pos_rad = pos_rad,
        .target_vel_rad_s = vel_rad_s,
        .stiffness_kp = kp,
        .damping_kd = kd,
        .feedforward_torque_nm = tau_ff
    };
    if (state_ == DriveState::ARMED) {
        state_ = DriveState::CLOSED_LOOP_IMPEDANCE;
    }
}

JointTelemetry Actuator::GetState() const {
    return last_telemetry_;
}

AutoTuner::TuningResult Actuator::Calibrate() {
    DriveState prev_state = state_;
    state_ = DriveState::CALIBRATING;
    auto result = AutoTuner::RunAutoCalibration(profile_.phase_resistance_ohm, profile_.phase_inductance_h, profile_.torque_constant_kt);
    
    // Apply tuned current loop gains & anti-cogging LUT
    current_pid_d_.SetGains(result.optimal_current_kp, result.optimal_current_ki, profile_.max_voltage_v);
    current_pid_q_.SetGains(result.optimal_current_kp, result.optimal_current_ki, profile_.max_voltage_v);
    anti_cogging_map_ = result.anti_cogging_lut;
    profile_.encoder_offset_rad = result.encoder_offset_rad;

    state_ = prev_state;
    return result;
}

std::string Actuator::DumpBlackBox() {
    return flight_recorder_.DumpForensicReport();
}

void Actuator::StepPhysics(float dt_s) {
    // 1. Calculate Target Current from Impedance Controller + Anti-Cogging Feedforward
    float target_iq = 0.0f;
    float target_id = 0.0f;

    if (state_ == DriveState::CLOSED_LOOP_IMPEDANCE || 
        state_ == DriveState::CLOSED_LOOP_TORQUE || 
        state_ == DriveState::CLOSED_LOOP_POSITION) {
        
        // Base impedance torque
        float base_iq = ImpedanceController::ComputeTorqueCurrent(
            last_command_, sim_pos_rad_, sim_vel_rad_s_, 
            profile_.torque_constant_kt, profile_.peak_current_a
        );

        // Anti-cogging torque compensation
        float tau_cogging_comp = anti_cogging_map_.Lookup(sim_pos_rad_);
        float iq_cogging = tau_cogging_comp / profile_.torque_constant_kt;

        target_iq = base_iq + iq_cogging;

        // Dynamic Field Weakening at high velocity
        float v_mag_est = std::sqrt(std::pow(sim_vel_rad_s_ * profile_.torque_constant_kt, 2.0f) + 
                                    std::pow(sim_current_iq_ * profile_.phase_resistance_ohm, 2.0f));
        target_id = FocMath::ComputeFieldWeakeningId(v_mag_est, sim_v_bus_, 15.0f);
    }

    // 2. Simulated Motor Dynamics
    const float J = 0.00045f;
    const float B = 0.0005f;
    
    // First-order current loop lag
    sim_current_iq_ += (target_iq - sim_current_iq_) * (dt_s / 0.0008f);
    sim_current_id_ += (target_id - sim_current_id_) * (dt_s / 0.0008f);

    // Motor Torque with Field Weakening (Reluctance torque + PM torque)
    float torque_motor = sim_current_iq_ * profile_.torque_constant_kt;
    
    // Acceleration: alpha = (Torque - B * vel) / J
    float accel = (torque_motor - B * sim_vel_rad_s_) / J;
    sim_vel_rad_s_ += accel * dt_s;
    sim_pos_rad_ += sim_vel_rad_s_ * dt_s;

    // Thermal heating: Stator winding Joule heating
    float total_current_sq = sim_current_iq_ * sim_current_iq_ + sim_current_id_ * sim_current_id_;
    float joule_power = total_current_sq * profile_.phase_resistance_ohm;
    sim_winding_temp_c_ += (joule_power * 0.04f - (sim_winding_temp_c_ - 30.0f) * 0.01f) * dt_s;

    // Inrush bus sag
    float bus_drop = std::sqrt(total_current_sq) * 0.035f;
    sim_v_bus_ = 48.0f - bus_drop;

    // 3. Sensor Readings
    SensorReadings s;
    float theta_e = sim_pos_rad_ * profile_.pole_pairs;
    s.i_phase_a = sim_current_iq_ * std::sin(theta_e) + sim_current_id_ * std::cos(theta_e);
    s.i_phase_b = sim_current_iq_ * std::sin(theta_e - 2.094395f) + sim_current_id_ * std::cos(theta_e - 2.094395f);
    s.i_phase_c = -(s.i_phase_a + s.i_phase_b);
    s.v_bus = sim_v_bus_;
    s.rotor_angle_rad = sim_pos_rad_;
    s.rotor_speed_rad_s = sim_vel_rad_s_;
    s.mosfet_temp_c = 35.0f + joule_power * 0.02f;
    s.winding_temp_c = sim_winding_temp_c_;

    // 4. Update Sliding Mode Sensorless Observer
    auto ab_currents = FocMath::Clarke(s.i_phase_a, s.i_phase_b, s.i_phase_c);
    smo_observer_.Update(sim_v_bus_ * 0.5f, sim_v_bus_ * 0.5f, ab_currents.alpha, ab_currents.beta, dt_s);

    // 5. Run Sub-microsecond Hardware eFuse Checks
    DriveState fault = DriveState::STANDBY;
    if (!safety_guard_.CheckHealth(s, dt_s, fault)) {
        state_ = fault;
        flight_recorder_.FreezeOnFault();
    }

    // 6. Record into Flight Black Box
    flight_recorder_.RecordSample(static_cast<uint32_t>(dt_s * 1e6f), s, state_);

    // 7. Update Telemetry
    last_telemetry_.node_id = node_id_;
    last_telemetry_.state = state_;
    last_telemetry_.position_rad = sim_pos_rad_;
    last_telemetry_.velocity_rad_s = sim_vel_rad_s_;
    last_telemetry_.torque_nm = torque_motor;
    last_telemetry_.current_iq_a = sim_current_iq_;
    last_telemetry_.v_bus_v = sim_v_bus_;
    last_telemetry_.temperature_c = sim_winding_temp_c_;
}

} // namespace apexdrive
