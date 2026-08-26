#include "../include/apexdrive/core/types.hpp"
#include "../include/apexdrive/control/foc_core.hpp"
#include "../include/apexdrive/control/mtpa_optimizer.hpp"
#include "../include/apexdrive/sim/isaac_mujoco_bridge.hpp"
#include "../include/apexdrive/host/actuator.hpp"
#include "../include/apexdrive/protocol/can_protocol_v2.hpp"

#include <iostream>
#include <cassert>
#include <cmath>

using namespace apexdrive;

void test_scenario_position_step() {
    std::cout << "[SCENARIO 1/6] Running 1.0 rad Position Step Closed-Loop Response... ";

    Actuator joint("can0", 0x14, true);
    joint.Arm();

    const float target_pos = 1.0f;
    // Stream 1 kHz command loop for 500 ms (500 outer loops * 25 FOC cycles = 12,500 FOC cycles)
    for (int ms = 0; ms < 500; ++ms) {
        joint.SetImpedance(target_pos, 0.0f, 60.0f, 3.5f, 0.0f);
        for (int i = 0; i < 25; ++i) {
            joint.StepPhysics(0.00004f);
        }
    }

    auto state = joint.GetState();
    float steady_state_error = std::abs(state.position_rad - target_pos);
    std::cout << "(Pos: " << state.position_rad << " rad, Err: " << steady_state_error << ") ";

    // Verify steady-state convergence within 5% of target
    assert(steady_state_error < 0.05f);
    std::cout << "\033[1;32mPASSED (Error: " << (steady_state_error * 1000.0f) << " mrad)\033[0m\n";
}

void test_scenario_torque_step() {
    std::cout << "[SCENARIO 2/6] Running 2.0 Nm Torque Step Closed-Loop Response... ";

    Actuator joint("can0", 0x14, true);
    joint.Arm();

    // Stream 1 kHz torque commands for 100 ms
    for (int ms = 0; ms < 100; ++ms) {
        joint.SetTorque(2.0f);
        for (int i = 0; i < 25; ++i) {
            joint.StepPhysics(0.00004f);
        }
    }

    auto state = joint.GetState();
    assert(state.torque_nm > 1.8f && state.torque_nm < 2.2f);
    std::cout << "\033[1;32mPASSED (Torque: " << state.torque_nm << " Nm)\033[0m\n";
}

void test_scenario_voltage_vector_saturation() {
    std::cout << "[SCENARIO 3/6] Running Vector-Space Voltage Saturation & Recovery... ";

    MotorParameters params;
    FocEngine engine(params);
    engine.SetCurrentLoopBandwidth(1500.0f);

    // Request huge current step exceeding bus voltage limit
    FocEngine::FocInputs in{
        .i_phase_a = 0.0f,
        .i_phase_b = 0.0f,
        .i_phase_c = 0.0f,
        .electrical_angle_rad = 0.5f,
        .electrical_speed_rad_s = 200.0f,
        .target_id_a = 0.0f,
        .target_iq_a = 40.0f, // Heavy torque demand at high speed
        .v_bus = 48.0f
    };

    auto out = engine.Step(in, 0.00004f);
    
    // Total voltage vector magnitude must never exceed V_max_available
    assert(out.is_saturated == true);
    assert(out.v_mag >= out.v_max_available);
    float sat_mag = std::sqrt(out.v_d_sat * out.v_d_sat + out.v_q_sat * out.v_q_sat);
    assert(sat_mag <= out.v_max_available + 1e-3f);

    std::cout << "\033[1;32mPASSED (Clamped to " << sat_mag << " V <= " << out.v_max_available << " V)\033[0m\n";
}

void test_scenario_can_watchdog_timeout() {
    std::cout << "[SCENARIO 4/6] Running 25ms CAN Command Watchdog Heartbeat Timeout... ";

    Actuator joint("can0", 0x14, true);
    joint.Arm();

    // Send valid command
    joint.SetImpedance(1.0f, 0.0f, 40.0f, 2.0f, 0.0f);
    assert(joint.GetSafetyState() == SafetyState::OK);

    // Cease sending commands and simulate 30ms of elapsed time
    for (int i = 0; i < 750; ++i) { // 750 * 40us = 30ms (> 25ms timeout)
        joint.StepPhysics(0.00004f);
    }

    auto state = joint.GetState();
    assert(state.safety_state == SafetyState::FAULT_STOP);
    assert((state.fault_flags & FaultFlag::WATCHDOG_TIMEOUT) != 0);
    (void)state;

    std::cout << "\033[1;32mPASSED (Safe Stop Triggered)\033[0m\n";
}

void test_scenario_can_v2_crc_validation() {
    std::cout << "[SCENARIO 5/6] Testing CAN-FD Protocol v2 CRC16 Corruption Rejection... ";

    ImpedanceCommand cmd{1.57f, 5.0f, 60.0f, 3.0f, 1.5f};
    uint8_t buffer[16];
    CanProtocolV2::EncodeCommand(cmd, OperatingMode::CLOSED_LOOP_IMPEDANCE, 1042, buffer);

    ImpedanceCommand decoded_cmd;
    OperatingMode decoded_mode;
    uint16_t decoded_seq;

    // 1. Valid Frame
    bool ok = CanProtocolV2::DecodeCommand(buffer, decoded_cmd, decoded_mode, decoded_seq);
    assert(ok == true);
    assert(decoded_seq == 1042);
    assert(decoded_mode == OperatingMode::CLOSED_LOOP_IMPEDANCE);
    (void)ok;

    // 2. Corrupt 1 bit in payload
    buffer[4] ^= 0x01;
    bool corrupt_ok = CanProtocolV2::DecodeCommand(buffer, decoded_cmd, decoded_mode, decoded_seq);
    assert(corrupt_ok == false); // Corrupted CRC rejected!
    (void)corrupt_ok;

    std::cout << "\033[1;32mPASSED\033[0m\n";
}

void test_scenario_motor_parameter_consistency() {
    std::cout << "[SCENARIO 6/6] Verifying Authoritative Motor Parameters & Derived Constants... ";

    MotorParameters params;
    float derived_kt = params.GetDerivedKt(); // 1.5 * 7 * 0.0068 = 0.0714 Nm/A
    float derived_ke = params.GetDerivedKe();
    float base_speed = params.GetBaseSpeed(48.0f);

    assert(derived_kt > 0.0f);
    assert(derived_ke > 0.0f);
    assert(base_speed > 100.0f);
    (void)derived_ke;
    (void)base_speed;

    // Verify peak vs RMS conversions
    float peak_current = 25.0f;
    float rms_current = Units::PhasePeakToRms(peak_current);
    assert(std::abs(Units::RmsToPhasePeak(rms_current) - peak_current) < 1e-4f);
    (void)rms_current;

    std::cout << "\033[1;32mPASSED (Derived Kt: " << derived_kt << " Nm/A)\033[0m\n";
}

void test_scenario_mtpa_and_field_weakening() {
    std::cout << "[SCENARIO 7/8] Testing Salient MTPA & Closed-Loop Field Weakening... ";

    MotorParameters params;
    params.inductance_d_h = 0.00010f; // Salient IPMSM: Lq > Ld
    params.inductance_q_h = 0.00015f;
    MtpaOptimizer mtpa(params);

    // 1. Demand 0.5 Nm torque under base speed
    auto opt1 = mtpa.ComputeOptimalCurrents(0.5f, 48.0f, 100.0f, 0.001f);
    assert(opt1.target_id_a < 0.0f); // Reluctance d-axis injection for salient machine!
    assert(opt1.target_iq_a > 0.0f);

    // 2. High-speed overspeed field weakening (5000 rad/s) for 50 ms
    MtpaOptimizer::OptimalCurrentVector opt2{};
    for (int i = 0; i < 50; ++i) {
        opt2 = mtpa.ComputeOptimalCurrents(0.5f, 48.0f, 5000.0f, 0.001f);
    }
    assert(opt2.target_id_a < opt1.target_id_a); // Deeper negative Id injected!

    std::cout << "\033[1;32mPASSED (MTPA Id: " << opt1.target_id_a << " A, FW Id: " << opt2.target_id_a << " A)\033[0m\n";
}

void test_scenario_isaac_mujoco_bridge() {
    std::cout << "[SCENARIO 8/8] Testing NVIDIA Isaac Sim & MuJoCo Joint Actuator Bridge... ";

    sim::IsaacMujocoJointBridge joint(0x10);
    joint.SetCommand(ImpedanceCommand{
        .target_pos_rad = 1.57f,
        .target_vel_rad_s = 0.0f,
        .stiffness_kp = 50.0f,
        .damping_kd = 2.5f,
        .feedforward_torque_nm = 0.5f
    });

    // Step physics at 1 kHz for 100 ms
    float current_pos = 0.0f;
    float current_vel = 0.0f;
    for (int ms = 0; ms < 100; ++ms) {
        float torque = joint.StepJointDynamics(current_pos, current_vel, 0.001f);
        float accel = torque / 0.00045f;
        current_vel += accel * 0.001f;
        current_pos += current_vel * 0.001f;
    }

    auto state = joint.GetJointState();
    assert(std::isfinite(state.applied_torque_nm));
    assert(state.position_rad > 0.5f); // Converging towards target 1.57 rad!

    std::cout << "\033[1;32mPASSED (Isaac Sim Applied Torque: " << state.applied_torque_nm << " Nm)\033[0m\n";
}

int main() {
    std::cout << "\n======================================================================\n";
    std::cout << "  RUNNING APEXDRIVE CLOSED-LOOP HARDWARE-READINESS TEST SCENARIOS     \n";
    std::cout << "======================================================================\n";

    test_scenario_position_step();
    test_scenario_torque_step();
    test_scenario_voltage_vector_saturation();
    test_scenario_can_watchdog_timeout();
    test_scenario_can_v2_crc_validation();
    test_scenario_motor_parameter_consistency();
    test_scenario_mtpa_and_field_weakening();
    test_scenario_isaac_mujoco_bridge();

    std::cout << "======================================================================\n";
    std::cout << "  \033[1;32mALL 8 HARDWARE-READINESS SCENARIOS PASSED (100% SUCCESS)\033[0m            \n";
    std::cout << "======================================================================\n\n";
    return 0;
}
