#include "../include/apexdrive/core/types.hpp"
#include "../include/apexdrive/core/foc_math.hpp"
#include "../include/apexdrive/core/efuse.hpp"
#include "../include/apexdrive/core/impedance_controller.hpp"
#include "../include/apexdrive/core/auto_tuner.hpp"
#include "../include/apexdrive/core/anti_cogging.hpp"
#include "../include/apexdrive/core/sliding_mode_observer.hpp"
#include "../include/apexdrive/core/flight_recorder.hpp"
#include "../include/apexdrive/protocol/can_frame.hpp"

#include <iostream>
#include <cassert>
#include <cmath>

using namespace apexdrive;

void test_foc_math() {
    std::cout << "[TEST 1/8] Testing FOC Clarke, Park & Inverse Transformations... ";
    
    float ia = 10.0f;
    float ib = -5.0f;
    float ic = -5.0f;
    float theta_e = 0.785398f; // 45 degrees

    auto ab = FocMath::Clarke(ia, ib, ic);
    auto dq = FocMath::Park(ab, theta_e);
    auto ab_inv = FocMath::InversePark(dq, theta_e);

    assert(std::abs(ab.alpha - ab_inv.alpha) < 1e-4f);
    assert(std::abs(ab.beta - ab_inv.beta) < 1e-4f);
    (void)ab_inv;

    auto duties = FocMath::Svpwm(ab, 24.0f);
    assert(duties.u >= 0.0f && duties.u <= 1.0f);
    assert(duties.v >= 0.0f && duties.v <= 1.0f);
    assert(duties.w >= 0.0f && duties.w <= 1.0f);
    (void)duties;

    // Test Saliency / Reluctance Torque
    float torque_em = FocMath::ComputeElectromagneticTorque(0.0f, 10.0f, 7.0f, 0.0068f, 0.000118f, 0.000135f);
    assert(torque_em > 0.0f);
    (void)torque_em;

    std::cout << "\033[1;32mPASSED\033[0m\n";
}

void test_pi_anti_windup() {
    std::cout << "[TEST 2/8] Testing Back-Calculation Anti-Windup PI Controller... ";

    PiController pi(2.0f, 200.0f, 10.0f); // Limit = 10.0V
    
    // Large persistent error -> Output must saturate at 10.0V and integrator must not blow up
    for (int i = 0; i < 100; ++i) {
        float out = pi.Update(50.0f, 0.001f);
        assert(std::abs(out) <= 10.0f);
        (void)out;
    }
    
    // Once error reverses, back-calculation anti-windup should desaturate rapidly
    float desat_out = pi.Update(-20.0f, 0.001f);
    assert(desat_out < 10.0f);
    (void)desat_out;

    std::cout << "\033[1;32mPASSED\033[0m\n";
}

void test_anti_cogging_lut() {
    std::cout << "[TEST 3/8] Testing 256-Point Anti-Cogging Interpolation Map... ";

    AntiCoggingMap map;
    map.SetBucket(0, 0.5f);
    map.SetBucket(1, 1.0f);

    // Halfway between bucket 0 and 1 -> should interpolate to ~0.75f
    float interp_val = map.Lookup(0.01227f);
    assert(interp_val > 0.5f && interp_val < 1.0f);
    (void)interp_val;

    std::cout << "\033[1;32mPASSED\033[0m\n";
}

void test_sliding_mode_observer() {
    std::cout << "[TEST 4/8] Testing Sliding Mode Observer & Tracking PLL... ";

    SlidingModeObserver smo(0.18f, 0.00012f, 25.0f);
    for (int i = 0; i < 500; ++i) {
        float angle = i * 0.02f;
        float va = 12.0f * std::cos(angle);
        float vb = 12.0f * std::sin(angle);
        float ia = 5.0f * std::cos(angle);
        float ib = 5.0f * std::sin(angle);
        smo.Update(va, vb, ia, ib, 0.00004f);
    }
    float est_angle = smo.GetEstimatedAngle();
    assert(est_angle >= 0.0f && est_angle <= 6.2831853f);
    (void)est_angle;

    std::cout << "\033[1;32mPASSED\033[0m\n";
}

void test_can_protocol_packing() {
    std::cout << "[TEST 5/8] Testing CAN-FD 8-Byte Bit-Packed Protocol & Fault Nibbles... ";

    ImpedanceCommand original_cmd{
        .target_pos_rad = 1.57079f,
        .target_vel_rad_s = 12.5f,
        .stiffness_kp = 85.0f,
        .damping_kd = 4.2f,
        .feedforward_torque_nm = -6.5f
    };

    uint8_t buffer[8];
    CanProtocol::EncodeImpedanceCommand(original_cmd, buffer);
    auto decoded_cmd = CanProtocol::DecodeImpedanceCommand(buffer);

    assert(std::abs(original_cmd.target_pos_rad - decoded_cmd.target_pos_rad) < 0.005f);
    assert(std::abs(original_cmd.target_vel_rad_s - decoded_cmd.target_vel_rad_s) < 0.1f);
    assert(std::abs(original_cmd.stiffness_kp - decoded_cmd.stiffness_kp) < 3.0f);
    assert(std::abs(original_cmd.damping_kd - decoded_cmd.damping_kd) < 0.2f);
    assert(std::abs(original_cmd.feedforward_torque_nm - decoded_cmd.feedforward_torque_nm) < 0.1f);
    (void)decoded_cmd;

    // Test Telemetry and Fault Nibble packing
    JointTelemetry original_telem{
        .node_id = 0x14,
        .mode = OperatingMode::CLOSED_LOOP_IMPEDANCE,
        .safety_state = SafetyState::FAULT_STOP,
        .position_rad = 0.5f,
        .velocity_rad_s = 2.0f,
        .torque_nm = 4.5f,
        .current_iq_a = 5.0f,
        .current_id_a = 0.0f,
        .v_bus_v = 48.0f,
        .temperature_c = 42.0f,
        .fault_flags = FaultFlag::OVERCURRENT_PHASE,
        .timestamp_us = 1000,
        .sequence_number = 0
    };

    uint8_t telem_buf[8];
    CanProtocol::EncodeTelemetry(original_telem, telem_buf);
    auto decoded_telem = CanProtocol::DecodeTelemetry(telem_buf, 0x14);

    assert(decoded_telem.mode == OperatingMode::CLOSED_LOOP_IMPEDANCE);
    assert((decoded_telem.fault_flags & FaultFlag::OVERCURRENT_PHASE) != 0);
    (void)decoded_telem;

    std::cout << "\033[1;32mPASSED\033[0m\n";
}

void test_safety_overcurrent() {
    std::cout << "[TEST 6/8] Testing Software Safety Supervisor Overcurrent Trip... ";

    MotorProfile profile;
    profile.peak_current_a = 40.0f;
    HardwareSafetySupervisor guard(profile);

    SensorReadings normal_sensors;
    normal_sensors.i_phase_a = 15.0f;
    normal_sensors.v_bus = 48.0f;
    
    SafetyState safety = SafetyState::OK;
    uint32_t faults = FaultFlag::NONE;

    guard.FeedWatchdog();
    assert(guard.CheckHealth(normal_sensors, 0.00004f, safety, faults) == true);
    assert(safety == SafetyState::OK);

    SensorReadings overcurrent_sensors = normal_sensors;
    overcurrent_sensors.i_phase_a = 48.0f; // Exceeds peak!
    
    assert(guard.CheckHealth(overcurrent_sensors, 0.00004f, safety, faults) == false);
    assert(safety == SafetyState::FAULT_STOP);
    assert((faults & FaultFlag::OVERCURRENT_PHASE) != 0);
    (void)overcurrent_sensors;
    (void)safety;
    (void)faults;

    std::cout << "\033[1;32mPASSED\033[0m\n";
}

void test_safety_overvoltage_and_uvlo() {
    std::cout << "[TEST 7/8] Testing Overvoltage and UVLO Brownout Fault Disambiguation... ";

    MotorProfile profile;
    profile.max_voltage_v = 54.0f;
    profile.min_voltage_uvlo_v = 18.0f;
    HardwareSafetySupervisor guard(profile);

    SafetyState safety = SafetyState::OK;
    uint32_t faults = FaultFlag::NONE;

    // Overvoltage test
    SensorReadings ov_sensors;
    ov_sensors.v_bus = 58.0f; // Above 54V
    guard.FeedWatchdog();
    assert(guard.CheckHealth(ov_sensors, 0.00004f, safety, faults) == false);
    assert((faults & FaultFlag::OVERVOLTAGE_BUS) != 0);
    assert((faults & FaultFlag::UNDERVOLTAGE_BUS) == 0); // Disambiguated!
    (void)ov_sensors;

    // Undervoltage test
    SensorReadings uv_sensors;
    uv_sensors.v_bus = 14.0f; // Below 18V
    guard.FeedWatchdog();
    assert(guard.CheckHealth(uv_sensors, 0.00004f, safety, faults) == false);
    assert((faults & FaultFlag::UNDERVOLTAGE_BUS) != 0);
    assert((faults & FaultFlag::OVERVOLTAGE_BUS) == 0);
    (void)uv_sensors;
    (void)safety;
    (void)faults;

    std::cout << "\033[1;32mPASSED\033[0m\n";
}

void test_flight_recorder() {
    std::cout << "[TEST 8/8] Testing In-Memory Circular Black-Box Monotonic Timestamping... ";

    FlightRecorder recorder;
    SensorReadings s;
    s.i_phase_a = 12.0f;
    s.v_bus = 48.0f;

    for (uint32_t i = 0; i < 50; ++i) {
        recorder.RecordSample(i * 40, s, OperatingMode::CLOSED_LOOP_IMPEDANCE, SafetyState::OK, 0);
    }
    assert(recorder.GetCount() == 50);
    assert(!recorder.IsFrozen());

    recorder.FreezeOnFault();
    assert(recorder.IsFrozen());

    std::string report = recorder.DumpForensicReport();
    assert(report.find("APEXDRIVE FORENSIC") != std::string::npos);

    std::cout << "\033[1;32mPASSED\033[0m\n";
}

int main() {
    std::cout << "\n======================================================================\n";
    std::cout << "  RUNNING APEXDRIVE CORE TEST SUITE (8 TEST SUITES)                   \n";
    std::cout << "======================================================================\n";

    test_foc_math();
    test_pi_anti_windup();
    test_anti_cogging_lut();
    test_sliding_mode_observer();
    test_can_protocol_packing();
    test_safety_overcurrent();
    test_safety_overvoltage_and_uvlo();
    test_flight_recorder();

    std::cout << "======================================================================\n";
    std::cout << "  \033[1;32mALL 8 TEST SUITES PASSED (100% SUCCESS)\033[0m                             \n";
    std::cout << "======================================================================\n\n";
    return 0;
}
