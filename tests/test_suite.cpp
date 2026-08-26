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
    std::cout << "[TEST 1/8] Testing FOC Clarke & Park Transform Inverses... ";
    
    float ia = 10.0f;
    float ib = -5.0f;
    float ic = -5.0f;
    float theta_e = 0.785398f; // 45 degrees

    auto ab = FocMath::Clarke(ia, ib, ic);
    auto dq = FocMath::Park(ab, theta_e);
    auto ab_inv = FocMath::InversePark(dq, theta_e);

    assert(std::abs(ab.alpha - ab_inv.alpha) < 1e-4f);
    assert(std::abs(ab.beta - ab_inv.beta) < 1e-4f);

    auto duties = FocMath::Svpwm(ab, 24.0f);
    assert(duties.u >= 0.0f && duties.u <= 1.0f);
    assert(duties.v >= 0.0f && duties.v <= 1.0f);
    assert(duties.w >= 0.0f && duties.w <= 1.0f);

    std::cout << "\033[1;32mPASSED\033[0m\n";
}

void test_field_weakening() {
    std::cout << "[TEST 2/8] Testing Dynamic Field Weakening Demagnetization Current... ";

    float v_bus = 48.0f;
    float v_nominal = 20.0f; // Below saturation limit
    float id_fw_none = FocMath::ComputeFieldWeakeningId(v_nominal, v_bus, 15.0f);
    assert(id_fw_none == 0.0f);

    float v_overspeed = 32.0f; // Above 48V * (1/sqrt3)*0.95 = 26.3V
    float id_fw_active = FocMath::ComputeFieldWeakeningId(v_overspeed, v_bus, 15.0f);
    assert(id_fw_active < 0.0f); // Negative demagnetizing current
    assert(id_fw_active >= -15.0f); // Clamped to max

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

    std::cout << "\033[1;32mPASSED\033[0m\n";
}

void test_sliding_mode_observer() {
    std::cout << "[TEST 4/8] Testing Sliding Mode Observer Back-EMF Tracking... ";

    SlidingModeObserver smo(0.18f, 0.00012f, 25.0f);
    for (int i = 0; i < 200; ++i) {
        float angle = i * 0.05f;
        float va = 12.0f * std::cos(angle);
        float vb = 12.0f * std::sin(angle);
        float ia = 5.0f * std::cos(angle);
        float ib = 5.0f * std::sin(angle);
        smo.Update(va, vb, ia, ib, 0.00004f);
    }
    float est_angle = smo.GetEstimatedAngle();
    assert(est_angle >= 0.0f && est_angle <= 6.2831853f);

    std::cout << "\033[1;32mPASSED\033[0m\n";
}

void test_can_protocol_packing() {
    std::cout << "[TEST 5/8] Testing 8-Byte Bit-Packed CAN-FD Protocol Serialization... ";

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
    assert(std::abs(original_cmd.stiffness_kp - decoded_cmd.stiffness_kp) < 2.0f);
    assert(std::abs(original_cmd.damping_kd - decoded_cmd.damping_kd) < 0.2f);
    assert(std::abs(original_cmd.feedforward_torque_nm - decoded_cmd.feedforward_torque_nm) < 0.1f);

    std::cout << "\033[1;32mPASSED\033[0m\n";
}

void test_efuse_protection() {
    std::cout << "[TEST 6/8] Testing Sub-Microsecond eFuse Instantaneous Overcurrent Breaker... ";

    MotorProfile profile;
    profile.peak_current_a = 40.0f;
    HardwareSafetySupervisor guard(profile);

    SensorReadings normal_sensors;
    normal_sensors.i_phase_a = 15.0f;
    normal_sensors.v_bus = 24.0f;
    DriveState fault = DriveState::STANDBY;

    assert(guard.CheckHealth(normal_sensors, 0.00004f, fault) == true);

    SensorReadings overcurrent_sensors = normal_sensors;
    overcurrent_sensors.i_phase_a = 48.0f; // Over peak limit!
    
    assert(guard.CheckHealth(overcurrent_sensors, 0.00004f, fault) == false);
    assert(fault == DriveState::FAULT_OVERCURRENT);

    std::cout << "\033[1;32mPASSED\033[0m\n";
}

void test_brownout_protection() {
    std::cout << "[TEST 7/8] Testing DC Bus Brownout / UVLO Cutoff... ";

    MotorProfile profile;
    profile.min_voltage_uvlo_v = 18.0f;
    HardwareSafetySupervisor guard(profile);

    SensorReadings sag_sensors;
    sag_sensors.v_bus = 14.2f; // Voltage collapsed below UVLO!
    DriveState fault = DriveState::STANDBY;

    assert(guard.CheckHealth(sag_sensors, 0.00004f, fault) == false);
    assert(fault == DriveState::FAULT_BROWNOUT);

    std::cout << "\033[1;32mPASSED\033[0m\n";
}

void test_flight_recorder() {
    std::cout << "[TEST 8/8] Testing In-Memory Flight Recorder Circular Buffering & Freeze... ";

    FlightRecorder recorder;
    SensorReadings s;
    s.i_phase_a = 12.0f;
    s.v_bus = 48.0f;

    for (uint32_t i = 0; i < 50; ++i) {
        recorder.RecordSample(i * 40, s, DriveState::ARMED);
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
    test_field_weakening();
    test_anti_cogging_lut();
    test_sliding_mode_observer();
    test_can_protocol_packing();
    test_efuse_protection();
    test_brownout_protection();
    test_flight_recorder();

    std::cout << "======================================================================\n";
    std::cout << "  \033[1;32mALL 8 TEST SUITES PASSED (100% SUCCESS)\033[0m                             \n";
    std::cout << "======================================================================\n\n";
    return 0;
}
