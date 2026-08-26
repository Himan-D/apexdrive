#pragma once

#include "../core/types.hpp"
#include <cstdint>
#include <cstring>
#include <algorithm>

namespace apexdrive {

/**
 * High-Speed 1Mbps CAN-FD Bit-Packed Binary Frame Protocol.
 * Packed for minimal latency on multi-axis daisy-chained robot buses.
 */
class CanProtocol {
public:
    static constexpr uint32_t CAN_ID_COMMAND_BASE    = 0x100;
    static constexpr uint32_t CAN_ID_TELEMETRY_BASE  = 0x200;
    static constexpr uint32_t CAN_ID_HEARTBEAT_BASE  = 0x300;

    // Packed 8-Byte Impedance Command Frame
    struct __attribute__((packed)) PackedCommandFrame {
        uint16_t target_pos_q16;     // Pos in radians mapped to [ -4pi, +4pi ]
        int16_t  target_vel_q12;     // Vel in rad/s mapped to [ -50, +50 ]
        uint8_t  stiffness_kp_q8;    // Kp mapped to [ 0, 500 ]
        uint8_t  damping_kd_q8;      // Kd mapped to [ 0, 25 ]
        int16_t  torque_ff_q12;      // Feedforward Torque mapped to [ -100, +100 ]
    };

    // Packed 8-Byte Telemetry Feedback Frame
    struct __attribute__((packed)) PackedTelemetryFrame {
        uint16_t actual_pos_q16;     // Pos in radians [ -4pi, +4pi ]
        int16_t  actual_vel_q12;     // Vel in rad/s [ -50, +50 ]
        int16_t  measured_torque_q12;// Torque in Nm [ -100, +100 ]
        uint8_t  temperature_c;      // Temperature in °C [ 0, 255 ]
        uint8_t  state_and_fault;    // Lower 4 bits: State, Upper 4 bits: Fault flag
    };

    // Serialization: Convert High-Level Command -> 8-Byte CAN Frame
    static void EncodeImpedanceCommand(const ImpedanceCommand& cmd, uint8_t out_data[8]) noexcept {
        PackedCommandFrame frame;
        
        // Map Position [-4pi, +4pi] -> [0, 65535]
        const float pos_clamped = std::clamp(cmd.target_pos_rad, -12.56637f, 12.56637f);
        frame.target_pos_q16 = static_cast<uint16_t>(((pos_clamped + 12.56637f) / 25.13274f) * 65535.0f);

        // Map Velocity [-50, +50] -> [-2048, 2047]
        const float vel_clamped = std::clamp(cmd.target_vel_rad_s, -50.0f, 50.0f);
        frame.target_vel_q12 = static_cast<int16_t>((vel_clamped / 50.0f) * 2047.0f);

        // Map Kp [0, 500] -> [0, 255]
        frame.stiffness_kp_q8 = static_cast<uint8_t>((std::clamp(cmd.stiffness_kp, 0.0f, 500.0f) / 500.0f) * 255.0f);

        // Map Kd [0, 25] -> [0, 255]
        frame.damping_kd_q8 = static_cast<uint8_t>((std::clamp(cmd.damping_kd, 0.0f, 25.0f) / 25.0f) * 255.0f);

        // Map Torque FF [-100, +100] -> [-2048, 2047]
        const float tau_clamped = std::clamp(cmd.feedforward_torque_nm, -100.0f, 100.0f);
        frame.torque_ff_q12 = static_cast<int16_t>((tau_clamped / 100.0f) * 2047.0f);

        std::memcpy(out_data, &frame, 8);
    }

    // Deserialization: Convert 8-Byte CAN Frame -> High-Level Command
    static ImpedanceCommand DecodeImpedanceCommand(const uint8_t in_data[8]) noexcept {
        PackedCommandFrame frame;
        std::memcpy(&frame, in_data, 8);

        ImpedanceCommand cmd;
        cmd.target_pos_rad = (static_cast<float>(frame.target_pos_q16) / 65535.0f) * 25.13274f - 12.56637f;
        cmd.target_vel_rad_s = (static_cast<float>(frame.target_vel_q12) / 2047.0f) * 50.0f;
        cmd.stiffness_kp = (static_cast<float>(frame.stiffness_kp_q8) / 255.0f) * 500.0f;
        cmd.damping_kd = (static_cast<float>(frame.damping_kd_q8) / 255.0f) * 25.0f;
        cmd.feedforward_torque_nm = (static_cast<float>(frame.torque_ff_q12) / 2047.0f) * 100.0f;

        return cmd;
    }

    // Serialization: Convert Telemetry -> 8-Byte CAN Frame
    static void EncodeTelemetry(const JointTelemetry& telem, uint8_t out_data[8]) noexcept {
        PackedTelemetryFrame frame;
        
        const float pos_clamped = std::clamp(telem.position_rad, -12.56637f, 12.56637f);
        frame.actual_pos_q16 = static_cast<uint16_t>(((pos_clamped + 12.56637f) / 25.13274f) * 65535.0f);

        const float vel_clamped = std::clamp(telem.velocity_rad_s, -50.0f, 50.0f);
        frame.actual_vel_q12 = static_cast<int16_t>((vel_clamped / 50.0f) * 2047.0f);

        const float tau_clamped = std::clamp(telem.torque_nm, -100.0f, 100.0f);
        frame.measured_torque_q12 = static_cast<int16_t>((tau_clamped / 100.0f) * 2047.0f);

        frame.temperature_c = static_cast<uint8_t>(std::clamp(telem.temperature_c, 0.0f, 255.0f));
        frame.state_and_fault = static_cast<uint8_t>(static_cast<uint8_t>(telem.state) & 0x0F);

        std::memcpy(out_data, &frame, 8);
    }

    // Deserialization: Convert 8-Byte CAN Frame -> Telemetry
    static JointTelemetry DecodeTelemetry(uint8_t node_id, const uint8_t in_data[8]) noexcept {
        PackedTelemetryFrame frame;
        std::memcpy(&frame, in_data, 8);

        JointTelemetry telem;
        telem.node_id = node_id;
        telem.position_rad = (static_cast<float>(frame.actual_pos_q16) / 65535.0f) * 25.13274f - 12.56637f;
        telem.velocity_rad_s = (static_cast<float>(frame.actual_vel_q12) / 2047.0f) * 50.0f;
        telem.torque_nm = (static_cast<float>(frame.measured_torque_q12) / 2047.0f) * 100.0f;
        telem.temperature_c = static_cast<float>(frame.temperature_c);
        telem.state = static_cast<DriveState>(frame.state_and_fault & 0x0F);

        return telem;
    }
};

} // namespace apexdrive
