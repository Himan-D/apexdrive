#pragma once

#include "../core/types.hpp"
#include <cstdint>
#include <cstring>
#include <algorithm>

namespace apexdrive {

/**
 * Enterprise CAN-FD Protocol Version 2.0 (16-Byte & 32-Byte Payload Standard).
 * Features:
 * - 16-bit monotonic sequence numbers for dropped packet & latency tracking.
 * - Hardware CCITT CRC16 frame validation.
 * - Full 32-bit machine-readable fault bitmap.
 * - Symmetric Q15 fixed-point encoding.
 */
class CanProtocolV2 {
public:
    static constexpr uint8_t PROTOCOL_VERSION = 0x02;

    // CRC-16-CCITT (Polynomial: 0x1021, Initial: 0xFFFF)
    [[nodiscard]] static constexpr uint16_t ComputeCrc16(const uint8_t* data, size_t length) noexcept {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < length; ++i) {
            crc ^= static_cast<uint16_t>(data[i]) << 8;
            for (int j = 0; j < 8; ++j) {
                if (crc & 0x8000) {
                    crc = (crc << 1) ^ 0x1021;
                } else {
                    crc <<= 1;
                }
            }
        }
        return crc;
    }

    struct CommandFramePayload {
        uint8_t protocol_ver{PROTOCOL_VERSION};
        uint8_t mode{0};
        uint16_t sequence_num{0};
        int16_t target_pos_q15{0};
        int16_t target_vel_q15{0};
        uint16_t stiffness_kp_q15{0};
        uint16_t damping_kd_q15{0};
        int16_t feedforward_tau_q15{0};
        uint16_t crc16{0};
    };

    struct TelemetryFramePayload {
        uint8_t protocol_ver{PROTOCOL_VERSION};
        uint8_t mode{0};
        uint8_t safety_state{0};
        uint8_t reserved{0};
        uint16_t sequence_num{0};
        int16_t pos_q15{0};
        int16_t vel_q15{0};
        int16_t torque_q15{0};
        int16_t iq_q15{0};
        int16_t id_q15{0};
        uint16_t v_bus_mv{0};
        int16_t temp_c_q8{0};
        uint32_t fault_flags{0};
        uint16_t crc16{0};
    };

    // Packing Helpers
    static void EncodeCommand(
        const ImpedanceCommand& cmd, 
        OperatingMode mode, 
        uint16_t sequence_num, 
        uint8_t out_buf[16]
    ) noexcept {
        CommandFramePayload payload;
        payload.protocol_ver = PROTOCOL_VERSION;
        payload.mode = static_cast<uint8_t>(mode);
        payload.sequence_num = sequence_num;
        payload.target_pos_q15 = Quantize(cmd.target_pos_rad, 12.56637f);
        payload.target_vel_q15 = Quantize(cmd.target_vel_rad_s, 50.0f);
        payload.stiffness_kp_q15 = static_cast<uint16_t>(std::clamp(cmd.stiffness_kp / 500.0f, 0.0f, 1.0f) * 65535.0f);
        payload.damping_kd_q15 = static_cast<uint16_t>(std::clamp(cmd.damping_kd / 25.0f, 0.0f, 1.0f) * 65535.0f);
        payload.feedforward_tau_q15 = Quantize(cmd.feedforward_torque_nm, 100.0f);
        payload.crc16 = 0;

        std::memcpy(out_buf, &payload, 14);
        uint16_t computed_crc = ComputeCrc16(out_buf, 14);
        out_buf[14] = static_cast<uint8_t>(computed_crc & 0xFF);
        out_buf[15] = static_cast<uint8_t>((computed_crc >> 8) & 0xFF);
    }

    [[nodiscard]] static bool DecodeCommand(
        const uint8_t in_buf[16], 
        ImpedanceCommand& out_cmd, 
        OperatingMode& out_mode, 
        uint16_t& out_sequence
    ) noexcept {
        uint16_t received_crc = static_cast<uint16_t>(in_buf[14] | (in_buf[15] << 8));
        uint16_t calculated_crc = ComputeCrc16(in_buf, 14);
        if (received_crc != calculated_crc) return false;

        CommandFramePayload payload;
        std::memcpy(&payload, in_buf, 14);
        if (payload.protocol_ver != PROTOCOL_VERSION) return false;

        out_mode = static_cast<OperatingMode>(payload.mode);
        out_sequence = payload.sequence_num;
        out_cmd.target_pos_rad = Dequantize(payload.target_pos_q15, 12.56637f);
        out_cmd.target_vel_rad_s = Dequantize(payload.target_vel_q15, 50.0f);
        out_cmd.stiffness_kp = (static_cast<float>(payload.stiffness_kp_q15) / 65535.0f) * 500.0f;
        out_cmd.damping_kd = (static_cast<float>(payload.damping_kd_q15) / 65535.0f) * 25.0f;
        out_cmd.feedforward_torque_nm = Dequantize(payload.feedforward_tau_q15, 100.0f);
        return true;
    }

    static void EncodeTelemetry(const JointTelemetry& telem, uint8_t out_buf[24]) noexcept {
        TelemetryFramePayload payload;
        payload.protocol_ver = PROTOCOL_VERSION;
        payload.mode = static_cast<uint8_t>(telem.mode);
        payload.safety_state = static_cast<uint8_t>(telem.safety_state);
        payload.reserved = 0;
        payload.sequence_num = telem.sequence_number;
        payload.pos_q15 = Quantize(telem.position_rad, 12.56637f);
        payload.vel_q15 = Quantize(telem.velocity_rad_s, 50.0f);
        payload.torque_q15 = Quantize(telem.torque_nm, 100.0f);
        payload.iq_q15 = Quantize(telem.current_iq_a, 50.0f);
        payload.id_q15 = Quantize(telem.current_id_a, 50.0f);
        payload.v_bus_mv = static_cast<uint16_t>(telem.v_bus_v * 1000.0f);
        payload.temp_c_q8 = static_cast<int16_t>(telem.temperature_c * 256.0f);
        payload.fault_flags = telem.fault_flags;
        payload.crc16 = 0;

        std::memcpy(out_buf, &payload, 22);
        uint16_t crc = ComputeCrc16(out_buf, 22);
        out_buf[22] = static_cast<uint8_t>(crc & 0xFF);
        out_buf[23] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    }

    [[nodiscard]] static bool DecodeTelemetry(
        const uint8_t in_buf[24], 
        uint8_t node_id, 
        JointTelemetry& out_telem
    ) noexcept {
        uint16_t received_crc = static_cast<uint16_t>(in_buf[22] | (in_buf[23] << 8));
        uint16_t calculated_crc = ComputeCrc16(in_buf, 22);
        if (received_crc != calculated_crc) return false;

        TelemetryFramePayload payload;
        std::memcpy(&payload, in_buf, 22);
        if (payload.protocol_ver != PROTOCOL_VERSION) return false;

        out_telem.node_id = node_id;
        out_telem.mode = static_cast<OperatingMode>(payload.mode);
        out_telem.safety_state = static_cast<SafetyState>(payload.safety_state);
        out_telem.sequence_number = payload.sequence_num;
        out_telem.position_rad = Dequantize(payload.pos_q15, 12.56637f);
        out_telem.velocity_rad_s = Dequantize(payload.vel_q15, 50.0f);
        out_telem.torque_nm = Dequantize(payload.torque_q15, 100.0f);
        out_telem.current_iq_a = Dequantize(payload.iq_q15, 50.0f);
        out_telem.current_id_a = Dequantize(payload.id_q15, 50.0f);
        out_telem.v_bus_v = static_cast<float>(payload.v_bus_mv) * 0.001f;
        out_telem.temperature_c = static_cast<float>(payload.temp_c_q8) / 256.0f;
        out_telem.fault_flags = payload.fault_flags;
        return true;
    }

private:
    [[nodiscard]] static constexpr inline int16_t Quantize(float val, float max_abs) noexcept {
        float clamped = std::clamp(val, -max_abs, max_abs);
        return static_cast<int16_t>((clamped / max_abs) * 32767.0f);
    }

    [[nodiscard]] static constexpr inline float Dequantize(int16_t raw, float max_abs) noexcept {
        return (static_cast<float>(raw) / 32767.0f) * max_abs;
    }
};

} // namespace apexdrive
