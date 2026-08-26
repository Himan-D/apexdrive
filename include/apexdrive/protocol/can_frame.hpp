#pragma once

#include "../core/types.hpp"
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <cmath>

namespace apexdrive {

/**
 * 8-Byte High-Efficiency Bit-Packed CAN-FD Binary Protocol.
 * 
 * Command Frame (Host -> Actuator, 8 Bytes):
 * [0..1] Target Position Q15: [-4pi, +4pi] rad  -> int16_t [-32768, +32767]
 * [2..3] Target Velocity Q15: [-50, +50] rad/s  -> int16_t [-32768, +32767]
 * [4]    Stiffness Kp:        [0, 500] Nm/rad   -> uint8_t  [0, 255]
 * [5]    Damping Kd:          [0, 25] Nm*s/rad  -> uint8_t  [0, 255]
 * [6..7] Feedforward Tau Q15: [-100, +100] Nm   -> int16_t [-32768, +32767]
 * 
 * Telemetry Frame (Actuator -> Host, 8 Bytes):
 * [0..1] Measured Pos Q15:    [-4pi, +4pi] rad  -> int16_t [-32768, +32767]
 * [2..3] Measured Vel Q15:    [-50, +50] rad/s  -> int16_t [-32768, +32767]
 * [4..5] Measured Torque Q15: [-100, +100] Nm   -> int16_t [-32768, +32767]
 * [6]    Winding Temp:        [0, 150] °C       -> uint8_t  [0, 255]
 * [7]    State & Faults:      [0..3] State (4b) | [4..7] Faults (4b)
 */
class CanProtocol {
public:
    static constexpr float POS_MAX_RAD = 12.566370614359172f; // 4 * pi
    static constexpr float VEL_MAX_RAD_S = 50.0f;
    static constexpr float TORQUE_MAX_NM = 100.0f;
    static constexpr float KP_MAX = 500.0f;
    static constexpr float KD_MAX = 25.0f;
    static constexpr float TEMP_MAX_C = 150.0f;

    // Symmetric Signed Q15 Quantization Helpers
    [[nodiscard]] static constexpr inline int16_t QuantizeQ15(float val, float max_val) noexcept {
        float clamped = std::clamp(val, -max_val, max_val);
        return static_cast<int16_t>((clamped / max_val) * 32767.0f);
    }

    [[nodiscard]] static constexpr inline float DequantizeQ15(int16_t raw, float max_val) noexcept {
        return (static_cast<float>(raw) / 32767.0f) * max_val;
    }

    // Command Packing
    static void EncodeImpedanceCommand(const ImpedanceCommand& cmd, uint8_t out_bytes[8]) noexcept {
        int16_t pos_q15 = QuantizeQ15(cmd.target_pos_rad, POS_MAX_RAD);
        int16_t vel_q15 = QuantizeQ15(cmd.target_vel_rad_s, VEL_MAX_RAD_S);
        uint8_t kp_u8   = static_cast<uint8_t>(std::clamp(cmd.stiffness_kp / KP_MAX, 0.0f, 1.0f) * 255.0f);
        uint8_t kd_u8   = static_cast<uint8_t>(std::clamp(cmd.damping_kd / KD_MAX, 0.0f, 1.0f) * 255.0f);
        int16_t tau_q15 = QuantizeQ15(cmd.feedforward_torque_nm, TORQUE_MAX_NM);

        out_bytes[0] = static_cast<uint8_t>(pos_q15 & 0xFF);
        out_bytes[1] = static_cast<uint8_t>((pos_q15 >> 8) & 0xFF);
        out_bytes[2] = static_cast<uint8_t>(vel_q15 & 0xFF);
        out_bytes[3] = static_cast<uint8_t>((vel_q15 >> 8) & 0xFF);
        out_bytes[4] = kp_u8;
        out_bytes[5] = kd_u8;
        out_bytes[6] = static_cast<uint8_t>(tau_q15 & 0xFF);
        out_bytes[7] = static_cast<uint8_t>((tau_q15 >> 8) & 0xFF);
    }

    [[nodiscard]] static ImpedanceCommand DecodeImpedanceCommand(const uint8_t in_bytes[8]) noexcept {
        int16_t pos_q15 = static_cast<int16_t>(in_bytes[0] | (in_bytes[1] << 8));
        int16_t vel_q15 = static_cast<int16_t>(in_bytes[2] | (in_bytes[3] << 8));
        uint8_t kp_u8   = in_bytes[4];
        uint8_t kd_u8   = in_bytes[5];
        int16_t tau_q15 = static_cast<int16_t>(in_bytes[6] | (in_bytes[7] << 8));

        return ImpedanceCommand{
            .target_pos_rad = DequantizeQ15(pos_q15, POS_MAX_RAD),
            .target_vel_rad_s = DequantizeQ15(vel_q15, VEL_MAX_RAD_S),
            .stiffness_kp = (static_cast<float>(kp_u8) / 255.0f) * KP_MAX,
            .damping_kd = (static_cast<float>(kd_u8) / 255.0f) * KD_MAX,
            .feedforward_torque_nm = DequantizeQ15(tau_q15, TORQUE_MAX_NM)
        };
    }

    // Telemetry Packing
    static void EncodeTelemetry(const JointTelemetry& telem, uint8_t out_bytes[8]) noexcept {
        int16_t pos_q15 = QuantizeQ15(telem.position_rad, POS_MAX_RAD);
        int16_t vel_q15 = QuantizeQ15(telem.velocity_rad_s, VEL_MAX_RAD_S);
        int16_t tau_q15 = QuantizeQ15(telem.torque_nm, TORQUE_MAX_NM);
        uint8_t temp_u8 = static_cast<uint8_t>(std::clamp(telem.temperature_c / TEMP_MAX_C, 0.0f, 1.0f) * 255.0f);
        
        // Low nibble: OperatingMode, High nibble: active fault flags (masked to 4 bits)
        uint8_t state_nibble = static_cast<uint8_t>(telem.mode) & 0x0F;
        uint8_t fault_nibble = static_cast<uint8_t>(telem.fault_flags & 0x0F) << 4;
        uint8_t status_byte  = state_nibble | fault_nibble;

        out_bytes[0] = static_cast<uint8_t>(pos_q15 & 0xFF);
        out_bytes[1] = static_cast<uint8_t>((pos_q15 >> 8) & 0xFF);
        out_bytes[2] = static_cast<uint8_t>(vel_q15 & 0xFF);
        out_bytes[3] = static_cast<uint8_t>((vel_q15 >> 8) & 0xFF);
        out_bytes[4] = static_cast<uint8_t>(tau_q15 & 0xFF);
        out_bytes[5] = static_cast<uint8_t>((tau_q15 >> 8) & 0xFF);
        out_bytes[6] = temp_u8;
        out_bytes[7] = status_byte;
    }

    [[nodiscard]] static JointTelemetry DecodeTelemetry(const uint8_t in_bytes[8], uint8_t node_id) noexcept {
        int16_t pos_q15 = static_cast<int16_t>(in_bytes[0] | (in_bytes[1] << 8));
        int16_t vel_q15 = static_cast<int16_t>(in_bytes[2] | (in_bytes[3] << 8));
        int16_t tau_q15 = static_cast<int16_t>(in_bytes[4] | (in_bytes[5] << 8));
        uint8_t temp_u8 = in_bytes[6];
        uint8_t status  = in_bytes[7];

        JointTelemetry telem;
        telem.node_id = node_id;
        telem.mode = static_cast<OperatingMode>(status & 0x0F);
        telem.fault_flags = static_cast<uint32_t>((status >> 4) & 0x0F);
        telem.safety_state = (telem.fault_flags != 0) ? SafetyState::FAULT_STOP : SafetyState::OK;
        telem.position_rad = DequantizeQ15(pos_q15, POS_MAX_RAD);
        telem.velocity_rad_s = DequantizeQ15(vel_q15, VEL_MAX_RAD_S);
        telem.torque_nm = DequantizeQ15(tau_q15, TORQUE_MAX_NM);
        telem.temperature_c = (static_cast<float>(temp_u8) / 255.0f) * TEMP_MAX_C;
        return telem;
    }
};

} // namespace apexdrive
