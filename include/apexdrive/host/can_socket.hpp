#pragma once

#include "../core/types.hpp"
#include <string>
#include <vector>
#include <optional>
#include <memory>

namespace apexdrive {

/**
 * True Linux SocketCAN-FD Network Transport Driver.
 * Supports:
 * - Real 64-byte MTU CAN-FD frames (`struct canfd_frame`) with Bit Rate Switch (BRS).
 * - Automatic Fallback to mock simulation when physical interfaces are absent.
 * - Non-blocking poll-based reception with microsecond timeouts.
 * - Hardware timestamping and multi-node discovery broadcast.
 */
class CanTransport {
public:
    explicit CanTransport(std::string interface_name = "can0", bool force_mock = false);
    ~CanTransport();

    CanTransport(const CanTransport&) = delete;
    CanTransport& operator=(const CanTransport&) = delete;

    CanTransport(CanTransport&& other) noexcept;
    CanTransport& operator=(CanTransport&& other) noexcept;

    /**
     * Transmits CAN-FD v2 Command Frame (16 Bytes, CRC16-checked) to target actuator node.
     */
    [[nodiscard]] bool SendCommand(uint8_t node_id, const ImpedanceCommand& cmd, OperatingMode mode, uint16_t sequence_num);

    /**
     * Non-blocking poll for incoming CAN-FD Telemetry Frame (24 Bytes, CRC16-checked).
     */
    [[nodiscard]] std::optional<JointTelemetry> ReceiveTelemetry(int timeout_ms = 1);

    /**
     * Broadcasts discovery frame and returns list of responsive node IDs.
     */
    [[nodiscard]] std::vector<uint8_t> ScanBus(int timeout_ms = 100);

    [[nodiscard]] bool IsHardwareOpen() const noexcept { return is_hardware_open_; }
    [[nodiscard]] const std::string& GetInterfaceName() const noexcept { return interface_name_; }

private:
    bool OpenSocketCAN();
    void CloseSocketCAN();

    std::string interface_name_;
    bool force_mock_{false};
    bool is_hardware_open_{false};
    int socket_fd_{-1};
};

} // namespace apexdrive
