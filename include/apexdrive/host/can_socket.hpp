#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <optional>
#include "../protocol/can_frame.hpp"

namespace apexdrive {

/**
 * SocketCAN & CAN-FD Transport Interface.
 * Handles hardware CAN socket communication on Linux and provides deterministic mock
 * transport when running in simulation mode or on macOS/Windows.
 */
class CanTransport {
public:
    explicit CanTransport(std::string interface_name, bool force_mock = false);
    ~CanTransport();

    // Disable copy
    CanTransport(const CanTransport&) = delete;
    CanTransport& operator=(const CanTransport&) = delete;

    // Enable move
    CanTransport(CanTransport&& other) noexcept;
    CanTransport& operator=(CanTransport&& other) noexcept;

    /**
     * Send an 8-byte CAN-FD Impedance Command to target node ID
     */
    [[nodiscard]] bool SendCommand(uint8_t node_id, const ImpedanceCommand& cmd);

    /**
     * Poll and receive telemetry frame from CAN bus (non-blocking with timeout)
     * @param timeout_ms Timeout in milliseconds (0 = non-blocking)
     */
    [[nodiscard]] std::optional<JointTelemetry> ReceiveTelemetry(int timeout_ms = 1);

    /**
     * Broadcast a discovery probe on the CAN bus and collect responding node IDs
     */
    [[nodiscard]] std::vector<uint8_t> ScanBus(int timeout_ms = 100);

    [[nodiscard]] bool IsHardwareOpen() const noexcept { return is_hardware_open_; }
    [[nodiscard]] const std::string& GetInterfaceName() const noexcept { return interface_name_; }

private:
    std::string interface_name_;
    bool force_mock_{false};
    bool is_hardware_open_{false};
    int socket_fd_{-1};

    bool OpenSocketCAN();
    void CloseSocketCAN();
};

} // namespace apexdrive
