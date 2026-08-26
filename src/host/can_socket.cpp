#include "../../include/apexdrive/host/can_socket.hpp"
#include <iostream>
#include <cstring>
#include <chrono>
#include <thread>

#ifdef __linux__
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <poll.h>
#endif

namespace apexdrive {

CanTransport::CanTransport(std::string interface_name, bool force_mock)
    : interface_name_(std::move(interface_name)), force_mock_(force_mock) {
    if (!force_mock_) {
        OpenSocketCAN();
    }
}

CanTransport::~CanTransport() {
    CloseSocketCAN();
}

CanTransport::CanTransport(CanTransport&& other) noexcept
    : interface_name_(std::move(other.interface_name_)),
      force_mock_(other.force_mock_),
      is_hardware_open_(other.is_hardware_open_),
      socket_fd_(other.socket_fd_) {
    other.socket_fd_ = -1;
    other.is_hardware_open_ = false;
}

CanTransport& CanTransport::operator=(CanTransport&& other) noexcept {
    if (this != &other) {
        CloseSocketCAN();
        interface_name_ = std::move(other.interface_name_);
        force_mock_ = other.force_mock_;
        is_hardware_open_ = other.is_hardware_open_;
        socket_fd_ = other.socket_fd_;
        other.socket_fd_ = -1;
        other.is_hardware_open_ = false;
    }
    return *this;
}

bool CanTransport::OpenSocketCAN() {
#ifdef __linux__
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        std::cerr << "[ApexDrive] Notice: Unable to create RAW CAN socket. Falling back to simulation mode.\n";
        is_hardware_open_ = false;
        return false;
    }

    // Enable CAN-FD Support
    int enable_canfd = 1;
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0) {
        std::cerr << "[ApexDrive] Warning: Interface does not support CAN-FD mode.\n";
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "[ApexDrive] Notice: CAN Interface '" << interface_name_ << "' not found on host. Using mock mode.\n";
        close(socket_fd_);
        socket_fd_ = -1;
        is_hardware_open_ = false;
        return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "[ApexDrive] Error: Failed to bind socket to interface '" << interface_name_ << "'.\n";
        close(socket_fd_);
        socket_fd_ = -1;
        is_hardware_open_ = false;
        return false;
    }

    is_hardware_open_ = true;
    return true;
#else
    // Non-Linux hosts (macOS/Windows) run in host simulation mode
    is_hardware_open_ = false;
    return false;
#endif
}

void CanTransport::CloseSocketCAN() {
#ifdef __linux__
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
#endif
    is_hardware_open_ = false;
}

bool CanTransport::SendCommand(uint8_t node_id, const ImpedanceCommand& cmd) {
    uint8_t payload[8];
    CanProtocol::EncodeImpedanceCommand(cmd, payload);

#ifdef __linux__
    if (is_hardware_open_ && socket_fd_ >= 0) {
        struct can_frame frame;
        frame.can_id = 0x200 | node_id;
        frame.can_dlc = 8;
        std::memcpy(frame.data, payload, 8);

        ssize_t bytes_sent = write(socket_fd_, &frame, sizeof(frame));
        return bytes_sent == sizeof(frame);
    }
#endif

    (void)node_id;
    return true; // Accepted by mock simulator
}

std::optional<JointTelemetry> CanTransport::ReceiveTelemetry(int timeout_ms) {
#ifdef __linux__
    if (is_hardware_open_ && socket_fd_ >= 0) {
        struct pollfd pfd;
        pfd.fd = socket_fd_;
        pfd.events = POLLIN;

        int ret = poll(&pfd, 1, timeout_ms);
        if (ret > 0 && (pfd.revents & POLLIN)) {
            struct can_frame frame;
            ssize_t bytes_read = read(socket_fd_, &frame, sizeof(frame));
            if (bytes_read == sizeof(frame) && frame.can_dlc >= 8) {
                uint8_t node_id = static_cast<uint8_t>(frame.can_id & 0xFF);
                return CanProtocol::DecodeTelemetry(frame.data, node_id);
            }
        }
    }
#else
    (void)timeout_ms;
#endif
    return std::nullopt;
}

std::vector<uint8_t> CanTransport::ScanBus(int timeout_ms) {
    std::vector<uint8_t> discovered_nodes;

#ifdef __linux__
    if (is_hardware_open_ && socket_fd_ >= 0) {
        // Send a ping probe frame to broadcast address 0x100
        struct can_frame probe_frame;
        probe_frame.can_id = 0x100;
        probe_frame.can_dlc = 1;
        probe_frame.data[0] = 0xAA; // Discovery opcode
        write(socket_fd_, &probe_frame, sizeof(probe_frame));

        auto start = std::chrono::steady_clock::now();
        while (true) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start).count();
            if (elapsed >= timeout_ms) break;

            auto tele = ReceiveTelemetry(10);
            if (tele.has_value()) {
                discovered_nodes.push_back(tele->node_id);
            }
        }
        return discovered_nodes;
    }
#else
    (void)timeout_ms;
#endif

    return discovered_nodes;
}

} // namespace apexdrive
