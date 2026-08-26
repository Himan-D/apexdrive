#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <string_view>

namespace apexdrive {

enum class HostOperatingSystem : uint8_t {
    LINUX,
    MACOS,
    WINDOWS,
    EMBEDDED_BAREMETAL,
    UNKNOWN
};

enum class ExecutionBackend : uint8_t {
    PHYSICAL_SOCKETCAN,   // Connected to real physical CAN transceiver (e.g. Jetson / PC with CAN card)
    VIRTUAL_SOCKETCAN,    // Connected to Linux kernel virtual CAN bus (vcan0)
    HOST_SIMULATION       // Running on developer workstation / laptop (macOS, Windows, or Linux without CAN)
};

struct PlatformProfile {
    HostOperatingSystem os{HostOperatingSystem::UNKNOWN};
    std::string os_name{"Unknown"};
    std::string cpu_arch{"Unknown"};
    bool is_realtime_kernel{false};        // Linux PREEMPT_RT active
    bool has_root_privileges{false};       // CAP_NET_ADMIN / sudo permissions
    std::vector<std::string> available_can_interfaces; // e.g. ["can0", "can1"]
    ExecutionBackend recommended_backend{ExecutionBackend::HOST_SIMULATION};
    std::string active_interface{"can0"};
};

/**
 * Intelligent Hardware & Host Platform Auto-Detector.
 * Dynamically queries kernel interfaces, network devices, and CPU capabilities to adapt
 * the runtime execution mode automatically without requiring manual configuration.
 */
class PlatformDetector {
public:
    [[nodiscard]] static PlatformProfile DetectEnvironment() noexcept;
    
    [[nodiscard]] static std::string GenerateDiagnosticSummary(const PlatformProfile& profile);
};

} // namespace apexdrive
