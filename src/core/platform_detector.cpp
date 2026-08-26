#include "../../include/apexdrive/core/platform_detector.hpp"
#include <sstream>
#include <fstream>
#include <iostream>
#include <algorithm>

#ifdef __linux__
#include <unistd.h>
#include <net/if.h>
#include <sys/types.h>
#include <dirent.h>
#elif defined(__APPLE__)
#include <sys/types.h>
#include <sys/sysctl.h>
#include <unistd.h>
#endif

namespace apexdrive {

PlatformProfile PlatformDetector::DetectEnvironment() noexcept {
    PlatformProfile profile;

    // 1. Detect Host OS & Architecture
#if defined(__linux__)
    profile.os = HostOperatingSystem::LINUX;
    profile.os_name = "Linux";
#elif defined(__APPLE__)
    profile.os = HostOperatingSystem::MACOS;
    profile.os_name = "macOS";
#elif defined(_WIN32)
    profile.os = HostOperatingSystem::WINDOWS;
    profile.os_name = "Windows";
#else
    profile.os = HostOperatingSystem::UNKNOWN;
    profile.os_name = "Generic POSIX";
#endif

    // Detect CPU Architecture
#if defined(__aarch64__) || defined(_M_ARM64)
    profile.cpu_arch = "ARM64 / AArch64";
#elif defined(__x86_64__) || defined(_M_X64)
    profile.cpu_arch = "x86_64 / AMD64";
#elif defined(__arm__)
    profile.cpu_arch = "ARM32";
#else
    profile.cpu_arch = "Generic Arch";
#endif

    // 2. Query Real-Time Kernel (Linux PREEMPT_RT)
#ifdef __linux__
    std::ifstream rt_file("/sys/kernel/realtime");
    if (rt_file.is_open()) {
        int val = 0;
        rt_file >> val;
        profile.is_realtime_kernel = (val == 1);
    }
    profile.has_root_privileges = (geteuid() == 0);
#endif

    // 3. Scan for Active CAN Interfaces
#ifdef __linux__
    DIR* dir = opendir("/sys/class/net");
    if (dir != nullptr) {
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string if_name(entry->d_name);
            if (if_name.rfind("can", 0) == 0 || if_name.rfind("vcan", 0) == 0 || if_name.rfind("slcan", 0) == 0) {
                profile.available_can_interfaces.push_back(if_name);
            }
        }
        closedir(dir);
    }

    std::sort(profile.available_can_interfaces.begin(), profile.available_can_interfaces.end());

    // 4. Adapt Execution Backend Automatically
    if (!profile.available_can_interfaces.empty()) {
        profile.active_interface = profile.available_can_interfaces.front();
        if (profile.active_interface.rfind("vcan", 0) == 0) {
            profile.recommended_backend = ExecutionBackend::VIRTUAL_SOCKETCAN;
        } else {
            profile.recommended_backend = ExecutionBackend::PHYSICAL_SOCKETCAN;
        }
    } else {
        profile.recommended_backend = ExecutionBackend::HOST_SIMULATION;
        profile.active_interface = "sim0";
    }
#else
    // Non-Linux hosts run in high-precision simulation testbench mode
    profile.recommended_backend = ExecutionBackend::HOST_SIMULATION;
    profile.active_interface = "sim0";
#endif

    return profile;
}

std::string PlatformDetector::GenerateDiagnosticSummary(const PlatformProfile& profile) {
    std::ostringstream oss;
    oss << "================================================================================\n";
    oss << "  APEXDRIVE PLATFORM AUTO-ADAPTATION REPORT                                     \n";
    oss << "================================================================================\n";
    oss << "  - Host Operating System : " << profile.os_name << " (" << profile.cpu_arch << ")\n";

    if (profile.os == HostOperatingSystem::LINUX) {
        oss << "  - Real-Time Kernel (RT) : " << (profile.is_realtime_kernel ? "ACTIVE (PREEMPT_RT Hard Real-Time)" : "Standard Linux Kernel") << "\n";
        oss << "  - Detected CAN Busses   : ";
        if (profile.available_can_interfaces.empty()) {
            oss << "None found in /sys/class/net\n";
        } else {
            for (size_t i = 0; i < profile.available_can_interfaces.size(); ++i) {
                oss << profile.available_can_interfaces[i] << (i + 1 < profile.available_can_interfaces.size() ? ", " : "\n");
            }
        }
    }

    oss << "  - Execution Backend     : ";
    switch (profile.recommended_backend) {
        case ExecutionBackend::PHYSICAL_SOCKETCAN:
            oss << "\033[1;32mPHYSICAL HARDWARE MODE (Bound to " << profile.active_interface << ")\033[0m\n";
            oss << "  - Strategy              : Native SocketCAN-FD kernel communication with transceivers\n";
            break;
        case ExecutionBackend::VIRTUAL_SOCKETCAN:
            oss << "\033[1;36mVIRTUAL BUS MODE (Bound to " << profile.active_interface << ")\033[0m\n";
            oss << "  - Strategy              : Linux kernel virtual CAN bus simulation\n";
            break;
        case ExecutionBackend::HOST_SIMULATION:
            oss << "\033[1;33mHIGH-FIDELITY SIMULATION TESTBENCH\033[0m\n";
            oss << "  - Strategy              : Continuous 25 kHz closed-loop PMSM physics & FOC testbench\n";
            break;
    }
    oss << "================================================================================\n";
    return oss.str();
}

void PlatformDetector::PrintReport() noexcept {
    auto profile = DetectEnvironment();
    std::cout << GenerateDiagnosticSummary(profile) << "\n";
}

} // namespace apexdrive
