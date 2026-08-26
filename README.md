# ApexDrive

[![CI Multi-Platform](https://github.com/Himan-D/apexdrive/actions/workflows/ci.yml/badge.svg)](https://github.com/Himan-D/apexdrive/actions/workflows/ci.yml)
[![PyPI](https://img.shields.io/pypi/v/apexdrive.svg)](https://pypi.org/project/apexdrive/)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS 2](https://img.shields.io/badge/ROS_2-Humble_%7C_Jazzy-brightgreen.svg)](https://docs.ros.org/en/humble/)
[![Platforms](https://img.shields.io/badge/Platforms-Linux_%7C_macOS_%7C_Windows_%7C_STM32G4-lightgrey.svg)](https://github.com/Himan-D/apexdrive)

**Robotics Actuator Control Engine, Inverter SDK & Simulation Suite**  
*Field-Oriented Control (FOC) • Linux SocketCAN Transport • ros2_control System Interface • STM32G4 Embedded Target*

---

## 1. Distribution & Installation Channels

| Channel | Method / Command | Description |
| :--- | :--- | :--- |
| **PyPI (Python Package Index)** | `pip install apexdrive` | High-level Python Client SDK & simulation bindings |
| **Homebrew (macOS / Linux)** | `brew install Himan-D/apexdrive/apexdrive` | Standalone Developer CLI diagnostic tool |
| **Debian / Ubuntu Package** | `sudo dpkg -i apexdrive_1.1.0_amd64.deb` | Pre-compiled Linux CLI & `libapexdrive_host.a` |
| **Docker (GHCR)** | `docker pull ghcr.io/himan-d/apexdrive:latest` | Multi-arch Linux runtime (`amd64`, `arm64`) |
| **ROS 2 ros2_control** | CMake `find_package(apexdrive_hardware)` | Standardized `hardware_interface::SystemInterface` plugin |
| **Source / Embedded C++20** | Git Submodule / CMake | Zero-dependency C++20 core (`include/apexdrive`) |

---

## 2. Architecture Overview

ApexDrive is organized into four decoupled architectural layers to bridge high-level robotics orchestration with bare-metal inverter electronics:

```
+-----------------------------------------------------------------------------+
| 1. ROBOTICS ORCHESTRATION & HIGH-LEVEL APIS                                 |
|    - ROS 2 ros2_control SystemInterface Plugin (C++)                        |
|    - Python Client SDK (import apexdrive)                                   |
|    - Developer Diagnostic CLI (apexdrive scan / monitor / bench / info)     |
+--------------------------------------┬--------------------------------------+
                                       | 1 kHz CAN-FD (CAN-FD v2 Protocol)
+--------------------------------------v--------------------------------------+
| 2. HOST TRANSPORT & PROTOCOL LAYER                                          |
|    - Linux SocketCAN Driver (socket(PF_CAN, SOCK_RAW, CAN_RAW))             |
|    - Symmetric Q15 Fixed-Point Frame Serialization & CRC16 Validation       |
|    - PlatformDetector: Dynamic Environment & Real-Time Kernel Auto-Discovery|
|    - Cross-Platform Deterministic Simulation Testbench (macOS / Linux)      |
+--------------------------------------┬--------------------------------------+
                                       | Bus Communication
+--------------------------------------v--------------------------------------+
| 3. CORE FOC VECTOR MATHEMATICS & SAFETY SUPERVISOR                          |
|    - Authoritative Single-Source MotorParameters (Analytical Kt, Ke)        |
|    - Forward/Inverse Clarke & Park Transformations                          |
|    - Space Vector Modulation (SVPWM) with Min/Max Common-Mode Injection     |
|    - Coupled Vector-Space Voltage Limiter with Anti-Windup Back-Calculation |
|    - Cross-Coupling Voltage Decoupling Feedforward                          |
|    - 256-Point Linear-Interpolated Anti-Cogging Harmonic Map                |
|    - Sliding Mode Observer (SMO) with Tracking Phase-Locked Loop (PLL)      |
|    - Continuous Salient PMSM Differential Dynamics Engine                   |
+--------------------------------------┬--------------------------------------+
                                       | Hardware Registers / DMA
+--------------------------------------v--------------------------------------+
| 4. BARE-METAL EMBEDDED FIRMWARE (firmware/stm32g4)                          |
|    - 25 kHz Injected ADC Conversion ISR (Phase Shunt Sampling)              |
|    - TIM1 Advanced Timer Center-Aligned Complementary PWM (120ns Dead-Time) |
|    - Hardware Safe Torque Off (STO) via TIM1 Break Input 1 (BKIN)           |
|    - SPI 14-Bit Magnetic Absolute Angle Encoder Driver (AS5047P / MA730)    |
+-----------------------------------------------------------------------------+
```

---

## 3. Core Capabilities

### Unified FOC Core (`apexdrive::FocEngine`)
* Zero dynamic memory allocation in control execution paths.
* Analytical motor parameter grounding: $K_t = 1.5 p \psi_f$, $K_e = \frac{\sqrt{3}}{2} p \psi_f$.
* Decoupled cross-coupling feedforward:
  $$V_d^* = V_{d,\text{PI}} - \omega_e L_q I_q$$
  $$V_q^* = V_{q,\text{PI}} + \omega_e (L_d I_d + \psi_f)$$
* Coupled vector-space voltage limiter ensuring $\sqrt{V_d^2 + V_q^2} \le V_{\max} = \frac{V_{\text{bus}}}{\sqrt{3}} \cdot 0.98$ with proportional back-calculation anti-windup integration.
* Shared single implementation across host simulation, hardware-in-the-loop (HIL) testing, and STM32 embedded firmware.

### Multi-Tier Safety Architecture
* **Hardware STO:** Direct analog comparator break input (`TIM1_BDTR.BKE`) tri-stating inverter gate drivers in $< 40\text{ ns}$ independently of software execution.
* **Software Safety Supervisor:** Continuous verification of peak phase overcurrent, DC bus overvoltage, under-voltage lockout (UVLO), stator/inverter thermal limits, and $I^2t$ continuous energy accumulation.
* **Command Watchdog:** Monotonic 25 ms heartbeat monitor requiring valid, CRC-verified frames to maintain torque generation.

### Compliant Motion Control
* Programmable virtual spring-damper impedance control law:
  $$\tau = K_p(\theta_d - \theta) + K_d(\dot{\theta}_d - \dot{\theta}) + \tau_{ff}$$
* Designed for multi-axis synchronization in legged and humanoid robotics.

---

## 4. Automatic Platform Auto-Adaptation

ApexDrive dynamically discovers the host machine environment at runtime via `apexdrive::PlatformDetector` and adjusts its execution backend without requiring manual configuration:

```bash
apexdrive info
```

```text
================================================================================
  APEXDRIVE PLATFORM AUTO-ADAPTATION REPORT                                     
================================================================================
  - Host Operating System : Linux (ARM64 / AArch64)
  - Real-Time Kernel (RT) : ACTIVE (PREEMPT_RT Hard Real-Time)
  - Detected CAN Busses   : can0, can1
  - Execution Backend     : PHYSICAL HARDWARE MODE (Bound to can0)
  - Strategy              : Native SocketCAN-FD kernel communication with transceivers
================================================================================
```

---

## 5. Building from Source & Running Tests

### Prerequisites
* C++20 compliant compiler (GCC 11+, Clang 14+, MSVC 2022, or Apple Clang)
* CMake 3.20+
* Linux with `libsocketcan-dev` (optional, required for physical CAN-FD bus communication)

### Build Commands
```bash
# Clone the repository
git clone https://github.com/Himan-D/apexdrive.git
cd apexdrive

# Configure and compile
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

# Run component unit tests and closed-loop hardware readiness scenarios
./test_suite
./control_scenarios_test
```

---

## 6. CLI Reference

```bash
# Display auto-detected host environment profile & active backend
./apexdrive info

# Scan CAN bus for physical joint actuators (falls back to simulation mode if no CAN hardware is present)
./apexdrive scan --interface can0

# Synthesize current loop PI gains and anti-cogging feedforward map
./apexdrive tune --id 0x14

# Launch real-time terminal telemetry monitor
./apexdrive monitor --id 0x14

# Output forensic circular black-box buffer
./apexdrive dump-blackbox

# Execute host-side 1,000,000-cycle timing benchmark
./apexdrive bench
```

---

## 7. ROS 2 Integration (`ros2_control`)

The `apexdrive_hardware` package provides a standardized `hardware_interface::SystemInterface` plugin for ROS 2 Humble, Iron, and Jazzy.

### URDF Configuration
```xml
<ros2_control name="ApexDriveSystem" type="system">
  <hardware>
    <plugin>apexdrive_hardware/ApexDriveHardware</plugin>
    <param name="can_interface">can0</param>
  </hardware>
  <joint name="joint_1">
    <param name="node_id">16</param>
    <command_interface name="position"/>
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

---

## 8. Python Client SDK

Install from PyPI:
```bash
pip install apexdrive
```

Usage Example:
```python
import apexdrive

# Initialize actuator connection
joint = apexdrive.Actuator(interface="can0", node_id=0x14)
joint.arm()

# Stream 1 kHz compliant impedance commands
# pos_rad: target angle, kp: stiffness (Nm/rad), kd: damping (Nm*s/rad), tau_ff: feedforward (Nm)
joint.set_impedance(pos_rad=1.57, vel_rad_s=0.0, kp=45.0, kd=2.5, tau_ff=1.2)

# Read telemetry snapshot
state = joint.get_state()
print(f"Position: {state.position_rad:.4f} rad | Torque: {state.torque_nm:.2f} Nm | Bus: {state.v_bus_v:.1f} V")
```

---

## 9. License

Distributed under the Apache 2.0 License. See `LICENSE` for details.
