# ApexDrive ⚡

> **Robotics Actuator Control Engine, Inverter SDK & Simulation Suite**  
> *FOC Mathematics • Linux SocketCAN Transport • ros2_control System Interface • Embedded STM32G4 Target*

---

## 🎯 Architecture Overview

ApexDrive is structured into four distinct, decoupled layers to bridge the gap between high-level robotics orchestration and low-level inverter electronics:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│ 1. ROBOTICS ORCHESTRATION & HIGH-LEVEL APIS                                 │
│    • ROS 2 ros2_control SystemInterface Plugin (C++)                        │
│    • Python Client SDK (`import apexdrive`)                                 │
│    • Developer Diagnostic CLI (`apexdrive scan / monitor / bench`)          │
└──────────────────────────────────────┬──────────────────────────────────────┘
                                       │ 1 kHz CAN-FD (8-Byte Bit-Packed Protocol)
┌──────────────────────────────────────▼──────────────────────────────────────┐
│ 2. HOST TRANSPORT & PROTOCOL LAYER                                          │
│    • Linux SocketCAN Driver (`socket(PF_CAN, SOCK_RAW, CAN_RAW)`)           │
│    • High-Resolution Fixed-Point Frame Serialization (Q16 Position/Torque)  │
│    • Cross-Platform Deterministic Simulation Testbench (macOS / Windows)    │
└──────────────────────────────────────┬──────────────────────────────────────┘
                                       │ Real-Time Bus Communication
┌──────────────────────────────────────▼──────────────────────────────────────┐
│ 3. CORE FOC VECTOR MATHEMATICS & SAFETY SUPERVISOR                          │
│    • Forward/Inverse Clarke & Park Transformations                          │
│    • Space Vector Modulation (SVPWM) with 3rd-Harmonic Neutral Point Shift  │
│    • 256-Point Linear-Interpolated Anti-Cogging Harmonic Map                │
│    • Cross-Coupling Voltage Decoupling Feedforward                          │
│    • Sliding Mode Observer (SMO) Sensorless Back-EMF & Flux Estimator      │
│    • I²t Thermal Energy Accumulator & Software Safety Supervisor            │
└──────────────────────────────────────┬──────────────────────────────────────┘
                                       │ Hardware Registers / DMA
┌──────────────────────────────────────▼──────────────────────────────────────┐
│ 4. BARE-METAL EMBEDDED FIRMWARE (`firmware/stm32g4`)                        │
│    • 25 kHz Injected ADC Conversion ISR (Phase Current Shunt Sampling)      │
│    • TIM1 Advanced Timer Center-Aligned Complementary PWM + 120ns Dead-Time │
│    • Hardware Break Input (BKIN): Analog Comparator Safe Torque Off (STO)   │
│    • 14-Bit SPI Magnetic Absolute Encoder Driver (AS5047P / MA730)          │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 🚀 Key Capabilities

* **⚡ FOC Vector Engine:** Zero-allocation Clarke/Park transforms, anti-windup current PI regulators, and SVPWM for +15.4% bus utilization.
* **🛡️ Multi-Tier Safety Guard:** Sub-microsecond hardware Safe Torque Off (STO) via TIM1 break inputs paired with real-time software eFuse checks (Overcurrent, Overvoltage, UVLO brownout, Overtemp, and $I^2t$ thermal budgeting).
* **🤖 Native Compliant Impedance Control:** Virtual programmable spring-damper control law ($\tau = K_p(\theta_d - \theta) + K_d(\dot{\theta}_d - \dot{\theta}) + \tau_{ff}$) designed for humanoids and quadrupeds.
* **🔧 Anti-Cogging Feedforward:** 256-point high-resolution linear-interpolated lookup table (LUT) to eliminate stator slotting torque ripple.
* **📡 Linux SocketCAN Transport:** Native SocketCAN-FD socket layer with automatic non-blocking frame polling and multi-axis discovery.
* **🦾 ROS 2 Integration:** Production `ros2_control` hardware plugin (`apexdrive_hardware::ApexDriveHardware`) ready for URDF integration.
* **📼 Edge Flight Recorder:** 25 kHz circular in-memory buffer that freezes on hardware fault for forensic incident investigation.

---

## 🛠️ Quickstart (Building from Source)

```bash
# Clone the repository
git clone https://github.com/Himan-D/apexdrive.git
cd apexdrive

# Build C++ Library, CLI, and Test Suite
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

# Run the unit test suite
./test_suite
```

---

## 💻 CLI Commands

```bash
# Scan the CAN bus for connected joint actuators (uses Linux SocketCAN or mock fallback)
./apexdrive scan --interface can0

# Synthesize decoupled PI gains and anti-cogging map
./apexdrive tune --id 0x14

# Launch real-time terminal telemetry HUD and oscilloscope
./apexdrive monitor --id 0x14

# Dump forensic black-box incident log
./apexdrive dump-blackbox

# Run host-side 1,000,000-cycle FOC math and supervisor latency benchmark
./apexdrive bench
```

---

## 🦾 ROS 2 Integration (`ros2_control`)

In your robot's URDF description:

```xml
<ros2_control name="ApexDriveSystem" type="system">
  <hardware>
    <plugin>apexdrive_hardware/ApexDriveHardware</plugin>
    <param name="can_interface">can0</param>
  </hardware>
  <joint name="knee_joint">
    <command_interface name="position"/>
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

---

## 🐍 Python SDK (`import apexdrive`)

```python
import apexdrive

# Connect to actuator node on CAN bus
joint = apexdrive.Actuator("can0", node_id=0x14)
joint.arm()

# Stream 1 kHz compliant impedance commands
joint.set_impedance(pos_rad=1.57, kp=45.0, kd=2.5, tau_ff=1.2)

state = joint.get_state()
print(f"Angle: {state.position_rad:.3f} rad | Torque: {state.torque_nm:.2f} Nm")
```

---

## 📄 License
Apache 2.0.
