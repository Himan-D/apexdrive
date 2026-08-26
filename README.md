# ApexDrive ⚡

> **Universal High-Performance Robotics Actuator & Motor Control Engine**  
> *Hard Real-Time 25 kHz FOC • Sub-Microsecond eFuse Protection • 1 kHz Compliant Impedance API • CAN-FD / EtherCAT*

---

## 🚀 Key Features

* **⚡ 25 kHz Zero-Allocation FOC Core:** Space Vector Modulation (SVPWM) with 3rd-harmonic neutral point shift (+15.4% DC bus utilization).
* **🛡️ Sub-Microsecond Hardware eFuse:** Instantaneous overcurrent circuit breaker (< 50ns) and real-time $I^2t$ thermal energy accumulator.
* **🤖 Native Compliant Impedance Control:** Virtual programmable spring-damper model ($K_p, K_d, \tau_{ff}$) designed for humanoids and quadrupeds.
* **🔧 10-Second Auto-Tuner:** Automated measurement of phase resistance ($R$), inductance ($L$), flux constant ($K_t$), and 360° anti-cogging ripple cancellation.
* **📦 Bit-Packed CAN-FD Protocol:** Ultra-compact 8-byte frames for multi-axis daisy-chained joint synchronization.
* **📼 Edge Flight Recorder:** Continuous 25 kHz circular in-memory ring buffer capturing pre- and post-incident forensic telemetry.

---

## 🛠️ Quickstart (Building from Source)

```bash
# Clone the repository
cd /Users/himand/apexdrive

# Build C++ Library, CLI, and Test Suite
mkdir build && cd build
cmake ..
make -j$(nproc)

# Run the unit test suite
./test_suite
```

---

## 💻 CLI Commands

```bash
# Scan the CAN-FD bus for connected joint actuators
./apexdrive scan

# Run 10-second automated electrical and anti-cogging calibration
./apexdrive tune --id 0x14

# Launch real-time terminal telemetry HUD and oscilloscope
./apexdrive monitor --id 0x14

# Dump forensic flight black-box incident report
./apexdrive dump-blackbox

# Run 100,000-cycle deterministic timing and FOC benchmark
./apexdrive bench
```

---

## 🦾 C++20 Client Integration Example

```cpp
#include <apexdrive/host/actuator.hpp>
#include <iostream>

int main() {
    // Connect to Joint 0x14 on CAN-FD bus
    apexdrive::Actuator knee("can0", 0x14);
    knee.Arm();

    // 1 kHz Compliant Control Loop
    while (true) {
        float target_pos_rad = 1.57f;  // 90 degrees
        float target_vel_rad_s = 0.0f;
        float stiffness_kp = 45.0f;    // 45 Nm/rad
        float damping_kd = 2.5f;       // 2.5 Nm/(rad/s)
        float tau_feedforward = 5.0f;  // Gravity compensation

        knee.SetImpedance(target_pos_rad, target_vel_rad_s, stiffness_kp, damping_kd, tau_feedforward);

        auto state = knee.GetState();
        std::cout << "Knee Position: " << state.position_rad 
                  << " rad | Torque: " << state.torque_nm << " Nm\n";
    }
}
```

---

## 🐍 Python SDK Integration

```python
import apexdrive

joint = apexdrive.Actuator("can0", node_id=0x14)
joint.arm()

# Set compliant impedance
joint.set_impedance(pos_rad=1.57, kp=40.0, kd=2.0, tau_ff=3.0)

state = joint.get_state()
print(f"Angle: {state.position_rad:.3f} rad | Temp: {state.temperature_c:.1f} °C")
```

---

## 📄 License
Apache 2.0 / Commercial Dual-License.
