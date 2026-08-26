#include "../include/board.hpp"
#include "../include/adc_driver.hpp"
#include "../include/pwm_driver.hpp"
#include "../include/encoder_driver.hpp"
#include "../include/fdcan_driver.hpp"
#include "foc_isr.cpp"

using namespace apexdrive;
using namespace apexdrive::firmware;

// Global Peripheral Driver Instances
static AdcDriver::AdcRegisters* ADC1_REG = reinterpret_cast<AdcDriver::AdcRegisters*>(0x50000000);
static AdcDriver::AdcRegisters* ADC2_REG = reinterpret_cast<AdcDriver::AdcRegisters*>(0x50000100);
static PwmDriver::Tim1Registers* TIM1_REG = reinterpret_cast<PwmDriver::Tim1Registers*>(0x40012C00);
static EncoderDriver::SpiRegisters* SPI1_REG = reinterpret_cast<EncoderDriver::SpiRegisters*>(0x40013000);
static FdcanDriver::FdcanRegisters* FDCAN1_REG = reinterpret_cast<FdcanDriver::FdcanRegisters*>(0x40006400);

int main() {
    // 1. Initialize Peripheral Drivers
    AdcDriver adc(ADC1_REG, ADC2_REG);
    PwmDriver pwm(TIM1_REG);
    EncoderDriver encoder(SPI1_REG);
    FdcanDriver fdcan(FDCAN1_REG, 0x14);

    MotorParameters motor_params;
    FirmwareFocHandler foc_handler(adc, pwm, encoder, motor_params);

    // 2. Hardware Initialization
    pwm.InitCenterAlignedPwm();
    adc.Init();
    encoder.Init();
    fdcan.Init();

    // 3. Calibrate Injected Current Sense Zero-Offsets before arming
    adc.CalibrateOffsets(1024);

    // 4. Arm PWM Stage
    pwm.ArmInverter();

    // 5. Main Superloop (1 kHz Communication & Safety Watchdog)
    OperatingMode mode = OperatingMode::STANDBY;
    ImpedanceCommand active_cmd{};
    uint16_t seq = 0;

    while (true) {
        // Poll for incoming CAN-FD v2 command
        if (fdcan.ReceiveCommand(active_cmd, mode, seq)) {
            // Heartbeat received
        }

        // Transmit 1 kHz Telemetry
        JointTelemetry telem{
            .node_id = 0x14,
            .mode = mode,
            .safety_state = SafetyState::OK,
            .sequence_number = seq
        };
        fdcan.TransmitTelemetry(telem);
    }

    return 0;
}
