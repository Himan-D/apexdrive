#include "../include/board.hpp"
#include "../include/adc_driver.hpp"
#include "../include/pwm_driver.hpp"
#include "../include/encoder_driver.hpp"
#include "../../../include/apexdrive/control/foc_core.hpp"

namespace apexdrive::firmware {

/**
 * 25 kHz Injected ADC Conversion ISR (High-Priority Real-Time Thread).
 * Target WCET: < 3.2 microseconds on 170 MHz Cortex-M4 with Hardware FPU.
 * 
 * Execution Pipeline:
 * 1. Read calibrated Injected Phase Currents (Ia, Ib, Ic)
 * 2. Read 14-bit Absolute Rotor Angle via 10.6 MHz SPI (AS5047P)
 * 3. Execute Unified FocEngine (Clarke, Park, PI, Decoupling, Vector Limiter, SVPWM)
 * 4. Write PWM Compare Registers (TIM1->CCR1, CCR2, CCR3)
 */
class FirmwareFocHandler {
public:
    FirmwareFocHandler(
        AdcDriver& adc, 
        PwmDriver& pwm, 
        EncoderDriver& encoder, 
        const MotorParameters& params
    ) noexcept 
        : adc_(adc), pwm_(pwm), encoder_(encoder), params_(params), foc_engine_(params) {
        foc_engine_.SetCurrentLoopBandwidth(1500.0f);
    }

    // Called strictly inside ADC1_2_IRQHandler (25,000 times/second)
    void OnAdcIsr(float target_iq, float v_bus = 48.0f) noexcept {
        // 1. Read calibrated phase currents
        float ia = 0.0f, ib = 0.0f, ic = 0.0f;
        adc_.ReadPhaseCurrents(ia, ib, ic);

        // 2. Read absolute rotor mechanical & electrical angles
        float theta_mech = 0.0f;
        if (!encoder_.ReadAngleRad(theta_mech)) {
            // Parity fault -> safe trip
            pwm_.EmergencySafeTorqueOff();
            return;
        }
        float theta_elec = theta_mech * static_cast<float>(params_.pole_pairs);

        // 3. Step Unified FOC Engine
        FocEngine::FocInputs foc_in{
            .i_phase_a = ia,
            .i_phase_b = ib,
            .i_phase_c = ic,
            .electrical_angle_rad = theta_elec,
            .electrical_speed_rad_s = 0.0f, // Estimated by PLL or velocity filter
            .target_id_a = 0.0f,
            .target_iq_a = target_iq,
            .v_bus = v_bus
        };

        auto foc_out = foc_engine_.Step(foc_in, 0.00004f); // 40 microseconds dt

        // 4. Update complementary PWM duty cycles
        pwm_.SetDutyCycles(foc_out.duty_u, foc_out.duty_v, foc_out.duty_w);
    }

    [[nodiscard]] FocEngine& GetFocEngine() noexcept { return foc_engine_; }

private:
    AdcDriver& adc_;
    PwmDriver& pwm_;
    EncoderDriver& encoder_;
    MotorParameters params_;
    FocEngine foc_engine_;
};

} // namespace apexdrive::firmware
