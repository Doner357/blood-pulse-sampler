#ifndef BPS_PRESSURE_CONTROLLER_HPP
#define BPS_PRESSURE_CONTROLLER_HPP

#include <stdfloat>
#include <cstdint>

#include <pico/stdlib.h>
#include <hardware/pwm.h>

#include "common.hpp"

namespace bps::sampler::pneumatic {

class PressureController {
    public:
        /**
         * @brief Initializes a controller for a pair of PWM channels (A and B) on a single PWM slice.
         *
         * This constructor configures the controller to manage both channels of a PWM slice.
         * The slice is identified by the GPIO pin number provided for Channel A.
         *
         * @param chan_a_gpio The GPIO pin number that is physically connected to a PWM channel A.
         *
         * @note The corresponding Channel B GPIO pin is automatically deduced and managed by this
         *       controller as well. For instance, if you provide GPIO 0 (Slice 0, Channel A),
         *       the controller will automatically manage GPIO 1 (Slice 0, Channel B).
         *
         * @warning The provided GPIO pin **must** be one that maps to a PWM Channel A
         *          (typically an even-numbered GPIO on the RP2040 & RP2350). Providing a GPIO that
         *          maps to a Channel B will result in an assertion failure during initialization.
         *          This constructor will also cause the taskDelay to the caller.
         */
        PressureController(uint const& chan_a_gpio, std::float32_t const& pressure_base_error = 0.0_pa) noexcept;

        void executePid(std::float32_t const& current_pressure, std::uint64_t const& current_time) noexcept;
        PressureController& setNewTargetPressure(std::float32_t const& new_target) noexcept;
        PressureController& setPressureError(std::float32_t const& error) noexcept;

    private:
        std::float32_t pressure_error;
        
        // PWM related
        static constexpr uint kPwmChanValve = PWM_CHAN_A;
        static constexpr uint kPwmChanPump  = PWM_CHAN_B;

        static constexpr float    kPwmClkDiv  = 150.0f;
        static constexpr uint16_t kPwmMaxWrap = 999;

        uint chan_a_gpio_pin;
        uint chan_b_gpio_pin;
        uint slice_num;
        std::uint16_t valve_pwm_level_percentage = 0;
        std::uint16_t pump_pwm_level_percentage  = 0;

        // PID related
        static constexpr std::float32_t kPidDeadBandThreshold = 100.0_pa;
        struct PidConstant {
            static constexpr std::float32_t kp = 0.0f;
            static constexpr std::float32_t ki = 0.0f;
            static constexpr std::float32_t kd = 0.0f;
        };
        std::float32_t pid_error_prev      = 0.0_pa;
        std::float32_t pid_integral        = 0.0_pa;
        std::float32_t pid_prev_pressure   = 0.0_pa;
        std::uint64_t  pid_prev_time       = 0u;
        std::float32_t pid_target_pressure = 0.0_pa;

        // EMA related
        static constexpr std::float32_t kEmaAlpha = 0.05f;

        // Set the output level percentage for valve control, the range of percentage is [0.0f, 1.0f]
        PressureController& setValvePwmPercentage(float const& percentage) noexcept;
        // Set the output level percentage for pump control, the range of percentage is [0.0f, 1.0f]
        PressureController& setPumpPwmPercentage(float const& percentage) noexcept;
};

} // namespace bps::sampler::pneumatic

#endif // BPS_PRESSURE_CONTROLLER_HPP