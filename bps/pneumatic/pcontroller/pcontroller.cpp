#include "pcontroller.hpp"

#include <FreeRTOS.h>

#include <pico/stdlib.h>
#include <hardware/pwm.h>

#include <algorithm>

#include "psensor.hpp"

namespace bps::pneumatic::pcontroller {

PressureController::PressureController(uint const& chan_a_gpio): 
    chan_a_gpio_pin(chan_a_gpio),
    chan_b_gpio_pin(chan_a_gpio + 1)
{
    // Initialize gpio function to pwm utilization
    gpio_set_function(this->chan_a_gpio_pin, GPIO_FUNC_PWM);
    gpio_set_function(this->chan_b_gpio_pin, GPIO_FUNC_PWM);
    // Find the pwm slice of specified gpio
    this->slice_num = pwm_gpio_to_slice_num(chan_a_gpio);
    configASSERT((this->slice_num == pwm_gpio_to_slice_num(this->chan_b_gpio_pin)));
    // Tweak the clock tick speed to 1MHz
    pwm_set_clkdiv(this->slice_num, PressureController::kPwmClkDiv);
    // Set the wrap, which tweak the maximum frequency to 1kHz
    pwm_set_wrap(this->slice_num, PressureController::kPwmMaxWrap);
    // Set pwm level to 0 (no output)
    setGatePwmPercentage(0.0f);
    setPumpPwmPercentage(0.0f);
    // Set pwm running
    pwm_set_enabled(this->slice_num, true);
}

void PressureController::executePid(std::float32_t const& current_pressure) noexcept {

}

void PressureController::setNewTargetPressure(std::float32_t const& new_target) noexcept {
    this->target_pressure = new_target;
}

void PressureController::setGatePwmPercentage(float const& percentage) noexcept {
    this->gate_pwm_level_percentage = percentage;
    std::uint16_t level = std::clamp(
                            static_cast<std::uint16_t>(percentage * PressureController::kPwmMaxWrap),
                            static_cast<std::uint16_t>(0),
                            PressureController::kPwmMaxWrap
                          );
    pwm_set_chan_level(this->slice_num, PressureController::kPwmChanGate, level);
}

void PressureController::setPumpPwmPercentage(float const& percentage) noexcept {
    this->pump_pwm_level_percentage = percentage;
    std::uint16_t level = std::clamp(
                            static_cast<std::uint16_t>(percentage * PressureController::kPwmMaxWrap),
                            static_cast<std::uint16_t>(0),
                            PressureController::kPwmMaxWrap
                          );
    pwm_set_chan_level(this->slice_num, PressureController::kPwmChanPump, level);
}

} // bps::pneumatic::pcontroller