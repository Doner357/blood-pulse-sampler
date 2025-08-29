#include "pcontroller.hpp"

#include <FreeRTOS.h>

#include <pico/stdlib.h>
#include <hardware/pwm.h>

#include <algorithm>
#include <stdfloat>
#include <cstdint>

#include "psensor.hpp"

namespace bps::sampler::pneumatic {

PressureController::PressureController(uint const& chan_a_gpio, std::float32_t const& pressure_base_error) noexcept: 
    pressure_error(pressure_base_error),
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
    pwm_set_clkdiv(this->slice_num, kPwmClkDiv);
    // Set the wrap, which tweak the maximum frequency to 1kHz
    pwm_set_wrap(this->slice_num, kPwmMaxWrap);
    // Set pwm level to 0 (no output)
    setGatePwmPercentage(0.0f);
    setPumpPwmPercentage(0.0f);
    // Set pwm running
    pwm_set_enabled(this->slice_num, true);
}

void PressureController::executePid(std::float32_t const& current_pressure, std::uint64_t const& current_time) noexcept {
    // Signal Processing (Exponential Moving Average)
    std::float32_t filtered_value = kEmaAlpha * current_pressure + (1 - kEmaAlpha) * pid_prev_pressure;
    
    // --- PID ---
    std::uint64_t interval_us = current_time - this->pid_prev_time;
    this->pid_prev_time = current_time;

    std::float32_t error = this->pid_target_pressure - current_pressure;
    // Deal the deadband
    if (std::abs(error) < kPidDeadBandThreshold) {
        this->pid_integral = 0;
        return;
    }

    this->pid_integral = this->pid_integral + (error * interval_us);
    std::float32_t derivative = (error - this->pid_error_prev) / interval_us;
    std::float32_t output = (PidConstant::kp * error) + 
                            (PidConstant::ki * this->pid_integral) + 
                            (PidConstant::kd * derivative);
    
    /* TODO: Update PWM part */
    
    this->pid_prev_pressure = filtered_value;
    this->pid_error_prev = error;
}

PressureController& PressureController::setNewTargetPressure(std::float32_t const& new_target) noexcept {
    this->pid_target_pressure = new_target;
    return *this;
}

PressureController& PressureController::setPressureError(std::float32_t const& error) noexcept {
    this->pressure_error = error;
    return *this;
}

PressureController& PressureController::setGatePwmPercentage(float const& percentage) noexcept {
    this->gate_pwm_level_percentage = percentage;
    std::uint16_t level = std::clamp(
                            static_cast<std::uint16_t>(percentage * kPwmMaxWrap),
                            static_cast<std::uint16_t>(0),
                            kPwmMaxWrap
                          );
    pwm_set_chan_level(this->slice_num, kPwmChanGate, level);
    
    return *this;
}

PressureController& PressureController::setPumpPwmPercentage(float const& percentage) noexcept {
    this->pump_pwm_level_percentage = percentage;
    std::uint16_t level = std::clamp(
                            static_cast<std::uint16_t>(percentage * kPwmMaxWrap),
                            static_cast<std::uint16_t>(0),
                            kPwmMaxWrap
                          );
    pwm_set_chan_level(this->slice_num, kPwmChanPump, level);

    return *this;
}

} // bps::sampler::pneumatic