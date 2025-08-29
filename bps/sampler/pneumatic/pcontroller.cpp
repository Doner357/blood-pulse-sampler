#include "pcontroller.hpp"

#include <FreeRTOS.h>

#include <pico/stdlib.h>
#include <hardware/pwm.h>

#include <algorithm>
#include <stdfloat>
#include <cstdint>
#include <array>
#include <cstring>

namespace bps::sampler::pneumatic {

std::uint8_t PressureController::task_counter = 0;

PressureController::PressureController(uint const& chan_a_gpio) noexcept: 
    chan_a_gpio_pin(chan_a_gpio),
    chan_b_gpio_pin(chan_a_gpio + 1),
    task_id(task_counter++)
{}

void PressureController::initialize() noexcept {
    // Initialize gpio function to pwm utilization
    gpio_set_function(this->chan_a_gpio_pin, GPIO_FUNC_PWM);
    gpio_set_function(this->chan_b_gpio_pin, GPIO_FUNC_PWM);
    // Find the pwm slice of specified gpio
    this->slice_num = pwm_gpio_to_slice_num(this->chan_a_gpio_pin);
    configASSERT((this->slice_num == pwm_gpio_to_slice_num(this->chan_b_gpio_pin)));
    // Tweak the clock tick speed to 1MHz
    pwm_set_clkdiv(this->slice_num, kPwmClkDiv);
    // Set the wrap, which tweak the maximum frequency to 1kHz
    pwm_set_wrap(this->slice_num, kPwmMaxWrap);
    // Set pwm level to 0 (no output)
    setValvePwmPercentage(0.0f);
    setPumpPwmPercentage(0.0f);
    // Set pwm running
    pwm_set_enabled(this->slice_num, true);
}

void PressureController::createTask(UBaseType_t const& priority) noexcept {
    static auto freertos_task = 
        [](void* context) {
            PressureController* controller = static_cast<PressureController*>(context);
            controller->taskLoop();
        };
    // Add task id as the suffix of created task.
    // Use this weird method so no dynamic resource allocation
    this->task_name[20] = static_cast<char>('0' + (this->task_id / 100));
    this->task_name[21] = static_cast<char>('0' + (this->task_id / 10));
    this->task_name[22] = static_cast<char>('0' + (this->task_id));
    this->task_name[23] = '\0';
    xTaskCreate(
        freertos_task,
        this->task_name.data(),
        1024,
        this,
        priority,
        &this->task_handle
    );
}

QueueReference<PressureController::PidTriggerPack> PressureController::getPidTriggerPackQueueRef() const noexcept {
    return this->pid_trigger_pack_queue;
}

QueueReference<std::float32_t> PressureController::getPidTargetPressureQueueRef() const noexcept {
    return this->pid_target_pressure_queue;
}

PressureController& PressureController::setValvePwmPercentage(float const& percentage) noexcept {
    this->valve_pwm_level_percentage = percentage;
    std::uint16_t level = std::clamp(
                            static_cast<std::uint16_t>(percentage * kPwmMaxWrap),
                            static_cast<std::uint16_t>(0),
                            kPwmMaxWrap
                          );
    pwm_set_chan_level(this->slice_num, kPwmChanValve, level);
    
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
    std::float32_t derivative = (error - this->pid_prev_error) / interval_us;
    std::float32_t output = (PidConstant::kp * error) + 
                            (PidConstant::ki * this->pid_integral) + 
                            (PidConstant::kd * derivative);
    // TODO: Update PWM part
    
    this->pid_prev_pressure = filtered_value;
    this->pid_prev_error = error;
}

void PressureController::taskLoop() noexcept {
    while (true) {
        std::expected<QueueHandle_t, std::nullptr_t> selected_handle{};
        if ((selected_handle = this->queue_set.selectFromSet(portMAX_DELAY))) {
            if (selected_handle == this->pid_trigger_pack_queue.getFreeRTOSQueueHandle()) {
                PidTriggerPack trigger_pack{};
                this->pid_trigger_pack_queue.receive(trigger_pack, pdTICKS_TO_MS(0));
                executePid(trigger_pack.current_pressure, trigger_pack.current_time_us);
            } else if (selected_handle == this->pid_target_pressure_queue.getFreeRTOSQueueHandle()) {
                std::float32_t target_pressure = 0.0f;
                this->pid_target_pressure_queue.receive(target_pressure, pdTICKS_TO_MS(0));
                this->pid_target_pressure = target_pressure;
            }
        }
    }
}

} // bps::sampler::pneumatic