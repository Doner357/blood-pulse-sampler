#include "pcontroller.hpp"

#include <FreeRTOS.h>
#include <semphr.h>

#include <pico/stdlib.h>
#include <hardware/pwm.h>

#include <algorithm>
#include <cmath>
#include <stdfloat>
#include <cstdint>
#include <array>
#include <cstring>
#include <cstdio>

#include "logger.hpp"

namespace bps::sampler::pneumatic {

std::uint8_t PressureController::task_counter = 0;

PressureController::PressureController(uint const& chan_a_gpio) noexcept: 
    pump_gpio_pin(chan_a_gpio),
    valve_gpio_pin(chan_a_gpio + 1),
    task_id(task_counter++)
{}

void PressureController::initialize() noexcept {
    // Initialize gpio function to pwm utilization
    gpio_set_function(this->pump_gpio_pin, GPIO_FUNC_PWM);
    gpio_set_function(this->valve_gpio_pin, GPIO_FUNC_PWM);
    // Find the pwm slice of specified gpio
    this->slice_num = pwm_gpio_to_slice_num(this->pump_gpio_pin);
    configASSERT((this->slice_num == pwm_gpio_to_slice_num(this->valve_gpio_pin)));
    // Tweak the clock tick speed to 1MHz
    pwm_set_clkdiv(this->slice_num, kPwmClkDiv);
    // Set the wrap, which tweak the maximum frequency to 1kHz
    pwm_set_wrap(this->slice_num, kPwmMaxWrap);
    // Set pwm level to 0 (no output)
    setPumpPwmPercentage(0.0f);
    setValvePwmPercentage(0.0f);
    // Set pwm running
    pwm_set_enabled(this->slice_num, true);
}

void PressureController::createTask(UBaseType_t const& priority) noexcept {
    this->valve_done_sem = xSemaphoreCreateBinaryStatic(&this->valve_sem_buffer);
    static auto freertos_task = 
        [](void* context) {
            PressureController* controller = static_cast<PressureController*>(context);
            controller->taskLoop();
        };
    // Add task id as the suffix of created task.
    // Use this weird method so no dynamic resource allocation
    snprintf(this->task_name.data(), this->task_name.size(), "PressureController-%d", this->task_id);
    xTaskCreate(
        freertos_task,
        this->task_name.data(),
        1024,
        this,
        priority,
        &this->task_handle
    );
}

QueueReference<PressureController::TriggerPack> PressureController::getTriggerPackQueueRef() const noexcept {
    return this->trigger_pack_queue;
}

QueueReference<std::float32_t> PressureController::getTargetPressureQueueRef() const noexcept {
    return this->target_pressure_queue;
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

void PressureController::controlPressure(std::float32_t const& current_pressure) noexcept {
    if (current_pressure > 90000.0_pa) {
        this->setPumpPwmPercentage(0.0f).setValvePwmPercentage(0.0f);
        return;
    }
    
    // Signal Processing (Exponential Moving Average)
    if (this->is_first_filtering) {
        this->prev_pressure = current_pressure;
        this->is_first_filtering = false;
    }
    std::float32_t filtered_value = kEmaAlpha * current_pressure + (1 - kEmaAlpha) * prev_pressure;
    
    // --- PID ---
    std::float32_t error = this->target_pressure - filtered_value;
    float output = kp * error;

    output = this->target_pressure < 100.0f ? 0.0f : std::clamp(output, -1.0f, 1.0f);
    if (output > 0.0f) {
        setValvePwmPercentage(1.0f);
        setPumpPwmPercentage(output);
    } else if (output < 0.0f) {
        pressureProcessRelease(output);
    } else {
        setValvePwmPercentage(0.0f);
        setPumpPwmPercentage(0.0f);
    }
    
    this->prev_pressure = filtered_value;
}

void PressureController::pressureProcessRelease(float const& p_output) noexcept {
    setPumpPwmPercentage(0.0f);

    constexpr std::uint64_t kMaxVentUsPerCycle = 7000;
    std::uint64_t open_time_us = static_cast<std::uint64_t>(std::pow(p_output, 2) * kMaxVentUsPerCycle);
    
    if (open_time_us < 50) {
        setValvePwmPercentage(1.0f);
        return;
    }

    // Release pressure
    setValvePwmPercentage(0.0f);
    // Create alarm callback which will be called after @open_time_us us
    static auto valve_alarm_callback = []([[maybe_unused]]alarm_id_t id, void* user_data) -> int64_t {
        PressureController* self = reinterpret_cast<PressureController*>(user_data);
        self->setValvePwmPercentage(1.0f);
        BaseType_t higher_priority_taskwoken = pdFALSE;
        xSemaphoreGiveFromISR(self->valve_done_sem, &higher_priority_taskwoken);
        portYIELD_FROM_ISR(higher_priority_taskwoken);
        return 0;
    };
    // Add the alarm
    add_alarm_in_us(open_time_us, valve_alarm_callback, this, true);
    xSemaphoreTake(this->valve_done_sem, portMAX_DELAY);
}

void PressureController::taskLoop() noexcept {
    while (true) {
        std::expected<QueueHandle_t, std::nullptr_t> selected_handle{};
        if ((selected_handle = this->queue_set.selectFromSet(portMAX_DELAY))) {
            if (selected_handle == this->trigger_pack_queue.getFreeRTOSQueueHandle()) {
                TriggerPack trigger_pack{};
                this->trigger_pack_queue.receive(trigger_pack, pdTICKS_TO_MS(0));
                controlPressure(trigger_pack.current_pressure);
            } else if (selected_handle == this->target_pressure_queue.getFreeRTOSQueueHandle()) {
                this->target_pressure_queue.receive(this->target_pressure, pdTICKS_TO_MS(0));
                BPS_LOG("%s: Set target pressure to %f\n", this->task_name.data(), static_cast<double>(this->target_pressure));
            }
        }
    }
}

} // bps::sampler::pneumatic