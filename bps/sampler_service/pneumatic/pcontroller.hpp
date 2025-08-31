#ifndef BPS_PRESSURE_CONTROLLER_HPP
#define BPS_PRESSURE_CONTROLLER_HPP

#include <FreeRTOS.h>
#include <semphr.h>

#include <pico/stdlib.h>
#include <hardware/pwm.h>

#include <stdfloat>
#include <cstdint>
#include <array>

#include "common.hpp"
#include "queue.hpp"

namespace bps::sampler::pneumatic {

class PressureController {
    public:
        struct TriggerPack {
            std::float32_t current_pressure = 0.0_pa;
        };

        PressureController(uint const& chan_a_gpio) noexcept;

        void initialize() noexcept;
        void createTask(UBaseType_t const& priority) noexcept;

        QueueReference<TriggerPack> getTriggerPackQueueRef() const noexcept;
        QueueReference<std::float32_t> getTargetPressureQueueRef() const noexcept;
        
    private:
        // PWM related
        static constexpr uint kPwmChanPump  = PWM_CHAN_A;
        static constexpr uint kPwmChanValve = PWM_CHAN_B;

        static constexpr float    kPwmClkDiv  = 200.0f;
        static constexpr uint16_t kPwmMaxWrap = 5000;

        uint pump_gpio_pin;
        uint valve_gpio_pin;
        uint slice_num;
        float pump_pwm_level_percentage  = 0.0f;
        float valve_pwm_level_percentage = 0.0f;

        // Portional Control related
        static constexpr std::float32_t kp = 0.0005f;
        std::float32_t target_pressure = 0.0_pa;

        void controlPressure(std::float32_t const& current_pressure) noexcept;
        void pressureProcessRelease(float const& p_output) noexcept;

        // EMA related
        static constexpr std::float32_t kEmaAlpha = 0.05f;
        std::float32_t prev_pressure   = 0.0_pa;
        bool is_first_filtering = true;

        StaticQueue<TriggerPack, 512> trigger_pack_queue{};
        StaticQueue<std::float32_t, 3> target_pressure_queue{};

        // Task related
        StaticQueueSet<
            decltype(trigger_pack_queue),
            decltype(target_pressure_queue)
        > queue_set {
            this->trigger_pack_queue,
            this->target_pressure_queue
        };
        
        // Set the output level percentage for pump control, the range of percentage is [0.0f, 1.0f]
        PressureController& setValvePwmPercentage(float const& percentage) noexcept;
        PressureController& setPumpPwmPercentage(float const& percentage) noexcept;
        
        // FreeRTOS task
        static constexpr std::size_t kMaxLenOfTaskName = 25;
        static std::uint8_t task_counter;
        std::uint8_t task_id;
        TaskHandle_t task_handle = nullptr;
        SemaphoreHandle_t valve_done_sem = nullptr;
        StaticSemaphore_t valve_sem_buffer{};
        std::array<char, kMaxLenOfTaskName> task_name{0};
        void taskLoop() noexcept;
};

} // namespace bps::sampler::pneumatic

#endif // BPS_PRESSURE_CONTROLLER_HPP