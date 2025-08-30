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
        struct PidTriggerPack {
            std::float32_t current_pressure = 0.0_pa;
            std::uint64_t  current_time_us  = 0;
        };

        PressureController(uint const& chan_a_gpio) noexcept;

        void initialize() noexcept;
        void createTask(UBaseType_t const& priority) noexcept;

        QueueReference<PidTriggerPack> getPidTriggerPackQueueRef() const noexcept;
        QueueReference<std::float32_t> getPidTargetPressureQueueRef() const noexcept;
        
    private:
        // PWM related
        static constexpr uint kPwmChanPump  = PWM_CHAN_A;
        static constexpr uint kPwmChanValve = PWM_CHAN_B;

        static constexpr float    kPwmClkDiv  = 150.0f;
        static constexpr uint16_t kPwmMaxWrap = 999;

        uint pump_gpio_pin;
        uint valve_gpio_pin;
        uint slice_num;
        float pump_pwm_level_percentage  = 0.0f;
        float valve_pwm_level_percentage = 0.0f;

        // PID related
        static constexpr std::float32_t kPidDeadBandThreshold = 50.0_pa;
        struct PidConstant {
            static constexpr std::float32_t kp = 0.0025f;
            static constexpr std::float32_t ki = 0.0f;
            static constexpr std::float32_t kd = 0.0f;
        };
        std::float32_t pid_integral        = 0.0_pa;
        std::float32_t pid_prev_error      = 0.0_pa;
        std::float32_t pid_prev_pressure   = 0.0_pa;
        float          pid_prev_output     = 0.0f;
        std::uint64_t  pid_prev_time       = 0u;
        std::float32_t pid_target_pressure = 0.0_pa;

        void executePid(std::float32_t const& current_pressure, std::uint64_t const& current_time) noexcept;
        void pidProcessRelease(float const& pid_output) noexcept;

        // EMA related
        static constexpr std::float32_t kEmaAlpha = 0.05f;
        bool is_first_filtering = true;

        StaticQueue<PidTriggerPack, 512> pid_trigger_pack_queue{};
        StaticQueue<std::float32_t, 3> pid_target_pressure_queue{};

        // Task related
        StaticQueueSet<
            decltype(pid_trigger_pack_queue),
            decltype(pid_target_pressure_queue)
        > queue_set {
            this->pid_trigger_pack_queue,
            this->pid_target_pressure_queue
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