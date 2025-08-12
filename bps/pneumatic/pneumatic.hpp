#ifndef BPS_PNEUMATIC_HPP
#define BPS_PNEUMATIC_HPP

// FreeRTOS
#include <FreeRTOS.h>
#include <task.h>
// Pico SDK
#include <hardware/i2c.h>

#include <cstdint>

#include "common.hpp"
#include "queue.hpp"

namespace bps::pneumatic {

class PneumaticService {
    public:
        static PneumaticService& getInstance() noexcept {
            static PneumaticService service;
            return service;
        }

        PneumaticService(PneumaticService const&) = delete;
        PneumaticService& operator=(PneumaticService const&) = delete;
        
        void initialize() noexcept;
        bool createTask(UBaseType_t const& priority) noexcept;

        // Get the input queue (like setters reference)
        ActionQueue_t& getActionQueue() noexcept;
        PressureBaseValueQueue_t& getPressureBaseValueQueue() noexcept;

        // Register action and pressure base value queue
        void registerPulseValueSetQueue(PulseValueSetQueue_t& queue) noexcept;

    private:
        PneumaticService();

        ActionQueue_t action_queue{};
        PressureBaseValueQueue_t pressure_base_value_queue{};
        PulseValueSetQueue_t* output_pulse_value_set_queue_ptr{nullptr};

        // FreeRTOS task
        TaskHandle_t task_handle{nullptr};
        void taskLoop() noexcept;
};

} // namespace bps::pneumatic

#endif