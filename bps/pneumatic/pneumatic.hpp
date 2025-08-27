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
        CommandQueue_t& getCommandQueue() noexcept;

        // Register command and pressure base value queue
        void registerPulseValueQueue(PulseValueQueue_t& queue) noexcept;

    private:
        PneumaticService();

        CommandQueue_t command_queue{};
        PulseValueQueue_t* output_pulse_value_queue_ptr{nullptr};

        StaticQueueSet<
            CommandQueue_t
        > queue_set = makeQueueSet(
            this->command_queue
        );

        // FreeRTOS task
        TaskHandle_t task_handle{nullptr};
        void taskLoop() noexcept;

        void updateCurrentStatus() noexcept;
        void processCurrentStatus() noexcept;

        // Status storage
        static constexpr std::size_t kNeedSamples = 2000;
        Command current_command{};
        std::size_t remain_samples = 0;
};

} // namespace bps::pneumatic

#endif // BPS_PNEUMATIC_HPP