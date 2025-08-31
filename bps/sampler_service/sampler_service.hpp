#ifndef BPS_SAMPLER_HPP
#define BPS_SAMPLER_HPP

// FreeRTOS
#include <FreeRTOS.h>
#include <task.h>
// Pico SDK
#include <hardware/i2c.h>

#include <cstdint>

#include "common.hpp"
#include "queue.hpp"
#include "pneumatic/phandler.hpp"

namespace bps::sampler {

// Meyers' Singleton Implementation
class SamplerService {
    public:
        // Predefined type for convenience usages
        static SamplerService& getInstance() noexcept {
            static SamplerService service;
            return service;
        }
        
        SamplerService(SamplerService const&) = delete;
        SamplerService& operator=(SamplerService const&) = delete;
        
        void initialize() noexcept;
        bool createTask(UBaseType_t const& priority) noexcept;

        // Get the input queue (like setters reference)
        QueueReference<Command> getCommandQueueRef() const noexcept;

        // Register command and pressure base value queue
        void registerPulseValueQueue(QueueReference<PulseValue> const& queue) noexcept;

    private:
        SamplerService();

        StaticQueue<Command, 3> command_queue{};
        QueueReference<PulseValue> output_pulse_value_queue_ref{};

        StaticQueueSet<
            decltype(command_queue)
        > queue_set = makeQueueSet(
            this->command_queue
        );

        pneumatic::PneumaticHandler& pneumatic_handler;

        // FreeRTOS task
        TaskHandle_t task_handle{nullptr};
        void taskLoop() noexcept;

        void updateCurrentStatus() noexcept;
        void processCurrentStatus() noexcept;

        // Status storage
        static constexpr std::size_t kNeedSamples = 2000;
        Command current_command{};
        CommandType prev_command_type;
        std::size_t remain_samples = 0;
};

} // namespace bps::sampler

#endif // BPS_SAMPLER_HPP