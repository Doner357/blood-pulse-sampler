#include "pneumatic.hpp"

#include "psensors/psensors.hpp"

namespace bps::pneumatic {

PneumaticService::PneumaticService() {}

void PneumaticService::initialize() noexcept {
    psensors::initializePressureSensors();
}

bool PneumaticService::createTask(UBaseType_t const& priority) noexcept {
    static auto freertos_task = 
        [](void* context) {
            PneumaticService* service = static_cast<PneumaticService*>(context);
            service->taskLoop();
        };
    return xTaskCreate(
        freertos_task,
        "Pneumatic Service",
        2048,
        this,
        priority,
        &this->task_handle
    ) == pdPASS;
}

// Get the input queue (like setters reference)
CommandQueue_t& PneumaticService::getCommandQueue() noexcept {
    return this->command_queue;
}

// Register command and pressure base value queue
void PneumaticService::registerPulseValueQueue(PulseValueQueue_t& queue) noexcept {
    this->output_pulse_value_queue_ptr = &queue;
}

void PneumaticService::taskLoop() noexcept {
    while (true) {
        updateCurrentStatus();
        processCurrentStatus();
    }
    /* Optional: Error handling */
}

void PneumaticService::updateCurrentStatus() noexcept {
    static std::expected<QueueHandle_t, std::nullptr_t> selected_handle{};
    if ((selected_handle = this->queue_set.selectFromSet(pdMS_TO_TICKS(0)))) {
        if (selected_handle == this->command_queue.getFreeRTOSQueueHandle()) {
            static Command new_command{};
            this->command_queue.receive(new_command, pdMS_TO_TICKS(0));
            if (
                new_command.command_type == CommandType::eStartSampling &&
                this->current_command.command_type != CommandType::eStartSampling
            ) {
                this->remain_samples = kNeedSamples;
            } else if (
                new_command.command_type == CommandType::eStopSampling &&
                this->current_command.command_type == CommandType::eStartSampling
            ) {
                this->remain_samples = 0;
            }
            this->current_command = new_command;
        }
    }
}

void PneumaticService::processCurrentStatus() noexcept {
    static std::expected<bps::PulseValue, bps::Error<int>> value{};
    switch (this->current_command.command_type) {
    case CommandType::eStopSampling:
        /* TODO: Stopping air pumps and open the valves */
        break;
    case CommandType::eStartSampling:
        value = psensors::readPressureSensorPipelinedBlocking();
        if (this->output_pulse_value_queue_ptr != nullptr && value.has_value()) {
            output_pulse_value_queue_ptr->send(value.value(), pdMS_TO_TICKS(0));
        }
        if ((--this->remain_samples) == 0) {
            this->current_command.command_type = CommandType::eStopSampling;
        }
        break;
    case CommandType::eSetPressure:
        break;
    case CommandType::eNull:
        vTaskDelay(pdMS_TO_TICKS(10));
        break;
    default:
        vTaskDelay(pdMS_TO_TICKS(10));
        break;
    }
}

} // namespace bps::pneumatic