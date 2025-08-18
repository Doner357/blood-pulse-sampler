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
ActionQueue_t& PneumaticService::getActionQueue() noexcept {
    return this->action_queue;
}

PressureBaseValueQueue_t& PneumaticService::getPressureBaseValueQueue() noexcept {
    return this->pressure_base_value_queue;
}

// Register action and pressure base value queue
void PneumaticService::registerPulseValueSetQueue(PulseValueSetQueue_t& queue) noexcept {
    this->output_pulse_value_set_queue_ptr = &queue;
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
        if (selected_handle == this->action_queue.getFreeRTOSQueueHandle()) {
            static Action new_action{};
            this->action_queue.receive(new_action, pdMS_TO_TICKS(0));
            if (
                new_action.action_type == ActionType::eStartSampling &&
                this->current_action.action_type == ActionType::eStopSampling
            ) {
                this->remain_samples = kNeedSamples;
            } else if (
                new_action.action_type == ActionType::eStopSampling &&
                this->current_action.action_type == ActionType::eStartSampling
            ) {
                this->remain_samples = 0;
            }
            this->current_action = new_action;
        } else if (selected_handle == this->pressure_base_value_queue.getFreeRTOSQueueHandle()) {
            this->pressure_base_value_queue.receive(this->current_pressure_base_value, pdMS_TO_TICKS(0));
        }
    }
}

void PneumaticService::processCurrentStatus() noexcept {
    static std::expected<bps::PulseValueSet, bps::Error<int>> value_set{};
    switch (this->current_action.action_type) {
    case ActionType::eStopSampling:
        /* TODO: Stopping air pumps and open the valves */
        break;
    case ActionType::eStartSampling:
        value_set = psensors::readPressureSensorPipelinedBlocking();
        if (this->output_pulse_value_set_queue_ptr != nullptr && value_set.has_value()) {
            output_pulse_value_set_queue_ptr->send(value_set.value(), pdMS_TO_TICKS(0));
        }
        if ((--this->remain_samples) == 0) {
            this->current_action.action_type = ActionType::eStopSampling;
        }
        break;
    case ActionType::eNull:
        vTaskDelay(pdMS_TO_TICKS(10));
        break;
    default:
        vTaskDelay(pdMS_TO_TICKS(10));
        break;
    }
}

} // namespace bps::pneumatic