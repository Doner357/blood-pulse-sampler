#include "sampler.hpp"

#include "pneumatic/psensors.hpp"
#include "pneumatic/phandler.hpp"

namespace bps::sampler {

SamplerService::SamplerService():
pneumatic_handler(pneumatic::PneumaticHandler::getInstance()) {}

void SamplerService::initialize() noexcept {
    auto& sensors = pneumatic::PressureSensors::getInstance();
    std::float32_t cun_baseline  = 0.0_pa;
    std::float32_t guan_baseline = 0.0_pa;
    std::float32_t chi_baseline  = 0.0_pa;
    std::uint8_t miss_sample = 0;
    for (std::uint8_t i = 0; i < 100; ++i) {
        auto baseline = sensors.readPressureSensorPipelinedSleeping();
        if (baseline) {
            cun_baseline  += baseline.value().cun;
            guan_baseline += baseline.value().guan;
            chi_baseline  += baseline.value().chi;
        } else {
            ++miss_sample;
        }
    }
    cun_baseline  /= (100 - miss_sample);
    guan_baseline /= (100 - miss_sample);
    chi_baseline  /= (100 - miss_sample);
    sensors.setBaseLine(cun_baseline, guan_baseline, chi_baseline);

    this->pneumatic_handler.initialize();
}

bool SamplerService::createTask(UBaseType_t const& priority) noexcept {
    this->pneumatic_handler.createTask(priority);

    static auto freertos_task = 
        [](void* context) {
            SamplerService* service = static_cast<SamplerService*>(context);
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
QueueReference<Command> SamplerService::getCommandQueueRef() const noexcept {
    return this->command_queue;
}

// Register command and pressure base value queue
void SamplerService::registerPulseValueQueue(QueueReference<PulseValue> const& queue) noexcept {
    this->output_pulse_value_queue_ref = queue;
}

void SamplerService::taskLoop() noexcept {
    while (true) {
        updateCurrentStatus();
        processCurrentStatus();
    }
    /* Optional: Error handling */
}

void SamplerService::updateCurrentStatus() noexcept {
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

void SamplerService::processCurrentStatus() noexcept {
    static std::expected<bps::PulseValue, bps::Error<int>> value{};
    value = pneumatic::PressureSensors::getInstance().readPressureSensorPipelinedBlocking();
    if (value) {
        this->pneumatic_handler.trigger(value.value());
    }
    switch (this->current_command.command_type) {
    case CommandType::eStopSampling:
        /* TODO: Stopping air pumps and open the valves */
        break;
    case CommandType::eStartSampling:
        if (this->output_pulse_value_queue_ref.isValid() && value.has_value()) {
            output_pulse_value_queue_ref.send(value.value(), pdMS_TO_TICKS(0));
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

} // namespace bps::sampler