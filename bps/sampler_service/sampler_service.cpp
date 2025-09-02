#include "sampler_service.hpp"

#include "pneumatic/psensors.hpp"
#include "pneumatic/phandler.hpp"
#include "logger.hpp"

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
        4096,
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

void SamplerService::registerMachineStatusQueue(QueueReference<MachineStatus> const& queue) noexcept {
    this->output_machine_status_queue_ref = queue;
}

void SamplerService::taskLoop() noexcept {
    while (true) {
        updateCurrentStatus();
        processCurrentStatus();
    }
    /* Optional: Error handling */
}

void SamplerService::updateCurrentStatus() noexcept {
    if (this->command_queue.receive(this->received_command, 0)) {
        switch (this->received_command.command_type) {
        case CommandType::eStopSampling:
            if (this->current_status == MachineStatus::eSampling) {
                this->current_status = MachineStatus::eIdle;
                BPS_LOG("Set BPS status to: Idle\n");
            }
            break;
        case CommandType::eStartSampling:
            if (this->current_status == MachineStatus::eIdle) {
                this->current_status = MachineStatus::eSampling;
                BPS_LOG("Set BPS status to: Sampling\n");
            }
            break;
        case CommandType::eSetPressure:
            if (this->current_status != MachineStatus::eSampling) {
                this->current_status = MachineStatus::eSettingPressure;
                this->need_to_set_pressure = true;
                BPS_LOG("Set BPS status to: SettingPressure\n");
            }
            break;
        case CommandType::eReset:
            this->received_command = Command{
                .command_type = CommandType::eSetPressure,
                .content = {
                    .pressure_settings = {
                        .cun = 0.0_pa,
                        .guan = 0.0_pa,
                        .chi = 0.0_pa
                    }
                }
            };
            this->current_status = MachineStatus::eSettingPressure;
            this->need_to_set_pressure = true;
            BPS_LOG("Set BPS status to: SettingPressure (for Reset)\n");
            break;
        default:
            break;
        }
    }
    this->output_machine_status_queue_ref.send(this->current_status, pdTICKS_TO_MS(1));
}

void SamplerService::processCurrentStatus() noexcept {
    std::expected<bps::PulseValue, bps::Error<int>> value{};
    switch (this->current_status) {
        case MachineStatus::eIdle:
            vTaskDelay(10);
            break;
        case MachineStatus::eSampling:
            value = pneumatic::PressureSensors::getInstance().readPressureSensorPipelinedBlocking();
            if (value) {
                this->output_pulse_value_queue_ref.send(value.value(), 0);
            }
            break;
        case MachineStatus::eSettingPressure:
            if (this->need_to_set_pressure) {
                this->pneumatic_handler.setCunPressure(this->received_command.content.pressure_settings.cun)
                                       .setGuanPressure(this->received_command.content.pressure_settings.guan)
                                       .setChiPressure(this->received_command.content.pressure_settings.chi);
                this->need_to_set_pressure = false;
                BPS_LOG("Set BPS status to received target\n");
            } else if (this->pneumatic_handler.isStable()) {
                this->current_status = MachineStatus::eIdle;
                BPS_LOG("Set machine status to: Idle\n");
            } else {
                value = pneumatic::PressureSensors::getInstance().readPressureSensorPipelinedBlocking();
                if (value) {
                    this->pneumatic_handler.trigger(value.value());
                }
            }
            break;
        default:
            vTaskDelay(10);
            break;
    }
}

} // namespace bps::sampler