#include "phandler.hpp"

#include <FreeRTOS.h>

#include <pico/stdlib.h>

#include <cstdint>
#include <stdfloat>

#include "pcontroller.hpp"

namespace bps::sampler::pneumatic {

PneumaticHandler::PneumaticHandler() noexcept {}

void PneumaticHandler::initialize() noexcept {
    this->cun_controller.initialize();
    this->guan_controller.initialize();
    this->chi_controller.initialize();
}

void PneumaticHandler::createTask(UBaseType_t const& priority) noexcept {
    this->cun_controller.createTask(priority);
    this->guan_controller.createTask(priority);
    this->chi_controller.createTask(priority);
}

void PneumaticHandler::trigger(PulseValue const& pulse_value) noexcept {
    static std::expected<QueueHandle_t, std::nullptr_t> selected_queue{};
    for (std::uint8_t i = 0; i < 3; ++i) {
        if ((selected_queue = this->queue_set.selectFromSet(0))) {
            if (selected_queue == this->cun_is_stable_queue.getFreeRTOSQueueHandle()) {
                this->cun_is_stable_queue.receive(this->cun_is_stable, 0);
            } else if (selected_queue == this->guan_is_stable_queue.getFreeRTOSQueueHandle()) {
                this->guan_is_stable_queue.receive(this->guan_is_stable, 0);
            } else if (selected_queue == this->chi_is_stable_queue.getFreeRTOSQueueHandle()) {
                this->chi_is_stable_queue.receive(this->chi_is_stable, 0);
            }
        }
    }
    this->cun_controller.getTriggerPackQueueRef().send(
        PressureController::TriggerPack{ pulse_value.cun },
        pdTICKS_TO_MS(0)
    );
    this->guan_controller.getTriggerPackQueueRef().send(
        PressureController::TriggerPack{ pulse_value.guan },
        pdTICKS_TO_MS(0)
    );
    this->chi_controller.getTriggerPackQueueRef().send(
        PressureController::TriggerPack{ pulse_value.chi },
        pdTICKS_TO_MS(0)
    );
}

PneumaticHandler& PneumaticHandler::setCunPressure(std::float32_t const& pressure) noexcept {
    this->cun_is_stable = false;
    this->cun_controller.getTargetPressureQueueRef().send(pressure, pdTICKS_TO_MS(0));
    return *this;
}

PneumaticHandler& PneumaticHandler::setGuanPressure(std::float32_t const& pressure) noexcept {
    this->guan_is_stable = false;
    this->guan_controller.getTargetPressureQueueRef().send(pressure, pdTICKS_TO_MS(0));
    return *this;
}

PneumaticHandler& PneumaticHandler::setChiPressure(std::float32_t const& pressure) noexcept {
    this->chi_is_stable = false;
    this->chi_controller.getTargetPressureQueueRef().send(pressure, pdTICKS_TO_MS(0));
    return *this;
}

bool PneumaticHandler::isStable() const noexcept {
    return this->cun_is_stable && this->guan_is_stable && this->chi_is_stable;
}

bool PneumaticHandler::cunIsStable() const noexcept {
    return this->cun_is_stable;
}

bool PneumaticHandler::guanIsStable() const noexcept {
    return this->guan_is_stable;
}

bool PneumaticHandler::chiIsStable() const noexcept {
    return this->chi_is_stable;
}

} // namespace bps::sampler::pneumatic