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
    this->cun_controller.getPidTriggerPackQueueRef().send(
        PressureController::PidTriggerPack{ pulse_value.cun, pulse_value.timestamp },
        pdTICKS_TO_MS(0)
    );
    this->guan_controller.getPidTriggerPackQueueRef().send(
        PressureController::PidTriggerPack{ pulse_value.guan, pulse_value.timestamp },
        pdTICKS_TO_MS(0)
    );
    this->chi_controller.getPidTriggerPackQueueRef().send(
        PressureController::PidTriggerPack{ pulse_value.chi, pulse_value.timestamp },
        pdTICKS_TO_MS(0)
    );
}

PneumaticHandler& PneumaticHandler::setCunPressure(std::float32_t const& pressure) noexcept {
    this->cun_controller.getPidTargetPressureQueueRef().send(pressure, pdTICKS_TO_MS(0));
    return *this;
}

PneumaticHandler& PneumaticHandler::setGuanPressure(std::float32_t const& pressure) noexcept {
    this->guan_controller.getPidTargetPressureQueueRef().send(pressure, pdTICKS_TO_MS(0));
    return *this;
}

PneumaticHandler& PneumaticHandler::setChiPressure(std::float32_t const& pressure) noexcept {
    this->chi_controller.getPidTargetPressureQueueRef().send(pressure, pdTICKS_TO_MS(0));
    return *this;
}

} // namespace bps::sampler::pneumatic