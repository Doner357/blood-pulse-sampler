#include "phandler.hpp"

#include <pico/stdlib.h>

#include <cstdint>
#include <stdfloat>

#include "psensor.hpp"

namespace bps::sampler::pneumatic {

PneumaticHandler::PneumaticHandler() noexcept {}

void PneumaticHandler::initialize() noexcept {
    constexpr std::uint8_t kSamples = 100;

    this->cun_controller.setValvePwmPercentage(1.0f).setPumpPwmPercentage(0.0f);
    this->guan_controller.setValvePwmPercentage(1.0f).setPumpPwmPercentage(0.0f);
    this->chi_controller.set
    
    std::float32_t cun_error  = 0.0_pa;
    std::float32_t guan_error = 0.0_pa;
    std::float32_t chi_error  = 0.0_pa;
    std::uint8_t num_miss_samples = 0;

    for (std::uint8_t i = 0; i < kSamples; ++i) {
        auto pulse_value = readPressureSensorPipelinedSleeping();
        if (pulse_value) {
            cun_error
        } else {
            ++num_miss_samples;
        }
    }
}

void PneumaticHandler::triger(PulseValue const& pulse_value) noexcept {
    this->cun_controller.executePid(pulse_value.cun, pulse_value.timestemp);
    this->guan_controller.executePid(pulse_value.guan, pulse_value.chi);
    this->chi_controller.executePid(pulse_value.chi, pulse_value.timestemp);
}

PneumaticHandler& PneumaticHandler::setCunPressure(std::float32_t const& pressure) noexcept {
    this->cun_controller.setTargetPressure(pressure);
}

PneumaticHandler& PneumaticHandler::setGaunPressure(std::float32_t const& pressure) noexcept {
    this->guan_controller.setTargetPressure(pressure);
}

PneumaticHandler& PneumaticHandler::setChiPressure(std::float32_t const& pressure) noexcept {
    this->chi_controller.setPressureError(pressure);
}

} // namespace bps::sampler::pneumatic