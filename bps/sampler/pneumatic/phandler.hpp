#ifndef BPS_PNEUMATIC_HPP
#define BPS_PNEUMATIC_HPP

#include <pico/stdlib.h>

#include <cstdint>
#include <cstdint>

#include "common.hpp"
#include "pcontroller.hpp"

namespace bps::sampler::pneumatic {

// Meyers' Singleton Implementation
class PneumaticHandler {
    public:
        // Meyers' Singleton basic constructor settings
        static PneumaticHandler& getInstance() noexcept {
            static PneumaticHandler handler;
            return handler;
        }
        PneumaticHandler(PneumaticHandler const&) = delete;
        PneumaticHandler& operator=(PneumaticHandler const&) = delete;

        void initialize() noexcept;
        void triger(PulseValue const& pulse_value) noexcept;
        PneumaticHandler& setCunPressure(std::float32_t const& pressure) noexcept;
        PneumaticHandler& setGaunPressure(std::float32_t const& pressure) noexcept;
        PneumaticHandler& setChiPressure(std::float32_t const& pressure) noexcept;

    private:
        PneumaticHandler() noexcept;

        static constexpr uint kCunValvePwmGpioPin  = 6;
        static constexpr uint kCunPumpPwmGpioPin   = kCunValvePwmGpioPin + 1;
        static constexpr uint kGuanValvePwmGpioPin = 8;
        static constexpr uint kGuanPumpPwmGpioPin  = kGuanValvePwmGpioPin + 1;
        static constexpr uint kChiValvePwmGpioPin  = 10;
        static constexpr uint kChiPumpPwmGpioPin   = kChiValvePwmGpioPin + 1;

        PressureController cun_controller{kCunValvePwmGpioPin};
        PressureController guan_controller{kGuanValvePwmGpioPin};
        PressureController chi_controller{kChiValvePwmGpioPin};
};

} // namespace bps::sampler::pneumatic

#endif // BPS_PNEUMATIC_HPP