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
        void createTask(UBaseType_t const& priority) noexcept;
        void trigger(PulseValue const& pulse_value) noexcept;
        PneumaticHandler& setCunPressure(std::float32_t const& pressure) noexcept;
        PneumaticHandler& setGuanPressure(std::float32_t const& pressure) noexcept;
        PneumaticHandler& setChiPressure(std::float32_t const& pressure) noexcept;
        bool isStable() const noexcept;
        bool cunIsStable() const noexcept;
        bool guanIsStable() const noexcept;
        bool chiIsStable() const noexcept;

    private:
        PneumaticHandler() noexcept;

        static constexpr uint kCunPumpPwmGpioPin   = 6;
        static constexpr uint kCunValvePwmGpioPin  = kCunPumpPwmGpioPin + 1;
        static constexpr uint kGuanPumpPwmGpioPin  = 8;
        static constexpr uint kGuanValvePwmGpioPin = kGuanPumpPwmGpioPin + 1;
        static constexpr uint kChiPumpPwmGpioPin   = 10;
        static constexpr uint kChiValvePwmGpioPin  = kChiPumpPwmGpioPin + 1;

        PressureController cun_controller{kCunPumpPwmGpioPin};
        PressureController guan_controller{kGuanPumpPwmGpioPin};
        PressureController chi_controller{kChiPumpPwmGpioPin};
        
        // Status Related
        StaticQueue<bool, 1> cun_is_stable_queue{};
        StaticQueue<bool, 1> guan_is_stable_queue{};
        StaticQueue<bool, 1> chi_is_stable_queue{};

        StaticQueueSet<
            decltype(cun_is_stable_queue),
            decltype(guan_is_stable_queue),
            decltype(chi_is_stable_queue)
        > queue_set {
            cun_is_stable_queue,
            guan_is_stable_queue,
            chi_is_stable_queue
        };

        bool cun_is_stable  = true;
        bool guan_is_stable = true;
        bool chi_is_stable  = true;
};

} // namespace bps::sampler::pneumatic

#endif // BPS_PNEUMATIC_HPP