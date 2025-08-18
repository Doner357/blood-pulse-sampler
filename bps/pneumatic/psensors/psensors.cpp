#include "psensors.hpp"

#include <pico/stdlib.h>
#include <hardware/i2c.h>
#include <pico/binary_info.h>
#include <pico/time.h>

#include <FreeRTOS.h>
#include <task.h>

#include <cstdint>
#include <expected>
#include <stdfloat>

namespace bps::pneumatic::psensors {

void initializePressureSensors() noexcept {
    i2c_init(kI2cPortInstance, kI2cBaudrateHz);
    gpio_set_function(kI2cSdaPinNum, GPIO_FUNC_I2C);
    gpio_set_function(kI2cSclPinNum, GPIO_FUNC_I2C);
    gpio_pull_up(kI2cSdaPinNum);
    gpio_pull_up(kI2cSclPinNum);

    bi_decl(bi_2pins_with_func(kI2cSdaPinNum, kI2cSclPinNum, GPIO_FUNC_I2C));
    bi_decl(bi_program_description("Reads 3 XGZP6857D pressure sensors via TCA9548A MUX."));

    // Disable all channels on the MUX initially (good practice)
    uint8_t disable_all_cmd = 0x00;
    i2c_write_blocking(kI2cPortInstance, kMuxI2cAddr, &disable_all_cmd, 1, false);
}

std::expected<PulseValueSet, Error<int>> readPressureSensorPipelinedBlocking() noexcept {
    // Request (Write) the pressure data
    for (std::size_t i = 0; i < kNumSensors; ++i) {
        if (!selectMuxChannel(i)) {
            return std::unexpected(Error<int>{ ErrorType::eFailedOperation, PICO_ERROR_GENERIC });
        }
        
        if (writeToSensor(std::array{ kSensorRegCmd, kSensorCmdStartComb }, false) == PICO_ERROR_GENERIC) {
            return std::unexpected(Error<int>{ ErrorType::eFailedOperation, PICO_ERROR_GENERIC });
        }
    }

    vTaskDelay(pdMS_TO_TICKS(kSampleRateMs));

    PulseValueSet value_set{};
    // Fetch (Read) the pressure data
    for (std::size_t i = 0; i < kNumSensors; ++i) {
        if (!selectMuxChannel(i)) {
            return std::unexpected(Error<int>{ ErrorType::eFailedOperation, PICO_ERROR_GENERIC });
        }

        std::array<uint8_t, 3> p_data_arr{};
        if (writeToSensor(std::array{ kSensorRegPressMsb }, true) < 0) {
            return std::unexpected(Error<int>{ ErrorType::eFailedOperation, PICO_ERROR_GENERIC });
        }
        if (readFromSensor(p_data_arr, false) != 3) {
            return std::unexpected(Error<int>{ ErrorType::eFailedOperation, PICO_ERROR_GENERIC });
        }
        int32_t pressure_adc_raw = static_cast<int32_t>(
            (static_cast<uint32_t>(p_data_arr[0]) << 16) |
            (static_cast<uint32_t>(p_data_arr[1]) << 8)  |
            (static_cast<uint32_t>(p_data_arr[2]))
        );

        std::float32_t pressure = 0.0_pa;
        if (pressure_adc_raw & 0x800000) {
            constexpr int32_t two_to_24 = 16777216L;
            pressure = static_cast<std::float32_t>(pressure_adc_raw - two_to_24) / kKValue;
        } else {
            pressure = static_cast<std::float32_t>(pressure_adc_raw) / kKValue;
        }

        switch (i) {
        case kCunSensorId:
            value_set.cun = pressure;
            break;
        case kGuanSensorId:
            value_set.guan = pressure;
            break;
        case kChiSensorId:
            value_set.chi = pressure;
        default:
            break;
        }
    }

    value_set.timestemp = get_absolute_time();

    return value_set;
}

} // bps::pneumatic::psensors