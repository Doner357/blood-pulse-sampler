#include "psensors.hpp"

// FreeRTOS
#include <FreeRTOS.h>
#include <task.h>
// Pico SDK
#include <pico/stdlib.h>
#include <hardware/i2c.h>
#include <pico/binary_info.h>
#include <pico/time.h>

#include <cstdint>
#include <expected>
#include <stdfloat>
#include <algorithm>

namespace bps::sampler::pneumatic {

PressureSensors::PressureSensors() noexcept {
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

std::expected<PulseValue, Error<int>> PressureSensors::readPressureSensorPipelinedSleeping() noexcept {
    // Request (Write) the pressure data
    for (std::size_t i = 0; i < kNumSensors; ++i) {
        if (!selectMuxChannel(i)) {
            return std::unexpected(Error<int>{ ErrorType::eFailedOperation, PICO_ERROR_GENERIC });
        }
        
        if (writeToSensor(std::array{ kSensorRegCmd, kSensorCmdStartComb }, false) == PICO_ERROR_GENERIC) {
            return std::unexpected(Error<int>{ ErrorType::eFailedOperation, PICO_ERROR_GENERIC });
        }
    }

    sleep_ms(kSampleRateMs);

    PulseValue value{};
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
            value.cun = std::max(pressure - this->pressure_baseline.cun, 0.0_pa);
            break;
        case kGuanSensorId:
            value.guan = std::max(pressure - this->pressure_baseline.guan, 0.0_pa);
            break;
        case kChiSensorId:
            value.chi = std::max(pressure - this->pressure_baseline.chi, 0.0_pa);
        default:
            break;
        }
    }

    value.timestamp = get_absolute_time();

    return value;
}

std::expected<PulseValue, Error<int>> PressureSensors::readPressureSensorPipelinedBlocking() noexcept {
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

    PulseValue value{};
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
            value.cun = std::max(pressure - this->pressure_baseline.cun, 0.0_pa);
            break;
        case kGuanSensorId:
            value.guan = std::max(pressure - this->pressure_baseline.guan, 0.0_pa);
            break;
        case kChiSensorId:
            value.chi = std::max(pressure - this->pressure_baseline.chi, 0.0_pa);
        default:
            break;
        }
    }

    value.timestamp = get_absolute_time();

    return value;
}

// Set baseline value to specified value
void PressureSensors::setBaseLine(std::float32_t const& cun_baseline, std::float32_t const& guan_baseline, std::float32_t const& chi_baseline) noexcept {
    this->pressure_baseline.cun  = cun_baseline;
    this->pressure_baseline.guan = guan_baseline;
    this->pressure_baseline.chi  = chi_baseline;
}

// Function to select a channel on the TCA9548A
bool PressureSensors::selectMuxChannel(std::uint8_t channel) noexcept {
    if (channel > 7) {
        return false;
    }
    std::uint8_t control_byte = 1 << channel; // Create a byte with only the bit for the desired channel set
    int result = i2c_write_blocking(kI2cPortInstance, kMuxI2cAddr, &control_byte, 1, false);
    if (result < 0) { // PICO_ERROR_GENERIC or PICO_ERROR_TIMEOUT
        return false;
    }
    return true;
}

bool PressureSensors::checkSensorConversionStatus() noexcept {
    uint8_t cmd_status_val = 0;
    std::array<uint8_t, 1> cmd_status_buf;
    if (writeToSensor(std::array{ kSensorRegCmd }, true) == PICO_ERROR_GENERIC) {
        return false;
    }
    if (readFromSensor(cmd_status_buf, false) != 1) {
        return false;
    }
    cmd_status_val = cmd_status_buf[0];
    if ((cmd_status_val & 0x08) != 0) {
        return false;
    }
    return true;
}

bool PressureSensors::checkSensorConversionStatusAttemptsBlocking(std::size_t const& attempts, UBaseType_t const& wait_ms) noexcept {
    for (std::size_t i = 0; i < attempts; ++i) {
        if (checkSensorConversionStatus()) {
            return true;
        }
        if (attempts == 1) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
    }

    return false;
}

} // bps::sampler::pneumatic