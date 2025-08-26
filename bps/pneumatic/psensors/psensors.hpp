#ifndef BPS_PRESSURE_SENSORS_HPP
#define BPS_PRESSURE_SENSORS_HPP

#include <FreeRTOS.h>
#include <task.h>

#include <hardware/i2c.h>

#include <cstdint>
#include <array>
#include <expected>

#include "common.hpp"

namespace bps::pneumatic::psensors {

// --- Sample Rate (can't less than 120Hz == 8 ms/sample) ---
// Note: This is the delay between triggering a pressure conversion and
//       waiting for the data to be ready.
//
// This value is NOT the final sample rate. The actual sample rate depends
// on the total cycle time, which includes this delay, data processing,
// and other overhead.
constexpr UBaseType_t kSampleRateMs = 6;

// --- I2C Multiplexer (TCA9548A) Constants ---
constexpr std::uint8_t kMuxI2cAddr         = 0x70;     // Default TCA9548A I2C address (A0,A1,A2 to GND)

// --- Sensor Specific Constants (XGZP6857D) ---
constexpr std::uint8_t kSensorI2cAddr      = 0x6D;
constexpr std::uint8_t kSensorRegCmd       = 0x30;
constexpr std::uint8_t kSensorCmdStartComb = 0x0A;
constexpr std::uint8_t kSensorRegPressMsb  = 0x06;
constexpr std::uint8_t kSensorRegPressCsb  = 0x07;
constexpr std::uint8_t kSensorRegPressLsb  = 0x08;
constexpr std::uint8_t kSensorRegTempMsb   = 0x09;
constexpr std::uint8_t kSensorRegTempLsb   = 0x0A;

// !!! IMPORTANT: Set kKValue based on your sensor's specific pressure range !!!
// Example for a 0-100kPa sensor, K is 64.
constexpr float kKValue = 64.0f;
constexpr std::float32_t kMaxTolerablePressurePa = 90000.0_pa;
constexpr int kNumSensors = 3; // We are reading three sensors

// --- Mapping Sensors ID to three measured position ---
constexpr int kCunSensorId  = 0;
constexpr int kGuanSensorId = 1;
constexpr int kChiSensorId  = 2;

// I2C Defines
constexpr i2c_inst_t* kI2cPortInstance = i2c0;
constexpr uint kI2cSdaPinNum = 4;        // GPIO4 for I2C0 SDA
constexpr uint kI2cSclPinNum = 5;        // GPIO5 for I2C0 SCL
constexpr uint32_t kI2cBaudrateHz = (400 * 1000); // 400KHz

// Initialize all the required driver and hardware settings
void initializePressureSensors() noexcept;

// Read the current pressure from three sensors, note that this will block the caller task for "kSampleRateMs" ms
std::expected<PulseValueSet, Error<int>> readPressureSensorPipelinedBlocking() noexcept;

// Function to select a channel on the TCA9548A
inline bool selectMuxChannel(std::uint8_t channel) noexcept {
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

// Function to write a byte to a sensor register (targets kSensorI2cAddr)
template <std::size_t N>
inline int writeToSensor(std::array<std::uint8_t, N> const& buffer, bool const& nostop) noexcept {
    return i2c_write_blocking(kI2cPortInstance, kSensorI2cAddr, buffer.data(), buffer.size(), nostop);
}

template <std::size_t N>
inline int wirteToSensorAttemptBlocking(
    std::array<std::uint8_t, N> const& buffer,
    bool const& nostop,
    std::size_t const& attempts,
    UBaseType_t const& wait_ms = 1
) noexcept {
    for (std::size_t i = 0; i < attempts; ++i) {
        int result = writeToSensor(buffer, nostop);
        if (result != PICO_ERROR_GENERIC) {
            return result;
        }
        if (attempts == 1) {
            return PICO_ERROR_GENERIC;
        }
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
    }

    return PICO_ERROR_GENERIC;
}

template <std::size_t N>
inline int readFromSensor(std::array<std::uint8_t, N>& buffer, bool const& nostop) noexcept {
    static_assert(N > 0, "I2C: must write at least one byte.");
    return i2c_read_blocking(kI2cPortInstance, kSensorI2cAddr, buffer.data(), buffer.size(), nostop);
}

template <std::size_t N>
inline int readFromSensorAttemptBlocking(
    std::array<std::uint8_t, N>& buffer,
    bool const& nostop,
    std::size_t const& attempts,
    UBaseType_t const& wait_ms = 1
) noexcept {
    static_assert(N > 0, "I2C: must write at least one byte.");
    for (std::size_t i = 0; i < attempts; ++i) {
        int result = readFromSensor(buffer, nostop);
        if (result != PICO_ERROR_GENERIC) {
            return true;
        }
        if (attempts == 1) {
            return PICO_ERROR_GENERIC;
        }
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
    }

    return PICO_ERROR_GENERIC;
}

inline bool checkSensorConversionStatus() noexcept {
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

inline bool checkSensorConversionStatusAttemptsBlocking(std::size_t const& attempts, UBaseType_t const& wait_ms = 1) noexcept {
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

} // bps::pneumatic::psensors

#endif