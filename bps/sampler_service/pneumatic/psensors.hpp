#ifndef BPS_PRESSURE_SENSORS_HPP
#define BPS_PRESSURE_SENSORS_HPP

#include <FreeRTOS.h>
#include <task.h>

#include <hardware/i2c.h>

#include <cstdint>
#include <stdfloat>
#include <array>
#include <expected>

#include "common.hpp"

namespace bps::sampler::pneumatic {

// Meyers' Singleton Implementation
class PressureSensors {
    public:
        // --- Sample Rate (can't less than 120Hz == 8 ms/sample) ---
        // Note: This is the delay between triggering a pressure conversion and
        //       waiting for the data to be ready.
        //
        // This value is NOT the final sample rate. The actual sample rate depends
        // on the total cycle time, which includes this delay, data processing,
        // and other overhead.
        static constexpr UBaseType_t kSampleRateMs = 6;

        // Meyers' Singleton basic constructor settings
        static PressureSensors& getInstance() noexcept {
            static PressureSensors sensors;
            return sensors;
        }
        PressureSensors(PressureSensors const&) = delete;
        PressureSensors& operator=(PressureSensors const&) = delete;

        // Read the current pressure from three sensors, note that this will sleep the caller task for "kSampleRateMs" ms
        // This can be called without using FreeRTOS
        std::expected<PulseValue, Error<int>> readPressureSensorPipelinedSleeping() noexcept;
        // Read the current pressure from three sensors, note that this will block the caller task for "kSampleRateMs" ms
        std::expected<PulseValue, Error<int>> readPressureSensorPipelinedBlocking() noexcept;
        // Set baseline value to specified value.
        void setBaseLine(std::float32_t const& cun_baseline, std::float32_t const& guan_baseline, std::float32_t const& chi_baseline) noexcept;

    private:
        // --- I2C Multiplexer (TCA9548A) Constants ---
        static constexpr std::uint8_t kMuxI2cAddr         = 0x70;     // Default TCA9548A I2C address (A0,A1,A2 to GND)

        // --- Sensor Specific Constants (XGZP6857D) ---
        static constexpr std::uint8_t kSensorI2cAddr      = 0x6D;
        static constexpr std::uint8_t kSensorRegCmd       = 0x30;
        static constexpr std::uint8_t kSensorCmdStartComb = 0x0A;
        static constexpr std::uint8_t kSensorRegPressMsb  = 0x06;
        static constexpr std::uint8_t kSensorRegPressCsb  = 0x07;
        static constexpr std::uint8_t kSensorRegPressLsb  = 0x08;
        static constexpr std::uint8_t kSensorRegTempMsb   = 0x09;
        static constexpr std::uint8_t kSensorRegTempLsb   = 0x0A;

        // !!! IMPORTANT: Set kKValue based on your sensor's specific pressure range !!!
        // Example for a 0-100kPa sensor, K is 64.
        static constexpr float kKValue = 64.0f;
        static constexpr std::float32_t kMaxTolerablePressurePa = 90000.0_pa;
        static constexpr int kNumSensors = 3; // We are reading three sensors

        // --- Mapping Sensors ID to three measured position ---
        static constexpr int kCunSensorId  = 0;
        static constexpr int kGuanSensorId = 1;
        static constexpr int kChiSensorId  = 2;

        // I2C Defines
        static constexpr i2c_inst_t* kI2cPortInstance = i2c0;
        static constexpr uint kI2cSdaPinNum = 4;        // GPIO4 for I2C0 SDA
        static constexpr uint kI2cSclPinNum = 5;        // GPIO5 for I2C0 SCL
        static constexpr uint32_t kI2cBaudrateHz = (400 * 1000); // 400KHz

        PressureSensors() noexcept;

        // Baseline value, the read value will be subtracted by this value
        struct {
            std::float32_t cun  = 0.0_pa;
            std::float32_t guan = 0.0_pa;
            std::float32_t chi  = 0.0_pa;
        } pressure_baseline{};

        // Function to select a channel on the TCA9548A
        bool selectMuxChannel(std::uint8_t channel) noexcept;
        bool checkSensorConversionStatus() noexcept;
        bool checkSensorConversionStatusAttemptsBlocking(std::size_t const& attempts, UBaseType_t const& wait_ms = 1) noexcept;

        // Function to write a byte to a sensor register (targets kSensorI2cAddr)
        template <std::size_t N>
        int writeToSensor(std::array<std::uint8_t, N> const& buffer, bool const& nostop) noexcept {
            return i2c_write_blocking(kI2cPortInstance, kSensorI2cAddr, buffer.data(), buffer.size(), nostop);
        }

        template <std::size_t N>
        int wirteToSensorAttemptBlocking(
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
        int readFromSensor(std::array<std::uint8_t, N>& buffer, bool const& nostop) noexcept {
            static_assert(N > 0, "I2C: must write at least one byte.");
            return i2c_read_blocking(kI2cPortInstance, kSensorI2cAddr, buffer.data(), buffer.size(), nostop);
        }

        template <std::size_t N>
        int readFromSensorAttemptBlocking(
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
};

} // namespace bps::sampler::pneumatic

#endif