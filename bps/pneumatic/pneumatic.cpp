#include "pneumatic.hpp"

#include <pico/stdlib.h>
#include <hardware/i2c.h>
#include <pico/binary_info.h>
#include <pico/time.h>

#include <FreeRTOS.h>
#include <task.h>

#include <expected>
#include <stdfloat>

namespace bps::pneumatic {

namespace {

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
constexpr std::uint8_t kSensorRegTempLsb = 0x0A;

// !!! IMPORTANT: Set kKValue based on your sensor's specific pressure range !!!
// Example for a 0-100kPa sensor, K is 64.
constexpr float kKValue = 64.0f;
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
inline int wirteToSensorAttempt(std::array<std::uint8_t, N> const& buffer, bool const& nostop, std::size_t const& attempts) noexcept {
    for (std::size_t i = 0; i < attempts; ++i) {
        int result = writeToSensor(buffer, nostop);
        if (result != PICO_ERROR_GENERIC) {
            return result;
        }
        if (attempts == 1) {
            return PICO_ERROR_GENERIC;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return PICO_ERROR_GENERIC;
}

template <std::size_t N>
inline int readFromSensor(std::array<std::uint8_t, N>& buffer, bool const& nostop) noexcept {
    static_assert(N > 0, "I2C: must write at least one byte.");
    return i2c_read_blocking(kI2cPortInstance, kSensorI2cAddr, buffer.data(), buffer.size(), nostop);
}

template <std::size_t N>
inline int readFromSensorAttempt(std::array<std::uint8_t, N>& buffer, bool const& nostop, std::size_t const& attempts) noexcept {
    static_assert(N > 0, "I2C: must write at least one byte.");
    for (std::size_t i = 0; i < attempts; ++i) {
        int result = readFromSensor(buffer, nostop);
        if (result != PICO_ERROR_GENERIC) {
            return true;
        }
        if (attempts == 1) {
            return PICO_ERROR_GENERIC;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
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

inline bool checkSensorConversionStatusAttempts(std::size_t const& attempts) noexcept {
    for (std::size_t i = 0; i < attempts; ++i) {
        if (checkSensorConversionStatus()) {
            return true;
        }
        if (attempts == 1) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return false;
}

std::expected<PulseValueSet, Error<int>> readPressureSensorPipelined() noexcept {
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
            pressure = static_cast<float>(pressure_adc_raw - two_to_24) / kKValue;
        } else {
            pressure = static_cast<float>(pressure_adc_raw) / kKValue;
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

} // anonymous namespace


PneumaticService::PneumaticService() {

}

void PneumaticService::initialize() noexcept {
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

bool PneumaticService::createTask(UBaseType_t const& priority) noexcept {
    static auto freertos_task = 
        [](void* context) {
            PneumaticService* service = static_cast<PneumaticService*>(context);
            service->taskLoop();
        };
    return xTaskCreate(
        freertos_task,
        "Pneumatic Service",
        2048,
        this,
        priority,
        &this->task_handle
    ) == pdPASS;
}

// Get the input queue (like setters reference)
ActionQueue_t& PneumaticService::getActionQueue() noexcept {
    return this->action_queue;
}

PressureBaseValueQueue_t& PneumaticService::getPressureBaseValueQueue() noexcept {
    return this->pressure_base_value_queue;
}

// Register action and pressure base value queue
void PneumaticService::registerPulseValueSetQueue(PulseValueSetQueue_t& queue) noexcept {
    this->output_pulse_value_set_queue_ptr = &queue;
}

void PneumaticService::taskLoop() noexcept {
    while (true) {
        updateCurrentStatus();
        processCurrentStatus();
    }
    /* Optional: Error handling */
}

void PneumaticService::updateCurrentStatus() noexcept {
    static std::expected<QueueHandle_t, std::nullptr_t> selected_handle{};
    if ((selected_handle = this->queue_set.selectFromSet(pdMS_TO_TICKS(0)))) {
        if (selected_handle == this->action_queue.getFreeRTOSQueueHandle()) {
            static Action new_action{};
            this->action_queue.receive(new_action, pdMS_TO_TICKS(0));
            if (
                new_action.action_type == ActionType::eStartSampling &&
                this->current_action.action_type == ActionType::eStopSampling
            ) {
                this->remain_samples = kNeedSamples;
            } else if (
                new_action.action_type == ActionType::eStopSampling &&
                this->current_action.action_type == ActionType::eStartSampling
            ) {
                this->remain_samples = 0;
            }
            this->current_action = new_action;
        } else if (selected_handle == this->pressure_base_value_queue.getFreeRTOSQueueHandle()) {
            this->pressure_base_value_queue.receive(this->current_pressure_base_value, pdMS_TO_TICKS(0));
        }
    }
}

void PneumaticService::processCurrentStatus() noexcept {
    static std::expected<bps::PulseValueSet, bps::Error<int>> value_set{};
    switch (this->current_action.action_type) {
    case ActionType::eStopSampling:
        /* TODO: Stopping air pumps and open the valves */
        break;
    case ActionType::eStartSampling:
        value_set = readPressureSensorPipelined();
        if (this->output_pulse_value_set_queue_ptr != nullptr && value_set.has_value()) {
            output_pulse_value_set_queue_ptr->send(value_set.value(), pdMS_TO_TICKS(0));
        }
        if ((--this->remain_samples) == 0) {
            this->current_action.action_type = ActionType::eStopSampling;
        }
        break;
    case ActionType::eNull:
        vTaskDelay(pdMS_TO_TICKS(10));
        break;
    default:
        vTaskDelay(pdMS_TO_TICKS(10));
        break;
    }
}

} // namespace bps::pneumatic