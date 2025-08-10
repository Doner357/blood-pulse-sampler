#include <cstdint>
#include <array>
#include <expected>

// Pico SDK headers
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "ble_service.hpp"

// --- I2C Multiplexer (TCA9548A) Constants ---
constexpr uint8_t MUX_I2C_ADDR        = 0x70;     // Default TCA9548A I2C address (A0,A1,A2 to GND)

// --- Sensor Specific Constants (XGZP6857D) ---
constexpr uint8_t SENSOR_I2C_ADDR       = 0x6D;
constexpr uint8_t SENSOR_REG_CMD        = 0x30;
constexpr uint8_t SENSOR_CMD_START_COMB = 0x0A;
constexpr uint8_t SENSOR_REG_PRESS_MSB  = 0x06;
constexpr uint8_t SENSOR_REG_PRESS_CSB  = 0x07;
constexpr uint8_t SENSOR_REG_PRESS_LSB  = 0x08;
constexpr uint8_t SENSOR_REG_TEMP_MSB   = 0x09;
constexpr uint8_t SENSOR_REG_TEMP_LSB   = 0x0A;

// !!! IMPORTANT: Set K_VALUE based on your sensor's specific pressure range !!!
// Example for a 0-100kPa sensor, K is 64.
constexpr float K_VALUE = 64.0f;
constexpr int NUM_SENSORS = 3; // We are reading three sensors

// I2C Defines
constexpr i2c_inst_t* I2C_PORT_INSTANCE = i2c0;
constexpr uint I2C_SDA_PIN_NUM = 4;        // GPIO4 for I2C0 SDA
constexpr uint I2C_SCL_PIN_NUM = 5;        // GPIO5 for I2C0 SCL
constexpr uint32_t I2C_BAUDRATE_HZ = (400 * 1000); // 400KHz

// Function to select a channel on the TCA9548A
bool select_mux_channel(uint8_t channel) {
    if (channel > 7) {
        return false;
    }
    uint8_t control_byte = 1 << channel; // Create a byte with only the bit for the desired channel set
    int result = i2c_write_blocking(I2C_PORT_INSTANCE, MUX_I2C_ADDR, &control_byte, 1, false);
    if (result < 0) { // PICO_ERROR_GENERIC or PICO_ERROR_TIMEOUT
        return false;
    }
    // Datasheet mentions "When a channel is selected, the channel becomes active after a stop condition
    // has been placed on the I2C bus." [cite: 149] The i2c_write_blocking with false for nostop
    // issues a stop condition.
    // A small delay might sometimes be beneficial after switching channels, though not strictly specified as required for all cases.
    // sleep_us(100); // Optional short delay
    return true;
}

// Function to write a byte to a sensor register (targets SENSOR_I2C_ADDR)
int write_to_sensor(uint8_t reg, uint8_t data) {
    std::array<uint8_t, 2> buffer = {reg, data};
    int result = i2c_write_blocking(I2C_PORT_INSTANCE, SENSOR_I2C_ADDR, buffer.data(), buffer.size(), false);
    if (result < 0) {
    }
    return result;
}

// Function template to read bytes from sensor registers into a std::array (targets SENSOR_I2C_ADDR)
template <size_t N>
int read_from_sensor(uint8_t reg_addr, std::array<uint8_t, N>& buffer) {
    int write_result = i2c_write_blocking(I2C_PORT_INSTANCE, SENSOR_I2C_ADDR, &reg_addr, 1, true);
    if (write_result < 0) {
        return write_result;
    }
    int read_result = i2c_read_blocking(I2C_PORT_INSTANCE, SENSOR_I2C_ADDR, buffer.data(), buffer.size(), false);
    if (read_result < 0) {
        return read_result;
    }
    if (static_cast<size_t>(read_result) != N) {
        return PICO_ERROR_GENERIC;
    }
    return read_result;
}

struct SensorData {
    float pressure_pa;
    float temperature_c;
    bool valid;
};

SensorData read_pressure_sensor() {
    SensorData data = {0.0f, 0.0f, false};

    if (write_to_sensor(SENSOR_REG_CMD, SENSOR_CMD_START_COMB) < 0) {
        return data;
    }

    uint8_t cmd_status_val = 0;
    std::array<uint8_t, 1> cmd_status_buf;
    int attempts = 0;
    bool conversion_done = false;
    while(attempts < 10) {
        if (read_from_sensor(SENSOR_REG_CMD, cmd_status_buf) != 1) {
            sleep_ms(2); attempts++; continue;
        }
        cmd_status_val = cmd_status_buf[0];
        if ((cmd_status_val & 0x08) == 0) {
            conversion_done = true;
            break;
        }
        sleep_ms(2); // Datasheet typical 5ms, wait a bit between polls
        attempts++;
    }

    if (!conversion_done) {
        // Attempting a read after a fixed longer delay as per datasheet alternative
        sleep_ms(25); // Datasheet suggests 20ms, adding a bit more
    }

    std::array<uint8_t, 3> p_data_arr;
    std::array<uint8_t, 2> t_data_arr;
    int32_t pressure_adc_raw;
    int16_t temperature_adc_raw;

    if (read_from_sensor(SENSOR_REG_PRESS_MSB, p_data_arr) != 3) {
        return data;
    }
    pressure_adc_raw = static_cast<int32_t>(
        (static_cast<uint32_t>(p_data_arr[0]) << 16) |
        (static_cast<uint32_t>(p_data_arr[1]) << 8)  |
        (static_cast<uint32_t>(p_data_arr[2]))
    );

    if (read_from_sensor(SENSOR_REG_TEMP_MSB, t_data_arr) != 2) {
        return data;
    }
    temperature_adc_raw = static_cast<int16_t>(
        (static_cast<uint16_t>(t_data_arr[0]) << 8) |
        (static_cast<uint16_t>(t_data_arr[1]))
    );

    if (pressure_adc_raw & 0x800000) {
        constexpr int32_t two_to_24 = 16777216L;
        data.pressure_pa = static_cast<float>(pressure_adc_raw - two_to_24) / K_VALUE;
    } else {
        data.pressure_pa = static_cast<float>(pressure_adc_raw) / K_VALUE;
    }

    if (temperature_adc_raw & 0x8000) {
        constexpr int32_t two_to_16 = 65536;
        data.temperature_c = static_cast<float>(temperature_adc_raw - two_to_16) / 256.0f;
    } else {
        data.temperature_c = static_cast<float>(temperature_adc_raw) / 256.0f;
    }
    data.valid = true;
    return data;
}

bps::PulseValueSet value_set{};

int main() {
    stdio_init_all();

    auto& gatt_server = bps::ble::gatt::GattServer::getInstance();
    gatt_server.initialize();
    auto result = gatt_server.on();
    if (!result) {
        return EXIT_FAILURE;
    }

    i2c_init(I2C_PORT_INSTANCE, I2C_BAUDRATE_HZ);
    gpio_set_function(I2C_SDA_PIN_NUM, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN_NUM, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN_NUM);
    gpio_pull_up(I2C_SCL_PIN_NUM);

    bi_decl(bi_2pins_with_func(I2C_SDA_PIN_NUM, I2C_SCL_PIN_NUM, GPIO_FUNC_I2C));
    bi_decl(bi_program_description("Reads 3 XGZP6857D pressure sensors via TCA9548A MUX."));

    // Disable all channels on the MUX initially (good practice)
    uint8_t disable_all_cmd = 0x00;
    i2c_write_blocking(I2C_PORT_INSTANCE, MUX_I2C_ADDR, &disable_all_cmd, 1, false);


    while (true) {
        for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
            if (!select_mux_channel(i)) {
                sleep_ms(500); // Wait a bit before trying next or looping
                continue;
            }
            
            // After selecting channel, there might be a very brief moment for lines to settle
            // or for the selected device to be fully "online" on the bus.
            // Usually not strictly necessary if I2C operations have proper start/stop.
            // sleep_us(200); 

            SensorData sensor_reading = read_pressure_sensor();
            switch (i) {
            case 0:
                value_set.cun = sensor_reading.pressure_pa;
                break;
            case 1:
                value_set.guan = sensor_reading.pressure_pa;
                break;
            case 2:
                value_set.chi = sensor_reading.pressure_pa;
                break;
            default:
                break;
            }
            sleep_ms(5); // Small delay between reading different sensors
        }
        gatt_server.sendPulseValueSet(value_set);
        sleep_ms(5); // Wait 2 seconds before reading all sensors again
    }

    return 0;
}