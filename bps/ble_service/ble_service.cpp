#include "ble_service.hpp"

#include <FreeRTOS.h>
#include <task.h>

#include <array>
#include <cstddef>
#include <utility>

#include "utils.hpp"
#include "gatt_server/gatt_server.hpp"

namespace bps::ble {

void BleService::initialize() noexcept {
    // Initialize GATT server
    auto& gatt_server = gatt::GattServer::getInstance();
    gatt_server.initialize();
}

bool BleService::createTask() noexcept {
    static auto freertos_task = 
        [](void* context) {
            BleService* service = static_cast<BleService*>(context);
            service->taskLoop();
        };
    return xTaskCreate(
        freertos_task,
        "BLE Service",
        2048,
        this,
        1,
        &this->task_handle
    );
}

bool BleService::sendMachineStatus(MachineStatus const& machine_status) noexcept {
    return this->machine_status_queue.send(machine_status, 15);
}

bool BleService::sendPulseValueSet(PulseValueSet const& value_set) noexcept {
    return this->pulse_value_set_queue.send(value_set, 15);
}

void BleService::registerActionCallback(actionCallback_t callback, void* context) noexcept {
    this->action_callback = callback;
    this->action_callback_context = context;

    // Initialize GATT action write callback
    gatt::GattServer& gatt_server = gatt::GattServer::getInstance();
    static auto server_action_callback = 
        [](void* pass_context, std::expected<Action, Error<std::byte>> action) {
            BleService* service = reinterpret_cast<BleService*>(pass_context);
            if (action) {
                service->action_queue.sendFromIsr(action.value(), nullptr);
            } else {
                /* TODO: Error Handling */
            }
        };
    gatt_server.registerActionCallback(server_action_callback, this);
}

void BleService::registerPressureBaseValueCallback(pressureBaseValueCallback_t callback, void* context) noexcept {
    this->pressure_base_value_callback = callback;
    this->pressure_base_value_context = context;

    // Initialize GATT action write callback
    gatt::GattServer& gatt_server = gatt::GattServer::getInstance();
    static auto server_base_value_callback = 
        [](void* pass_context, PressureBaseValue const& base_value) {
            BleService* service = reinterpret_cast<BleService*>(pass_context);
            service->pressure_base_value_queue.sendFromIsr(base_value, nullptr);
        };
    gatt_server.registerPressureBaseValueCallback(server_base_value_callback, this);
}

void BleService::taskLoop() noexcept {
    static Action action{};
    static PressureBaseValue base_value{};
    static MachineStatus machine_status{};
    static PulseValueSet pulse_value_set{};
    while (true) {
        // Deal the values
        gatt::GattServer& gatt_server = gatt::GattServer::getInstance();
        
        if (this->machine_status_queue.receive(machine_status, 0)) {
            gatt_server.setMachineStatus(std::move(machine_status));
        }

        if (this->pulse_value_set_queue.receive(pulse_value_set, 0)) {
            gatt_server.setPulseValueSet(std::move(pulse_value_set));
        }

        if (this->action_callback != nullptr && this->action_queue.receive(action, 0)) {
            this->action_callback(this->action_callback_context, std::move(action));
        }

        if (
            this->pressure_base_value_callback != nullptr &&
            this->pressure_base_value_queue.receive(base_value, 0)
        ) {
            this->pressure_base_value_callback(
                this->pressure_base_value_context,
                std::move(base_value)
            );
        }

        /* Do something else */
        vTaskDelay(TickType_t(5));
    }
    
    /* Optional: Error Handling */
}

} // bps::ble namespace