#include "ble_service.hpp"

#include <FreeRTOS.h>
#include <task.h>

#include <array>
#include <cstddef>
#include <utility>
#include <expected>

#include "common.hpp"
#include "queue.hpp"
#include "gatt_server/gatt_server.hpp"

namespace bps::ble {

BleService::BleService() {}

void BleService::initialize() noexcept {
    // Initialize GATT server
    auto& gatt_server = gatt::GattServer::getInstance();
    gatt_server.initialize();
    gatt_server.on();
}

bool BleService::createTask(UBaseType_t const& priority) noexcept {
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
        priority,
        &this->task_handle
    );
}

bool BleService::sendMachineStatus(MachineStatus const& machine_status) noexcept {
    return this->machine_status_queue.send(machine_status, pdMS_TO_TICKS(5));
}

bool BleService::sendPulseValueSet(PulseValueSet const& value_set) noexcept {
    return this->pulse_value_set_queue.send(value_set, pdMS_TO_TICKS(5));
}

void BleService::registerActionQueue(QueueHandle_t const& handle) noexcept {
    if (!handle) return;
    this->action_queue_handle = handle;
    static auto action_callback = [](void* context, std::expected<Action, Error<std::byte>> action) {
        QueueHandle_t queue_handle = reinterpret_cast<QueueHandle_t>(context);
        if (action) {
            xQueueSend(queue_handle, &action.value(), pdMS_TO_TICKS(1000));
        } else {
            /* Error Handling */
        }
    };
    gatt::GattServer::getInstance().registerActionCallback(action_callback, handle);
}

void BleService::registerPressureBaseValueQueue(QueueHandle_t const& handle) noexcept {
    if (!handle) return;
    this->pressure_base_value_handle = handle;
    static auto pressure_base_value_callback = [](void* context, PressureBaseValue const& base_value) {
        QueueHandle_t queue_handle = reinterpret_cast<QueueHandle_t>(context);
        xQueueSend(queue_handle, &base_value, pdMS_TO_TICKS(1000));
    };
    gatt::GattServer::getInstance().registerPressureBaseValueCallback(pressure_base_value_callback, handle);
}

void BleService::taskLoop() noexcept {
    while (true) {
        static std::expected<QueueHandle_t, std::nullptr_t> selected_handle{};
        if ((selected_handle = this->queue_set.selectFromSet(pdMS_TO_TICKS(portMAX_DELAY)))) {
            if (selected_handle == this->machine_status_queue.getFreeRTOSQueueHandle()) {
                static MachineStatus status{};
                if (this->machine_status_queue.receive(status, pdMS_TO_TICKS(1000))) {
                    gatt::GattServer::getInstance().setMachineStatus(status);
                } else {
                    /* Error Handling */
                }
                
            } else if (selected_handle == this->pulse_value_set_queue.getFreeRTOSQueueHandle()) {
                static PulseValueSet value_set{};
                if (this->pulse_value_set_queue.receive(value_set, pdMS_TO_TICKS(5))) {
                    gatt::GattServer::getInstance().setPulseValueSet(value_set);
                } else {
                    /* Error Handling */
                }

            }

        } else {
            /* TODO: Error Handling */
        }
    }
    
    /* Optional: Error Handling */
}

} // bps::ble namespace