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
    ) == pdPASS;
}

MachineStatusQueue_t& BleService::getMachineStatusQueue() noexcept {
    return this->machine_status_queue;
}

PulseValueSetQueue_t& BleService::getPulseValueSetQueue() noexcept {
    return this->pulse_value_set_queue;
}

void BleService::registerActionQueue(ActionQueue_t& queue) noexcept {
    if (!queue.isValid()) return;
    this->output_action_queue_ptr = &queue;
    static auto action_callback = [](void* context, std::expected<Action, Error<std::byte>> action) {
        auto send_queue = reinterpret_cast<ActionQueue_t*>(context);
        if (action) {
            // This lambda will be called by the GattServer, so there shouldn't be any delay.
            send_queue->sendFromIsr(action.value(), nullptr);
        } else {
            /* Error Handling */
        }
    };
    gatt::GattServer::getInstance().registerActionCallback(
        action_callback,
        this->output_action_queue_ptr
    );
}

void BleService::registerPressureBaseValueQueue(PressureBaseValueQueue_t& queue) noexcept {
    if (!queue.isValid()) return;
    this->output_pressure_base_value_queue_ptr = &queue;
    static auto pressure_base_value_callback = [](void* context, PressureBaseValue const& base_value) {
        auto send_queue = reinterpret_cast<PressureBaseValueQueue_t*>(context);
        // This lambda will be called by the GattServer, so there shouldn't be any delay.
        send_queue->sendFromIsr(base_value, nullptr);
    };
    gatt::GattServer::getInstance().registerPressureBaseValueCallback(
        pressure_base_value_callback,
        this->output_pressure_base_value_queue_ptr
    );
}

void BleService::taskLoop() noexcept {
    while (true) {
        static std::expected<QueueHandle_t, std::nullptr_t> selected_handle{};
        if ((selected_handle = this->queue_set.selectFromSet(pdMS_TO_TICKS(portMAX_DELAY)))) {
            if (selected_handle == this->machine_status_queue.getFreeRTOSQueueHandle()) {
                static MachineStatus status{};
                if (this->machine_status_queue.receive(status, pdMS_TO_TICKS(1000))) {
                    gatt::GattServer::getInstance().sendMachineStatus(status);
                } else {
                    /* Error Handling */
                }
                
            } else if (selected_handle == this->pulse_value_set_queue.getFreeRTOSQueueHandle()) {
                static PulseValueSet value_set{};
                if (this->pulse_value_set_queue.receive(value_set, pdMS_TO_TICKS(5))) {
                    gatt::GattServer::getInstance().sendPulseValueSet(value_set);
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