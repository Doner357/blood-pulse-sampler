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

QueueReference<MachineStatus> BleService::getMachineStatusQueueRef() const noexcept {
    return this->machine_status_queue;
}

QueueReference<PulseValue> BleService::getPulseValueQueueRef() const noexcept {
    return this->pulse_value_queue;
}

void BleService::registerCommandQueue(QueueReference<Command> const& queue) noexcept {
    if (!queue.isValid()) return;
    this->output_command_queue_ref = queue;
    static auto command_callback = [](void* context, std::expected<Command, Error<std::byte>> command) {
        auto send_queue = reinterpret_cast<QueueReference<Command>*>(context);
        if (command) {
            // This lambda will be called by the GattServer, so there shouldn't be any delay.
            send_queue->sendFromIsr(command.value(), nullptr);
        } else {
            /* Error Handling */
        }
    };
    gatt::GattServer::getInstance().registerCommandCallback(
        command_callback,
        &this->output_command_queue_ref
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
                
            } else if (selected_handle == this->pulse_value_queue.getFreeRTOSQueueHandle()) {
                static PulseValue value{};
                if (this->pulse_value_queue.receive(value, pdMS_TO_TICKS(5))) {
                    gatt::GattServer::getInstance().sendPulseValue(value);
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