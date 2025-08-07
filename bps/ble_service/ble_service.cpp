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
    return this->machine_status_queue.send(machine_status, pdTICKS_TO_MS(5));
}

bool BleService::sendPulseValueSet(PulseValueSet const& value_set) noexcept {
    return this->pulse_value_set_queue.send(value_set, pdTICKS_TO_MS(5));
}

bool BleService::receiveAction(Action& action) noexcept {
    return this->action_queue.receive(action, 5) == pdPASS;
}

bool BleService::receivePressureBaseValue(PressureBaseValue& base_value) noexcept {
    return this->pressure_base_value_queue.receive(base_value, 5) == pdPASS;
}

void BleService::taskLoop() noexcept {
}

} // bps::ble namespace