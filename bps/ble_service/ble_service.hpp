#ifndef BPS_BLE_SERVICE_HPP
#define BPS_BLE_SERVICE_HPP

#include <btstack_run_loop.h>

#include <cstdint>
#include <optional>

#include "common.hpp"
#include "queue.hpp"
#include "gatt_server/gatt_server.hpp"

namespace bps::ble {

// Meyers' Singleton Implementation
class BleService {
    public:

        // Meyers' Singleton basic constructor settings
        static BleService& getInstance() noexcept {
            static BleService service;
            return service;
        }

        BleService(BleService const&) = delete;
        BleService& operator=(BleService const&) = delete;
        
        // Initialize all resource
        // ! This must be done once before running !
        void initialize() noexcept;

        // Create a freertos task
        // ! This must be done once before running !
        bool createTask(UBaseType_t const& priority) noexcept;
        
        // Get the FreeRTOS queue handle
        MachineStatusQueue_t& getMachineStatusQueue() noexcept;
        PulseValueSetQueue_t& getPulseValueSetQueue() noexcept;

        // Register action and pressure base value queue
        void registerActionQueue(ActionQueue_t& queue) noexcept;
        void registerPressureBaseValueQueue(PressureBaseValueQueue_t& queue) noexcept;

    private:
        BleService();

        std::optional<ActionQueue_t*> action_queue{};
        std::optional<PressureBaseValueQueue_t*> pressure_base_value_queue{};
        MachineStatusQueue_t machine_status_queue{};
        PulseValueSetQueue_t pulse_value_set_queue{};

        StaticQueueSet<
            MachineStatusQueue_t,
            PulseValueSetQueue_t
        > queue_set = makeQueueSet(
            this->machine_status_queue,
            this->pulse_value_set_queue
        );

        std::optional<QueueHandle_t> action_queue_handle{};
        std::optional<QueueHandle_t> pressure_base_value_handle{};

        // FreeRTOS task
        TaskHandle_t task_handle{nullptr};
        void taskLoop() noexcept;
};

} // bps::gatt

#endif // BPS_GATT_SERVICE_HPP