#ifndef BPS_BLE_SERVICE_HPP
#define BPS_BLE_SERVICE_HPP

#include <btstack_run_loop.h>

#include <cstdint>

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
        
        // Get the input queue (like setters reference)
        MachineStatusQueue_t& getMachineStatusQueue() noexcept;
        PulseValueSetQueue_t& getPulseValueSetQueue() noexcept;

        // Register action and pressure base value queue
        void registerActionQueue(ActionQueue_t& queue) noexcept;
        void registerPressureBaseValueQueue(PressureBaseValueQueue_t& queue) noexcept;

    private:
        BleService();

        ActionQueue_t* output_action_queue_ptr{nullptr};
        PressureBaseValueQueue_t* output_pressure_base_value_queue_ptr{nullptr};
        MachineStatusQueue_t machine_status_queue{};
        PulseValueSetQueue_t pulse_value_set_queue{};

        StaticQueueSet<
            MachineStatusQueue_t,
            PulseValueSetQueue_t
        > queue_set = makeQueueSet(
            this->machine_status_queue,
            this->pulse_value_set_queue
        );

        // FreeRTOS task
        TaskHandle_t task_handle{nullptr};
        void taskLoop() noexcept;
};

} // bps::gatt

#endif // BPS_GATT_SERVICE_HPP