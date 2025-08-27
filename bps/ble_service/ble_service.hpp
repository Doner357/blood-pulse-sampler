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
        PulseValueQueue_t& getPulseValueQueue() noexcept;

        // Register command and pressure base value queue
        void registerCommandQueue(CommandQueue_t& queue) noexcept;

    private:
        BleService();

        CommandQueue_t* output_command_queue_ptr{nullptr};
        MachineStatusQueue_t machine_status_queue{};
        PulseValueQueue_t pulse_value_queue{};

        StaticQueueSet<
            MachineStatusQueue_t,
            PulseValueQueue_t
        > queue_set = makeQueueSet(
            this->machine_status_queue,
            this->pulse_value_queue
        );

        // FreeRTOS task
        TaskHandle_t task_handle{nullptr};
        void taskLoop() noexcept;
};

} // bps::gatt

#endif // BPS_GATT_SERVICE_HPP