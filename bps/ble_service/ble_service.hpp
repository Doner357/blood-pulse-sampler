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
        
        // Senders, send data to the ouput queue
        // == Note ============================================================================
        //     This will block the caller's task for below time:
        //         Machine Status : 5 ms
        //         Pulse Value Set: 5 ms
        // ====================================================================================
        bool sendMachineStatus(MachineStatus const& machine_status) noexcept;
        bool sendPulseValueSet(PulseValueSet const& value_set) noexcept;

        // Register action and pressure base value queue
        void registerActionQueue(QueueHandle_t const& handle) noexcept;
        void registerPressureBaseValueQueue(QueueHandle_t const& handle) noexcept;

    private:
        BleService();
        static constexpr UBaseType_t kLengthOfMachineStatusQueue     = 3;
        static constexpr UBaseType_t kLengthOfPulseValueSetQueue     = 255;
        using MachineStatusQueue_t     =  StaticQueue<
                                            MachineStatus, 
                                            kLengthOfMachineStatusQueue
                                        >;
        using PulseValueSetQueue_t     = StaticQueue<
                                            PulseValueSet, 
                                            kLengthOfPulseValueSetQueue
                                        >;

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