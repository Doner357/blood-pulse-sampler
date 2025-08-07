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
        // Predefined type for convenience usages
        using actionCallback_t = void (*)(void* context, Action action);
        using pressureBaseValueCallback_t = void (*)(void* context, PressureBaseValue base_value);

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

        // Recive writable data (by ble client) from BLE service
        // == Note ============================================================================
        //     This will block the caller's task indefinitely until there is a data can receive
        //     Return false as error
        // ====================================================================================
        bool receiveAction(Action& action) noexcept;
        bool receivePressureBaseValue(PressureBaseValue& base_value) noexcept;
        
        // Senders, send data to the ouput queue
        // == Note ============================================================================
        //     This will block the caller's task for below time:
        //         Machine Status : 5 ms
        //         Pulse Value Set: 5 ms
        // ====================================================================================
        bool sendMachineStatus(MachineStatus const& machine_status) noexcept;
        bool sendPulseValueSet(PulseValueSet const& value_set) noexcept;

    private:
        BleService() {}

        static constexpr UBaseType_t kLengthOfActionQueue            = 3;
        static constexpr UBaseType_t kLengthOfPressureBaseValueQueue = 3;
        static constexpr UBaseType_t kLengthOfMachineStatusQueue     = 3;
        static constexpr UBaseType_t kLengthOfPulseValueSetQueue     = 255;
        using ActionQueue_t            = StaticQueue<
                                            Action,
                                            kLengthOfActionQueue
                                        >;
        using PressureBaseValueQueue_t = StaticQueue<
                                            PressureBaseValue,
                                            kLengthOfPressureBaseValueQueue
                                        >;
        using MachineStatusQueue_t     =  StaticQueue<
                                            MachineStatus, 
                                            kLengthOfMachineStatusQueue
                                        >;
        using PulseValueSetQueue_t     = StaticQueue<
                                            PulseValueSet, 
                                            kLengthOfPulseValueSetQueue
                                        >;
        ActionQueue_t            action_queue{};
        PressureBaseValueQueue_t pressure_base_value_queue{};
        MachineStatusQueue_t     machine_status_queue{};
        PulseValueSetQueue_t     pulse_value_set_queue{};

        // FreeRTOS task
        TaskHandle_t task_handle{nullptr};
        void taskLoop() noexcept;
};

} // bps::gatt

#endif // BPS_GATT_SERVICE_HPP