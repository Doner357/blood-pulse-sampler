#ifndef BPS_GATT_SERVER_HPP
#define BPS_GATT_SERVER_HPP

#include <btstack.h>
#include <ble/att_db.h>
#include <ble/att_server.h>

#include <cstdint>
#include <cstddef>
#include <array>
#include <stdfloat>
#include <expected>

#include "gatt_database.hpp"
#include "utils.hpp"

#define APP_AD_FLAGS 0x06

namespace bps::gatt {

// Meyers' Singleton Implementation
class GattServer {
    public:
        // Predefined type for convenience usages
        using actionCallback_t = void (*)(void* context, std::expected<Action, Error<std::byte>>);
        using pressureBaseValueCallback_t = void (*)(void* context, PressureBaseValue const& base_value);

        static GattServer& getInstance() noexcept {
            static GattServer instance;
            return instance;
        }

        GattServer(GattServer const&) = delete;
        GattServer& operator=(GattServer const&) = delete;

        void initialize() noexcept;
        std::expected<int, Error<int>> on() noexcept;
        std::expected<int, Error<int>> off() noexcept;

        // Setters, only allow to set Writable data
        GattServer& setMachineStatus(
            MachineStatus const& status
        ) noexcept;

        GattServer& setPulseValueSet(
            PulseValueSet const& value_set
        ) noexcept;

        GattServer& setPulseValueSet(
            std::float64_t const& timestemp,
            std::float32_t const& cun,
            std::float32_t const& guan,
            std::float32_t const& chi
        ) noexcept;
        
        // Getters
        [[nodiscard]] std::expected<Action, Error<std::byte>> getAction() const noexcept {
            return this->characteristics.getAction();
        }
        [[nodiscard]] PressureBaseValue getPressureBaseValue() const noexcept {
            return this->characteristics.getPressureBaseValue();
        }
        [[nodiscard]] std::expected<MachineStatus, Error<std::byte>> getMachineStatus() const noexcept {
            return this->characteristics.getMachineStatus();
        }
        [[nodiscard]] std::uint16_t getMachineStatusClientConfiguration() const noexcept {
            return this->characteristics.getMachineStatusClientConfiguration();
        }
        [[nodiscard]] PulseValueSet getPulseValueSet() const noexcept {
            return this->characteristics.getPulseValueSet();
        }
        [[nodiscard]] std::uint16_t getPulseValueSetClientConfiguration() const noexcept {
            return this->characteristics.getPulseValueSetClientConfiguration();
        }

        // Register the action & pressure base value callback which will be called
        // when value has been written
        void registerActionCallback(actionCallback_t callback, void* context) noexcept;
        void registerPressureBaseValueCallback(pressureBaseValueCallback_t callback, void* context) noexcept;

    private:

        // Private Constructor for singletton design model
        GattServer();
        
        // Connection handle for service
        hci_con_handle_t      hci_con_handle;
        att_service_handler_t service_handler;
        
        btstack_packet_callback_registration_t hci_event_callback_registration;

        // Custom characteristic
        CustomCharacteristics characteristics;
        
        bool notification_pending_machine_status{false};
        bool notification_pending_pulse_value_set{false};

        // action & pressure base value callback registered by user
        actionCallback_t action_callback{nullptr};
        pressureBaseValueCallback_t pressure_base_value_callback{nullptr};
        void* action_callback_context{nullptr};
        void* pressure_base_value_context{nullptr};

        // Btstack packet handlers
        static void packetHandler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size);
        void handleEvent(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size);

        // Trampoline function for btstack callback functions
        // Keep the type same as the interface provided by btstack
        static uint16_t attReadCallbackTrampoline(
            hci_con_handle_t con_handle,
            uint16_t attribute_handle,
            uint16_t offset,
            uint8_t* buffer,
            uint16_t buffer_size
        ) noexcept;
        static int attWriteCallbackTrampoline(
            hci_con_handle_t con_handle,
            uint16_t attribute_handle,
            uint16_t transaction_mode,
            uint16_t offset,
            unsigned char *buffer,
            uint16_t buffer_size
        ) noexcept;

        // Real att read / write callback
        uint16_t attReadCallback(
            hci_con_handle_t const& con_handle,
            uint16_t const& attribute_handle,
            uint16_t const& offset,
            uint8_t* buffer,
            uint16_t const& buffer_size
        ) noexcept;
        int attWriteCallback(
            hci_con_handle_t const& con_handle,
            uint16_t const& attribute_handle,
            uint16_t const& transaction_mode,
            uint16_t const& offset,
            unsigned char *buffer,
            uint16_t const& buffer_size
        ) noexcept;
};
    
} // namespace bps

#endif // BPS_GATT_SERVER_HPP