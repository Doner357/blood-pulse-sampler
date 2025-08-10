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
#include <string_view>

#include "gatt_database.hpp"
#include "common.hpp"
#include "utils.hpp"

#define APP_AD_FLAGS 0x06

namespace bps::ble::gatt {

// Meyers' Singleton Implementation
class GattServer {
    public:
        // Predefined type for convenience usages
        using actionCallback_t = void (*)(void* context, std::expected<Action, Error<std::byte>> action);
        using pressureBaseValueCallback_t = void (*)(void* context, PressureBaseValue const& base_value);

        // Meyers' Singleton basic constructor settings
        static GattServer& getInstance() noexcept {
            static GattServer instance;
            return instance;
        }
        GattServer(GattServer const&) = delete;
        GattServer& operator=(GattServer const&) = delete;
        
        // Initialize all necessary server settings
        void initialize() noexcept;
        // Start Gatt Server
        std::expected<int, Error<int>> on() noexcept;
        // Turn off Gatt Server
        std::expected<int, Error<int>> off() noexcept;

        // =========================================================
        // == Setters, only allow to set data readable by client  ==
        // =========================================================
        
        GattServer& sendMachineStatus(
            MachineStatus const& status
        ) noexcept;

        GattServer& sendPulseValueSet(
            PulseValueSet const& value_set
        ) noexcept;

        GattServer& sendPulseValueSet(
            std::float64_t const& timestemp,
            std::float32_t const& cun,
            std::float32_t const& guan,
            std::float32_t const& chi
        ) noexcept;

        // =========================================================
        // == Getters                                             ==
        // =========================================================

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
        // ================================================================================================
        // == Nest class: CustomCaracteristics                                                           ==
        // ================================================================================================
        
        class CustomCharacteristics {
            public:
                // Descriptions for each characteristic
                static constexpr inline std::string_view action_description 
                = "Control machine movements";
                static constexpr inline std::string_view pressure_base_value_description
                = "Pressure reference values at three measuring locations";
                static constexpr inline std::string_view machine_status_description
                = "Status of sampler";
                static constexpr inline std::string_view pulse_value_set_description
                = "Measured pulsed value";

                CustomCharacteristics();

                // =========================================================
                // == Setters                                             ==
                // =========================================================
                CustomCharacteristics& setAction(
                    Action const& action
                ) noexcept;

                CustomCharacteristics& setAction(
                    ActionType const& action_type,
                    PressureType const& cun,
                    PressureType const& guan,
                    PressureType const& chi
                ) noexcept;

                CustomCharacteristics& setPressureBaseValue(
                    PressureBaseValue const& value
                ) noexcept;

                CustomCharacteristics& setPressureBaseValue(
                    std::float32_t floating,
                    std::float32_t middle,
                    std::float32_t deep
                ) noexcept;

                CustomCharacteristics& setMachineStatus(
                    MachineStatus const& status
                ) noexcept;

                CustomCharacteristics& setMachineStatusClientConfiguration(
                    std::uint16_t const& configuration
                ) noexcept;

                CustomCharacteristics& setPulseValueSet(
                    PulseValueSet const& value_set
                ) noexcept;

                CustomCharacteristics& setPulseValueSet(
                    std::float64_t const& timestemp,
                    std::float32_t const& cun,
                    std::float32_t const& guan,
                    std::float32_t const& chi
                ) noexcept;

                CustomCharacteristics& setPulseValueSetClientConfiguration(
                    std::uint16_t configuration
                ) noexcept;
                

                // =========================================================
                // == Getters                                             ==
                // =========================================================

                [[nodiscard]] std::expected<Action, Error<std::byte>>  getAction() const noexcept;
                [[nodiscard]] PressureBaseValue getPressureBaseValue() const noexcept;
                [[nodiscard]] std::expected<MachineStatus, Error<std::byte>> getMachineStatus() const noexcept;
                [[nodiscard]] std::uint16_t getMachineStatusClientConfiguration() const noexcept;
                [[nodiscard]] PulseValueSet getPulseValueSet() const noexcept;
                [[nodiscard]] std::uint16_t getPulseValueSetClientConfiguration() const noexcept;
                // Data array reference getter
                [[nodiscard]] auto& getActionArray() noexcept { return this->action; };
                [[nodiscard]] auto& getPressureBaseValueArray() noexcept { return this->pressure_base_value; };
                [[nodiscard]] auto& getMachineStatusArray() noexcept { return this->machine_status; };
                [[nodiscard]] auto& getPulseValueSetArray() noexcept { return this->pulse_value_set; };
                
            private:
                // =========================================================
                // == Serialized data and client configuration            ==
                // =========================================================
                
                // Characteristic Action information
                std::array<std::byte, 1> action;

                // Characteristic Pressure base value information
                std::array<std::byte, 12> pressure_base_value;

                // Characteristic Machine status information
                std::array<std::byte, 1> machine_status;
                std::uint16_t            machine_status_client_configuration;

                // Characteristic Pulse value set information
                std::array<std::byte, 20> pulse_value_set;
                std::uint16_t             pulse_value_set_client_configuration;

        } characteristics{};
        // ================================================================================================
        // == End of CustomCaracteristics                                                                ==
        // ================================================================================================

        // Private Constructor for singletton design model
        GattServer();
        
        // Connection handle for service
        hci_con_handle_t      hci_con_handle;
        att_service_handler_t service_handler;
        
        // Long life cycle callback registration
        btstack_packet_callback_registration_t hci_event_callback_registration;
        
        // Notifycation flags, true when there is one or more data need to be notified
        bool notification_pending_machine_status{false};
        bool notification_pending_pulse_value_set{false};

        // action & pressure base value callback registered by user
        actionCallback_t action_callback{nullptr};
        pressureBaseValueCallback_t pressure_base_value_callback{nullptr};
        void* action_callback_context{nullptr};
        void* pressure_base_value_context{nullptr};

        // Btstack packet handlers
        void packetHandler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size);

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