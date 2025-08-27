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
        using commandCallback_t = void (*)(void* context, std::expected<Command, Error<std::byte>> command);

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

        GattServer& sendPulseValue(
            PulseValue const& value
        ) noexcept;

        GattServer& sendPulseValue(
            std::uint64_t  const& timestemp,
            std::float32_t const& cun,
            std::float32_t const& guan,
            std::float32_t const& chi
        ) noexcept;

        // =========================================================
        // == Getters                                             ==
        // =========================================================

        [[nodiscard]] std::expected<Command, Error<std::byte>> getCommand() const noexcept {
            return this->characteristics.getCommand();
        }
        [[nodiscard]] std::expected<MachineStatus, Error<std::byte>> getMachineStatus() const noexcept {
            return this->characteristics.getMachineStatus();
        }
        [[nodiscard]] std::uint16_t getMachineStatusClientConfiguration() const noexcept {
            return this->characteristics.getMachineStatusClientConfiguration();
        }
        [[nodiscard]] PulseValue getPulseValue() const noexcept {
            return this->characteristics.getPulseValue();
        }
        [[nodiscard]] std::uint16_t getPulseValueClientConfiguration() const noexcept {
            return this->characteristics.getPulseValueClientConfiguration();
        }

        // Register the Command & pressure base value callback which will be called
        // when value has been written
        void registerCommandCallback(commandCallback_t callback, void* context) noexcept;

    private:
        // ================================================================================================
        // == Nest class: CustomCaracteristics                                                           ==
        // ================================================================================================
        
        class CustomCharacteristics {
            public:
                // Descriptions for each characteristic
                static constexpr inline std::string_view command_description 
                = "Control machine movements";
                static constexpr inline std::string_view machine_status_description
                = "Status of sampler";
                static constexpr inline std::string_view pulse_value_description
                = "Measured pulsed value";

                CustomCharacteristics();

                // =========================================================
                // == Setters                                             ==
                // =========================================================
                CustomCharacteristics& setMachineStatus(
                    MachineStatus const& status
                ) noexcept;

                CustomCharacteristics& setMachineStatusClientConfiguration(
                    std::uint16_t const& configuration
                ) noexcept;

                CustomCharacteristics& setPulseValue(
                    PulseValue const& pulse_value
                ) noexcept;

                CustomCharacteristics& setPulseValue(
                    std::uint64_t  const& timestemp,
                    std::float32_t const& cun,
                    std::float32_t const& guan,
                    std::float32_t const& chi
                ) noexcept;

                CustomCharacteristics& setPulseValueClientConfiguration(
                    std::uint16_t configuration
                ) noexcept;
                

                // =========================================================
                // == Getters                                             ==
                // =========================================================

                [[nodiscard]] std::expected<Command, Error<std::byte>>  getCommand() const noexcept;
                [[nodiscard]] std::expected<MachineStatus, Error<std::byte>> getMachineStatus() const noexcept;
                [[nodiscard]] std::uint16_t getMachineStatusClientConfiguration() const noexcept;
                [[nodiscard]] PulseValue getPulseValue() const noexcept;
                [[nodiscard]] std::uint16_t getPulseValueClientConfiguration() const noexcept;
                // Data array reference getter
                [[nodiscard]] auto& getCommandArray() noexcept { return this->command; };
                [[nodiscard]] auto& getMachineStatusArray() noexcept { return this->machine_status; };
                [[nodiscard]] auto& getPulseValueArray() noexcept { return this->pulse_value; };
                
            private:
                // =========================================================
                // == Serialized data and client configuration            ==
                // =========================================================
                
                // Characteristic Command information
                std::array<std::byte, 13> command{ std::byte{0} };

                // Characteristic Machine status information
                std::array<std::byte, 1> machine_status{ std::byte{0} };
                std::uint16_t            machine_status_client_configuration = 0;

                // Characteristic Pulse value set information
                std::array<std::byte, 20> pulse_value{ std::byte{0} };
                std::uint16_t             pulse_value_client_configuration = 0;

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
        bool notification_pending_pulse_value{false};

        // command & pressure base value callback registered by user
        commandCallback_t command_callback{nullptr};
        void* command_callback_context{nullptr};

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