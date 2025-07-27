#include "gatt_server.hpp"

#include <btstack.h>
#include <pico/cyw43_arch.h>
#include <pico/btstack_cyw43.h>

#include <cstdint>
#include <cstring>
#include <algorithm>

#include "utils.hpp"
#include "gatt_database.hpp"

namespace bps::gatt {

namespace {

auto gap_adv_data = make_bytes(
    // Flags general discoverable
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
    // Name
    0x04, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'B', 'S', 'P',
    // Custom Service UUID
    0x11, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS,
    0x0a, 0x35, 0xf3, 0x0e, 0x20, 0x30, 0x28, 0x88, 0xbc, 0x41, 0x53, 0xc6, 0xc0, 0x47, 0x2c, 0x65 
);
static_assert(gap_adv_data.size() <= 31);

constexpr uint16_t adv_int_min = 800;
constexpr uint16_t adv_int_max = 800;
constexpr std::uint8_t adv_type = 0;
bd_addr_t null_addr = {0};

} // anymous namespace

GattServer::GattServer() : hci_con_handle(HCI_CON_HANDLE_INVALID) {
    this->service_handler.start_handle = Att::Handle::CustomCharacteristic::kStart;
    this->service_handler.end_handle = Att::Handle::CustomCharacteristic::kEnd;
    this->service_handler.read_callback = &GattServer::attReadCallbackTrampoline;
    this->service_handler.write_callback = &GattServer::attWriteCallbackTrampoline;
}

void GattServer::initialize() noexcept {
    if (cyw43_arch_init()) {
        /* TODO: Error Handling */
    }

    l2cap_init();
    sm_init();

    // Initialize ATT server, no general read/write callbacks
    // because we'll set one up for each service
    att_server_init(reinterpret_cast<uint8_t const*>(Att::Database::kProfileData.data()), nullptr, nullptr);
    
    att_server_register_service_handler(&this->service_handler);
    
    // inform about BTstack state
    this->hci_event_callback_registration.callback = &GattServer::packetHandler;
    hci_add_event_handler(&this->hci_event_callback_registration);

    // register for ATT event
    att_server_register_packet_handler(&GattServer::packetHandler);
}

int GattServer::on() noexcept {
    return hci_power_control(HCI_POWER_ON);
}

int GattServer::off() noexcept {
    return hci_power_control(HCI_POWER_OFF);
}

void GattServer::packetHandler (
    uint8_t packet_type,
    uint16_t channel,
    uint8_t* packet,
    uint16_t size
) {
    GattServer& instance = GattServer::getInstance();
    instance.handleEvent(packet_type, channel, packet, size);
}

void GattServer::handleEvent(
    uint8_t packet_type,
    [[maybe_unused]] uint16_t channel,
    uint8_t* packet,
    [[maybe_unused]] uint16_t size
) {
    bd_addr_t local_addr = {0};
    if (packet_type != HCI_EVENT_PACKET) return;

    std::uint8_t event_type = hci_event_packet_get_type(packet);

    switch (event_type) {
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
        gap_local_bd_addr(local_addr);
        gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
        gap_advertisements_set_data(gap_adv_data.size(), reinterpret_cast<uint8_t*>(gap_adv_data.data()));
        gap_advertisements_enable(1);
        break;

    case HCI_EVENT_CONNECTION_COMPLETE:
        this->hci_con_handle = hci_event_connection_complete_get_connection_handle(packet);
        /* Log handling */
        break;

    case HCI_EVENT_DISCONNECTION_COMPLETE:
        /* Log handling */
        this->hci_con_handle = HCI_CON_HANDLE_INVALID;
        gap_advertisements_enable(1);
        break;

    case ATT_EVENT_CAN_SEND_NOW:
        if (this->notification_pending_machine_status) {
            this->notification_pending_machine_status = false;
            att_server_notify(
                this->hci_con_handle,
                Att::Handle::CustomCharacteristic::MachineStatus::kValue,
                reinterpret_cast<uint8_t*>(this->characteristics.getMachineStatusArray().data()),
                this->characteristics.getMachineStatusArray().size()
            );
            att_server_request_can_send_now_event(this->hci_con_handle);
        } else if (this->notification_pending_pulse_value_set) {
            this->notification_pending_pulse_value_set = false;
            att_server_notify(
                this->hci_con_handle,
                Att::Handle::CustomCharacteristic::PulseDataSet::kValue,
                reinterpret_cast<uint8_t*>(this->characteristics.getPulseValueSetArray().data()),
                this->characteristics.getPulseValueSetArray().size()
            );
            att_server_request_can_send_now_event(this->hci_con_handle);
        }
        break;
        
    default:
        break;
    }
}

// Setters, only allow to set Writable data
GattServer& GattServer::setMachineStatus(
    MachineStatus const& status
) noexcept {
    this->characteristics.setMachineStatus(status);
    if (this->characteristics.getMachineStatusClientConfiguration() == 
    GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION &&
    (this->hci_con_handle != HCI_CON_HANDLE_INVALID)) {
        this->notification_pending_machine_status = true;
        att_server_request_can_send_now_event(this->hci_con_handle);
    }
    return *this;
}

GattServer& GattServer::setPulseValueSet(
    PulseValueSet const& value_set
) noexcept {
    this->characteristics.setPulseValueSet(value_set);
    if (this->characteristics.getPulseValueSetClientConfiguration() == 
    GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION) {
        this->notification_pending_pulse_value_set = true;
        att_server_request_can_send_now_event(this->hci_con_handle);
    }
    return *this;
}

GattServer& GattServer::setPulseValueSet(
    std::float64_t const& timestemp,
    std::float32_t const& cun,
    std::float32_t const& guan,
    std::float32_t const& chi
) noexcept {
    this->setPulseValueSet(
        PulseValueSet {
            .timestemp = timestemp,
            .cun = cun,
            .guan = guan,
            .chi = chi
        }
    );

    return *this;
}

uint16_t GattServer::attReadCallbackTrampoline(
    hci_con_handle_t con_handle,
    uint16_t attribute_handle,
    uint16_t offset,
    uint8_t* buffer,
    uint16_t buffer_size
) noexcept {
    GattServer& gatt_server = GattServer::getInstance();
    return gatt_server.attReadCallback(con_handle, attribute_handle, offset, buffer, buffer_size);
}

int GattServer::attWriteCallbackTrampoline(
    hci_con_handle_t con_handle,
    uint16_t attribute_handle,
    uint16_t transaction_mode,
    uint16_t offset,
    unsigned char *buffer,
    uint16_t buffer_size
) noexcept {
    GattServer& gatt_server = GattServer::getInstance();
    return gatt_server.attWriteCallback(con_handle, attribute_handle, transaction_mode, offset, buffer, buffer_size);
}

// Real att read / write callback
uint16_t GattServer::attReadCallback(
    [[maybe_unused]] hci_con_handle_t const& con_handle,
    uint16_t const& attribute_handle,
    uint16_t const& offset,
    uint8_t* buffer,
    uint16_t const& buffer_size
) noexcept {
    switch (attribute_handle) {
    case Att::Handle::CustomCharacteristic::Action::kValue:
        return att_read_callback_handle_blob(
            reinterpret_cast<uint8_t const*>(this->characteristics.getActionArray().data()),
            this->characteristics.getActionArray().size(),
            offset,
            buffer,
            buffer_size
        );

    case Att::Handle::CustomCharacteristic::Action::kUserDescription:
        return att_read_callback_handle_blob(
            reinterpret_cast<uint8_t const*>(CustomCharacteristics::action_description.data()),
            CustomCharacteristics::action_description.size(),
            offset,
            buffer,
            buffer_size
        );

    case Att::Handle::CustomCharacteristic::PressureBaseValue::kValue:
        return att_read_callback_handle_blob(
            reinterpret_cast<uint8_t const*>(this->characteristics.getPressureBaseValueArray().data()),
            this->characteristics.getPressureBaseValueArray().size(),
            offset,
            buffer,
            buffer_size
        );

    case Att::Handle::CustomCharacteristic::PressureBaseValue::kUserDescription:
        return att_read_callback_handle_blob(
            reinterpret_cast<uint8_t const*>(CustomCharacteristics::pressure_base_value_description.data()),
            CustomCharacteristics::pressure_base_value_description.size(),
            offset,
            buffer,
            buffer_size
        );

    case Att::Handle::CustomCharacteristic::MachineStatus::kValue:
        return att_read_callback_handle_blob(
            reinterpret_cast<uint8_t const*>(this->characteristics.getMachineStatusArray().data()),
            this->characteristics.getMachineStatusArray().size(),
            offset,
            buffer,
            buffer_size
        );

    case Att::Handle::CustomCharacteristic::MachineStatus::kClientConfiguration:
        return att_read_callback_handle_little_endian_16(
            this->characteristics.getMachineStatusClientConfiguration(),
            offset,
            buffer,
            buffer_size
        );

    case Att::Handle::CustomCharacteristic::MachineStatus::kUserDescription:
        return att_read_callback_handle_blob(
            reinterpret_cast<uint8_t const*>(CustomCharacteristics::machine_status_description.data()),
            CustomCharacteristics::machine_status_description.size(),
            offset,
            buffer,
            buffer_size
        );

    case Att::Handle::CustomCharacteristic::PulseDataSet::kValue:
        return att_read_callback_handle_blob(
            reinterpret_cast<uint8_t const*>(this->characteristics.getPulseValueSetArray().data()),
            this->characteristics.getPulseValueSetArray().size(),
            offset,
            buffer,
            buffer_size
        );

    case Att::Handle::CustomCharacteristic::PulseDataSet::kClientConfiguration:
        return att_read_callback_handle_little_endian_16(
            this->characteristics.getPulseValueSetClientConfiguration(),
            offset,
            buffer,
            buffer_size
        );

    case Att::Handle::CustomCharacteristic::PulseDataSet::kUserDescription:
        return att_read_callback_handle_blob(
            reinterpret_cast<uint8_t const*>(CustomCharacteristics::pulse_value_set_description.data()),
            CustomCharacteristics::pulse_value_set_description.size(),
            offset,
            buffer,
            buffer_size
        );

    default:
        break;
    }

    return 0;
}

int GattServer::attWriteCallback(
    [[maybe_unused]] hci_con_handle_t const& con_handle,
    uint16_t const& attribute_handle,
    [[maybe_unused]] uint16_t const& transaction_mode,
    [[maybe_unused]] uint16_t const& offset,
    unsigned char *buffer,
    [[maybe_unused]] uint16_t const& buffer_size
) noexcept {
    switch (attribute_handle) {
    case Att::Handle::CustomCharacteristic::Action::kValue:
        std::memcpy(
            this->characteristics.getActionArray().data(),
            buffer,
            std::min(static_cast<std::size_t>(buffer_size), this->characteristics.getActionArray().size())
        );
        break;
        
    case Att::Handle::CustomCharacteristic::PressureBaseValue::kValue:
        std::memcpy(
            this->characteristics.getPressureBaseValueArray().data(),
            buffer,
            std::min(static_cast<std::size_t>(buffer_size), this->characteristics.getPressureBaseValueArray().size())
        );
        break;

    case Att::Handle::CustomCharacteristic::MachineStatus::kClientConfiguration:
        this->characteristics.setMachineStatusClientConfiguration(little_endian_read_16(buffer, 0));
        break;
        
    case Att::Handle::CustomCharacteristic::PulseDataSet::kClientConfiguration:
        this->characteristics.setPulseValueSetClientConfiguration(little_endian_read_16(buffer, 0));
        break;
        
    default:
        break;
    }

    return 0;
}

} // namespace bps