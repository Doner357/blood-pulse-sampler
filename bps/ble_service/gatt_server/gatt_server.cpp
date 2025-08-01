#include "gatt_server.hpp"

#include <btstack.h>
#include <pico/cyw43_arch.h>
#include <pico/btstack_cyw43.h>

#include <cstdint>
#include <cstring>
#include <algorithm>
#include <array>
#include <cstddef>
#include <expected>
#include <string_view>

#include "utils.hpp"
#include "gatt_database.hpp"

namespace bps::ble::gatt {

namespace {

auto gap_adv_data = makeBytes(
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

} // anymous namespace

// ================================================================================================
// == GattServer::CustomCaracteristics                                                           ==
// ================================================================================================
GattServer::CustomCharacteristics::CustomCharacteristics() : 
    action({std::byte{0}}),
    pressure_base_value({std::byte{0}})
{
    setMachineStatus(MachineStatus::eNull);
    setMachineStatusClientConfiguration(0);
    setPulseValueSet(0.0, 0.0_pa, 0.0_pa, 0.0_pa);
    setPulseValueSetClientConfiguration(0);
}


GattServer::CustomCharacteristics& GattServer::CustomCharacteristics::setAction(
    Action const& act
) noexcept {
    auto packed_action = 
        std::byte{(std::byte{std::to_underlying(act.action_type)} & std::byte{0x03}) << 0} |
        std::byte{(std::byte{std::to_underlying(act.cun)} & std::byte{0x0c}) << 2} |
        std::byte{(std::byte{std::to_underlying(act.guan)} & std::byte{0x30}) << 4} |
        std::byte{(std::byte{std::to_underlying(act.chi)} & std::byte{0xc0}) << 6};

    this->action[0] = packed_action;
    return *this;
}

GattServer::CustomCharacteristics& GattServer::CustomCharacteristics::setAction(
    ActionType const& action_type,
    PressureType const& cun,
    PressureType const& guan,
    PressureType const& chi
) noexcept {
    return setAction(
        Action {
            .action_type = action_type,
            .cun = cun,
            .guan = guan,
            .chi = chi
        }
    );
}

GattServer::CustomCharacteristics& GattServer::CustomCharacteristics::setPressureBaseValue(
    PressureBaseValue const& value
) noexcept {
    std::size_t offset = 0;
    
    writeAsLittleEndian(value.floating, &this->pressure_base_value[offset]);
    offset += sizeof(value.floating);
    
    writeAsLittleEndian(value.middle, &this->pressure_base_value[offset]);
    offset += sizeof(value.middle);
    
    writeAsLittleEndian(value.deep, &this->pressure_base_value[offset]);
    offset += sizeof(value.deep);

    return *this;
}

GattServer::CustomCharacteristics& GattServer::CustomCharacteristics::setPressureBaseValue(
    std::float32_t floating,
    std::float32_t middle,
    std::float32_t deep
) noexcept {
    return setPressureBaseValue(
        PressureBaseValue {
            .floating = floating,
            .middle = middle,
            .deep = deep
        }
    );
}

GattServer::CustomCharacteristics& GattServer::CustomCharacteristics::setMachineStatus(
    MachineStatus const& status
) noexcept {
    this->machine_status[0] = std::byte{std::to_underlying(status)};
    return *this;
}

GattServer::CustomCharacteristics& GattServer::CustomCharacteristics::setMachineStatusClientConfiguration(
    std::uint16_t const& configuration
) noexcept {
    this->machine_status_client_configuration = configuration;
    return *this;
}

GattServer::CustomCharacteristics& GattServer::CustomCharacteristics::setPulseValueSet(
    PulseValueSet const& value_set
) noexcept {
    std::size_t offset = 0;

    writeAsLittleEndian(value_set.timestemp, &this->pulse_value_set[offset]);
    offset += sizeof(value_set.timestemp);

    writeAsLittleEndian(value_set.cun, &this->pulse_value_set[offset]);
    offset += sizeof(value_set.cun);

    writeAsLittleEndian(value_set.guan, &this->pulse_value_set[offset]);
    offset += sizeof(value_set.guan);

    writeAsLittleEndian(value_set.chi, &this->pulse_value_set[offset]);

    return *this;
}

GattServer::CustomCharacteristics& GattServer::CustomCharacteristics::setPulseValueSet(
    std::float64_t const& timestemp,
    std::float32_t const& cun,
    std::float32_t const& guan,
    std::float32_t const& chi
) noexcept {
    return setPulseValueSet(
        PulseValueSet{
            .timestemp = timestemp,
            .cun       = cun,
            .guan      = guan,
            .chi       = chi
        }
    );
}

GattServer::CustomCharacteristics& GattServer::CustomCharacteristics::setPulseValueSetClientConfiguration(
    std::uint16_t configuration
) noexcept {
    this->pulse_value_set_client_configuration = configuration;
    return *this;
}

// Getters
std::expected<Action, Error<std::byte>> GattServer::CustomCharacteristics::getAction() const noexcept {
    using mask = std::byte;
    auto action_type   = toActionType((this->action[0] & mask{0x03}) >> 0);
    auto cun_pressure  = toPressureType((this->action[0] & mask{0x0c}) >> 2);
    auto guan_pressure = toPressureType((this->action[0] & mask{0x30}) >> 4);
    auto chi_pressure  = toPressureType((this->action[0] & mask{0xc0}) >> 6);
    
    if (!action_type || !cun_pressure || !guan_pressure || !chi_pressure) {
        return std::unexpected(
            Error{ ErrorType::eInvalidValue, this->action[0] }
        );
    }

    return Action{ action_type.value(), cun_pressure.value(), guan_pressure.value(), chi_pressure.value() };
}

PressureBaseValue GattServer::CustomCharacteristics::getPressureBaseValue() const noexcept {
    PressureBaseValue value{};

    std::size_t offset = 0;
    
    readAsNativeEndian(&this->pressure_base_value[offset], value.floating);
    offset += sizeof(value.floating);

    readAsNativeEndian(&this->pressure_base_value[offset], value.middle);
    offset += sizeof(value.middle);

    readAsNativeEndian(&this->pressure_base_value[offset], value.deep);
    
    return value;
}

std::expected<MachineStatus, Error<std::byte>> GattServer::CustomCharacteristics::getMachineStatus() const noexcept {
    auto status = toMachineStatus(this->machine_status[0]);
    if (!status) {
        return std::unexpected(
            Error{ ErrorType::eInvalidValue, this->machine_status[0] }
        );
    }
    return status.value();
}

std::uint16_t GattServer::CustomCharacteristics::getMachineStatusClientConfiguration() const noexcept {
    return this->machine_status_client_configuration;
}

PulseValueSet GattServer::CustomCharacteristics::getPulseValueSet() const noexcept {
    PulseValueSet value_set{};

    std::size_t offset = 0;

    readAsNativeEndian(&this->pulse_value_set[offset], value_set.timestemp);
    offset += sizeof(value_set.timestemp);

    readAsNativeEndian(&this->pulse_value_set[offset], value_set.cun);
    offset += sizeof(value_set.cun);

    readAsNativeEndian(&this->pulse_value_set[offset], value_set.guan);
    offset += sizeof(value_set.guan);

    readAsNativeEndian(&this->pulse_value_set[offset], value_set.chi);
    
    return value_set;
}

std::uint16_t GattServer::CustomCharacteristics::getPulseValueSetClientConfiguration() const noexcept {
    return this->pulse_value_set_client_configuration;
}

// ================================================================================================
// == GattServer                                                                                 ==
// ================================================================================================
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

std::expected<int, Error<int>> GattServer::on() noexcept {
    int status = hci_power_control(HCI_POWER_ON);
    if(status != 0) {
        return std::unexpected(Error{ErrorType::eFailedOperation, status});
    }
    return status;
}

std::expected<int, Error<int>> GattServer::off() noexcept {
    int status = hci_power_control(HCI_POWER_OFF);
    if(status != 0) {
        return std::unexpected(Error{ErrorType::eFailedOperation, status});
    }
    return status;
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
    bd_addr_t null_addr = {0};

    switch (event_type) {
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
        gap_local_bd_addr(local_addr);
        gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
        gap_advertisements_set_data(gap_adv_data.size(), reinterpret_cast<uint8_t*>(gap_adv_data.data()));
        gap_advertisements_enable(1);
        break;

    case HCI_EVENT_CONNECTION_COMPLETE:
        break;
    
    case HCI_EVENT_LE_META:
        switch (hci_event_le_meta_get_subevent_code(packet)) {
        case HCI_SUBEVENT_LE_CONNECTION_COMPLETE:
            this->hci_con_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
            if (hci_subevent_le_connection_complete_get_status(packet) == 0) {
                gap_request_connection_parameter_update(this->hci_con_handle, 6, 12, 0, 0x0048);
            } else {
                this->hci_con_handle = HCI_CON_HANDLE_INVALID;
            }
            break;
        default:
            break;
        }
        break;

    case HCI_EVENT_DISCONNECTION_COMPLETE:
        /* Log handling */
        this->hci_con_handle = HCI_CON_HANDLE_INVALID;
        this->characteristics = CustomCharacteristics{};
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
    GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION &&
    (this->hci_con_handle != HCI_CON_HANDLE_INVALID)) {
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
    return this->setPulseValueSet(
        PulseValueSet {
            .timestemp = timestemp,
            .cun = cun,
            .guan = guan,
            .chi = chi
        }
    );
}

void GattServer::registerActionCallback(actionCallback_t callback, void* context) noexcept {
    this->action_callback = callback;
    this->action_callback_context = context;
}

void GattServer::registerPressureBaseValueCallback(pressureBaseValueCallback_t callback, void* context) noexcept {
    this->pressure_base_value_callback = callback;
    this->pressure_base_value_context = context;
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
    case Att::Handle::CustomCharacteristic::Action::kUserDescription:
        return att_read_callback_handle_blob(
            reinterpret_cast<uint8_t const*>(CustomCharacteristics::action_description.data()),
            CustomCharacteristics::action_description.size(),
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
        if (this->action_callback) {
            auto action = this->characteristics.getAction();
            this->action_callback(this->action_callback_context, action);
        }
        break;
        
    case Att::Handle::CustomCharacteristic::PressureBaseValue::kValue:
        std::memcpy(
            this->characteristics.getPressureBaseValueArray().data(),
            buffer,
            std::min(static_cast<std::size_t>(buffer_size), this->characteristics.getPressureBaseValueArray().size())
        );
        if (this->pressure_base_value_callback) {
            auto pressure_base_value = this->characteristics.getPressureBaseValue();
            this->pressure_base_value_callback(this->pressure_base_value_context, pressure_base_value);
        }
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