#include "gatt_server.hpp"

#include <btstack.h>

#include <cstdint>
#include <cstring>

#include "gatt_database.hpp"

namespace bps::gatt {

GattServer::GattServer() {
    this->service_handler.start_handle = Att::Handle::CustomCharacteristic::kStart;
    this->service_handler.end_handle   = Att::Handle::CustomCharacteristic::kEnd;
    this->service_handler.read_callback  = &GattServer::attReadCallbackTrampoline;
    this->service_handler.write_callback = &GattServer::attWriteCallbackTrampoline;
    att_server_register_service_handler(&this->service_handler);
}

// Setters, only allow to set Writable data
GattServer& GattServer::setMachineStatus(
    MachineStatus const& status
) noexcept {
    this->characteristics.setMachineStatus(status);
    if (this->characteristics.getMachineStatusClientConfiguration() == 
    GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION) {
        btstack_context_callback_registration_t callback_register{};
        callback_register.callback = &GattServer::characteristicMachineStatusCallbackTrampoline;
        callback_register.context = this;
        att_server_register_can_send_now_callback(&callback_register, this->hci_con_handle);
    }
    return *this;
}

GattServer& GattServer::setPulseValueSet(
    PulseValueSet const& value_set
) noexcept {
    this->characteristics.setPulseValueSet(value_set);
    if (this->characteristics.getPulseValueSetClientConfiguration() == 
    GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION) {
        btstack_context_callback_registration_t callback_register{};
        callback_register.callback = &GattServer::characteristicPulseValueSetCallbackTrampoline;
        callback_register.context = this;
        att_server_register_can_send_now_callback(&callback_register, this->hci_con_handle);
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

void GattServer::characteristicMachineStatusCallbackTrampoline([[maybe_unused]] void* context) {
    GattServer& gatt_server = GattServer::getInstance();
    gatt_server.characteristicMachineStatusCallback();
}

void GattServer::characteristicPulseValueSetCallbackTrampoline([[maybe_unused]] void* context) {
    GattServer& gatt_server = GattServer::getInstance();
    gatt_server.characteristicPulseValueSetCallback();
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
        break;

    case Att::Handle::CustomCharacteristic::Action::kUserDescription:
        return att_read_callback_handle_blob(
            reinterpret_cast<uint8_t const*>(CustomCharacteristics::action_description.data()),
            CustomCharacteristics::action_description.size(),
            offset,
            buffer,
            buffer_size
        );
        break;

    case Att::Handle::CustomCharacteristic::PressureBaseValue::kValue:
        return att_read_callback_handle_blob(
            reinterpret_cast<uint8_t const*>(this->characteristics.getPressureBaseValueArray().data()),
            this->characteristics.getPressureBaseValueArray().size(),
            offset,
            buffer,
            buffer_size
        );
        break;

    case Att::Handle::CustomCharacteristic::PressureBaseValue::kUserDescription:
        return att_read_callback_handle_blob(
            reinterpret_cast<uint8_t const*>(CustomCharacteristics::pressure_base_value_description.data()),
            CustomCharacteristics::pressure_base_value_description.size(),
            offset,
            buffer,
            buffer_size
        );
        break;

    case Att::Handle::CustomCharacteristic::MachineStatus::kValue:
        return att_read_callback_handle_blob(
            reinterpret_cast<uint8_t const*>(this->characteristics.getMachineStatusArray().data()),
            this->characteristics.getMachineStatusArray().size(),
            offset,
            buffer,
            buffer_size
        );
        break;

    case Att::Handle::CustomCharacteristic::MachineStatus::kClientConfiguration:
        return att_read_callback_handle_little_endian_16(
            this->characteristics.getMachineStatusClientConfiguration(),
            offset,
            buffer,
            buffer_size
        );
        break;

    case Att::Handle::CustomCharacteristic::MachineStatus::kUserDescription:
        return att_read_callback_handle_blob(
            reinterpret_cast<uint8_t const*>(CustomCharacteristics::machine_status_description.data()),
            CustomCharacteristics::machine_status_description.size(),
            offset,
            buffer,
            buffer_size
        );
        break;

    case Att::Handle::CustomCharacteristic::PulseDataSet::kValue:
        return att_read_callback_handle_blob(
            reinterpret_cast<uint8_t const*>(this->characteristics.getPulseValueSetArray().data()),
            this->characteristics.getPulseValueSetArray().size(),
            offset,
            buffer,
            buffer_size
        );
        break;

    case Att::Handle::CustomCharacteristic::PulseDataSet::kClientConfiguration:
        return att_read_callback_handle_little_endian_16(
            this->characteristics.getPulseValueSetClientConfiguration(),
            offset,
            buffer,
            buffer_size
        );
        break;

    case Att::Handle::CustomCharacteristic::PulseDataSet::kUserDescription:
        return att_read_callback_handle_blob(
            reinterpret_cast<uint8_t const*>(CustomCharacteristics::pulse_value_set_description.data()),
            CustomCharacteristics::pulse_value_set_description.size(),
            offset,
            buffer,
            buffer_size
        );
        break;

    default:
        break;
    }

    return 0;
}

int GattServer::attWriteCallback(
    hci_con_handle_t const& con_handle,
    uint16_t const& attribute_handle,
    [[maybe_unused]] uint16_t const& transaction_mode,
    [[maybe_unused]] uint16_t const& offset,
    unsigned char *buffer,
    [[maybe_unused]] uint16_t const& buffer_size
) noexcept {
    switch (attribute_handle) {
    case Att::Handle::CustomCharacteristic::Action::kValue:
        std::memcpy(this->characteristics.getActionArray().data(), buffer, buffer_size);
        break;
        
    case Att::Handle::CustomCharacteristic::PressureBaseValue::kValue:
        std::memcpy(this->characteristics.getPressureBaseValueArray().data(), buffer, buffer_size);
        break;

    case Att::Handle::CustomCharacteristic::MachineStatus::kClientConfiguration:
        this->characteristics.setMachineStatusClientConfiguration(little_endian_read_16(buffer, 0));
        this->hci_con_handle = con_handle;
        break;
        
    case Att::Handle::CustomCharacteristic::PulseDataSet::kClientConfiguration:
        this->characteristics.setPulseValueSetClientConfiguration(little_endian_read_16(buffer, 0));
        this->hci_con_handle = con_handle;
        break;
        
    default:
        break;
    }

    return 0;
}

void GattServer::characteristicMachineStatusCallback() noexcept {
    att_server_notify(
        this->hci_con_handle,
        Att::Handle::CustomCharacteristic::MachineStatus::kValue,
        reinterpret_cast<uint8_t*>(this->characteristics.getMachineStatusArray().data()),
        this->characteristics.getMachineStatusArray().size()
    );
}

void GattServer::characteristicPulseValueSetCallback() noexcept {
    att_server_notify(
        this->hci_con_handle,
        Att::Handle::CustomCharacteristic::PulseDataSet::kValue,
        reinterpret_cast<uint8_t*>(this->characteristics.getPulseValueSetArray().data()),
        this->characteristics.getPulseValueSetArray().size()
    );
}

} // namespace bps