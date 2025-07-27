#include "gatt_database.hpp"

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <utility>
#include <expected>

#include "utils.hpp"

namespace bps::gatt {

CustomCharacteristics::CustomCharacteristics() : 
    action({std::byte{0}}),
    pressure_base_value({std::byte{0}})
{
    setMachineStatus(MachineStatus::eNull);
    setMachineStatusClientConfiguration(0);
    setPulseValueSet(0.0, 0.0_pa, 0.0_pa, 0.0_pa);
    setPulseValueSetClientConfiguration(0);
}


CustomCharacteristics& CustomCharacteristics::setAction(
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

CustomCharacteristics& CustomCharacteristics::setAction(
    ActionType const& action_type,
    PressureType const& cun,
    PressureType const& guan,
    PressureType const& chi
) noexcept {
    setAction(
        Action {
            .action_type = action_type,
            .cun = cun,
            .guan = guan,
            .chi = chi
        }
    );

    return *this;
}

CustomCharacteristics& CustomCharacteristics::setPressureBaseValue(
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

CustomCharacteristics& CustomCharacteristics::setPressureBaseValue(
    std::float32_t floating,
    std::float32_t middle,
    std::float32_t deep
) noexcept {
    setPressureBaseValue(
        PressureBaseValue {
            .floating = floating,
            .middle = middle,
            .deep = deep
        }
    );
    return *this;
}

CustomCharacteristics& CustomCharacteristics::setMachineStatus(
    MachineStatus const& status
) noexcept {
    this->machine_status[0] = std::byte{std::to_underlying(status)};
    return *this;
}

CustomCharacteristics& CustomCharacteristics::setMachineStatusClientConfiguration(
    std::uint16_t const& configuration
) noexcept {
    this->machine_status_client_configuration = configuration;
    return *this;
}

CustomCharacteristics& CustomCharacteristics::setPulseValueSet(
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

CustomCharacteristics& CustomCharacteristics::setPulseValueSet(
    std::float64_t const& timestemp,
    std::float32_t const& cun,
    std::float32_t const& guan,
    std::float32_t const& chi
) noexcept {
    setPulseValueSet(
        PulseValueSet{
            .timestemp = timestemp,
            .cun       = cun,
            .guan      = guan,
            .chi       = chi
        }
    );
    return *this;
}

CustomCharacteristics& CustomCharacteristics::setPulseValueSetClientConfiguration(
    std::uint16_t configuration
) noexcept {
    this->pulse_value_set_client_configuration = configuration;
    return *this;
}

// Getters
std::expected<Action, Error<std::byte>> CustomCharacteristics::getAction() const noexcept {
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

PressureBaseValue CustomCharacteristics::getPressureBaseValue() const noexcept {
    PressureBaseValue value{};

    std::size_t offset = 0;
    
    readAsNativeEndian(&this->pressure_base_value[offset], value.floating);
    offset += sizeof(value.floating);

    readAsNativeEndian(&this->pressure_base_value[offset], value.middle);
    offset += sizeof(value.middle);

    readAsNativeEndian(&this->pressure_base_value[offset], value.deep);
    
    return value;
}

std::expected<MachineStatus, Error<std::byte>> CustomCharacteristics::getMachineStatus() const noexcept {
    auto status = toMachineStatus(this->machine_status[0]);
    if (!status) {
        return std::unexpected(
            Error{ ErrorType::eInvalidValue, this->machine_status[0] }
        );
    }
    return status.value();
}

std::uint16_t CustomCharacteristics::getMachineStatusClientConfiguration() const noexcept {
    return this->machine_status_client_configuration;
}

PulseValueSet CustomCharacteristics::getPulseValueSet() const noexcept {
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

std::uint16_t CustomCharacteristics::getPulseValueSetClientConfiguration() const noexcept {
    return this->pulse_value_set_client_configuration;
}

} // bps::gatt