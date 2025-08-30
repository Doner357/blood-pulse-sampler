#ifndef BPS_COMMON_HPP
#define BPS_COMMON_HPP

#include <utility>
#include <concepts>
#include <cstdint>
#include <cstddef>
#include <stdfloat>
#include <optional>

#include "queue.hpp"

// UDL for 'pa' unit, return 32-bits float
consteval std::float32_t operator""_pa(long double pa) {
    return static_cast<std::float32_t>(pa);
}

namespace bps {
    
// Simple Error enum
enum class ErrorType {
    eInvalidValue,
    eFailedOperation
};

// Template which holds the error value
template <typename ValueType>
struct Error {
    ErrorType type;
    ValueType value;
};

// Treat each type with a size of 1 byte as a byte type
template<typename T>
concept ByteTypes = (sizeof(T) == 1u);

// Type of Command
enum class CommandType : std::uint8_t {
    eNull          = 0X00,
    eStopSampling  = 0x01,
    eStartSampling = 0x02,
    eSetPressure   = 0x03
};
// Helper function, convert each byte type value to CommandType enum class
// Return std::nullopt optional if there is no matched enum
inline std::optional<CommandType> toCommandType(ByteTypes auto value) noexcept {
    auto const enum_value = static_cast<std::underlying_type<CommandType>::type>(value);
    switch (enum_value) {
    case std::to_underlying(CommandType::eStartSampling):
        return CommandType::eStartSampling;
    case std::to_underlying(CommandType::eStopSampling):
        return CommandType::eStopSampling;
    case std::to_underlying(CommandType::eSetPressure):
        return CommandType::eSetPressure;
    default:
        return std::nullopt;
    }
}

// Represent Sampler machine status
enum class MachineStatus : std::uint8_t {
    eNull            = 0x00,
    eIdle            = 0x01,
    eSampling        = 0x02,
    eSettingPressure = 0x03
};
// Helper function, convert each byte type value to MachineStatus enum class
// Return std::nullopt optional if there is no matched enum
inline std::optional<MachineStatus> toMachineStatus(ByteTypes auto value) noexcept {
    auto const enum_value = static_cast<std::underlying_type<MachineStatus>::type>(value);
    switch (enum_value) {
    case std::to_underlying(MachineStatus::eIdle):
        return MachineStatus::eIdle;
    case std::to_underlying(MachineStatus::eSampling):
        return MachineStatus::eSampling;
    case std::to_underlying(MachineStatus::eSettingPressure):
        return MachineStatus::eSettingPressure;
    default:
        return std::nullopt;
    }
}

// Represent three measured positions on hand
enum class Position : std::uint8_t {
    eNull = 0x00,
    eCun,
    eGuan,
    eChi
};
// Helper function, convert each byte type value to Position enum class
// Return std::nullopt optional if there is no matched enum
inline std::optional<Position> toPosition(ByteTypes auto value) noexcept {
    auto const enum_value = static_cast<std::underlying_type<Position>::type>(value);
    switch (enum_value) {
    case std::to_underlying(Position::eCun):
        return Position::eCun;
    case std::to_underlying(Position::eGuan):
        return Position::eGuan;
    case std::to_underlying(Position::eChi):
        return Position::eChi;
    default:
        return std::nullopt;
    }
}

// Three type of pressure
enum class PressureType : std::uint8_t {
    eNull   = 0x00,
    eFloat  = 0x01,
    eMiddle = 0x02,
    eDeep   = 0x03
};
// Helper function, convert each byte type value to PressureType enum class
// Return std::nullopt optional if there is no matched enum
inline std::optional<PressureType> toPressureType(ByteTypes auto value) noexcept {
    auto const enum_value = static_cast<std::underlying_type<PressureType>::type>(value);
    switch (enum_value) {
    case std::to_underlying(PressureType::eFloat):
        return PressureType::eFloat;
    case std::to_underlying(PressureType::eMiddle):
        return PressureType::eMiddle;
    case std::to_underlying(PressureType::eDeep):
        return PressureType::eDeep;
    default:
        return std::nullopt;
    }
}

// Hold Common the machine should do
struct Command {
    CommandType command_type = CommandType::eNull;
    union Content {
        // For eStartSampling command
        std::uint64_t sample_time_ms;
        // For eSetPressure command
        struct PressureInfo {
            std::float32_t cun;
            std::float32_t guan;
            std::float32_t chi;
        } pressure_settings;
    } content;
};

// Pack one pulse sample information
struct PulseValue {
    std::uint64_t  timestamp = 0;
    std::float32_t cun  = 0.0_pa;
    std::float32_t guan = 0.0_pa;
    std::float32_t chi  = 0.0_pa;
};

} // namespace bps

#endif // BPS_COMMON_HPP