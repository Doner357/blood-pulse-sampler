#ifndef TRADITIONAL_CHINESE_MEDICINE_MODEL
#define TRADITIONAL_CHINESE_MEDICINE_MODEL

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <optional>
#include <memory_resource>
#include <concepts>
#include <utility>
#include <stdfloat>
#include <bit>
#include <algorithm>

// UDL for 'pa' unit, return 32-bits float
consteval std::float32_t operator""_pa(long double pa) {
    return static_cast<std::float32_t>(pa);
}

namespace bps {

// Simple Error enum
enum class ErrorType {
    eInvalidValue,
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

// Type of Action
enum class ActionType : std::uint8_t {
    eNull          = 0X00,
    eStopSampling  = 0x40,
    eStartSampling = 0x50
};
// Helper function, convert each byte type value to ActionType enum class
// Return std::nullopt optional if there is no matched enum
inline std::optional<ActionType> toActionType(ByteTypes auto value) noexcept {
    auto const enum_value = static_cast<std::underlying_type<ActionType>::type>(value);
    switch (enum_value) {
    case std::to_underlying(ActionType::eStartSampling):
        return ActionType::eStartSampling;
    case std::to_underlying(ActionType::eStopSampling):
        return ActionType::eStopSampling;
    default:
        return std::nullopt;
    }
}

// Represent Sampler machine status
enum class MachineStatus : std::uint8_t {
    eNull     = 0x00,
    eIdle     = 0x01,
    eSampling = 0x02
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

// Hold action the machine should do
struct Action {
    ActionType action_type = ActionType::eNull;
    PressureType cun  = PressureType::eNull;
    PressureType guan = PressureType::eNull;
    PressureType chi  = PressureType::eNull;
};

// Hold base value of pressure type
struct PressureBaseValue {
    std::float32_t floating = 0.0_pa;
    std::float32_t middle   = 0.0_pa;
    std::float32_t deep     = 0.0_pa;
};

// Pack one pulse sample information
struct PulseValueSet {
    std::float64_t timestemp = 0.0;
    std::float32_t cun  = 0.0_pa;
    std::float32_t guan = 0.0_pa;
    std::float32_t chi  = 0.0_pa;
};

// Helper function to generate std::byte array
template<typename... Ts>
constexpr std::array<std::byte, sizeof...(Ts)> make_bytes(Ts&&... args) noexcept {
    return{std::byte(std::forward<Ts>(args))...};
}

// Write given value into given std::byte array with little endian order
template<typename T>
void writeAsLittleEndian(T const& value, std::byte* dest) {
    std::array<std::byte, sizeof(T)> bytes;
    std::memcpy(bytes.data(), &value, sizeof(T));
    // If the system is big endian, reverse the order
    if constexpr (std::endian::native == std::endian::big) {
        std::ranges::reverse(bytes);
    }
    std::ranges::copy(bytes, dest);
}

// Write given std::byte array into given value with system native endian order
template<typename T>
void readAsNativeEndian(std::byte const* src, T& dest) {
    std::array<std::byte, sizeof(T)> bytes;
    std::memcpy(bytes.data(), src, sizeof(T));
    // From little endian (gatt default) convert to system order
    if constexpr (std::endian::native == std::endian::big) {
        std::ranges::reverse(bytes);
    }
    std::memcpy(&dest, bytes.data(), sizeof(T));
}

} // namespace bps

#endif // TRADITIONAL_CHINESE_MEDICINE_MODEL