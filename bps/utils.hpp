#ifndef BPS_UTILS_HPP
#define BPS_UTILS_HPP

#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>

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
#include <concepts>

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
constexpr std::array<std::byte, sizeof...(Ts)> makeBytes(Ts&&... args) noexcept {
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

template<typename T, UBaseType_t Length>
class StaticQueue {
    public:
        using ContentType = T;
        // Constructor
        StaticQueue() noexcept {
            this->queue_handle = xQueueCreateStatic(
                Length,
                sizeof(T),
                reinterpret_cast<uint8_t*>(buffer.data()),
                &this->static_queue_cb
            );
            configASSERT(this->queue_handle != nullptr);
        }

        // Destructor to release the queue handle
        ~StaticQueue() {
            if (this->queue_handle != nullptr) {
                vQueueDelete(this->queue_handle);
            }
        }

        StaticQueue(StaticQueue const&) = delete;
        StaticQueue& operator=(StaticQueue const&) = delete;
        StaticQueue(StaticQueue&& other) = delete;
        StaticQueue& operator=(StaticQueue&& other) = delete;

        // --- API Methods ---

        // Send by copying a const reference
        bool send(T const& object, TickType_t wait_ms) noexcept {
            if (
                this->queue_handle == nullptr || 
                xQueueSend(
                    this->queue_handle, 
                    reinterpret_cast<void const*>(&object), 
                    wait_ms
                ) != pdPASS
            ) return false;
            return true;
        }

        bool sendFromIsr(T const& object, BaseType_t* higher_priority_task_to_woken) noexcept {
            if (
                this->queue_handle == nullptr ||
                xQueueSendFromISR(
                    this->queue_handle,
                    reinterpret_cast<void const*>(&object),
                    higher_priority_task_to_woken
                ) != pdPASS
            ) return false;
            return true;
        }

        // Receive an object
        bool receive(T& receive_buffer, TickType_t wait_ms) noexcept {
            if (
                this->queue_handle == nullptr || 
                xQueueReceive(
                    this->queue_handle,
                    reinterpret_cast<void*>(&receive_buffer),
                    pdTICKS_TO_MS(wait_ms)
                ) != pdPASS
            ) return false;
            return true;
        }

        // Get the raw FreeRTOS queue handle
        QueueHandle_t getFreeRTOSQueueHandle() const noexcept {
            return this->queue_handle;
        }

        // Helper to check if the queue was created successfully
        bool isValid() const noexcept {
            return this->queue_handle != nullptr;
        }

        // Get the length of the queue
        static constexpr UBaseType_t length() {
            return Length;
        }

        UBaseType_t size() const noexcept {
            return uxQueueMessagesWaiting(this->queue_handle);
        }

    private:
        std::array<std::byte, Length * sizeof(T)> buffer{};
        StaticQueue_t static_queue_cb{};
        QueueHandle_t queue_handle{nullptr};
};

} // namespace bps

#endif // BPS_UTILS_HPP