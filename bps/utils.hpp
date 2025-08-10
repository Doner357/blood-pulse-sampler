#ifndef BPS_UTILS_HPP
#define BPS_UTILS_HPP

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <concepts>
#include <bit>
#include <algorithm>
#include <concepts>

namespace bps {

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

} // namespace bps

#endif // BPS_UTILS_HPP