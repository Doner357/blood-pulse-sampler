#ifndef BPS_LOGGER_HPP
#define BPS_LOGGER_HPP

// Only include headers and define code if not in release mode (NDEBUG is not defined)
#ifndef NDEBUG
#include <pico/stdio.h> // Pico's standard I/O library
#include <cstdio>
#include <cstdarg>      // For va_list, va_start, vprintf, va_end
#endif

namespace bps::logger {

inline bool initializeLogger() noexcept {
#ifndef NDEBUG
    return stdio_init_all();
#else
    return false
#endif
}

// --- Low-level logging implementation function (for internal use) ---
// This is an inline function defined in the header, suitable for C++.
// It takes file, line, and function context, followed by a format string and variadic args.
inline void log_impl(const char* file, int line, const char* func, const char* fmt, ...) noexcept {
#ifndef NDEBUG
    static bool has_initialize = false;
    if (!has_initialize) {
        stdio_init_all();
        has_initialize = true;
    }
    // 1. Print the prefix with file, line, and function info.
    //    For Pico, printf() typically outputs to UART or USB serial.
    std::printf("[%s:%d] [%s] ", file, line, func);

    // 2. Process the variadic arguments using vprintf.
    va_list args;
    va_start(args, fmt);
    std::vprintf(fmt, args);
    va_end(args);

    // 3. Print a newline for better readability.
    std::printf("\n");
#endif // NDEBUG
}

} // namespace bps::logger

// --- The user-facing macro ---
// This macro is the intended way to call the logger.
// It automatically captures the call site's context.
// ##__VA_ARGS__ handles cases where no optional arguments are provided.
#define BPS_LOG(fmt, ...) bps::logger::log_impl(__FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)

#endif // BPS_LOGGER_HPP