cmake_minimum_required(VERSION 3.13)

# Custom compile options
add_library(compile_options INTERFACE)
target_compile_options(compile_options INTERFACE
    $<$<CONFIG:Debug>:$<BUILD_INTERFACE:-Wall -Wextra -Wshadow -Wformat=2 -Wunused>>
)

# Declare main linking target interface
add_library(bps_common INTERFACE)

target_include_directories(bps_common
INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

# --- Gatt ---
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/gatt_service")

# Declare main linking target interface
add_library(bps_service INTERFACE)

# Main target
target_link_libraries(bps_service INTERFACE
    bps_gatt
)