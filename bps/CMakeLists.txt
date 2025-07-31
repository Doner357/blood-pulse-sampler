cmake_minimum_required(VERSION 3.13)

# Declare main linking target interface
add_library(bps_common INTERFACE)

target_include_directories(bps_common
INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/ble_service")

add_library(bps_service STATIC)
target_link_libraries(bps_service
    PUBLIC
        bps_ble_service
)