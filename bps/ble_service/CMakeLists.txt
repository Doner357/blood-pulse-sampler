cmake_minimum_required(VERSION 3.13)

add_subdirectory(gatt_server)

add_library(bps_ble_service STATIC
    ${CMAKE_CURRENT_LIST_DIR}/ble_service.cpp
)

target_include_directories(bps_ble_service
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}"
)

target_link_libraries(bps_ble_service
    PRIVATE
        compile_options
    PUBLIC
        bps_common
        bps_logger
        bps_gatt_server
)