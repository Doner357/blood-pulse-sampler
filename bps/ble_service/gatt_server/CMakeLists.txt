cmake_minimum_required(VERSION 3.13)

add_library(bps_gatt_server
    STATIC
        "${CMAKE_CURRENT_LIST_DIR}/gatt_server.cpp"
)

target_include_directories(bps_gatt_server
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
)

# Use btstack python compiler to generate btstack_database.h
pico_btstack_make_gatt_header(bps_gatt_server
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/gatt_database.gatt"
)

target_link_libraries(bps_gatt_server
    PRIVATE
        compile_options
    PUBLIC
        bps_common
        pico_async_context_freertos
        pico_cyw43_arch_none
        pico_btstack_cyw43
        pico_btstack_ble
)
