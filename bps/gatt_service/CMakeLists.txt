cmake_minimum_required(VERSION 3.13)

add_library(bps_gatt STATIC
    "${CMAKE_CURRENT_LIST_DIR}/gatt_database.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/gatt_server.cpp"
)

target_include_directories(bps_gatt PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

pico_btstack_make_gatt_header(bps_gatt PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/gatt_database.gatt"
)

target_link_libraries(bps_gatt PRIVATE
    compile_options
)

target_link_libraries(bps_gatt PUBLIC
    bps_common
    pico_btstack_ble
    pico_btstack_cyw43
    pico_cyw43_arch_none
)