cmake_minimum_required(VERSION 3.13)

# Declare main linking target interface
add_library(bps-service INTERFACE)

# --- Gatt ---
add_library(bps-gatt STATIC
    "${CMAKE_CURRENT_LIST_DIR}/gatt_service/gatt_database.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/gatt_service/gatt_server.cpp"
)

target_include_directories(bps-gatt PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/gatt_service
)

pico_btstack_make_gatt_header(bps-gatt PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/gatt_service/gatt_database.gatt"
)

target_link_libraries(bps-gatt PRIVATE
    compile-options
)

target_link_libraries(bps-gatt PUBLIC
    compile-options
    pico_btstack_ble
    pico_btstack_cyw43
    pico_cyw43_arch_none
)

# Main target
target_link_libraries(bps-service INTERFACE
    bps-gatt
)