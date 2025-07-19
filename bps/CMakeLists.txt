cmake_minimum_required(VERSION 3.13)

add_library(bps-utils STATIC
    "${CMAKE_CURRENT_LIST_DIR}/gatt_service/gatt_database.cpp"
)

target_include_directories(bps-utils PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/gatt_service
)

pico_btstack_make_gatt_header(bps-utils PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/gatt_service/gatt_database.gatt"
)

target_link_libraries(bps-utils PRIVATE
    compile-options
)