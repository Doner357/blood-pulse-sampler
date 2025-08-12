cmake_minimum_required(VERSION 3.11)

add_library(bps_pneumatic STATIC
    "${CMAKE_CURRENT_LIST_DIR}/pneumatic.cpp"
)

target_include_directories(bps_pneumatic
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}"
)

target_link_libraries(bps_pneumatic
    PRIVATE
        compile_options
    PUBLIC
        pico_stdlib
        pico_time
        hardware_i2c
        bps_common
)