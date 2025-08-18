cmake_minimum_required(VERSION 3.11)

add_library(bps_pressure_sensors STATIC
    "${CMAKE_CURRENT_LIST_DIR}/psensors.cpp"
)

target_include_directories(bps_pressure_sensors PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
)

target_link_libraries(bps_pressure_sensors
    PRIVATE
        compile_options
    PUBLIC
        pico_stdlib
        pico_time
        hardware_i2c
        bps_common
        freertos_kernel
)