cmake_minimum_required(VERSION 3.11)

add_library(bps_pressure_controller STATIC
    "${CMAKE_CURRENT_LIST_DIR}/psensor.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/pcontroller.cpp"
)

target_include_directories(bps_pressure_controller PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
)

target_link_libraries(bps_pressure_controller
    PRIVATE
        compile_options
    PUBLIC
        bps_common
        pico_time
        pico_stdlib
        hardware_i2c
        hardware_pwm
        freertos_kernel
)