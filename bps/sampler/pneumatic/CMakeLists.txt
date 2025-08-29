cmake_minimum_required(VERSION 3.11)

add_library(bps_pneumatic STATIC
    "${CMAKE_CURRENT_LIST_DIR}/psensors.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/pcontroller.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/phandler.cpp"
)

target_include_directories(bps_pneumatic PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
)

target_link_libraries(bps_pneumatic
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