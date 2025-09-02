cmake_minimum_required(VERSION 3.11)

add_library(bps_logger INTERFACE)

target_include_directories(bps_logger
    INTERFACE
        "${CMAKE_CURRENT_LIST_DIR}"   
)

target_link_libraries(bps_logger
    INTERFACE
        pico_stdlib
)