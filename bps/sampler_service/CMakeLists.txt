cmake_minimum_required(VERSION 3.11)

add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/pneumatic")

add_library(bps_sampler STATIC
    "${CMAKE_CURRENT_LIST_DIR}/sampler_service.cpp"
)

target_include_directories(bps_sampler
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}"
)

target_link_libraries(bps_sampler
    PRIVATE
        compile_options
    PUBLIC
        bps_logger
        bps_pneumatic
)