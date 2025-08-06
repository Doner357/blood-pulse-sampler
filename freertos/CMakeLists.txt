cmake_minimum_required(VERSION 3.11)

set(FREERTOS_KERNEL_PATH "${CMAKE_CURRENT_LIST_DIR}/freertos-kernel")
# Pico-provided freertos import file
include(FreeRTOS_Kernel_import.cmake)

add_library(freertos_kernel INTERFACE)

target_include_directories(freertos_kernel
    INTERFACE
        "${CMAKE_CURRENT_LIST_DIR}"
)

target_link_libraries(freertos_kernel
    INTERFACE
        FreeRTOS-Kernel-Static
        FreeRTOS-Kernel-Heap4
)