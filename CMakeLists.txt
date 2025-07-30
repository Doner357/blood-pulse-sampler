# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(blood-pulse-sampler C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Fetch RTOS
FetchContent_Declare( freertos_kernel
  GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
  GIT_TAG        V11.2.0 #Note: Best practice to use specific git-hash or tagged version
)

# FreeRTOS Configurations
add_library(freertos_config INTERFACE)

target_include_directories(freertos_config SYSTEM
INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/freertos"
)

target_compile_definitions(freertos_config
INTERFACE
    projCOVERAGE_TEST=0
)

# In case of cross compilation
# target_compile_definitions(freertos_config INTERFACE ${definitions})
# target_compile_options(freertos_config INTERFACE ${options})

# Select TEMPLATE as port since RP2350 has not been add to
# FreeRTOS default provided port
set( FREERTOS_HEAP "4" CACHE STRING "" FORCE)
# Select the native compile PORT
set( FREERTOS_PORT "TEMPLATE" CACHE STRING "" FORCE)
# Select the cross-compile PORT
if (CMAKE_CROSSCOMPILING)
  set(FREERTOS_PORT "TEMPLATE" CACHE STRING "" FORCE)
endif()

FetchContent_MakeAvailable(freertos_kernel)
set(FREERTOS_KERNEL_PATH ${freertos_kernel_SOURCE_DIR})
include(FreeRTOS_Kernel_import.cmake)

# Add bps subdirectory for main library usage
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/bps")

# Add executable. Default name is the project name, version 0.1
add_executable(blood-pulse-sampler blood-pulse-sampler.cpp )

pico_set_program_name(blood-pulse-sampler "blood-pulse-sampler")
pico_set_program_version(blood-pulse-sampler "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(blood-pulse-sampler 0)
pico_enable_stdio_usb(blood-pulse-sampler 0)

# Add the standard library to the build
target_link_libraries(blood-pulse-sampler
        pico_stdlib
)

# Add the standard include files to the build
target_include_directories(blood-pulse-sampler PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(blood-pulse-sampler
        hardware_i2c
        bps_service
)

# Link to compile options interface
target_link_libraries(blood-pulse-sampler
    compile_options
)

pico_add_extra_outputs(blood-pulse-sampler)