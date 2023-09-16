# Minimum CMake version required
cmake_minimum_required(VERSION 3.13)

# Project name and language
project(pico C CXX ASM)

# Set the Raspberry Pi Pico SDK path
set(PICO_SDK_PATH ~/pico/pico-sdk)

# Initialize the Raspberry Pi Pico SDK
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

# Add your source file(s) here
add_executable(pico
    pico.cpp
)

# Include Raspberry Pi Pico SDK directories
target_include_directories(pico PRIVATE ${PICO_SDK_PATH}/src/rp2_common/pico_stdlib/include)

# Link Raspberry Pi Pico SDK libraries
target_link_libraries(pico PRIVATE pico_stdlib)

# Add the Raspberry Pi Pico SDK as a subdirectory
add_subdirectory(${PICO_SDK_PATH} pico_sdk)

# Enable CMake to use the Raspberry Pi Pico toolchain
pico_set_program_name(pico "pico")
pico_set_program_version(pico "1.0")

# Include the default project configuration

# Enable the Raspberry Pi Pico SDK FreeRTOS support (optional, comment if not needed)
# pico_enable_freeduino(pico 0)

# Enable SPI, I2C, and UART support (optional, add if needed)
# target_link_libraries(pico PRIVATE pico_multicore)

# Include the Raspberry Pi Pico SDK FreeRTOS (optional, add if needed)
# target_link_libraries(pico PRIVATE pico_multicore)

# Create map/bin/hex/uf2 file etc. (removed this line)
# pico_add_extra_outputs(pico)
