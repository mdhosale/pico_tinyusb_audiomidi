cmake_minimum_required(VERSION 3.13)

# Include build functions from Pico SDK
include(pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C++ standards
project(pacis_source_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_compile_definitions(
#    PICO_HEAP_SIZE=2048
    PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64
)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

project(pico_tinyusb_audiomidi_project)

add_subdirectory("src")
