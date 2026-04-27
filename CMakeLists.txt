# =============================================================================
# KLAVIATOR V4.0 - CMake Build File
# =============================================================================
cmake_minimum_required(VERSION 3.20.0)

# Set board here or pass via: west build -b nrf54l15dk/nrf54l15/cpuapp
# set(BOARD nrf54l15dk/nrf54l15/cpuapp)

set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/app.overlay")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(klaviator
    VERSION     4.0.0
    DESCRIPTION "Klaviator PCA9685 16-solenoid piano controller"
    LANGUAGES   C
)

# Select which main file to compile:
# - src/main_led_test.c     : Basic LED wiring confirmation (simple on/off)
# - src/main.c              : Solenoid pattern test (kick/hold/chromatic/chords)
#                             Test on LEDs first, then use with real solenoids
# - src/main_simple_test.c  : Other test

target_sources(app PRIVATE
    src/main.c
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_options(app PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
)
