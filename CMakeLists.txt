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

# ═══════════════════════════════════════════════════════════════════════════
# MAIN FIRMWARE SELECTION
# ═══════════════════════════════════════════════════════════════════════════
#
# PRIMARY FIRMWARE (solenoid controller):
#   src/main.c              - ⭐ PRODUCTION SOLENOID CONTROLLER ⭐
#                             - 16-channel PCA9685 control
#                             - Kick-hold phases
#                             - Velocity mapping
#                             - Tested & working on LEDs
#                             - Ready for real solenoids
#
# TEST/UTILITY FIRMWARE:
#   src/test_led_basic.c    - Simple LED wiring test for troubleshooting
#
# ═══════════════════════════════════════════════════════════════════════════

target_sources(app PRIVATE
    src/main.c              # ← ACTIVE: Production solenoid controller
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_options(app PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
)
