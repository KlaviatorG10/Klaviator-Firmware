# =============================================================================
# KLAVIATOR V4.0 - CMake Build File
# =============================================================================
cmake_minimum_required(VERSION 3.20.0)

set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/app.overlay")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(klaviator
    VERSION     4.0.0
    DESCRIPTION "Klaviator PCA9685 16-solenoid piano controller and DM860E"
    LANGUAGES   C
)

target_sources(app PRIVATE src/main.c)

target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_compile_options(app PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
)
