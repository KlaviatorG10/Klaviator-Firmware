# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hello_world)

# Select which main file to build:
# - main_led_test.c     : Simple LED on/off test with PCA9685
# - main.c              : Full solenoid + motor + UART firmware
# - main_simple_test.c  : Other test

target_sources(app PRIVATE main_led_test.c)
