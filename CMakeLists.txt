# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Force the flash runner to be nrfjprog instead of nrfutil
set(ZEPHYR_BOARD_FLASH_RUNNER nrfjprog)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hello_world)

target_sources(app PRIVATE src/main.c)
