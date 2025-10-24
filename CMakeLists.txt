# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(BOARD skoll)

list(APPEND EXTRA_ZEPHYR_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/ext/modules/ms5611
    ${CMAKE_CURRENT_SOURCE_DIR}/ext/modules/icm20948
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(skoll LANGUAGES C)

zephyr_include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_sources(app PRIVATE 
    src/main.c
)

