# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(yv3-dl)

set(common_path ../../common)
FILE(GLOB app_sources src/ipmi/*.c src/platform/*.c src/lib/*.c)
FILE(GLOB common_sources ${common_path}/service/*.c ${common_path}/service/*/*.c ${common_path}/hal/*.c ${common_path}/dev/*.c ${common_path}/lib/*.c ${common_path}/shell/*.c)

target_sources(app PRIVATE ${app_sources})
target_sources(app PRIVATE ${common_sources})
target_include_directories(app PRIVATE ${ZEPHYR_BASE}/include/portability)
target_include_directories(app PRIVATE ${common_path} ${common_path}/service/ipmi/include ${common_path}/service/host ${common_path}/service/sensor ${common_path}/service/usb ${common_path}/service/ipmb ${common_path}/service/mctp ${common_path}/service/pldm ${common_path}/hal ${common_path}/dev/include ${common_path}/lib ${common_path}/shell)
target_include_directories(app PRIVATE src/ipmi/include src/lib src/platform)

# Fail build if there are any warnings 
target_compile_options(app PRIVATE -Werror)
