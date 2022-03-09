# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(yv35-cl)

set(common_path ../../common)
FILE(GLOB app_sources src/*.c src/ipmi/*.c src/sensor/*.c src/sensor/dev/*.c src/platform/*.c )
FILE(GLOB common_sources ${common_path}/*.c ${common_path}/ipmi/*.c ${common_path}/host/*.c ${common_path}/sensor/*.c ${common_path}/util/*.c ${common_path}/usb/*.c ${common_path}/sensor/dev/*.c) 
target_sources(app PRIVATE ${app_sources})
target_sources(app PRIVATE ${common_sources})
target_include_directories(app PRIVATE ${ZEPHYR_BASE}/include/portability)
target_include_directories(app PRIVATE ${common_path} ${common_path}/ipmi/include ${common_path}/host ${common_path}/sensor ${common_path}/sensor/includes ${common_path}/util ${common_path}/usb)
target_include_directories(app PRIVATE src/ipmi/include src/platform src/sensor)
