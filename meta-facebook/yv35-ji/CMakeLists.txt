# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(yv35-ji)

set(common_path ../../common)
FILE(GLOB app_sources src/ipmi/*.c src/platform/*.c src/lib/*.c)
FILE(GLOB common_sources ${common_path}/service/*.c ${common_path}/service/*/*.c ${common_path}/hal/*.c ${common_path}/dev/*.c ${common_path}/shell/*.c ${common_path}/shell/commands/*.c)

target_sources(app PRIVATE ${app_sources})
target_sources(app PRIVATE ${common_sources})

# Common Lib
target_sources(app PRIVATE ${common_path}/lib/libutil.c)
target_sources(app PRIVATE ${common_path}/lib/power_status.c)
target_sources(app PRIVATE ${common_path}/lib/timer.c)
target_sources(app PRIVATE ${common_path}/lib/util_pmbus.c)
target_sources(app PRIVATE ${common_path}/lib/util_spi.c)
target_sources(app PRIVATE ${common_path}/lib/util_sys.c)
target_sources(app PRIVATE ${common_path}/lib/util_worker.c)

target_include_directories(app PRIVATE ${ZEPHYR_BASE}/include/portability)

# Common Includes
target_include_directories(app PRIVATE ${common_path}) 
target_include_directories(app PRIVATE ${common_path}/include)
target_include_directories(app PRIVATE ${common_path}/dev/include) 
target_include_directories(app PRIVATE ${common_path}/hal) 
target_include_directories(app PRIVATE ${common_path}/lib) 
target_include_directories(app PRIVATE ${common_path}/service/host)
target_include_directories(app PRIVATE ${common_path}/service/ipmb) 
target_include_directories(app PRIVATE ${common_path}/service/ipmi/include) 
target_include_directories(app PRIVATE ${common_path}/service/logging) 
target_include_directories(app PRIVATE ${common_path}/service/mctp) 
target_include_directories(app PRIVATE ${common_path}/service/pldm) 
target_include_directories(app PRIVATE ${common_path}/service/sensor) 
target_include_directories(app PRIVATE ${common_path}/service/usb) 
target_include_directories(app PRIVATE ${common_path}/shell/commands)
target_include_directories(app PRIVATE ${common_path}/shell)

# Platform Specific Includes
target_include_directories(app PRIVATE src/ipmi/include)
target_include_directories(app PRIVATE src/lib)
target_include_directories(app PRIVATE src/platform)

# Fail build if there are any warnings 
target_compile_options(app PRIVATE -Werror)
