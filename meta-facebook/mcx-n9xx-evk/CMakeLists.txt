# SPDX-License-Identifier: Apache-2.0
#
# Minimal bring-up target: this intentionally does NOT glob common/service,
# common/hal, or common/dev like other meta-facebook boards do. Those trees
# are written against a pre-Zephyr-3.0 API surface (<zephyr.h>, legacy
# drivers/i2c.h, CONFIG_GPIO_ASPEED/CONFIG_GPIO_NPCM4XX gating, etc.) that
# does not exist in mainline zephyrproject-rtos/zephyr, which is what
# actually carries NXP MCXN947 board support. Porting common/ to build
# against mainline Zephyr is tracked as follow-up "full bring-up" work -
# see meta-facebook/mcx-n9xx-evk/README.md.
#
# This target only proves the OpenBIC repo/build/west pipeline produces a
# bootable image for this board and reaches an interactive shell.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mcx-n9xx-evk)

target_sources(app PRIVATE src/main.c)
target_include_directories(app PRIVATE src)
