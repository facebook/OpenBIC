# SPDX-License-Identifier: Apache-2.0
#
# Full board port: builds real common/ subsystems (hal_wdt, hal_i2c,
# lib/timer, lib/libutil, sensor framework, FRU/eeprom, IPMI/IPMB)
# ported to mainline Zephyr's API surface, in place of the bring-up
# branch's from-scratch stand-ins. See README.md for exactly what was
# ported, what's stubbed, and why.

cmake_minimum_required(VERSION 3.20.0)

# DMTF libspdm (SPDM attestation responder, see src/plat_spdm.c) is
# pulled in as an extra Zephyr module so its zephyr_library() registers
# before find_package(Zephyr) finalises ZEPHYR_LIBS. Must precede it.
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/libspdm)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mcx-n9xx-evk)

set(common_path ${CMAKE_CURRENT_SOURCE_DIR}/../../common)

target_sources(app PRIVATE
  src/main.c
  src/plat_gpio.c
  src/plat_hwinfo.c
  src/plat_mbox.c
  src/plat_shell.c
  src/plat_storage.c
  src/plat_sensor_table.c
  src/plat_sdr_table.c
  src/plat_fru.c
  src/plat_ipmb.c
  src/plat_mctp.c
  src/plat_pldm_monitor.c
  src/plat_spdm.c
  src/plat_spdm_platform.c
  src/plat_stubs.c
  ${common_path}/lib/timer.c
  ${common_path}/lib/libutil.c
  ${common_path}/lib/util_pmbus.c
  ${common_path}/hal/hal_wdt.c
  ${common_path}/hal/hal_i2c.c
  ${common_path}/service/sensor/sensor.c
  ${common_path}/service/sensor/sdr.c
  ${common_path}/service/mctp/mctp.c
  ${common_path}/service/mctp/mctp_ctrl.c
  ${common_path}/service/mctp/mctp_smbus.c
  ${common_path}/service/pldm/pldm.c
  ${common_path}/service/pldm/pldm_base.c
  ${common_path}/service/pldm/pldm_oem.c
  ${common_path}/service/pldm/pldm_smbios.c
  ${common_path}/service/pldm/pldm_firmware_update.c
  ${common_path}/service/ipmb/ipmb.c
  ${common_path}/service/ipmi/ipmi.c
  ${common_path}/service/ipmi/app_handler.c
  ${common_path}/service/ipmi/chassis_handler.c
  ${common_path}/service/ipmi/sensor_handler.c
  ${common_path}/service/ipmi/storage_handler.c
  ${common_path}/service/ipmi/oem_handler.c
  ${common_path}/shell/commands/sensor_shell.c
)

# common/service/ipmi/oem_1s_handler.c implements Meta's proprietary
# OEM-1S NetFn (0x38) command set (fan/GPIO/JTAG/PECI/APML etc, all
# x86-host-adjacent BMC/BIC functionality). It unconditionally
# includes hal_gpio.h and hal_peci.h, neither portable to this board
# (see above/README.md), and none of its commands apply to an EVK with
# no host, fans, or PECI. ipmi.c's dispatch table calls
# IPMI_OEM_1S_handler() unconditionally for that NetFn regardless -
# stubbed (CC_INVALID_CMD, the honest "not implemented" answer) in
# src/plat_stubs.c.

# common/service/sensor/sensor.c's driver-dispatch table
# (sensor_drive_tbl) unconditionally references every possible sensor
# chip driver's init/read functions by type, regardless of what any
# given board's (empty, here) sensor table actually configures - every
# OpenBIC board compiles the whole common/dev/*.c glob for this reason.
# Excluded:
# - snoop.c/pcc.c: genuinely Aspeed-only hardware (SNOOP/POST Code
#   Capture peripherals) with no mainline equivalent, and (unlike
#   everything else here) not referenced by sensor_drive_tbl at all -
#   nothing needs them to link.
# - cx7.c/nv_satmc.c/mpro.c/pm8702.c/vistara.c: all need
#   pldm_monitor.c's pldm_platform_monitor_read(), which needs
#   hal_gpio.c - a hand-rolled, Aspeed/NPCM4XX-register-address-
#   specific GPIO driver, not a portable wrapper. Porting it is new
#   low-level driver work for MCXN947's own GPIO registers, out of
#   scope here (see README.md). cx7_init - the only one of these five
#   sensor_drive_tbl actually needs - is stubbed in plat_stubs.c.
file(GLOB dev_sources ${common_path}/dev/*.c)
list(REMOVE_ITEM dev_sources
  ${common_path}/dev/snoop.c
  ${common_path}/dev/pcc.c
  ${common_path}/dev/cx7.c
  ${common_path}/dev/nv_satmc.c
  ${common_path}/dev/mpro.c
  ${common_path}/dev/pm8702.c
  ${common_path}/dev/vistara.c
)
target_sources(app PRIVATE ${dev_sources})

target_include_directories(app PRIVATE
  src
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${common_path}
  ${common_path}/hal
  ${common_path}/lib
  ${common_path}/dev/include
  ${common_path}/service/sensor
  ${common_path}/service/ipmb
  ${common_path}/service/pldm
  ${common_path}/service/mctp
  ${common_path}/service/ipmi/include
  ${common_path}/service/usb
  ${common_path}/shell/commands
)
