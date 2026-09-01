# SPDX-License-Identifier: Apache-2.0
#
# Orchestrates the optional dual-core build: builds remote/ (cpu1) as a
# second Zephyr image alongside this app (cpu0), only when invoked with
# `west build --sysbuild`. Mirrors the pattern used by Zephyr's own
# samples/drivers/mbox sample for this exact board.

if("${SB_CONFIG_REMOTE_BOARD}" STREQUAL "")
  message(FATAL_ERROR "Target ${BOARD}/${BOARD_QUALIFIERS} has no remote board configured "
    "in Kconfig.sysbuild - dual-core build is only wired up for mcx_n9xx_evk/mcxn947/cpu0.")
endif()

# cpu0's SoC init (soc/nxp/mcx/mcxn/soc.c: second_core_boot(), a
# PRE_KERNEL_2 SYS_INIT) sets cpu1's boot address to slot1_partition and
# releases cpu1 from reset. That must happen ONLY when a cpu1 image
# actually exists there - i.e. this dual-core sysbuild. Enabling it via
# the plain cpu0 boards/*.conf instead released cpu1 into an empty
# slot1 on every single-core build too: cpu1 wild-branches through
# blank flash and wedges the system (dead / garbled console) before
# cpu0 finishes booting. So it is set here, scoped to sysbuild.
set_config_bool(${DEFAULT_IMAGE} CONFIG_SECOND_CORE_MCUX y)

set(REMOTE_APP remote)

ExternalZephyrProject_Add(
  APPLICATION ${REMOTE_APP}
  SOURCE_DIR  ${APP_DIR}/${REMOTE_APP}
  BOARD       ${SB_CONFIG_REMOTE_BOARD}
  BOARD_REVISION ${BOARD_REVISION}
)

add_dependencies(${DEFAULT_IMAGE} ${REMOTE_APP})
