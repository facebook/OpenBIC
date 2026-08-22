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

set(REMOTE_APP remote)

ExternalZephyrProject_Add(
  APPLICATION ${REMOTE_APP}
  SOURCE_DIR  ${APP_DIR}/${REMOTE_APP}
  BOARD       ${SB_CONFIG_REMOTE_BOARD}
  BOARD_REVISION ${BOARD_REVISION}
)

add_dependencies(${DEFAULT_IMAGE} ${REMOTE_APP})
