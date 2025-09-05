/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <shell/shell.h>
#include <zephyr.h>
#include <stdio.h>
#include <zephyr.h>
#include "util_sys.h"
#include "soc_common.h"

const char *npcm4xx_reset_reason_to_str(uint8_t reason)
{
	switch (reason) {
	case NPCM4XX_RESET_REASON_VCC_POWERUP:
		return "VCC Power-up Reset";
	case NPCM4XX_RESET_REASON_WDT_RST:
		return "Watchdog Reset";
	case NPCM4XX_RESET_REASON_DEBUGGER_RST:
		return "Debugger Reset";
	case NPCM4XX_RESET_REASON_INVALID:
		return "Invalid Reset";
	default:
		return "Unknown Reset Reason";
	}
}

void cmd_plat_get_reset_cause(struct shell *shell, size_t argc, char **argv)
{
	uint8_t reason = npcm4xx_get_reset_reason();
	shell_print(shell, "reset reason[0x%02x]: %s", reason, npcm4xx_reset_reason_to_str(reason));
}

SHELL_CMD_REGISTER(get_reset_cause, NULL, "get reset cause command", cmd_plat_get_reset_cause);
