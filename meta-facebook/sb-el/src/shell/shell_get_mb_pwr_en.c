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
#include <stdlib.h>
#include <logging/log.h>
#include "plat_cpld.h"

LOG_MODULE_REGISTER(plat_get_mb_pwr_en_shell, LOG_LEVEL_DBG);

static int cmd_get_mb_pwr_en(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t data = 0;

	if (!plat_read_cpld(VR_EN_PIN_READING_5, &data, 1)) {
		shell_error(shell, "read CPLD fail 0x%x", VR_EN_PIN_READING_5);
		return -1;
	}

	shell_info(shell, "MB PWR_EN : %d", data & BIT(0));

	return 0;
}

/* Root of command */
SHELL_CMD_REGISTER(get_mb_pwr_en, NULL, "get MB PWR EN status", cmd_get_mb_pwr_en);
