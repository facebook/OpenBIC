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
#include "plat_isr.h"
#include "plat_i2c.h"

#define AEGIS_CPLD_ADDR (0x4C >> 1)
#define AEGIS_BOARD_POWER_ENABLE 0xFF
#define AEGIS_BOARD_POWER_DISABLE 0xEF

LOG_MODULE_REGISTER(plat_soc_pwron_reset_shell, LOG_LEVEL_DBG);

static int cmd_soc_pwron_reset_set(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t data = 0;
	if (!strcmp(argv[0], "override")) {
		if (!strcmp(argv[1], "1")) {
			data = AEGIS_BOARD_POWER_ENABLE;
			if (!plat_i2c_write(I2C_BUS5, AEGIS_CPLD_ADDR, 0x00, &data, 1)) {
				shell_error(shell, "plat soc_pwron_reset set failed");
				return -1;
			}
		} else if (!strcmp(argv[1], "0")) {
			data = AEGIS_BOARD_POWER_DISABLE;
			if (!plat_i2c_write(I2C_BUS5, AEGIS_CPLD_ADDR, 0x00, &data, 1)) {
				shell_error(shell, "plat soc_pwron_reset set failed");
				return -1;
			}
		} else {
			shell_warn(
				shell,
				"Help: soc_pwron_reset override <drive-level> should only accept 0 or 1");
			return -1;
		}
	} else if (!strcmp(argv[0], "passthru")) {
		data = AEGIS_BOARD_POWER_ENABLE;
		if (!plat_i2c_write(I2C_BUS5, AEGIS_CPLD_ADDR, 0x00, &data, 1)) {
			shell_error(shell, "plat soc_pwron_reset set failed");
			return -1;
		}
	} else {
		shell_warn(shell, "Help: soc_pwron_reset <override/passthru> <drive-level>");
		return -1;
	}
	shell_print(shell, "plat soc_pwron_reset control finish");
	return 0;
}

/* level 1 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_soc_pwron_reset_cmds,
			       SHELL_CMD_ARG(override, NULL,
					     "soc_pwron_reset override <drive-level>",
					     cmd_soc_pwron_reset_set, 2, 1),
			       SHELL_CMD(passthru, NULL, "soc_pwron_reset passthru",
					 cmd_soc_pwron_reset_set),
			       SHELL_SUBCMD_SET_END);

/* Root of command soc_pwron_reset */
SHELL_CMD_REGISTER(soc_pwron_reset, &sub_soc_pwron_reset_cmds, "soc_pwron_reset commands", NULL);
