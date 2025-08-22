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

#include <stdlib.h>
#include <logging/log.h>
#include "plat_hook.h"
#include "plat_class.h"
#include "hal_i2c.h"
#include "plat_i2c.h"
#include "plat_user_setting.h"
#include "plat_fru.h"

LOG_MODULE_REGISTER(plat_thermaltrip_switch_shell, LOG_LEVEL_DBG);

#define CPLD_ADDR (0x4C >> 1)
#define I2C_BUS_CPLD I2C_BUS11
#define CPLD_THERMALTRIP_SWITCH_ADDR 0x3A

void cmd_thermaltrip_status_get(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t read_back_data = 0;
	if (!plat_read_cpld(CPLD_THERMALTRIP_SWITCH_ADDR, &read_back_data, 1)) {
		shell_error(shell, "Failed to read thermaltrip switch status");
		return;
	}

	shell_print(shell, "thermaltrip switch %s", (read_back_data) ? "enable" : "disable");
}

void cmd_thermaltrip_status_en(const struct shell *shell, size_t argc, char **argv)
{
	bool is_perm = false;

	if (argc == 2) {
		if (!strcmp(argv[1], "perm")) {
			is_perm = true;
		} else {
			shell_error(shell, "The last argument must be <perm>");
			return;
		}
	}

	set_thermaltrip_user_settings(true, is_perm);
}

void cmd_thermaltrip_status_dis(const struct shell *shell, size_t argc, char **argv)
{
	bool is_perm = false;

	if (argc == 2) {
		if (!strcmp(argv[1], "perm")) {
			is_perm = true;
		} else {
			shell_error(shell, "The last argument must be <perm>");
			return;
		}
	}

	set_thermaltrip_user_settings(false, is_perm);
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_plat_thermaltrip_set_cmd,
			       SHELL_CMD(enable, NULL, "enable", cmd_thermaltrip_status_en),
			       SHELL_CMD(disable, NULL, "disable", cmd_thermaltrip_status_dis),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_thermaltrip_switch_cmds,
			       SHELL_CMD(get, NULL, "get thermaltrip switch status",
					 cmd_thermaltrip_status_get),
			       SHELL_CMD(set, &sub_plat_thermaltrip_set_cmd,
					 "set thermaltrip switch status", NULL),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(thermaltrip_switch, &sub_thermaltrip_switch_cmds, "thermal trip switch command",
		   NULL);
