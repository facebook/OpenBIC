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
#include <string.h>
#include <stdio.h>
#include <zephyr.h>
#include "plat_hook.h"
#include "plat_pwrlevel_shell.h"

void cmd_set_pwrlevel(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2 && argc != 3) {
		shell_warn(shell, "Help: pwrlevel1 set <alert-level in mA>|default [perm]");
		return;
	}

	bool is_default = false;
	int32_t setting_value = 0;

	if (strcmp(argv[1], "default") == 0) {
		is_default = true;
		set_alert_level_to_default_or_user_setting(is_default, 0);
		if (argc != 3) {
			shell_print(shell, "pwrlevel1 set default success!");
			return;
		}
	} else {
		setting_value = strtol(argv[1], NULL, 10);

		set_alert_level_to_default_or_user_setting(is_default, setting_value);

		if (argc != 3) {
			shell_print(shell, "pwrlevel1 set %d success!", setting_value);
			return;
		}
	}

	if (strcmp(argv[2], "perm") == 0) {
		if (strcmp(argv[1], "default") == 0) {
			setting_value = 0xffffffff;
		}

		char setting_data[4] = { 0 };
		int result = 0;

		setting_data[0] = setting_value & 0xff;
		setting_data[1] = (setting_value & 0xff00) >> 8;
		setting_data[2] = (setting_value & 0xff0000) >> 16;
		setting_data[3] = (setting_value & 0xff000000) >> 24;

		//write to eeprom
		result =
			set_user_settings_alert_level_to_eeprom(setting_data, sizeof(setting_data));
		if (result != 0) {
			shell_error(shell, "write alert level to eeprom error");
			return;
		}

		if (strcmp(argv[1], "default") == 0) {
			shell_print(shell, "pwrlevel1 set default perm success!");
		} else {
			shell_print(shell, "pwrlevel1 set %d perm success!", setting_value);
		}
	}

	return;
}

void cmd_get_pwrlevel(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_warn(shell, "Help: pwrlevel1 get");
		return;
	}

	int32_t result = 0, setting_value = 0, default_value = 0;
	bool is_assert = false;

	result = get_alert_level_info(&is_assert, &default_value, &setting_value);

	if (result != 0) {
		shell_error(shell, "Fail to get pwrlevel1");
	} else {
		if (is_assert == false) {
			shell_print(
				shell,
				"power alert current status: deassert | alert level: %dmA | default alert level: %dmA",
				setting_value, default_value);
		} else {
			shell_print(
				shell,
				"power alert current status: assert | alert level: %dmA | default alert level: %dmA",
				setting_value, default_value);
		}
	}

	return;
}

/* Sub-command Level 1 of command pwrlevel1 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_pwrlevel_cmds,
			       SHELL_CMD(set, NULL, "set_pwrlevel", cmd_set_pwrlevel),
			       SHELL_CMD(get, NULL, "get_pwrlevel", cmd_get_pwrlevel),
			       SHELL_SUBCMD_SET_END);

/* Root of command uart-pwrevent */
SHELL_CMD_REGISTER(pwrlevel1, &sub_pwrlevel_cmds, "pwrlevel1 commands for AG", NULL);
