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
#include "plat_hook.h"
#include "plat_cpld.h"
#include "plat_user_setting.h"

LOG_MODULE_REGISTER(plat_delay_module_pg_shell, LOG_LEVEL_DBG);

static int cmd_delay_delay_module_pg(const struct shell *shell, size_t argc, char **argv)
{
	bool is_perm = false;

	if (argc == 3) {
		if (!strcmp(argv[2], "perm")) {
			is_perm = true;
		} else {
			shell_error(shell, "The last argument must be <perm>");
			return -1;
		}
	}

	uint8_t setting_value = 0;
	if (!strcmp(argv[1], "default")) {
		shell_info(shell, "Set module PG delay to default, %svolatile\n",
			   (argc == 3) ? "non-" : "");
	} else {
		setting_value = strtol(argv[1], NULL, 10);
		if (setting_value < 0 || setting_value > 255) {
			shell_warn(shell, "Help: N range from 0 to 255");
			return -1;
		}
		uint16_t delay_time = setting_value * 10;
		shell_info(shell, "Set module PG delay to %d ms, %svolatile\n", delay_time,
			   (argc == 3) ? "non-" : "");
	}

	if (!plat_write_cpld(CPLD_OFFSET_MODULE_PG_DELAY, &setting_value)) {
		shell_error(shell, "plat delay_module_pg set failed");
		return -1;
	}

	//write to eeprom
	if (is_perm) {
		if (!set_user_settings_delay_module_pg_to_eeprom(&setting_value,
								sizeof(setting_value))) {
			shell_error(shell, "write module PG delay to eeprom error");
			return -1;
		}
	}

	return 0;
}

/* Root of command delay_asic_rst */
SHELL_CMD_ARG_REGISTER(delay_module_pg, NULL, "delay_module_pg [N (* 10ms)]|default [perm]",
		       cmd_delay_delay_module_pg, 2, 1);
