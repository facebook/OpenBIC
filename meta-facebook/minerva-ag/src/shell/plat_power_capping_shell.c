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
#include "plat_pldm_sensor.h"

LOG_MODULE_REGISTER(plat_power_capping_shell, LOG_LEVEL_DBG);

void cmd_power_capping_get(const struct shell *shell, size_t argc, char **argv)
{
	for (int i = 0; i < POWER_CAPPING_INDEX_MAX; i++) {
		uint8_t *rail_name = NULL;
		if (!power_capping_rail_name_get((uint8_t)i, &rail_name)) {
			shell_print(shell, "Can't find power_capping_rail_name by rail index: %d",
				    i);
			continue;
		}

		if (power_capping_user_settings.user_setting_value[i] == 0xffff) {
			shell_print(shell, "%4d|%-50s|not set", i, rail_name);
		} else {
			shell_print(shell, "%4d|%-50s|%4d", i, rail_name,
				    power_capping_user_settings.user_setting_value[i]);
		}
	}
}

int cmd_power_capping_set(const struct shell *shell, size_t argc, char **argv)
{
	if (argc == 4) {
		if (!strcmp(argv[1], "HC_LC")) {
			uint16_t set_value_HC = strtol(argv[2], NULL, 10);
			uint16_t set_value_LC = strtol(argv[3], NULL, 10);
			if (!plat_set_power_capping_command(POWER_CAPPING_INDEX_HC,
							    &set_value_HC)) {
				shell_error(shell, "Can't set value by rail index: %d",
					    POWER_CAPPING_INDEX_HC);
				return -1;
			}
			if (!plat_set_power_capping_command(POWER_CAPPING_INDEX_LC,
							    &set_value_LC)) {
				shell_error(shell, "Can't set value by rail index: %d",
					    POWER_CAPPING_INDEX_LC);
				return -1;
			}
			shell_print(shell, "set HC value: %d, LC value: %d successfully",
				    set_value_HC, set_value_LC);
		} else {
			shell_error(shell, "only support HC_LC");
			return -1;
		}
	} else if (argc == 3) {
		if (!strcmp(argv[1], "interval")) {
			long interval_val = strtol(argv[2], NULL, 10);
			if (interval_val < ATH_VDD_INTERVAL_MS) {
				shell_error(
					shell,
					"The interval value is smaller than the ATH_VDD polling interval, which is %d ms",
					ATH_VDD_INTERVAL_MS);
				return -1;
			}
			if (interval_val > 0xFFFF) {
				shell_error(shell, "The interval value is too large (max %d)",
					    0xFFFF);
				return -1;
			}

			uint16_t set_value = (uint16_t)interval_val;
			if (!plat_set_power_capping_command(POWER_CAPPING_INDEX_INTERVAL,
							    &set_value)) {
				shell_error(shell, "Can't set value by rail index: %d",
					    POWER_CAPPING_INDEX_INTERVAL);
				return -1;
			}
			shell_print(shell, "set interval value: %d successfully", set_value);
		} else {
			shell_error(shell, "only support interval");
			return -1;
		}
	}

	return 0;
}

static const struct shell_static_entry power_capping_name_entries[] = {
	{ .syntax = "interval",
	  .handler = NULL,
	  .subcmd = NULL,
	  .help = "set <interval> <interval_value>" },
	{ .syntax = "HC_LC",
	  .handler = NULL,
	  .subcmd = NULL,
	  .help = "set HC_LC <HC_value> <LC_value>" },
};

static void power_capping_rname_get(size_t idx, struct shell_static_entry *entry)
{
	if (idx < ARRAY_SIZE(power_capping_name_entries)) {
		*entry = power_capping_name_entries[idx];
	} else {
		entry->syntax = NULL;
	}
}

SHELL_DYNAMIC_CMD_CREATE(power_capping_rname, power_capping_rname_get);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_voltage_get_cmds,
			       SHELL_CMD(all, NULL, "get power capping all setting",
					 cmd_power_capping_get),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_power_capping_cmds,
	SHELL_CMD(get, &sub_voltage_get_cmds, "get power capping all ", NULL),
	SHELL_CMD_ARG(set, &power_capping_rname,
		      "set <interval|HC_LC> <interval_value|HC_value> <LC_value>",
		      cmd_power_capping_set, 3, 1),
	SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(power_capping, &sub_power_capping_cmds, "power capping set/get command", NULL);
