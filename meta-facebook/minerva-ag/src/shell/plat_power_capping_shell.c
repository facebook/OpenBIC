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

static bool validate_power_capping_cmd(int argc, char **argv)
{
	if (argc < 3)
		return false;

	if (strcmp(argv[0], "set") != 0)
		return false;

	if (strcmp(argv[1], "HC_LC") == 0) {
		// set HC_LC <HC_value|default> <LC_value|default> [perm]
		if (argc != 4 && argc != 5)
			return false;
		if (argc == 5 && strcmp(argv[4], "perm") != 0)
			return false;
		return true;

	} else if (strcmp(argv[1], "interval_ms") == 0) {
		// set interval_ms <value|default> [perm]
		if (argc != 3 && argc != 4)
			return false;
		if (argc == 4 && strcmp(argv[3], "perm") != 0)
			return false;
		return true;

	} else if (strcmp(argv[1], "switch") == 0) {
		// set switch <enable|disable|default> [perm]
		if (argc != 3 && argc != 4)
			return false;
		if (strcmp(argv[2], "enable") != 0 && strcmp(argv[2], "disable") != 0 &&
		    strcmp(argv[2], "default") != 0)
			return false;
		if (argc == 4 && strcmp(argv[3], "perm") != 0)
			return false;
		return true;
	} else if (strcmp(argv[1], "average_time_ms") == 0) {
		// set average_time_ms <50|100|default> [perm]
		if (argc != 3 && argc != 4)
			return false;
		if (strcmp(argv[2], "50") != 0 && strcmp(argv[2], "100") != 0 &&
		    strcmp(argv[2], "default") != 0)
			return false;
		if (argc == 4 && strcmp(argv[3], "perm") != 0)
			return false;
		return true;
	}

	return false;
}

void cmd_power_capping_get(const struct shell *shell, size_t argc, char **argv)
{
	for (int i = 0; i < POWER_CAPPING_INDEX_MAX; i++) {
		uint8_t *rail_name = NULL;
		uint16_t value = 0;
		if (!power_capping_rail_name_get((uint8_t)i, &rail_name)) {
			shell_print(shell, "Can't find power_capping_rail_name by rail index: %d",
				    i);
			continue;
		}

		if (!plat_get_power_capping_command((uint8_t)i, &value)) {
			shell_print(shell, "Can't get value by rail index: %d", i);
			continue;
		}

		if (i == POWER_CAPPING_INDEX_SWITCH) {
			shell_print(shell, "%4d|%-50s|%s", i, rail_name,
				    value ? "enable" : "disable");
		} else {
			shell_print(shell, "%4d|%-50s|%4d", i, rail_name, value);
		}
	}
}

int cmd_power_capping_set(const struct shell *shell, size_t argc, char **argv)
{
	if (!validate_power_capping_cmd(argc, argv)) {
		shell_error(shell, "Invalid command format. Allowed formats:\n"
				   "  set HC_LC <HC_value|default> <LC_value|default> [perm]\n"
				   "  set interval_ms <value|default> [perm]\n"
				   "  set switch <enable|disable|default> [perm]\n"
				   "  set average_time_ms <50|100|default> [perm]");
		return -EINVAL;
	}

	bool is_perm = false;

	if (strcmp(argv[1], "HC_LC") == 0) {
		uint16_t set_value_HC = strtol(argv[2], NULL, 10);
		uint16_t set_value_LC = strtol(argv[3], NULL, 10);
		if (!strcmp(argv[4], "perm")) {
			is_perm = true;
		}
		if (!strcmp(argv[2], "default")) {
			set_value_HC = POWER_CAPPING_HC_DEFAULT;
		}
		if (!strcmp(argv[3], "default")) {
			set_value_LC = POWER_CAPPING_LC_DEFAULT;
		}
		if (!plat_set_power_capping_command(POWER_CAPPING_INDEX_HC, &set_value_HC,
						    is_perm)) {
			shell_error(shell, "Can't set value by rail index: %d",
				    POWER_CAPPING_INDEX_HC);
			return -1;
		}
		if (!plat_set_power_capping_command(POWER_CAPPING_INDEX_LC, &set_value_LC,
						    is_perm)) {
			shell_error(shell, "Can't set value by rail index: %d",
				    POWER_CAPPING_INDEX_LC);
			return -1;
		}
		shell_info(shell, "Set %s HC:%d, LC:%d ,%svolatile\n", argv[1], set_value_HC,
			   set_value_LC, (argc == 5) ? "non-" : "");
	} else if (strcmp(argv[1], "interval_ms") == 0) {
		long interval_val = strtol(argv[2], NULL, 10);
		if (!strcmp(argv[3], "perm")) {
			is_perm = true;
		}
		if (!strcmp(argv[2], "default")) {
			interval_val = ATH_VDD_INTERVAL_MS;
		}
		if (interval_val < ATH_VDD_INTERVAL_MS) {
			shell_error(
				shell,
				"The interval_ms value is smaller than the ATH_VDD polling interval_ms, which is %d ms",
				ATH_VDD_INTERVAL_MS);
			return -1;
		}
		if (interval_val > 0xFFFF) {
			shell_error(shell, "The interval_ms value is too large (max %d)", 0xFFFF);
			return -1;
		}

		uint16_t set_value = (uint16_t)interval_val;
		if (!plat_set_power_capping_command(POWER_CAPPING_INDEX_INTERVAL, &set_value,
						    is_perm)) {
			shell_error(shell, "Can't set value by rail index: %d",
				    POWER_CAPPING_INDEX_INTERVAL);
			return -1;
		}
		shell_info(shell, "Set %s:%d, %svolatile\n", argv[1], interval_val,
			   (argc == 4) ? "non-" : "");
	} else if (strcmp(argv[1], "switch") == 0) {
		bool enable = strcmp(argv[2], "enable") == 0;
		if (!strcmp(argv[3], "perm")) {
			is_perm = true;
		}
		if (!strcmp(argv[2], "default")) {
			enable = POWER_CAPPING_DISABLE;
		}
		if (enable) {
			uint16_t set_value = POWER_CAPPING_ENABLE;
			if (!plat_set_power_capping_command(POWER_CAPPING_INDEX_SWITCH, &set_value,
							    is_perm)) {
				shell_error(shell, "Can't set value by rail index: %d",
					    POWER_CAPPING_INDEX_SWITCH);
				return -1;
			}
		} else {
			uint16_t set_value = POWER_CAPPING_DISABLE;
			if (!plat_set_power_capping_command(POWER_CAPPING_INDEX_SWITCH, &set_value,
							    is_perm)) {
				shell_error(shell, "Can't set value by rail index: %d",
					    POWER_CAPPING_INDEX_SWITCH);
				return -1;
			}
		}
		shell_info(shell, "Set %s:%s, %svolatile\n", argv[1],
			   (enable ? "enable" : "disable"), (argc == 4) ? "non-" : "");
	} else if (strcmp(argv[1], "average_time_ms") == 0) {
		uint16_t set_value = strtol(argv[2], NULL, 10);
		if (!strcmp(argv[3], "perm")) {
			is_perm = true;
		}
		if (!strcmp(argv[2], "default")) {
			set_value = POWER_CAPPING_AVERAGE_TIME_MS_DEFAULT;
		}

		if (!plat_set_power_capping_command(POWER_CAPPING_INDEX_AVERAGE, &set_value,
						    is_perm)) {
			shell_error(shell, "Can't set value by rail index: %d",
				    POWER_CAPPING_INDEX_AVERAGE);
			return -1;
		}
		shell_info(shell, "Set %s:%d, %svolatile\n", argv[1], set_value,
			   (argc == 4) ? "non-" : "");
	}

	return 0;
}

static const struct shell_static_entry power_capping_name_entries[] = {
	{ .syntax = "HC_LC",
	  .handler = NULL,
	  .subcmd = NULL,
	  .help = "set HC_LC <HC_value|default> <LC_value|default> [perm]" },
	{ .syntax = "interval_ms",
	  .handler = NULL,
	  .subcmd = NULL,
	  .help = "set interval_ms <interval_value|default> [perm]" },
	{ .syntax = "switch",
	  .handler = NULL,
	  .subcmd = NULL,
	  .help = "set switch <enable|disable|default> [perm]" },
	{ .syntax = "average_time_ms",
	  .handler = NULL,
	  .subcmd = NULL,
	  .help = "set average_time_ms <50|100|default> [perm]" },

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

SHELL_STATIC_SUBCMD_SET_CREATE(sub_power_capping_get_cmds,
			       SHELL_CMD(all, NULL, "get power capping all setting",
					 cmd_power_capping_get),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_power_capping_cmds,
	SHELL_CMD(get, &sub_power_capping_get_cmds, "get power capping all ", NULL),
	SHELL_CMD(set, &power_capping_rname,
		  "Power capping control\n"
		  "  set HC_LC <HC_value|default> <LC_value|default> [perm]\n"
		  "  set interval_ms <value|default> [perm]\n"
		  "  set switch <enable|disable|default> [perm]\n"
		  "  set average_time_ms <50|100|default> [perm]",
		  cmd_power_capping_set),
	SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(power_capping, &sub_power_capping_cmds, "power capping set/get command", NULL);