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
#include "sensor.h"
#include "plat_hook.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_temp_status_shell, LOG_LEVEL_DBG);

static int cmd_temp_status_get(const struct shell *shell, size_t argc, char **argv)
{
	if (!(argc == 2)) {
		shell_error(shell, "temp_status get <sensor>");
		return -1;
	}

	enum PLAT_TEMP_INDEX_E rail;
	if (temp_sensor_rail_enum_get(argv[1], &rail) == false) {
		shell_error(shell, "Invalid rail name: %s", argv[1]);
		return -1;
	}

	uint8_t temp_status = 0xFF;

	if (!plat_get_temp_status(rail, &temp_status)) {
		shell_error(shell, "Can't find temp status by rail index: %x", rail);
		return -1;
	}

	if (rail == TEMP_INDEX_ON_DIE_1_2 || rail == TEMP_INDEX_ON_DIE_3_4) {
		shell_print(shell, "0x%02x", rail, argv[1],
			    (get_tmp_type() == TMP_EMC1413) ? "EMC1413" : "TMP432", temp_status);
	} else {
		shell_print(shell, "0x%02x (ALERT_N)", rail, argv[1], "TMP75", temp_status);
	}

	return 0;
}

static int cmd_temp_status_clear(const struct shell *shell, size_t argc, char **argv)
{
	if (!(argc == 2)) {
		shell_error(shell, "temp_status clear all|<sensor>");
		return -1;
	}

	if (!strcmp(argv[1], "all")) {
		for (int i = 0; i < TEMP_INDEX_MAX; i++) {
			uint8_t *rail_name = NULL;
			if (!temp_sensor_rail_name_get((uint8_t)i, &rail_name)) {
				shell_error(shell, "Can't find temp_rail_name by rail index: %x",
					    i);
				continue;
			}

			if (!plat_clear_temp_status(i)) {
				shell_error(shell, "Can't clear temp status by rail index: %x", i);
				continue;
			}
		}
		shell_print(shell, "All Temp clear temp status finish");
		return 0;
	} else {
		enum PLAT_TEMP_INDEX_E rail;
		if (temp_sensor_rail_enum_get(argv[1], &rail) == false) {
			shell_error(shell, "Invalid rail name: %s", argv[1]);
			return -1;
		}

		if (!plat_clear_temp_status(rail)) {
			shell_error(shell, "Can't clear temp status by rail index: %x", rail);
			return -1;
		}
		shell_print(shell, "[%-2x]%-40s clear temp status finish", rail, argv[1]);
		return 0;
	}
}

static void temp_sensor_rname_get_for_get_cmd(size_t idx, struct shell_static_entry *entry)
{
	uint8_t *name = NULL;
	temp_sensor_rail_name_get((uint8_t)idx, &name);

	entry->syntax = (name) ? (const char *)name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

static void temp_sensor_rname_get_for_clear_cmd(size_t idx, struct shell_static_entry *entry)
{
	uint8_t *name = NULL;
	temp_sensor_rail_name_get((uint8_t)idx, &name);

	if (idx == TEMP_INDEX_MAX)
		name = (uint8_t *)"all";

	entry->syntax = (name) ? (const char *)name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(temp_sensor_rname_for_temp_status_get, temp_sensor_rname_get_for_get_cmd);
SHELL_DYNAMIC_CMD_CREATE(temp_sensor_rname_for_temp_status_clear,
			 temp_sensor_rname_get_for_clear_cmd);

/* level 1 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_temp_status_cmds,
			       SHELL_CMD(get, &temp_sensor_rname_for_temp_status_get,
					 "get <sensor>", cmd_temp_status_get),
			       SHELL_CMD(clear, &temp_sensor_rname_for_temp_status_clear,
					 "clear all|<sensor>", cmd_temp_status_clear),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(temp_status, &sub_temp_status_cmds,
		   "temperature fault status get/clear commands", NULL);
