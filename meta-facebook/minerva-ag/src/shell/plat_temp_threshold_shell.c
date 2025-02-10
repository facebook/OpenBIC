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
#include "plat_hook.h"

LOG_MODULE_REGISTER(plat_temp_threshold_shell, LOG_LEVEL_DBG);

static int temp_threshold_set_all_default(const struct shell *shell)
{
	uint32_t temperature = 0;

	bool all_success = true;

	for (int i = 0; i < PLAT_TEMP_INDEX_THRESHOLD_TYPE_MAX; i++) {
		if (!plat_set_temp_threshold(i, &temperature, true, false)) {
			shell_print(shell, "Can't set temp threshold index: %d", i);
			all_success = false;
		}
	}

	if (all_success) {
		shell_print(shell, "Set all temp threshold to default");
	}

	return 0;
}

static int cmd_temp_threshold_get_all(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(
		shell,
		"  id|              temp_threshold_name               |temperature(millidegree C)");
	for (int i = 0; i < PLAT_TEMP_INDEX_THRESHOLD_TYPE_MAX; i++) {
		uint32_t temperature = 0;
		uint8_t *temp_index_threshold_name = NULL;
		if (!temp_index_threshold_type_name_get((uint8_t)i, &temp_index_threshold_name)) {
			shell_print(shell, "Can't find type name by index: %d", i);
			continue;
		}

		if (!plat_get_temp_threshold(i, &temperature)) {
			shell_print(shell, "Can't find temp threshold by index: %d", i);
			continue;
		}

		shell_print(shell, "%4d|%-50s|%4d", i, temp_index_threshold_name, temperature);
	}

	return 0;
}

static int cmd_temp_threshold_set(const struct shell *shell, size_t argc, char **argv)
{
	bool is_default = false;
	bool is_perm = false;

	if (argc == 4) {
		if (!strcmp(argv[3], "perm")) {
			is_perm = true;
		} else {
			shell_error(shell, "The last argument must be <perm>");
			return -1;
		}
	}

	if (!strcmp(argv[1], "all")) {
		if (!strcmp(argv[2], "default")) {
			temp_threshold_set_all_default(shell);
			return 0;
		}
		shell_error(shell, "Only support \"temp_threshold set all default\" ");
		return -1;
	}

	/* covert string to enum */
	enum PLAT_TEMP_INDEX_THRESHOLD_TYPE_E temp_index_threshold_type;
	if (temp_threshold_type_enum_get(argv[1], &temp_index_threshold_type) == false) {
		shell_error(shell, "Invalid temp threshold name: %s", argv[1]);
		return -1;
	}

	uint32_t temperature = strtol(argv[2], NULL, 0);
	if (!strcmp(argv[2], "default")) {
		is_default = true;
		shell_info(shell, "Set %s(%d) to default, %svolatile\n", argv[1],
			   temp_index_threshold_type, (argc == 4) ? "non-" : "");
	} else {
		shell_info(shell, "Set %s(%d) to %d millidegree C, %svolatile\n", argv[1],
			   temp_index_threshold_type, temperature, (argc == 4) ? "non-" : "");
	}

	if (!plat_set_temp_threshold(temp_index_threshold_type, &temperature, is_default,
				     is_perm)) {
		shell_print(shell, "Can't set temp_threshold by temp index: %d",
			    temp_index_threshold_type);
		return -1;
	}

	return 0;
}

static void temp_index_threshold_type_name_get_(size_t idx, struct shell_static_entry *entry)
{
	uint8_t *name = NULL;
	temp_index_threshold_type_name_get((uint8_t)idx, &name);

	if (idx == PLAT_TEMP_INDEX_THRESHOLD_TYPE_MAX)
		name = (uint8_t *)"all";

	entry->syntax = (name) ? (const char *)name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(temp_index_threshold_type_name, temp_index_threshold_type_name_get_);

/* level 2 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_temp_threshold_get_cmds,
			       SHELL_CMD(all, NULL, "temp_threshold get all",
					 cmd_temp_threshold_get_all),
			       SHELL_SUBCMD_SET_END);

/* level 1 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_temp_threshold_cmds,
			       SHELL_CMD(get, &sub_temp_threshold_get_cmds, "get all", NULL),
			       SHELL_CMD_ARG(set, &temp_index_threshold_type_name,
					     "set <temp_threshold> <value>|default [perm]",
					     cmd_temp_threshold_set, 3, 1),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(temp_threshold, &sub_temp_threshold_cmds, "temp_threshold set/get commands",
		   NULL);
