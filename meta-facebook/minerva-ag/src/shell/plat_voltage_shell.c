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

LOG_MODULE_REGISTER(plat_voltage_shell, LOG_LEVEL_DBG);

static int cmd_voltage_get_all(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "  id|              sensor_name               |vout(mV) ");
	/* list all vr sensor value */
	for (int i = 0; i < VR_RAIL_E_MAX; i++) {
		if ((get_board_type() == MINERVA_AEGIS_BD) && (i == 0))
			continue; // skip osfp p3v3 on AEGIS BD

		uint16_t vout = 0;
		uint8_t *rail_name = NULL;
		if (!vr_rail_name_get((uint8_t)i, &rail_name)) {
			shell_print(shell, "Can't find vr_rail_name by rail index: %d", i);
			continue;
		}

		if (!plat_get_vout_command(i, &vout)) {
			shell_print(shell, "Can't find vout by rail index: %d", i);
			continue;
		}

		shell_print(shell, "%4d|%-50s|%4d", i, rail_name, vout);
	}

	return 0;
}

static int cmd_voltage_set(const struct shell *shell, size_t argc, char **argv)
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

	/* covert rail string to enum */
	enum VR_RAIL_E rail;
	if (vr_rail_enum_get(argv[1], &rail) == false) {
		shell_error(shell, "Invalid rail name: %s", argv[1]);
		return -1;
	}

	uint16_t millivolt = strtol(argv[2], NULL, 0);
	if (!strcmp(argv[2], "default")) {
		is_default = true;
		shell_info(shell, "Set %s(%d) to default, %svolatile\n", argv[1], rail,
			   (argc == 4) ? "non-" : "");
	} else {
		shell_info(shell, "Set %s(%d) to %d mV, %svolatile\n", argv[1], rail, millivolt,
			   (argc == 4) ? "non-" : "");
	}

	/* set the vout */
	if ((get_board_type() == MINERVA_AEGIS_BD) && (rail == 0)) {
		shell_print(shell, "There is no osfp p3v3 on AEGIS BD");
		return 0;
	}
	if (!plat_set_vout_command(rail, &millivolt, is_default, is_perm)) {
		shell_print(shell, "Can't set vout by rail index: %d", rail);
		return -1;
	}

	return 0;
}

static void voltage_rname_get(size_t idx, struct shell_static_entry *entry)
{
	if ((get_board_type() == MINERVA_AEGIS_BD))
		idx++;

	uint8_t *name = NULL;
	vr_rail_name_get((uint8_t)idx, &name);

	entry->syntax = (name) ? (const char *)name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(voltage_rname, voltage_rname_get);

/* level 2 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_voltage_get_cmds,
			       SHELL_CMD(all, NULL, "get voltage all vout command",
					 cmd_voltage_get_all),
			       SHELL_SUBCMD_SET_END);

/* level 1 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_voltage_cmds,
			       SHELL_CMD(get, &sub_voltage_get_cmds, "get voltage all", NULL),
			       SHELL_CMD_ARG(set, &voltage_rname,
					     "set <voltage-rail> <new-voltage>|default [perm]",
					     cmd_voltage_set, 3, 1),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(voltage, &sub_voltage_cmds, "voltage set/get commands", NULL);
