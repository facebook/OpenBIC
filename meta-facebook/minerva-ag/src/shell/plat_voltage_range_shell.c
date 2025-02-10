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

LOG_MODULE_REGISTER(plat_voltage_range_shell, LOG_LEVEL_DBG);

static int cmd_voltage_range_get(const struct shell *shell, size_t argc, char **argv)
{
	if (!((argc == 2) || (argc == 3))) {
		shell_error(shell, "voltage_range get all");
		shell_error(shell, "voltage_range get <voltage-rail> min|max");
		return -1;
	}

	if ((argc == 2)) {
		if (!strcmp(argv[1], "all")) {
			/* print all range */
			for (int i = 0; i < VR_RAIL_E_MAX; i++) {
				if ((get_board_type() == MINERVA_AEGIS_BD) && (i == 0))
					continue;

				uint8_t *rail_name = NULL;
				if (!vr_rail_name_get((uint8_t)i, &rail_name)) {
					shell_print(shell,
						    "Can't find vr_rail_name by rail index: %d", i);
					continue;
				}

				uint16_t millivolt = 0xFFFF;
				plat_get_vout_min(i, &millivolt);
				shell_print(shell, "%-50s vout min: %dmV", rail_name, millivolt);
				plat_get_vout_max(i, &millivolt);
				shell_print(shell, "%-50s vout max: %dmV", rail_name, millivolt);
			}
			return 0;
		} else {
			shell_error(shell, "voltage_range get all");
			shell_error(shell, "voltage_range get <voltage-rail> min|max");
			return -1;
		}
	}

	/* covert rail string to enum */
	enum VR_RAIL_E rail;
	if (vr_rail_enum_get(argv[1], &rail) == false) {
		shell_error(shell, "Invalid rail name: %s", argv[1]);
		return -1;
	}

	if ((get_board_type() == MINERVA_AEGIS_BD) && (rail == 0)) {
		shell_print(shell, "There is no osfp p3v3 on AEGIS BD");
		return 0;
	}

	uint16_t millivolt = 0xFFFF;

	if (!strcmp(argv[2], "min")) {
		/*get min range*/
		plat_get_vout_min(rail, &millivolt);

	} else if (!strcmp(argv[2], "max")) {
		/*get max range*/
		plat_get_vout_max(rail, &millivolt);
	} else {
		shell_error(shell, "voltage_range get all");
		shell_error(shell, "voltage_range get <voltage-rail> min|max");
		return -1;
	}

	shell_print(shell, "voltage_range get %s %s: %dmV", argv[1], argv[2], millivolt);

	return 0;
}

static int cmd_voltage_range_set(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 4) {
		shell_error(shell, "voltage_range set <voltage-rail> <new-voltage> min|max");
		return -1;
	}

	/* covert rail string to enum */
	enum VR_RAIL_E rail;
	if (vr_rail_enum_get(argv[1], &rail) == false) {
		shell_error(shell, "Invalid rail name: %s", argv[1]);
		return -1;
	}

	if ((get_board_type() == MINERVA_AEGIS_BD) && (rail == 0)) {
		shell_print(shell, "There is no osfp p3v3 on AEGIS BD");
		return 0;
	}

	/* covert voltage to millivolt */
	uint16_t voltage_set_val_mv = strtol(argv[2], NULL, 0);

	uint16_t voltage_set_val_mv_backup = voltage_set_val_mv;

	if (!strcmp(argv[3], "min")) {
		/*set min range*/
		if (!plat_set_vout_min(rail, &voltage_set_val_mv)) {
			shell_error(shell, "voltage_range set %s %s: %dmV failed", argv[1], argv[2],
				    voltage_set_val_mv_backup);
			return -1;
		}
		shell_print(shell, "voltage_range set %s %s: %dmV", argv[1], argv[2],
			    voltage_set_val_mv);
	} else if (!strcmp(argv[3], "max")) {
		/*set max range*/
		if (!plat_set_vout_max(rail, &voltage_set_val_mv)) {
			shell_error(shell, "voltage_range set %s %s: %dmV failed", argv[1], argv[2],
				    voltage_set_val_mv_backup);
			return -1;
		}
		shell_print(shell, "voltage_range set %s %s: %dmV", argv[1], argv[2],
			    voltage_set_val_mv);
	} else {
		shell_error(shell, "voltage_range set <voltage-rail> <new-voltage> min|max");
		return -1;
	}

	return 0;
}

static void vr_rname_get_for_get_voltrage(size_t idx, struct shell_static_entry *entry)
{
	if ((get_board_type() == MINERVA_AEGIS_BD))
		idx++;

	uint8_t *name = NULL;
	vr_rail_name_get((uint8_t)idx, &name);

	if (idx == VR_RAIL_E_MAX)
		name = (uint8_t *)"all";

	entry->syntax = (name) ? (const char *)name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

static void vr_rname_get_for_set_voltrage(size_t idx, struct shell_static_entry *entry)
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

SHELL_DYNAMIC_CMD_CREATE(vr_rname_for_get_voltrage, vr_rname_get_for_get_voltrage);
SHELL_DYNAMIC_CMD_CREATE(vr_rname_for_set_voltrage, vr_rname_get_for_set_voltrage);

/* level 1 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_voltage_range_cmds,
			       SHELL_CMD(get, &vr_rname_for_get_voltrage,
					 "get all|<voltage-rail> min|max", cmd_voltage_range_get),
			       SHELL_CMD(set, &vr_rname_for_set_voltrage,
					 "set <voltage-rail> <new-voltage> min|max",
					 cmd_voltage_range_set),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(voltage_range, &sub_voltage_range_cmds, "voltage_range set/get commands", NULL);
