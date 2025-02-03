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

#ifndef PLAT_VOLTAGE_PEAK_SHELL_H
#define PLAT_VOLTAGE_PEAK_SHELL_H

#include <shell/shell.h>
#include "plat_class.h"

void cmd_get_voltage_peak(const struct shell *shell, size_t argc, char **argv);
void cmd_clear_voltage_peak(const struct shell *shell, size_t argc, char **argv);

static void voltage_rname_get(size_t idx, struct shell_static_entry *entry)
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

SHELL_DYNAMIC_CMD_CREATE(voltage_rname, voltage_rname_get);

/* Sub-command Level 1 of command voltage-peak */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_voltage_peak_cmds,
			       SHELL_CMD(get, &voltage_rname, "get_voltage_peak",
					 cmd_get_voltage_peak),
			       SHELL_CMD(clear, &voltage_rname, "clear_voltage_peak",
					 cmd_clear_voltage_peak),
			       SHELL_SUBCMD_SET_END);

/* Root of command voltage-peak */
SHELL_CMD_REGISTER(voltage_peak, &sub_voltage_peak_cmds, "Voltage Peak commands for AG", NULL);

#endif
