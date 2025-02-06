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
#include <stdio.h>
#include <string.h>
#include <zephyr.h>
#include <logging/log.h>
#include "plat_hook.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_voltage_peak_shell);

static int cmd_get_voltage_peak(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: get <voltage-rail>|all");
		return -1;
	}

	int peak_value;

	if (!strcmp(argv[1], "all")) {
		for (int i = 0; i < VR_RAIL_E_MAX; i++) {
			if ((get_board_type() == MINERVA_AEGIS_BD) && (i == 0))
				continue;

			uint8_t *rail_name = NULL;
			if (!vr_rail_name_get((uint8_t)i, &rail_name)) {
				shell_print(shell, "Can't find vr_rail_name by rail index: %d", i);
				continue;
			}

			if (vr_rail_voltage_peak_get(rail_name, &peak_value) == false) {
				shell_error(shell, "Invalid rail name to get peak value");
				continue;
			}

			if (peak_value == 0xffffffff) {
				shell_print(shell, "%-50s peak value: This is default peak value",
					    rail_name);
			} else {
				float sensor_reading = 0, decimal = 0;
				int16_t integer = 0;

				// Convert two byte integer, two byte decimal sensor format to float
				integer = peak_value & 0xffff;
				decimal = (float)(peak_value >> 16) / 1000.0;

				if (integer >= 0) {
					sensor_reading = (float)integer + decimal;
				} else {
					sensor_reading = (float)integer - decimal;
				}

				shell_print(shell, "%-50s peak value: %10.3f", rail_name,
					    sensor_reading);
			}
		}
	} else {
		enum VR_RAIL_E rail;
		if (vr_rail_enum_get(argv[1], &rail) == false) {
			shell_error(shell, "Invalid rail name: %s", argv[1]);
			return -1;
		}

		if ((get_board_type() == MINERVA_AEGIS_BD) && (rail == 0)) {
			shell_print(shell, "There is no osfp p3v3 on AEGIS BD");
			return 0;
		}

		if (vr_rail_voltage_peak_get(argv[1], &peak_value) == false) {
			shell_error(shell, "Invalid rail name to get peak value: %s", argv[1]);
			return -1;
		}

		if (peak_value == 0xffffffff) {
			shell_print(shell, "%-50s peak value: This is default peak value", argv[1]);
		} else {
			float sensor_reading = 0, decimal = 0;
			int16_t integer = 0;

			// Convert two byte integer, two byte decimal sensor format to float
			integer = peak_value & 0xffff;
			decimal = (float)(peak_value >> 16) / 1000.0;

			if (integer >= 0) {
				sensor_reading = (float)integer + decimal;
			} else {
				sensor_reading = (float)integer - decimal;
			}

			shell_print(shell, "%-50s peak value: %10.3f", argv[1], sensor_reading);
		}
	}

	return 0;
}

static int cmd_clear_voltage_peak(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: clear <voltage-rail>|all");
		return -1;
	}

	if (!strcmp(argv[1], "all")) {
		bool result = true;
		for (int i = 0; i < VR_RAIL_E_MAX; i++) {
			if ((get_board_type() == MINERVA_AEGIS_BD) && (i == 0))
				continue;

			result = vr_rail_voltage_peak_clear(i);
			if (result != true) {
				uint8_t *rail_name = NULL;
				if (!vr_rail_name_get((uint8_t)i, &rail_name)) {
					shell_print(shell,
						    "Can't find vr_rail_name by rail index: %d", i);
					continue;
				}
				shell_print(shell, "%-50s voltage peak clear failed", rail_name);
				continue;
			}
		}
		shell_print(shell, "voltage_peak clear all done!");
	} else {
		enum VR_RAIL_E rail;
		bool result = true;

		if (vr_rail_enum_get(argv[1], &rail) == false) {
			shell_error(shell, "Invalid rail name: %s", argv[1]);
			return -1;
		}

		result = vr_rail_voltage_peak_clear((uint8_t)rail);
		if (result != true) {
			shell_print(shell, "%-50s voltage peak clear failed", argv[1]);
			return -1;
		}

		shell_print(shell, "voltage_peak clear %s done!", argv[1]);
	}

	return 0;
}

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
