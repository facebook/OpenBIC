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

LOG_MODULE_REGISTER(plat_vr_status_shell, LOG_LEVEL_DBG);

static int cmd_vr_status_get(const struct shell *shell, size_t argc, char **argv)
{
	if (!(argc == 3)) {
		shell_error(shell, "vr_status get all|<voltage-rail> all|<which-status>");
		return -1;
	}

	if (!strcmp(argv[1], "all")) {
		for (int i = 0; i < VR_RAIL_E_MAX; i++) {
			if ((get_board_type() == MINERVA_AEGIS_BD) && (i == 0))
				continue; // skip osfp p3v3 on AEGIS BD

			uint8_t *rail_name = NULL;
			if (!vr_rail_name_get((uint8_t)i, &rail_name)) {
				shell_error(shell, "Can't find vr_rail_name by rail index: %x", i);
				continue;
			}

			if (!strcmp(argv[2], "all")) {
				shell_print(shell, "[%-2x]%-50s", i, rail_name);
				for (int j = 0; j < VR_STAUS_E_MAX; j++) {
					uint8_t *vr_status_name = NULL;
					if (!vr_status_name_get((uint8_t)j, &vr_status_name)) {
						shell_error(
							shell,
							"Can't find vr_status_name by rail index: %x",
							j);
						continue;
					}
					uint16_t vr_status = 0xFFFF;

					if (!plat_get_vr_status(i, j, &vr_status)) {
						shell_error(
							shell,
							"Can't find vr status[%x] by rail index: %x",
							j, i);
						continue;
					}

					shell_print(shell, "    %-10s:0x%02x", vr_status_name,
						    vr_status);
				}
			} else {
				enum VR_STAUS_E vr_status_rail;
				if (vr_status_enum_get(argv[2], &vr_status_rail) == false) {
					shell_error(shell, "Invalid vr_status rail name: %s",
						    argv[2]);
					return -1;
				}

				uint16_t vr_status = 0xFFFF;

				if (!plat_get_vr_status(i, vr_status_rail, &vr_status)) {
					shell_error(shell,
						    "Can't find vr status[%x] by rail index: %x",
						    vr_status_rail, i);
					return -1;
				}

				shell_print(shell, "[%-2x]%-50s %-10s:0x%02x", i, rail_name,
					    argv[2], vr_status);
			}
		}
		return 0;
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

		if (!strcmp(argv[2], "all")) {
			for (int j = 0; j < VR_STAUS_E_MAX; j++) {
				uint8_t *vr_status_name = NULL;
				if (!vr_status_name_get((uint8_t)j, &vr_status_name)) {
					shell_error(shell,
						    "Can't find vr_status_name by rail index: %x",
						    j);
					continue;
				}
				uint16_t vr_status = 0xFFFF;

				if (!plat_get_vr_status(rail, j, &vr_status)) {
					shell_error(shell,
						    "Can't find vr status[%x] by rail index: %x", j,
						    rail);
					continue;
				}

				shell_print(shell, "[%-2x]%-50s %-10s:0x%02x", rail, argv[1],
					    vr_status_name, vr_status);
			}
			return 0;
		} else {
			enum VR_STAUS_E vr_status_rail;
			if (vr_status_enum_get(argv[2], &vr_status_rail) == false) {
				shell_error(shell, "Invalid vr_status rail name: %s", argv[2]);
				return -1;
			}

			uint16_t vr_status = 0xFFFF;

			if (!plat_get_vr_status(rail, vr_status_rail, &vr_status)) {
				shell_error(shell, "Can't find vr status[%x] by rail index: %x",
					    vr_status_rail, rail);
				return -1;
			}

			shell_print(shell, "[%-2x]%-50s %-10s:0x%02x", rail, argv[1], argv[2],
				    vr_status);
			return 0;
		}
	}
}

static int cmd_vr_status_clear(const struct shell *shell, size_t argc, char **argv)
{
	if (!(argc == 2)) {
		shell_error(shell, "vr_status clear all|<voltage-rail>");
		return -1;
	}

	if (!strcmp(argv[1], "all")) {
		for (int i = 0; i < VR_RAIL_E_MAX; i++) {
			if ((get_board_type() == MINERVA_AEGIS_BD) && (i == 0))
				continue; // skip osfp p3v3 on AEGIS BD

			uint8_t *rail_name = NULL;
			if (!vr_rail_name_get((uint8_t)i, &rail_name)) {
				shell_error(shell, "Can't find vr_rail_name by rail index: %x", i);
				continue;
			}

			if (!plat_clear_vr_status(i)) {
				shell_error(shell, "Can't clear vr status by rail index: %x", i);
				continue;
			}
		}
		shell_print(shell, "All VR clear vr status finish");
		return 0;
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

		if (!plat_clear_vr_status(rail)) {
			shell_error(shell, "Can't clear vr status by rail index: %x", rail);
			return -1;
		}
		shell_print(shell, "[%-2x]%-50s clear vr status finish", rail, argv[1]);
		return 0;
	}
}

static void vr_status_rname_get(size_t idx, struct shell_static_entry *entry)
{
	if ((get_board_type() == MINERVA_AEGIS_BD))
		idx++;

	uint8_t *name = NULL;
	vr_status_name_get((uint8_t)idx, &name);
	if (idx == VR_STAUS_E_MAX)
		name = (uint8_t *)"all";

	entry->syntax = (name) ? (const char *)name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(vr_status_rname_for_vr_status, vr_status_rname_get);

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
	entry->subcmd = &vr_status_rname_for_vr_status;
}

static void voltage_rname_clear(size_t idx, struct shell_static_entry *entry)
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

SHELL_DYNAMIC_CMD_CREATE(vr_rname_for_vr_status, voltage_rname_get);
SHELL_DYNAMIC_CMD_CREATE(vr_rname_for_vr_status_clear, voltage_rname_clear);

/* level 1 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_vr_status_cmds,
			       SHELL_CMD(get, &vr_rname_for_vr_status,
					 "get all|<voltage-rail> all|<which-status>",
					 cmd_vr_status_get),
			       SHELL_CMD(clear, &vr_rname_for_vr_status_clear,
					 "clear all|<voltage-rail>", cmd_vr_status_clear),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(vr_status, &sub_vr_status_cmds, "vr fault status set/get commands", NULL);
