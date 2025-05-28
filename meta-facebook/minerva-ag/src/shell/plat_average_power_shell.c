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
#include "plat_gpio.h"
#include "plat_event.h"

LOG_MODULE_REGISTER(plat_average_power_shell, LOG_LEVEL_DBG);

static int cmd_power_get(const struct shell *shell, size_t argc, char **argv)
{
	/* is_ubc_enabled_delayed_enabled() is to wait for all VR to be enabled  */
	/* (gpio_get(FM_PLD_UBC_EN_R) == GPIO_HIGH) is to shut down polling immediately when UBC is disabled */
	if (!((gpio_get(FM_PLD_UBC_EN_R) == GPIO_HIGH) && is_ubc_enabled_delayed_enabled())) {
		shell_error(shell, "Can't get power command because VR has no power yet.");
		return -1;
	}

	if (!(argc == 2)) {
		shell_error(shell, "average_power get <ubc_vr_name>|all");
		return -1;
	}
	if (!strcmp(argv[1], "all")) {
		for (int i = 0; i < UBC_VR_RAIL_E_MAX; i++) {
			if ((get_board_type() == MINERVA_AEGIS_BD) && (i == 2))
				continue;

			uint8_t *rail_name = NULL;
			if (!ubc_vr_rail_name_get((uint8_t)i, &rail_name)) {
				shell_print(shell, "Can't find ubc_vr_rail_name by rail index: %d",
					    i);
				continue;
			}

			int16_t average_power = 0;
			if (!get_average_power(i, &average_power)) {
				shell_print(shell, "Can't find average_power by rail index: %d", i);
				continue;
			}
			sensor_val tmp_reading;
			tmp_reading.integer = (int16_t)(average_power & 0xFFFF);
			tmp_reading.fraction = (int16_t)((average_power >> 16) & 0xFFFF);

			shell_print(shell, "%4x|%-50s| %5d.%03dW", i, rail_name,
				    tmp_reading.integer, tmp_reading.fraction);
		}
	} else {
		enum UBC_VR_RAIL_E rail;
		if (ubc_vr_rail_enum_get(argv[1], &rail) == false) {
			shell_error(shell, "Invalid rail name: %s", argv[1]);
			return -1;
		}
		if ((get_board_type() == MINERVA_AEGIS_BD) && (rail == 2)) {
			shell_print(shell, "There is no osfp p3v3 on AEGIS BD");
			return 0;
		}

		int16_t average_power = 0;
		if (!get_average_power(rail, &average_power)) {
			shell_print(shell, "Can't find average_power by rail index: %d", rail);
			return -1;
		}
		sensor_val tmp_reading;
		tmp_reading.integer = (int16_t)(average_power & 0xFFFF);
		tmp_reading.fraction = (int16_t)((average_power >> 16) & 0xFFFF);

		shell_print(shell, "%4x|%-50s| %5d.%03dW", rail, argv[1], tmp_reading.integer,
			    tmp_reading.fraction);
	}

	return 0;
}

static void ubc_vr_rname_get_for_get_power(size_t idx, struct shell_static_entry *entry)
{
	if ((get_board_type() == MINERVA_AEGIS_BD) && (idx == 2))
		idx++;

	uint8_t *name = NULL;
	ubc_vr_rail_name_get((uint8_t)idx, &name);

	if (idx == UBC_VR_RAIL_E_MAX)
		name = (uint8_t *)"all";

	entry->syntax = (name) ? (const char *)name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(ubc_vr_rname_for_get_power, ubc_vr_rname_get_for_get_power);

/* level 1 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_average_power_cmds,
			       SHELL_CMD(get, &ubc_vr_rname_for_get_power,
					 "average power get power commands", cmd_power_get),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(average_power, &sub_average_power_cmds, "average power commands", NULL);
