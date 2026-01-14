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
#include "plat_hook.h"
#include "plat_cpld.h"
#include "plat_vr_test_mode.h"

LOG_MODULE_REGISTER(plat_voltage_shell, LOG_LEVEL_DBG);

static int cmd_voltage_get_all(const struct shell *shell, size_t argc, char **argv)
{
	/* is_ubc_enabled_delayed_enabled() is to wait for all VR to be enabled  */
	/* (gpio_get(FM_PLD_UBC_EN_R) == GPIO_HIGH) is to shut down polling immediately when UBC is disabled */
	if (!((gpio_get(FM_PLD_UBC_EN_R) == GPIO_HIGH) && is_ubc_enabled_delayed_enabled())) {
		shell_error(shell, "Can't get voltage command because VR has no power yet.");
		return -1;
	}

	shell_print(shell, "  id|              sensor_name               |vout(mV) ");
	/* list all vr sensor value */
	for (int i = 0; i < VR_RAIL_E_MAX; i++) {
		if (((get_asic_board_id() != ASIC_BOARD_ID_EVB)) &&
		    (i == VR_RAIL_E_P3V3_OSFP_VOLT_V))
			continue; // skip osfp p3v3 on BD

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

		shell_print(shell, "%4d|%-40s|%4d", i, rail_name, vout);
	}

	return 0;
}

static int cmd_voltage_set(const struct shell *shell, size_t argc, char **argv)
{
	if (!get_vr_test_mode_flag()) {
		shell_warn(shell, "This command is only for VR test mode");
		return -1;
	}

	/* is_ubc_enabled_delayed_enabled() is to wait for all VR to be enabled  */
	/* (gpio_get(FM_PLD_UBC_EN_R) == GPIO_HIGH) is to shut down polling immediately when UBC is disabled */
	if (!((gpio_get(FM_PLD_UBC_EN_R) == GPIO_HIGH) && is_ubc_enabled_delayed_enabled())) {
		shell_error(shell, "Can't set voltage command because VR has no power yet.");
		return -1;
	}

	if (argc >= 4) {
		shell_error(shell, "voltage set <voltage-rail> <new-voltage>");
		return -1;
	}

	/* covert rail string to enum */
	enum VR_RAIL_E rail;
	if (vr_rail_enum_get(argv[1], &rail) == false) {
		shell_error(shell, "Invalid rail name: %s", argv[1]);
		return -1;
	}

	uint16_t millivolt = strtol(argv[2], NULL, 0);
	uint16_t vout_max_millivolt = vout_range_user_settings.change_vout_max[rail];
	uint16_t vout_min_millivolt = vout_range_user_settings.change_vout_min[rail];
	if (millivolt < vout_min_millivolt || millivolt > vout_max_millivolt) {
		shell_error(shell, "vout[%d] cannot be less than %dmV or greater than %dmV", rail,
			    vout_min_millivolt, vout_max_millivolt);
		return -1;
	}
	// can't set voltage for osfp p3v3
	if (rail == VR_RAIL_E_P3V3_OSFP_VOLT_V) {
		shell_warn(shell, "OSFP P3V3 can't set voltage");
		return -1;
	}
	shell_info(shell, "Set %s(%d) to %d mV, %svolatile\n", argv[1], rail, millivolt,
		   (argc == 4) ? "non-" : "");

	/* set the vout */
	if ((get_asic_board_id() != ASIC_BOARD_ID_EVB) && (rail == VR_RAIL_E_P3V3_OSFP_VOLT_V)) {
		shell_print(shell, "There is no osfp p3v3");
		return 0;
	}
	// if vr is MPS, read back uvp and keep it >= 200mv
	uint8_t vr = get_vr_module();
	if (vr == VR_MODULE_MPS) {
		if (rail == VR_RAIL_E_ASIC_P0V85_MEDHA0_VDD ||
		    rail == VR_RAIL_E_ASIC_P0V85_MEDHA1_VDD) {
			uint16_t uvp = 0;
			uint16_t vout_offset = 0;
			if (get_vr_mp29816a_reg(rail, &vout_offset, VOUT_OFFSET)) {
				shell_error(shell, "get vr %d vout cmd fail", rail);
				return -1;
			}
			if (get_vr_mp29816a_reg(rail, &uvp, UVP)) {
				shell_error(shell, "get vr %d uvp fail", rail);
				return -1;
			}
			// Vout cmd = 200(limit uvp) + 500(test mode uvp_threshold) - vout offset
			uint16_t limit_vout_cmd = 200 + 500 - vout_offset;
			;
			if (millivolt < limit_vout_cmd) {
				shell_warn(shell, "uvp is too low, set vr %d vout cmd to %d", rail,
					   limit_vout_cmd);
				millivolt = limit_vout_cmd;
			}
		}
	}

	if (!plat_set_vout_command(rail, &millivolt)) {
		shell_error(shell, "Can't set vout by rail index: %d", rail);
		return -1;
	}

	return 0;
}

static void voltage_rname_get(size_t idx, struct shell_static_entry *entry)
{
	if ((get_asic_board_id() == ASIC_BOARD_ID_EVB))
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
