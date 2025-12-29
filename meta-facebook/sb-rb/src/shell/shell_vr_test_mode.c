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
#include <shell/shell.h>

#include "plat_pldm_sensor.h"
#include "plat_vr_test_mode.h"
#include "plat_hook.h"
#include "plat_util.h"

void cmd_vr_test_mode_start(const struct shell *shell, size_t argc, char **argv)
{
	if (!is_dc_on()) {
		shell_print(shell, "please iris power on first");
		return;
	}

	vr_test_mode_enable(true);
}
void cmd_vr_test_mode_exit(const struct shell *shell, size_t argc, char **argv)
{
	if (!is_dc_on()) {
		shell_print(shell, "please iris power on first");
		return;
	}

	vr_test_mode_enable(false);
}

static void vr_test_mode_show(const struct shell *shell, bool is_default)
{
	const vr_test_mode_setting_t *table =
		is_default ? vr_test_mode_table_default : vr_test_mode_table;
	uint8_t table_size = is_default ? vr_test_mode_table_dafault_size : vr_test_mode_table_size;

	shell_print(shell, "%-30s | %-11s | %-11s | %-7s | %-7s | %-9s | %-7s | %-7s ",
		    "VR RAIL NAME", "FAST OCP(A)", "SLOW OCP(A)", "UVP(mV)", "OVP(mV)", "V MAX(mV)",
		    "LCR(mV)", "UCR(mV)");
	shell_print(
		shell,
		"----------------------------------------------------------------------------------------------------------------");

	for (uint8_t i = 0; i < table_size; i++) {
		uint8_t *rail_name = NULL;
		if (vr_rail_name_get((uint8_t)i, &rail_name)) {
			const vr_test_mode_setting_t *cfg = &table[i];
			shell_print(shell,
				    "%-30s | %-11d | %-11d | %-7d | %-7d | %-9d | %-7d | %-7d ",
				    (char *)rail_name, (cfg->fast_ocp / 10), ((cfg->slow_ocp / 10)),
				    cfg->uvp, cfg->ovp, cfg->vout_max, cfg->lcr, cfg->ucr);
		}
	}
}
void cmd_vr_test_mode_show_default(const struct shell *shell, size_t argc, char **argv)
{
	vr_test_mode_show(shell, true);
}
void cmd_vr_test_mode_show_val(const struct shell *shell, size_t argc, char **argv)
{
	vr_test_mode_show(shell, false);
}
static int get_vr_reg_to_int(uint8_t vr_rail, uint8_t reg)
{
	uint8_t data[2] = { 0 };
	if (!get_raw_data_from_sensor_id(vr_rail_table[vr_rail].sensor_id, reg, data, 2))
		return -1;

	uint16_t raw_val = (data[1] << 8) | data[0];
	return (int)raw_val;
}

void cmd_vr_test_mode_show_real(const struct shell *shell, size_t argc, char **argv)
{
	if (!is_dc_on()) {
		shell_print(shell, "please iris power on first");
		return;
	}

	shell_print(shell, "%-30s | %-11s | %-11s | %-7s | %-7s | %-9s | %-7s | %-7s ",
		    "VR RAIL NAME", "FAST OCP(A)", "SLOW OCP(A)", "UVP(mV)", "OVP(mV)", "V MAX(mV)",
		    "LCR(mV)", "UCR(mV)");
	shell_print(
		shell,
		"----------------------------------------------------------------------------------------------------------------");
	// not include P3V3
	for (uint8_t i = 0; i < VR_RAIL_E_P3V3_OSFP_VOLT_V; i++) {
		uint8_t *rail_name = NULL;
		if (vr_rail_name_get((uint8_t)i, &rail_name)) {
			shell_print(shell,
				    "%-30s | %-11d | %-11d | %-7d | %-7d | %-9d | %-7d | %-7d ",
				    (char *)rail_name, (get_vr_reg_to_int(i, VR_FAST_OCP_REG) / 10),
				    (get_vr_reg_to_int(i, VR_SLOW_OCP_REG) / 10),
				    get_vr_reg_to_int(i, VR_UVP_REG),
				    get_vr_reg_to_int(i, VR_OVP_REG),
				    get_vr_reg_to_int(i, VR_VOUT_MAX_REG),
				    vout_range_user_settings.change_vout_min[i],
				    vout_range_user_settings.change_vout_max[i]);
		}
	}
}

void cmd_vr_test_mode_get_status(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", get_vr_test_mode_flag() ? "vr test mode" : "normal mode");
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_vr_test_mode_show_cmds,
	SHELL_CMD(default, NULL, "show  vr test mode default val", cmd_vr_test_mode_show_default),
	SHELL_CMD(test_mode_val, NULL, "show vr test mode val", cmd_vr_test_mode_show_val),
	SHELL_CMD(real, NULL, "show vr test mode real val", cmd_vr_test_mode_show_real),
	SHELL_SUBCMD_SET_END);
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_vr_test_mode_cmds,
	SHELL_CMD(start, NULL, "vr test mode start command", cmd_vr_test_mode_start),
	SHELL_CMD(exit, NULL, "vr test mode exit command", cmd_vr_test_mode_exit),
	SHELL_CMD(show, &sub_vr_test_mode_show_cmds, "show vr test mode val command", NULL),
	SHELL_CMD(get_status, NULL, "get vr current modecommand", cmd_vr_test_mode_get_status),
	SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(vr_test_mode, &sub_vr_test_mode_cmds, "VR test mode commands", NULL);
