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
#include "plat_class.h"

static bool cmd_is_not_ready(const struct shell *shell)
{
	uint8_t vr = get_vr_module();
	if (vr == VR_MODULE_MPS)
 	{
		shell_warn(shell, "MPS test mode is not ready");
		return false;
	}

	return true;
}

static bool cmd_is_dc_on(const struct shell *shell)
{
	if (!is_dc_on()) {
		shell_warn(shell, "please iris power on first");
		return false;
	}

	return true;
}

void cmd_vr_test_mode_start(const struct shell *shell, size_t argc, char **argv)
{
	if (!cmd_is_dc_on(shell))
		return;
	if (!cmd_is_not_ready(shell))
		return;
	vr_test_mode_enable(true);
}
void cmd_vr_test_mode_exit(const struct shell *shell, size_t argc, char **argv)
{
	if (!cmd_is_dc_on(shell))
		return;
	if (!cmd_is_not_ready(shell))
		return;
	vr_test_mode_enable(false);
}

void cmd_vr_test_mode_show_default(const struct shell *shell, size_t argc, char **argv)
{
	if (!cmd_is_dc_on(shell))
		return;
	if (!cmd_is_not_ready(shell))
		return;
	uint8_t vr = get_vr_module();
	if (vr == VR_MODULE_RNS) {
		// RNS
		shell_print(shell, "%-30s | %-11s | %-11s | %-7s | %-7s | %-9s | %-7s | %-7s ",
			    "VR RAIL NAME", "FAST OCP(A)", "SLOW OCP(A)", "UVP(mV)", "OVP(mV)",
			    "V MAX(mV)", "LCR(mV)", "UCR(mV)");
		shell_print(
			shell,
			"----------------------------------------------------------------------------------------------------------------");

		for (uint8_t i = 0; i < vr_test_mode_table_dafault_size; i++) {
			uint8_t *rail_name = NULL;
			if (vr_rail_name_get((uint8_t)i, &rail_name)) {
				const vr_test_mode_setting_t *cfg = &vr_test_mode_table_default[i];

				uint16_t uvp = 0;
				uint16_t ovp = 0;
				if (!use_offset_uvp_ovp(i)) {
					uvp = cfg->uvp;
					ovp = cfg->ovp;
				} else if (!get_vr_offset_uvp_ovp(i, &uvp, &ovp)) {
					shell_warn(shell, "get vr %d uvp/ovp fail", i);
				}

				shell_print(
					shell,
					"%-30s | %-11d | %-11d | %-7d | %-7d | %-9d | %-7d | %-7d ",
					(char *)rail_name, (cfg->fast_ocp / 10),
					((cfg->slow_ocp / 10)), uvp, ovp, cfg->vout_max, cfg->lcr,
					cfg->ucr);
			}
		}
	} else if (vr == VR_MODULE_MPS) {

		// MPS
		shell_print(shell, "MPS");
		shell_print(shell, "%-30s | %-12s | %-7s | %-8s | %-9s | %-9s | %-7s | %-7s ",
			    "VR RAIL NAME", "Total OCP(A)", "UVP(mv)", "OVP1(mV)", "OVP2(mV)",
			    "V MAX(mV)", "LCR(mV)", "UCR(mV)");
		shell_print(
			shell,
			"----------------------------------------------------------------------------------------------------------------");
		// mp29816c
		for (uint8_t i = 0; i < vr_mps_normal_mode_table_size; i++) {
			uint8_t *rail_name = NULL;
			if (vr_rail_name_get((uint8_t)i, &rail_name)) {
				const mps_vr_test_mode_setting_t *cfg =
					&vr_mps_normal_mode_table[i];
				shell_print(
					shell,
					"%-30s | %-12d | %-7d | %-8d | %-9d | %-9d | %-7d | %-7d",
					(char *)rail_name, cfg->total_ocp, cfg->uvp, cfg->ovp1,
					cfg->ovp2, cfg->vout_max, cfg->lcr, cfg->ucr);
			}
		}
	} else {
		shell_error(shell, "vr module not support");
	}
}

void cmd_vr_test_mode_show_val(const struct shell *shell, size_t argc, char **argv)
{
	if (!cmd_is_not_ready(shell))
		return;
	uint8_t vr = get_vr_module();
	if (vr == VR_MODULE_RNS) {
		// RNS
		shell_print(shell, "%-30s | %-11s | %-11s | %-7s | %-7s | %-9s | %-7s | %-7s ",
			    "VR RAIL NAME", "FAST OCP(A)", "SLOW OCP(A)", "UVP(mV)", "OVP(mV)",
			    "V MAX(mV)", "LCR(mV)", "UCR(mV)");
		shell_print(
			shell,
			"----------------------------------------------------------------------------------------------------------------");
		for (uint8_t i = 0; i < vr_test_mode_table_size; i++) {
			uint8_t *rail_name = NULL;
			if (vr_rail_name_get((uint8_t)i, &rail_name)) {
				const vr_test_mode_setting_t *cfg = &vr_test_mode_table[i];

				shell_print(
					shell,
					"%-30s | %-11d | %-11d | %-7d | %-7d | %-9d | %-7d | %-7d ",
					(char *)rail_name, (cfg->fast_ocp / 10),
					((cfg->slow_ocp / 10)), cfg->uvp, cfg->ovp, cfg->vout_max,
					cfg->lcr, cfg->ucr);
			}
		}
	} else if (vr == VR_MODULE_MPS) {
		// MPS
		shell_print(shell, "MPS");
		shell_print(shell, "%-30s | %-12s | %-7s | %-8s | %-9s | %-9s | %-7s | %-7s ",
			    "VR RAIL NAME", "Total OCP(A)", "UVP(mv)", "OVP1(mV)", "OVP2(mV)",
			    "V MAX(mV)", "LCR(mV)", "UCR(mV)");
		shell_print(
			shell,
			"----------------------------------------------------------------------------------------------------------------");
		// mp29816c
		char ovp2_str[16];
		char uvp_str[16];
		snprintf(ovp2_str, sizeof(ovp2_str), "no action");
		for (uint8_t i = 0; i < vr_mps_test_mode_table_size; i++) {
			uint8_t *rail_name = NULL;
			if (vr_rail_name_get((uint8_t)i, &rail_name)) {
				const mps_vr_test_mode_setting_t *cfg = &vr_mps_test_mode_table[i];
				uint8_t rail = cfg->vr_rail;
				uint16_t vout = 0;
				int32_t uvp_show = 0;
				get_vr_mp2971_reg(rail, &vout, VOUT_COMMAND);
				if (i >= VR_RAIL_E_ASIC_P0V9_OWL_E_TRVDD){
					/* uvp = vout- 400 mv (default) */
					uvp_show = (int32_t)vout - 400;
					if (uvp_show < 0)
						uvp_show = 0;
					snprintf(uvp_str, sizeof(uvp_str), "%d", uvp_show);
				}else{
					snprintf(uvp_str, sizeof(uvp_str), ">=200");
				}
				shell_print(
					shell,
					"%-30s | %-12d | %-7s | %-8d | %-9s | %-9d | %-7d | %-7d",
					(char *)rail_name, cfg->total_ocp, uvp_str, cfg->ovp1,
					ovp2_str, cfg->vout_max, cfg->lcr, cfg->ucr);
			}
		}
	} else {
		shell_error(shell, "vr module not support");
	}
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
	if (!cmd_is_dc_on(shell))
		return;
	if (!cmd_is_not_ready(shell))
		return;
	uint8_t vr = get_vr_module();
	if (vr == VR_MODULE_RNS) {
		// RNS
		shell_print(shell, "%-30s | %-11s | %-11s | %-7s | %-7s | %-9s | %-7s | %-7s ",
			    "VR RAIL NAME", "FAST OCP(A)", "SLOW OCP(A)", "UVP(mV)", "OVP(mV)",
			    "V MAX(mV)", "LCR(mV)", "UCR(mV)");
		shell_print(
			shell,
			"----------------------------------------------------------------------------------------------------------------");
		// not include P3V3
		for (uint8_t i = 0; i < VR_RAIL_E_P3V3_OSFP_VOLT_V; i++) {
			uint8_t *rail_name = NULL;
			if (vr_rail_name_get((uint8_t)i, &rail_name)) {
				uint16_t uvp = 0;
				uint16_t ovp = 0;
				if (get_vr_fixed_uvp_ovp_enable(i)) {
					uvp = get_vr_reg_to_int(i, VR_UVP_REG);
					ovp = get_vr_reg_to_int(i, VR_OVP_REG);
				} else if (!get_vr_offset_uvp_ovp(i, &uvp, &ovp))
					shell_error(shell, "get vr %d uvp/ovp fail", i);

				shell_print(
					shell,
					"%-30s | %-11d | %-11d | %-7d | %-7d | %-9d | %-7d | %-7d ",
					(char *)rail_name,
					(get_vr_reg_to_int(i, VR_FAST_OCP_REG) / 10),
					(get_vr_reg_to_int(i, VR_SLOW_OCP_REG) / 10), uvp, ovp,
					get_vr_reg_to_int(i, VR_VOUT_MAX_REG),
					vout_range_user_settings.change_vout_min[i],
					vout_range_user_settings.change_vout_max[i]);
			}
		}
	} else if (vr == VR_MODULE_MPS) {
		// MPS
		uint16_t uvp = 0;
		uint16_t vout_max = 0;
		uint16_t vout_command = 0;
		uint16_t vout_offset = 0;
		uint16_t total_ocp = 0;
		uint16_t ovp_1 = 0;
		uint16_t ovp_2 = 0;
		char ovp2_str[16];
		uint8_t test_mode = get_vr_test_mode_flag() ? 1 : 0;

		shell_print(shell, "MPS");
		shell_print(shell, "%-30s | %-12s | %-7s | %-8s | %-9s | %-9s | %-7s | %-7s ",
			    "VR RAIL NAME", "Total OCP(A)", "UVP(mv)", "OVP1(mV)", "OVP2(mV)",
			    "V MAX(mV)", "LCR(mV)", "UCR(mV)");
		shell_print(
			shell,
			"----------------------------------------------------------------------------------------------------------------");
		// not include P3V3
		for (uint8_t i = 0; i <= VR_RAIL_E_ASIC_P0V85_MEDHA1_VDD; i++) {
			uint8_t *rail_name = NULL;
			get_vr_mp29816a_reg(i, &uvp, UVP);
			get_vr_mp29816a_reg(i, &vout_max, VOUT_MAX);
			get_vr_mp29816a_reg(i, &vout_command, VOUT_COMMAND);
			get_vr_mp29816a_reg(i, &vout_offset, VOUT_OFFSET);
			get_vr_mp29816a_reg(i, &total_ocp, TOTAL_OCP);
			get_vr_mp29816a_reg(i, &ovp_1, OVP_1);
			get_vr_mp29816a_reg(i, &ovp_2, OVP_2);
			if (test_mode) {
				// test mode
				snprintf(ovp2_str, sizeof(ovp2_str), "no action");
			} else {
				// normal mode
				snprintf(ovp2_str, sizeof(ovp2_str), "%d", ovp_2);
			}
			if (vr_rail_name_get((uint8_t)i, &rail_name)) {
				shell_print(
					shell,
					"%-30s | %-12d | %-7d | %-8d | %-9s | %-9d | %-7d | %-7d",
					(char *)rail_name, total_ocp, uvp, ovp_1, ovp2_str,
					vout_max, vout_range_user_settings.change_vout_min[i],
					vout_range_user_settings.change_vout_max[i]);
			}
		}
		for (uint8_t j = VR_RAIL_E_ASIC_P0V9_OWL_E_TRVDD; j < VR_RAIL_E_P3V3_OSFP_VOLT_V;
		     j++) {
			uint8_t *rail_name = NULL;
			get_vr_mp2971_reg(j, &uvp, UVP);
			get_vr_mp2971_reg(j, &vout_max, VOUT_MAX);
			get_vr_mp2971_reg(j, &vout_command, VOUT_COMMAND);
			get_vr_mp2971_reg(j, &vout_offset, VOUT_OFFSET);
			get_vr_mp2971_reg(j, &total_ocp, TOTAL_OCP);
			get_vr_mp2971_reg(j, &ovp_1, OVP_1);
			get_vr_mp2971_reg(j, &ovp_2, OVP_2);
			if (test_mode) {
				// test mode
				snprintf(ovp2_str, sizeof(ovp2_str), "no action");
			} else {
				// normal mode
				snprintf(ovp2_str, sizeof(ovp2_str), "%d", ovp_2);
			}
			if (vr_rail_name_get((uint8_t)j, &rail_name)) {
				shell_print(
					shell,
					"%-30s | %-12d | %-7d | %-8d | %-9s | %-9d | %-7d | %-7d",
					(char *)rail_name, total_ocp, uvp, ovp_1, ovp2_str,
					vout_max, vout_range_user_settings.change_vout_min[j],
					vout_range_user_settings.change_vout_max[j]);
			}
		}
	} else {
		// unknown
		shell_error(shell, "unknown vr module");
	}
}

void cmd_vr_test_mode_get_status(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", get_vr_test_mode_flag() ? "vr test mode" : "normal mode");
}

void cmd_vr_test_mode_get_page(const struct shell *shell, size_t argc, char **argv)
{
	if (!cmd_is_dc_on(shell))
		return;

	enum VR_RAIL_E rail;
	if (vr_rail_enum_get(argv[1], &rail) == false) {
		shell_error(shell, "Invalid rail name: %s", argv[1]);
		return;
	}

	shell_print(shell, "vr %d get page %d", rail, get_vr_page(rail));
}

void cmd_vr_dma_read(const struct shell *shell, size_t argc, char **argv)
{
	if (!cmd_is_dc_on(shell))
		return;
	uint8_t vr = get_vr_module();
	if (vr != VR_MODULE_RNS) {
		shell_error(shell, "vr dma read only support RNS");
		return;
	}
	enum VR_RAIL_E rail;
	if (vr_rail_enum_get(argv[1], &rail) == false) {
		shell_error(shell, "Invalid rail name: %s", argv[1]);
		return;
	}

	uint16_t reg = strtoul(argv[2], NULL, 16);

	uint8_t data[2];
	memset(data, 0, 2);
	if (!dma_read_vr(rail, reg, data, 2)) {
		shell_warn(shell, "vr %d dma read from 0x%x fail", rail, reg);
		return;
	}

	shell_print(shell, "vr %d dma read from 0x%x:", rail, reg);
	shell_hexdump(shell, data, 2);
}
void cmd_vr_dma_write(const struct shell *shell, size_t argc, char **argv)
{
	if (!cmd_is_dc_on(shell))
		return;
	uint8_t vr = get_vr_module();
	if (vr != VR_MODULE_RNS) {
		shell_error(shell, "vr dma write only support RNS");
		return;
	}
	enum VR_RAIL_E rail;
	if (vr_rail_enum_get(argv[1], &rail) == false) {
		shell_error(shell, "Invalid rail name: %s", argv[1]);
		return;
	}

	uint16_t reg = strtoul(argv[2], NULL, 16);

	uint8_t data[2];
	data[0] = strtoul(argv[3], NULL, 16);
	data[1] = strtoul(argv[4], NULL, 16);
	if (!dma_write_vr(rail, reg, data, 2)) {
		shell_warn(shell, "vr %d dma write to 0x%x fail", rail, reg);
		return;
	}

	shell_print(shell, "vr %d dma write to 0x%x:", rail, reg);
}

void cmd_vr_pmbus_read(const struct shell *shell, size_t argc, char **argv)
{
	if (!cmd_is_dc_on(shell))
		return;

	enum VR_RAIL_E rail;
	if (vr_rail_enum_get(argv[1], &rail) == false) {
		shell_error(shell, "Invalid rail name: %s", argv[1]);
		return;
	}

	uint8_t reg = strtoul(argv[2], NULL, 16);

	uint8_t data[2];
	memset(data, 0, 2);
	if (!get_raw_data_from_sensor_id(vr_rail_table[rail].sensor_id, reg, data, 2)) {
		shell_warn(shell, "vr %d pmbus read %d from 0x%x fail", rail, reg);
		return;
	}

	shell_print(shell, "vr %d pmbus read from 0x%x:", rail, reg);
	shell_hexdump(shell, data, 2);
}
void cmd_vr_pmbus_write(const struct shell *shell, size_t argc, char **argv)
{
	if (!cmd_is_dc_on(shell))
		return;

	enum VR_RAIL_E rail;
	if (vr_rail_enum_get(argv[1], &rail) == false) {
		shell_error(shell, "Invalid rail name: %s", argv[1]);
		return;
	}

	uint8_t reg = strtoul(argv[2], NULL, 16);

	uint8_t data[2];
	data[0] = strtoul(argv[3], NULL, 16);
	data[1] = strtoul(argv[4], NULL, 16);
	if (!plat_set_vr_reg(rail, reg, data, 2)) {
		shell_warn(shell, "vr %d pmbus write to 0x%x fail", rail, reg);
		return;
	}

	shell_print(shell, "vr %d pmbus write to 0x%x:", rail, reg);
}

static void vr_rname_get_subcmds(size_t idx, struct shell_static_entry *entry)
{
	if (idx >= VR_RAIL_E_P3V3_OSFP_VOLT_V) {
		entry->syntax = NULL;
	} else {
		entry->syntax = (const char *)vr_rail_table[idx].sensor_name;
	}

	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}
SHELL_DYNAMIC_CMD_CREATE(sub_vr_test_mode_rname_cmds, vr_rname_get_subcmds);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_vr_test_mode_show_cmds,
	SHELL_CMD(default, NULL, "show  vr test mode default val", cmd_vr_test_mode_show_default),
	SHELL_CMD(test_mode_val, NULL, "show vr test mode val", cmd_vr_test_mode_show_val),
	SHELL_CMD(real, NULL, "show vr test mode real val", cmd_vr_test_mode_show_real),
	SHELL_SUBCMD_SET_END);
SHELL_STATIC_SUBCMD_SET_CREATE(sub_vr_dma_cmds,
			       SHELL_CMD(read, &sub_vr_test_mode_rname_cmds, "vr dma read",
					 cmd_vr_dma_read),
			       SHELL_CMD(write, &sub_vr_test_mode_rname_cmds, "vr dma write",
					 cmd_vr_dma_write),
			       SHELL_SUBCMD_SET_END);
SHELL_STATIC_SUBCMD_SET_CREATE(sub_vr_pmbus_cmds,
			       SHELL_CMD(read, &sub_vr_test_mode_rname_cmds, "vr pmbus read",
					 cmd_vr_pmbus_read),
			       SHELL_CMD(write, &sub_vr_test_mode_rname_cmds, "vr pmbus write",
					 cmd_vr_pmbus_write),
			       SHELL_SUBCMD_SET_END);
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_vr_test_mode_cmds,
	SHELL_CMD(start, NULL, "vr test mode start command", cmd_vr_test_mode_start),
	SHELL_CMD(exit, NULL, "vr test mode exit command", cmd_vr_test_mode_exit),
	SHELL_CMD(show, &sub_vr_test_mode_show_cmds, "show vr test mode val command", NULL),
	SHELL_CMD(get_status, NULL, "get vr current modecommand", cmd_vr_test_mode_get_status),
	SHELL_CMD(get_page, &sub_vr_test_mode_rname_cmds, "vr get page command",
		  cmd_vr_test_mode_get_page),
	SHELL_CMD(dma, &sub_vr_dma_cmds, "vr dma command", NULL),
	SHELL_CMD(pmbus, &sub_vr_pmbus_cmds, "vr pmbus command", NULL), SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(vr_test_mode, &sub_vr_test_mode_cmds, "VR test mode commands", NULL);
