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
#include "plat_class.h"
#include "plat_hook.h"
#include "plat_hamsa_avdd_pcie.h"

LOG_MODULE_REGISTER(plat_hamsa_avdd_pcie_shell, LOG_LEVEL_DBG);

static int get_vr_reg_to_int(uint8_t vr_rail, uint8_t reg)
{
	uint8_t data[2] = { 0 };
	if (!get_raw_data_from_sensor_id(vr_rail_table[vr_rail].sensor_id, reg, data, 2))
		return -1;

	uint16_t raw_val = (data[1] << 8) | data[0];

	return (int)raw_val;
}

static int cmd_hamsa_avdd_pcie_get(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t vr = get_vr_module();
	if (vr == VR_MODULE_RNS) {
		// RNS
		uint8_t i = VR_RAIL_E_ASIC_P0V8_HAMSA_AVDD_PCIE;
		shell_print(shell, "RNS: get_vr_fixed_uvp_ovp_enable_state: %s",
			    get_vr_fixed_uvp_ovp_enable(i) ? "enable" : "disable");
		shell_print(shell, "%-30s | %-7s | %-7s | %-9s | %-7s | %-7s ", "VR RAIL NAME",
			    "UVP(mV)", "OVP(mV)", "V MAX(mV)", "LCR(mV)", "UCR(mV)");
		shell_print(
			shell,
			"----------------------------------------------------------------------------------------------------------------");
		uint8_t *rail_name = NULL;
		if (vr_rail_name_get((uint8_t)i, &rail_name)) {
			uint16_t uvp = 0;
			uint16_t ovp = 0;
			if (get_vr_fixed_uvp_ovp_enable(i)) {
				uvp = get_vr_reg_to_int(i, VR_UVP_REG);
				ovp = get_vr_reg_to_int(i, VR_OVP_REG);
			} else if (!get_vr_offset_uvp_ovp(i, &uvp, &ovp)) {
				shell_error(shell, "get vr %d uvp/ovp fail1", i);
			}

			shell_print(shell, "%-30s | %-7d | %-7d | %-9d | %-7d | %-7d ",
				    (char *)rail_name, uvp, ovp,
				    get_vr_reg_to_int(i, VR_VOUT_MAX_REG),
				    vout_range_user_settings.change_vout_min[i],
				    vout_range_user_settings.change_vout_max[i]);
		}
	} else if (vr == VR_MODULE_MPS) {
		shell_print(shell, "MPS");
		shell_print(shell, "%-30s | %-9s | %-7s | %-7s ", "VR RAIL NAME", "V MAX(mV)",
			    "LCR(mV)", "UCR(mV)");
		shell_print(
			shell,
			"----------------------------------------------------------------------------------------------------------------");
		uint8_t i = VR_RAIL_E_ASIC_P0V8_HAMSA_AVDD_PCIE;
		uint8_t *rail_name = NULL;
		if (vr_rail_name_get((uint8_t)i, &rail_name)) {
			uint16_t vout_max = 0;
			get_vr_mp2971_reg(i, &vout_max, VOUT_MAX);

			shell_print(shell, "%-30s | %-9d | %-7d | %-7d", (char *)rail_name,
				    vout_max, vout_range_user_settings.change_vout_min[i],
				    vout_range_user_settings.change_vout_max[i]);
		}
	} else {
		shell_error(shell, "vr module not support");
	}

	return 0;
}

static int cmd_hamsa_avdd_pcie_set(const struct shell *shell, size_t argc, char **argv)
{
	bool is_perm = false;

	if (is_mb_dc_on()) {
		shell_error(shell, "Can't set hamsa_avdd_pcie voltage because iris power on");
		return -1;
	}

	if (argc == 3) {
		if (!strcmp(argv[2], "perm")) {
			is_perm = true;
		} else {
			shell_error(shell, "The last argument must be <perm>");
			return -1;
		}
	} else {
		shell_error(shell, "The command must be: hamsa_avdd_pcie set <new-voltage> perm");
		return -1;
	}

	uint8_t vr = get_vr_module();
	uint16_t millivolt = strtol(argv[1], NULL, 0);
	uint16_t vout_max_millivolt = 0;
	uint16_t vout_min_millivolt = 0;
	if (vr == VR_MODULE_MPS) {
		vout_max_millivolt = 840;
		vout_min_millivolt = 790;
	} else if (vr == VR_MODULE_RNS) {
		vout_max_millivolt = 850;
		vout_min_millivolt = 800;
	} else {
		shell_error(shell, "VR module not support");
		return -1;
	}

	if (millivolt < vout_min_millivolt || millivolt > vout_max_millivolt) {
		shell_error(shell, "hamsa_avdd_pcie cannot be less than %dmV or greater than %dmV",
			    vout_min_millivolt, vout_max_millivolt);
		return -1;
	}

	shell_info(shell, "Set hamsa_avdd_pcie vout to %d mV, %svolatile\n", millivolt,
		   (argc == 3) ? "non-" : "");

	if (!set_hamsa_avdd_pcie(&millivolt, is_perm)) {
		shell_error(shell, "set hamsa_avdd_pcie failed");
		return -1;
	}

	return 0;
}

/* level 1 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_hamsa_avdd_pcie_cmds,
			       SHELL_CMD(get, NULL, "get", cmd_hamsa_avdd_pcie_get),
			       SHELL_CMD(set, NULL, "set <new-voltage> perm",
					 cmd_hamsa_avdd_pcie_set),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(hamsa_avdd_pcie, &sub_hamsa_avdd_pcie_cmds,
		   "hamsa_avdd_pcie voltage set/get commands", NULL);
