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

LOG_MODULE_REGISTER(plat_perm_config_shell, LOG_LEVEL_DBG);

/* If any perm parameter are added, remember to update this function accordingly.　*/
static int cmd_perm_config_get(const struct shell *shell, size_t argc, char **argv)
{
	int config_count = 0;
	for (int i = 0; i < VR_RAIL_E_MAX; i++) {
		if ((get_board_type() == MINERVA_AEGIS_BD) && (i == 0))
			continue; // skip osfp p3v3 on AEGIS BD
		if (user_settings.vout[i] != 0xffff) {
			uint8_t *rail_name = NULL;
			if (!vr_rail_name_get((uint8_t)i, &rail_name)) {
				LOG_ERR("Can't find vr_rail_name by rail index: %x", i);
				continue;
			}
			shell_print(shell, "[%2d]%-50s val=%d", i, rail_name,
				    user_settings.vout[i]);
			config_count++;
		}
	}
	for (int i = 0; i < PLAT_TEMP_INDEX_THRESHOLD_TYPE_MAX; i++) {
		if (temp_threshold_user_settings.temperature_reg_val[i] != 0xffffffff) {
			uint8_t *temp_index_threshold_name = NULL;
			if (!temp_index_threshold_type_name_get((uint8_t)i,
								&temp_index_threshold_name)) {
				LOG_ERR("Can't find vr_rail_name by rail index: %x", i);
				continue;
			}
			shell_print(shell, "[%2d]%-50s val=%d", i, temp_index_threshold_name,
				    temp_threshold_user_settings.temperature_reg_val[i]);
			config_count++;
		}
	}

	char setting_data[4] = { 0 };

	if (get_user_settings_alert_level_from_eeprom(setting_data, sizeof(setting_data)) == -1) {
		LOG_ERR("get alert level user settings failed");
	} else {
		int32_t alert_level_value = ((setting_data[3] << 24) | (setting_data[2] << 16) |
					     (setting_data[1] << 8) | setting_data[0]);
		if (alert_level_value != 0xffffffff) {
			shell_print(shell, "power alert level                            val=%d",
				    alert_level_value);
			config_count++;
		}
	}

	uint8_t setting_data_for_soc_pcie_perst = 0xFF;
	if (!get_user_settings_soc_pcie_perst_from_eeprom(
		    &setting_data_for_soc_pcie_perst, sizeof(setting_data_for_soc_pcie_perst))) {
		LOG_ERR("get soc_pcie_perst user settings failed");
	} else {
		if (setting_data_for_soc_pcie_perst != 0xFF) {
			shell_print(shell, "soc_pcie_perst                            val=%d",
				    setting_data_for_soc_pcie_perst);
			config_count++;
		}
	}

	for (int i = 0; i < STRAP_INDEX_MAX; i++) {
		if (bootstrap_user_settings.user_setting_value[i] != 0xffff) {
			uint8_t *rail_name = NULL;
			if (!strap_name_get((uint8_t)i, &rail_name)) {
				LOG_ERR("Can't find strap_rail_name by rail index: %x", i);
				continue;
			}
			uint8_t drive_level = bootstrap_user_settings.user_setting_value[i] & 0xff;
			shell_print(shell, "[%2d]%-50s val=%d", i, rail_name, drive_level);
			config_count++;
		}
	}

	uint8_t setting_data_for_thermaltrip = 0xFF;
	if (!get_user_settings_thermaltrip_from_eeprom(&setting_data_for_thermaltrip,
						       sizeof(setting_data_for_thermaltrip))) {
		LOG_ERR("get thermaltrip user settings failed");
	} else {
		if (setting_data_for_thermaltrip != 0xFF) {
			shell_print(shell, "thermaltrip                            val=%s",
				    (setting_data_for_thermaltrip ? "enable" : "disable"));
			config_count++;
		}
	}

	uint8_t setting_data_for_throttle = 0xFF;
	if (!get_user_settings_throttle_from_eeprom(&setting_data_for_throttle,
						    sizeof(setting_data_for_throttle))) {
		LOG_ERR("get throttle user settings failed");
	} else {
		if (setting_data_for_throttle != 0xFF) {
			shell_print(shell, "throttle                            %s",
				    ((setting_data_for_throttle == 0x00) ?
					     "sense0 disable, sense1 disable" :
				     (setting_data_for_throttle == 0x40) ?
					     "sense0 disable, sense1 enable" :
				     (setting_data_for_throttle == 0x80) ?
					     "sense0 enable, sense1 disable" :
				     (setting_data_for_throttle == 0xC0) ?
					     "sense0 enable, sense1 enable" :
					     "unknown"));
			config_count++;
		}
	}

	if (!config_count) {
		shell_print(shell, "no perm parameter exist");
	}

	return 0;
}

static int cmd_perm_config_clear(const struct shell *shell, size_t argc, char **argv)
{
	if (!perm_config_clear()) {
		shell_print(shell, "Can't clear perm_config");
	}
	shell_print(shell, "Clear perm_config finish");
	return 0;
}

/* level 1 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_perm_config_cmds,
			       SHELL_CMD(get, NULL, "get perm_config", cmd_perm_config_get),
			       SHELL_CMD(clear, NULL, "clear perm_config", cmd_perm_config_clear),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(perm_config, &sub_perm_config_cmds, "perm_config get/clear commands", NULL);
