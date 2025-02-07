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
#include <string.h>
#include <stdio.h>
#include <zephyr.h>
#include "sensor_threshold_shell.h"
#include "plat_pldm_sensor.h"

#define MINERVA_THRESHOLD_UNIT 0.001

void cmd_set_sensor_threshold(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 4) {
		shell_warn(shell, "Help: set <sensor ID> <UCT/LCT> <value>");
		return;
	}

	uint32_t sensor_max_count = 0;
	sensor_max_count = plat_get_pdr_size(PLDM_NUMERIC_SENSOR_PDR);

	int sensorID = strtol(argv[1], NULL, 16);

	if (sensorID < UBC1_P12V_TEMP_C || sensorID > sensor_max_count) {
		shell_error(shell, "Help: Sensor ID: 0x%x is higher than 0x%x or lower than 0x1",
			    sensorID, sensor_max_count);
		return;
	}

	int result = 0;
	result = check_supported_threshold_with_sensor_id(sensorID);
	if (result != 0) {
		shell_error(shell, "Sensor ID 0x%x unsupported thresholds", sensorID);
		return;
	}

	char threshold_type[4] = { 0 };
	int value = 0;
	float threshold_value = 0;

	snprintf(threshold_type, sizeof(threshold_type), "%s", argv[2]);
	value = strtol(argv[3], NULL, 10);
	threshold_value = (float)value *
			  MINERVA_THRESHOLD_UNIT; // If user want to send 3.3V, "value" will be 3300

	if (strcmp(threshold_type, "UCT") == 0) {
		result = change_pdr_table_critical_high_with_sensor_id(sensorID, threshold_value);
		if (result != 0) {
			shell_error(shell, "Change critical high to pdr table failed");
			return;
		}
	} else if (strcmp(threshold_type, "LCT") == 0) {
		result = change_pdr_table_critical_low_with_sensor_id(sensorID, threshold_value);
		if (result != 0) {
			shell_error(shell, "Change critical low to pdr table failed");
			return;
		}
	} else {
		shell_error(shell, "Help: Need to send <UCT/LCT>");
		return;
	}

	shell_print(shell, "sensor_threshold set 0x%x %s %d success!", sensorID, threshold_type,
		    value);

	return;
}

void cmd_get_sensor_threshold(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: get <sensor ID>/all");
		return;
	}

	char threshold_all[4] = { 0 };
	float critical_high = 0;
	float critical_low = 0;
	int result = 0;

	uint32_t sensor_max_count = 0;
	sensor_max_count = plat_get_pdr_size(PLDM_NUMERIC_SENSOR_PDR);

	snprintf(threshold_all, sizeof(threshold_all), "%s", argv[1]);

	if (strcmp(threshold_all, "All") == 0 || strcmp(threshold_all, "all") == 0) {
		for (int i = UBC1_P12V_TEMP_C; i <= sensor_max_count; i++) {
			result = check_supported_threshold_with_sensor_id(i);
			if (result == 0) {
				char sensor_name[MAX_AUX_SENSOR_NAME_LEN] = { 0 };

				get_pdr_table_critical_high_and_low_with_sensor_id(
					i, &critical_high, &critical_low);

				pldm_get_sensor_name_via_sensor_id(i, sensor_name,
								   sizeof(sensor_name));

				shell_print(
					shell,
					"sensor ID: 0x%x  |  sensor name: %-50s  |  critical high: %10.3f  |  critical low: %10.3f",
					i, sensor_name, critical_high, critical_low);
				critical_high = 0;
				critical_low = 0;
			}
		}
	} else {
		int sensorID = strtol(argv[1], NULL, 16);

		if (sensorID < UBC1_P12V_TEMP_C || sensorID > sensor_max_count) {
			shell_error(shell, "Sensor ID 0x%x is higher than 0x%x or lower than 0x1",
				    sensorID, sensor_max_count);
			return;
		}

		result = check_supported_threshold_with_sensor_id(sensorID);
		if (result != 0) {
			shell_error(shell, "Sensor ID 0x%x unsupported thresholds", sensorID);
			return;
		}

		result = get_pdr_table_critical_high_and_low_with_sensor_id(
			sensorID, &critical_high, &critical_low);
		if (result != 0) {
			shell_error(shell, "Get sensor threshold failed");
			return;
		}

		char sensor_name[MAX_AUX_SENSOR_NAME_LEN] = { 0 };
		pldm_get_sensor_name_via_sensor_id(sensorID, sensor_name, sizeof(sensor_name));

		shell_print(
			shell,
			"sensor ID: 0x%x  |  sensor name: %-50s  |  critical high: %10.3f  |  critical low: %10.3f",
			sensorID, sensor_name, critical_high, critical_low);
	}

	return;
}

/* Sub-command Level 1 of command sensor threshold */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_sensor_threshold_cmds,
			       SHELL_CMD(set, NULL, "set_sensor_threshold",
					 cmd_set_sensor_threshold),
			       SHELL_CMD(get, NULL, "get_sensor_threshold",
					 cmd_get_sensor_threshold),
			       SHELL_SUBCMD_SET_END);

/* Root of command sensor threshold */
SHELL_CMD_REGISTER(sensor_threshold, &sub_sensor_threshold_cmds, "threshold commands for AG", NULL);
