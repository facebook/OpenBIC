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

#include "sdr.h"
#include "sensor.h"
#include "sensor_shell.h"
#include <stdlib.h>
#include <string.h>

/*
 * Constants
 *
 * These constants are in the source file instead of the header file due to build problems 
 * with using the SHELL_STATIC_SUBCMD_SET_CREATE macro. 
 */

#define PINMASK_RESERVE_CHECK 1
#define GPIO_RESERVE_PREFIX "Reserve"
#define NUM_OF_GPIO_IS_DEFINE 167

// clang-format off
const char *const sensor_status_name[] = {
	sensor_name_to_num(read_success)
	sensor_name_to_num(read_acur_success)
	sensor_name_to_num(not_found)
	sensor_name_to_num(not_accesible) 
	sensor_name_to_num(fail_to_access)
	sensor_name_to_num(init_status)
	sensor_name_to_num(unspecified_err)
	sensor_name_to_num(polling_disable)
	sensor_name_to_num(pre_read_error)
	sensor_name_to_num(post_read_error)
	sensor_name_to_num(api_unregister)
	sensor_name_to_num(4byte_acur_read_success)
	sensor_name_to_num(sensor_not_present)
};
// clang-format on

/*
 * Helper Functions
 */

static bool sensor_access_check(uint8_t sensor_num)
{
	bool (*access_checker)(uint8_t);
	access_checker = sensor_config[sensor_config_index_map[sensor_num]].access_checker;

	return (access_checker)(sensor_config[sensor_config_index_map[sensor_num]].num);
}

static int sensor_get_idx_by_sensor_num(uint16_t sensor_num)
{
	for (int sen_idx = 0; sen_idx < sensor_config_count; sen_idx++) {
		if (sensor_num == sensor_config[sen_idx].num)
			return sen_idx;
	}

	return -1;
}

static int get_sdr_index_by_sensor_num(uint8_t sensor_num)
{
	for (int index = 0; index < sdr_count; ++index) {
		if (sensor_num == full_sdr_table[index].sensor_num) {
			return index;
		}
	}

	return -1;
}

static int sensor_access(const struct shell *shell, int sensor_num, enum SENSOR_ACCESS mode)
{
	if (!shell) {
		return -1;
	}

	if (sensor_num >= SENSOR_NUM_MAX || sensor_num < 0) {
		return -1;
	}

	int sdr_index = get_sdr_index_by_sensor_num(sensor_num);
	if (sdr_index == -1) {
		shell_error(shell, "[%s] can't find sensor number in sdr table.\n", __func__);
		return -1;
	}

	switch (mode) {
	/* Get sensor info by "sensor_config" table */
	case SENSOR_READ:;
		int sen_idx = sensor_get_idx_by_sensor_num(sensor_num);
		if (sen_idx == -1) {
			shell_error(shell, "No such sensor number!");
			return -1;
		}
		char sensor_name[MAX_SENSOR_NAME_LENGTH] = { 0 };
		snprintf(sensor_name, sizeof(sensor_name), "%s", full_sdr_table[sdr_index].ID_str);

		char check_access =
			(sensor_access_check(sensor_config[sen_idx].num) == true) ? 'O' : 'X';
		char check_poll = (sensor_config[sen_idx].is_enable_polling == true) ? 'O' : 'X';

		if (check_access == 'O') {
			if (sensor_config[sen_idx].cache_status == SENSOR_READ_4BYTE_ACUR_SUCCESS) {
				int16_t fraction = sensor_config[sen_idx].cache >> 16;
				int16_t integer = sensor_config[sen_idx].cache & 0xFFFF;
				shell_print(
					shell,
					"[0x%-2x] %-25s: %-10s | access[%c] | poll[%c] %-4d sec | %-25s | %5d.%03d",
					sensor_config[sen_idx].num, sensor_name,
					sensor_type_name[sensor_config[sen_idx].type], check_access,
					check_poll, (int)sensor_config[sen_idx].poll_time,
					sensor_status_name[sensor_config[sen_idx].cache_status],
					integer, fraction);
				break;
			} else if (sensor_config[sen_idx].cache_status == SENSOR_READ_SUCCESS ||
				   sensor_config[sen_idx].cache_status ==
					   SENSOR_READ_ACUR_SUCCESS) {
				shell_print(
					shell,
					"[0x%-2x] %-25s: %-10s | access[%c] | poll[%c] %-4d sec | %-25s | %-8d",
					sensor_config[sen_idx].num, sensor_name,
					sensor_type_name[sensor_config[sen_idx].type], check_access,
					check_poll, (int)sensor_config[sen_idx].poll_time,
					sensor_status_name[sensor_config[sen_idx].cache_status],
					sensor_config[sen_idx].cache);
				break;
			}
		}

		shell_print(shell,
			    "[0x%-2x] %-25s: %-10s | access[%c] | poll[%c] %-4d sec | %-25s | na",
			    sensor_config[sen_idx].num, sensor_name,
			    sensor_type_name[sensor_config[sen_idx].type], check_access, check_poll,
			    (int)sensor_config[sen_idx].poll_time,
			    sensor_status_name[sensor_config[sen_idx].cache_status]);
		break;

	case SENSOR_WRITE:
		/* TODO */
		break;

	default:
		break;
	}

	return 0;
}

/*
 * Command Functions
 */

void cmd_sensor_cfg_list_all(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1 && argc != 2) {
		shell_warn(shell, "Help: platform sensor list_all <keyword(optional)>");
		return;
	}

	char *keyword = NULL;
	if (argc == 2)
		keyword = argv[1];

	if (sensor_config_count == 0) {
		shell_warn(shell, "[%s]: sensor monitor count is zero", __func__);
		return;
	}

	shell_print(
		shell,
		"---------------------------------------------------------------------------------");
	for (int sen_idx = 0; sen_idx < sensor_config_count; sen_idx++) {
		int sdr_index = get_sdr_index_by_sensor_num(sensor_config[sen_idx].num);
		if (sdr_index == -1) {
			shell_error(shell, "[%s] can't find sensor number in sdr table.\n",
				    __func__);
			return;
		}
		if (keyword && !strstr(full_sdr_table[sdr_index].ID_str, keyword) &&
		    !strstr(sensor_type_name[sensor_config[sen_idx].type], keyword))
			continue;

		sensor_access(shell, sensor_config[sen_idx].num, SENSOR_READ);
	}

	shell_print(
		shell,
		"---------------------------------------------------------------------------------");
}

void cmd_sensor_cfg_get(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: platform sensor get <sensor_num>");
		return;
	}

	int sen_num = strtol(argv[1], NULL, 16);

	sensor_access(shell, sen_num, SENSOR_READ);
}

void cmd_control_sensor_polling(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_warn(shell, "[%s]: input parameter count is invalid", __func__);
		return;
	}

	uint8_t sensor_num = strtol(argv[1], NULL, 16);
	uint8_t operation = strtol(argv[2], NULL, 16);

	uint8_t is_set_all = 0;

	int sensor_index = sensor_get_idx_by_sensor_num(sensor_num);
	if (sensor_index == -1) {
		/* Set all sensor polling mode only suppot 1-base sensor number */
		if (sensor_num == 0) {
			is_set_all = 1;
		} else {
			shell_warn(
				shell,
				"[%s]: can't find sensor number in sensor config table, sensor number: 0x%x",
				__func__, sensor_num);
			return;
		}
	}

	if ((operation != DISABLE_SENSOR_POLLING) && (operation != ENABLE_SENSOR_POLLING)) {
		shell_warn(shell, "[%s]: operation is invalid, operation: %d", __func__, operation);
		return;
	}

	if (is_set_all) {
		for (int sen_idx = 0; sen_idx < sensor_config_count; sen_idx++) {
			sensor_config[sen_idx].is_enable_polling = operation;
		}
		shell_print(shell, "All Sensors' polling successfully %s.",
			    ((operation == DISABLE_SENSOR_POLLING) ? "disabled" : "enabled"));
		return;
	}

	sensor_config[sensor_index].is_enable_polling =
		((operation == DISABLE_SENSOR_POLLING) ? DISABLE_SENSOR_POLLING :
							       ENABLE_SENSOR_POLLING);
	shell_print(shell, "Sensor number 0x%x %s sensor polling success", sensor_num,
		    ((operation == DISABLE_SENSOR_POLLING) ? "disable" : "enable"));
	return;
}
