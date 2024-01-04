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
#include "libutil.h"
#include "sensor_shell.h"
#include <stdlib.h>
#include <string.h>
#include <logging/log.h>

/*
 * Constants
 *
 * These constants are in the source file instead of the header file due to build problems 
 * with using the SHELL_STATIC_SUBCMD_SET_CREATE macro. 
 */

#define PINMASK_RESERVE_CHECK 1
#define GPIO_RESERVE_PREFIX "Reserve"
#define NUM_OF_GPIO_IS_DEFINE 167

char default_sensor_name[] = "Not support sensor name";

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
	sensor_name_to_num(pec_error)
	sensor_name_to_num(parameter_not_valid)
};
// clang-format on

/*
 * Helper Functions
 */

static bool table_access_check(uint16_t table_idx)
{
	if (table_idx < sensor_monitor_count) {
		bool (*access_checker)(uint8_t);
		access_checker = sensor_monitor_table[table_idx].access_checker;

		if (access_checker == NULL) {
			return true;
		} else {
			return (access_checker)(sensor_monitor_table[table_idx].access_checker_arg);
		}
	} else {
		printk("[%s] table index is invalid, table index: 0x%x\n", __func__, table_idx);
		return false;
	}
}

static sensor_cfg *sensor_get_idx_by_sensor_num(uint16_t table_idx, uint16_t sensor_num)
{
	if (table_idx < sensor_monitor_count) {
		sensor_cfg *cfg_table = sensor_monitor_table[table_idx].monitor_sensor_cfg;
		if (cfg_table == NULL) {
			printk("[%s] cfg table is NULL, table index: 0x%x\n", __func__, table_idx);
			return NULL;
		}

		uint8_t cfg_count = sensor_monitor_table[table_idx].cfg_count;

		return find_sensor_cfg_via_sensor_num(cfg_table, cfg_count, sensor_num);
	} else {
		printk("[%s] table index is invalid, table index: 0x%x\n", __func__, table_idx);
		return NULL;
	}
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

static int sensor_access(const struct shell *shell, uint16_t table_idx, sensor_cfg *cfg,
			 enum SENSOR_ACCESS mode)
{
	if (shell == NULL || cfg == NULL) {
		return -1;
	}

	if (cfg->num >= SENSOR_NUM_MAX || table_idx >= sensor_monitor_count) {
		return -1;
	}

	int sdr_index = -1;

	switch (mode) {
	/* Get sensor info by "sensor_config" table */
	case SENSOR_READ:
		if (table_idx == COMMON_SENSOR_TABLE_INDEX) {
			sdr_index = get_sdr_index_by_sensor_num(cfg->num);
			if (sdr_index == -1) {
				shell_error(shell, "[%s] can't find sensor number in sdr table.\n",
					    __func__);
				return -1;
			}
		}

		char sensor_name[MAX_SENSOR_NAME_LENGTH] = { 0 };
		if (sdr_index < 0) {
			snprintf(sensor_name, sizeof(sensor_name), "%s", default_sensor_name);
		} else {
			snprintf(sensor_name, sizeof(sensor_name), "%s",
				 full_sdr_table[sdr_index].ID_str);
		}

		char check_access = ((cfg->access_checker(cfg->num) == true) ? 'O' : 'X');
		char check_poll = ((cfg->is_enable_polling == true) ? 'O' : 'X');

		if (check_access == 'O') {
			if (cfg->cache_status == SENSOR_READ_4BYTE_ACUR_SUCCESS) {
				int16_t fraction = cfg->cache >> 16;
				int16_t integer = cfg->cache & 0xFFFF;
				shell_print(
					shell,
					"[0x%-2x] %-25s: %-10s | access[%c] | poll[%c] %-4d sec | %-25s | %5d.%03d",
					cfg->num, sensor_name, sensor_type_name[cfg->type],
					check_access, check_poll, (int)cfg->poll_time,
					sensor_status_name[cfg->cache_status], integer, fraction);
				break;
			} else if (cfg->cache_status == SENSOR_READ_SUCCESS ||
				   cfg->cache_status == SENSOR_READ_ACUR_SUCCESS) {
				shell_print(
					shell,
					"[0x%-2x] %-25s: %-10s | access[%c] | poll[%c] %-4d sec | %-25s | %-8d",
					cfg->num, sensor_name, sensor_type_name[cfg->type],
					check_access, check_poll, (int)cfg->poll_time,
					sensor_status_name[cfg->cache_status], cfg->cache);
				break;
			}
		}

		shell_print(shell,
			    "[0x%-2x] %-25s: %-10s | access[%c] | poll[%c] %-4d sec | %-25s | na",
			    cfg->num, sensor_name, sensor_type_name[cfg->type], check_access,
			    check_poll, (int)cfg->poll_time, sensor_status_name[cfg->cache_status]);
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

void cmd_sensor_cfg_list_all_table(const struct shell *shell, size_t argc, char **argv)
{
	if (shell == NULL) {
		return;
	}

	if (argc != 1) {
		shell_warn(shell, "Help: platform sensor list_all_table");
		return;
	}

	uint16_t table_index = 0;
	char check_access = 'X';

	for (table_index = 0; table_index < sensor_monitor_count; ++table_index) {
		char table_name[MAX_SENSOR_NAME_LENGTH] = { 0 };
		snprintf(table_name, sizeof(table_name), "%s",
			 sensor_monitor_table[table_index].table_name);

		check_access = ((table_access_check(table_index) == true) ? 'O' : 'X');

		shell_print(shell,
			    "Table name: %s | index: 0x%-2x | sensor count: 0x%-2x | access[%c]",
			    table_name, table_index, sensor_monitor_table[table_index].cfg_count,
			    check_access);
	}
}

void cmd_sensor_cfg_list_all_sensor(const struct shell *shell, size_t argc, char **argv)
{
	if (shell == NULL) {
		return;
	}

	if (argc != 1 && argc != 2) {
		shell_warn(shell, "Help: platform sensor list_all_sensor <keyword(optional)>");
		return;
	}

	int sdr_index = -1;
	uint16_t table_idx = 0;
	uint8_t sensor_idx = 0;
	char *keyword = NULL;
	if (argc == 2)
		keyword = argv[1];

	for (table_idx = 0; table_idx < sensor_monitor_count; ++table_idx) {
		if (sensor_monitor_table[table_idx].monitor_sensor_cfg == NULL) {
			shell_warn(shell, "[%s]: table idx: 0x%x sensor table is NULL, so skip",
				   __func__, table_idx);
			continue;
		}

		char table_name[MAX_SENSOR_NAME_LENGTH] = { 0 };
		char check_access = ((table_access_check(table_idx) == true) ? 'O' : 'X');
		sensor_cfg *cfg_table = sensor_monitor_table[table_idx].monitor_sensor_cfg;

		snprintf(table_name, sizeof(table_name), "%s",
			 sensor_monitor_table[table_idx].table_name);

		shell_print(
			shell,
			"---------------------------------------------------------------------------------");
		shell_print(shell,
			    "Table name: %s | index: 0x%-2x | sensor count: 0x%-2x | access[%c]\n",
			    table_name, table_idx, sensor_monitor_table[table_idx].cfg_count,
			    check_access);

		if (sensor_monitor_table[table_idx].cfg_count != 0) {
			for (sensor_idx = 0; sensor_idx < sensor_monitor_table[table_idx].cfg_count;
			     ++sensor_idx) {
				sensor_cfg *cfg = &cfg_table[sensor_idx];

				if (table_idx == COMMON_SENSOR_TABLE_INDEX) {
					sdr_index = get_sdr_index_by_sensor_num(cfg->num);
					if (sdr_index == -1) {
						shell_error(
							shell,
							"[%s] can't find sensor number: 0x%x in sdr table.\n",
							__func__, cfg->num);
						continue;
					}

					if (keyword &&
					    !strstr(full_sdr_table[sdr_index].ID_str, keyword) &&
					    !strstr(sensor_type_name[sensor_config[sensor_idx].type],
						    keyword)) {
						continue;
					}
				}

				sensor_access(shell, table_idx, cfg, SENSOR_READ);
			}
		}

		shell_print(
			shell,
			"---------------------------------------------------------------------------------");
	}
}

void cmd_sensor_cfg_get_table_all_sensor(const struct shell *shell, size_t argc, char **argv)
{
	if (shell == NULL) {
		return;
	}

	if (argc != 2) {
		shell_warn(shell, "Help: platform sensor get_table_all_senosr <table_index>");
		return;
	}

	uint8_t table_idx = strtol(argv[1], NULL, 16);

	if (sensor_monitor_table[table_idx].monitor_sensor_cfg == NULL) {
		shell_warn(shell, "[%s]: table idx: 0x%x sensor table is NULL", __func__,
			   table_idx);
		return;
	}

	uint8_t sensor_idx = 0;
	char table_name[MAX_SENSOR_NAME_LENGTH] = { 0 };
	char check_access = ((table_access_check(table_idx) == true) ? 'O' : 'X');
	sensor_cfg *cfg_table = sensor_monitor_table[table_idx].monitor_sensor_cfg;

	snprintf(table_name, sizeof(table_name), "%s", sensor_monitor_table[table_idx].table_name);

	shell_print(
		shell,
		"---------------------------------------------------------------------------------");
	shell_print(shell, "Table name: %s | index: 0x%-2x | sensor count: 0x%-2x | access[%c]\n",
		    table_name, table_idx, sensor_monitor_table[table_idx].cfg_count, check_access);

	if (sensor_monitor_table[table_idx].cfg_count != 0) {
		for (sensor_idx = 0; sensor_idx < sensor_monitor_table[table_idx].cfg_count;
		     ++sensor_idx) {
			sensor_cfg *cfg = &cfg_table[sensor_idx];

			if (table_idx == COMMON_SENSOR_TABLE_INDEX) {
				int sdr_index = get_sdr_index_by_sensor_num(cfg->num);
				if (sdr_index == -1) {
					shell_error(
						shell,
						"[%s] can't find sensor number: 0x%x in sdr table.\n",
						__func__, cfg->num);
					continue;
				}
			}

			sensor_access(shell, table_idx, cfg, SENSOR_READ);
		}
	}

	shell_print(
		shell,
		"---------------------------------------------------------------------------------");
}

void cmd_sensor_cfg_get_table_single_sensor(const struct shell *shell, size_t argc, char **argv)
{
	if (shell == NULL) {
		return;
	}

	if (argc != 3) {
		shell_warn(
			shell,
			"Help: platform sensor get_table_single_sensor <table_index> <sensor_num>");
		return;
	}

	uint8_t table_idx = strtol(argv[1], NULL, 16);
	uint8_t sensor_num = strtol(argv[2], NULL, 16);
	sensor_cfg *cfg = sensor_get_idx_by_sensor_num(table_idx, sensor_num);
	if (cfg == NULL) {
		shell_warn(shell, "[%s] fail to get sensor cfg, table idx: 0x%x, sensor num: 0x%x",
			   table_idx, sensor_num);
		return;
	}

	char table_name[MAX_SENSOR_NAME_LENGTH] = { 0 };
	char check_access = ((table_access_check(table_idx) == true) ? 'O' : 'X');

	snprintf(table_name, sizeof(table_name), "%s", sensor_monitor_table[table_idx].table_name);

	shell_print(
		shell,
		"---------------------------------------------------------------------------------");
	shell_print(shell, "Table name: %s | index: 0x%-2x | sensor count: 0x%-2x | access[%c]\n",
		    table_name, table_idx, sensor_monitor_table[table_idx].cfg_count, check_access);

	sensor_access(shell, table_idx, cfg, SENSOR_READ);
}

void cmd_control_sensor_polling(const struct shell *shell, size_t argc, char **argv)
{
	if (shell == NULL) {
		return;
	}

	if ((argc != 2) && (argc != 4)) {
		shell_warn(
			shell,
			"Help: platform sensor control_sensor_polling <table_index> <sensor_num> <operation>");
		return;
	}

	if (argc == 2) {
		/* Disable all table sensor monitor */
		uint8_t operation = strtol(argv[1], NULL, 16);

		if ((operation != DISABLE_SENSOR_POLLING) && (operation != ENABLE_SENSOR_POLLING)) {
			shell_warn(shell, "[%s]: operation is invalid, operation: %d", __func__,
				   operation);
			return;
		}

		uint16_t table_idx = 0;
		uint8_t sensor_index = 0;

		for (table_idx = 0; table_idx < sensor_monitor_count; ++table_idx) {
			sensor_cfg *cfg_table = sensor_monitor_table[table_idx].monitor_sensor_cfg;
			if (cfg_table == NULL) {
				shell_warn(shell, "[%s] cfg table is NULL, skip table index: 0x%x",
					   __func__, table_idx);
				continue;
			}

			for (sensor_index = 0;
			     sensor_index < sensor_monitor_table[table_idx].cfg_count;
			     ++sensor_index) {
				sensor_cfg *cfg = &cfg_table[sensor_index];
				cfg->is_enable_polling = operation;
			}
		}

		shell_print(shell, "All table sensors' polling successfully %s.",
			    ((operation == DISABLE_SENSOR_POLLING) ? "disabled" : "enabled"));
		return;
	}

	uint8_t table_idx = strtol(argv[1], NULL, 16);
	uint8_t sensor_num = strtol(argv[2], NULL, 16);
	uint8_t operation = strtol(argv[3], NULL, 16);

	uint8_t is_set_all = 0;

	sensor_cfg *cfg = sensor_get_idx_by_sensor_num(table_idx, sensor_num);
	if (cfg == NULL) {
		/* Set all sensor polling mode only suppot 1-base sensor number */
		if (sensor_num == 0) {
			is_set_all = 1;
		} else {
			shell_warn(
				shell,
				"[%s]: can't find sensor number in sensor config table, table index: 0x%x, sensor number: 0x%x",
				__func__, table_idx, sensor_num);
			return;
		}
	}

	if ((operation != DISABLE_SENSOR_POLLING) && (operation != ENABLE_SENSOR_POLLING)) {
		shell_warn(shell, "[%s]: operation is invalid, operation: %d", __func__, operation);
		return;
	}

	if (is_set_all) {
		uint8_t sensor_index = 0;
		sensor_cfg *cfg_table = sensor_monitor_table[table_idx].monitor_sensor_cfg;
		if (cfg_table == NULL) {
			shell_warn(
				shell,
				"[%s]: cfg table is NULL, table index: 0x%x, sensor number: 0x%x",
				__func__, table_idx, sensor_num);
			return;
		}

		for (sensor_index = 0; sensor_index < sensor_monitor_table[table_idx].cfg_count;
		     ++sensor_index) {
			sensor_cfg *sensor_cfg = &cfg_table[sensor_index];
			sensor_cfg->is_enable_polling = operation;
		}
		shell_print(shell, "All table: 0x%x sensors' polling successfully %s.", table_idx,
			    ((operation == DISABLE_SENSOR_POLLING) ? "disabled" : "enabled"));
		return;
	}

	cfg->is_enable_polling = ((operation == DISABLE_SENSOR_POLLING) ? DISABLE_SENSOR_POLLING :
									  ENABLE_SENSOR_POLLING);
	shell_print(shell, "Table idx: 0x%x, Sensor number 0x%x %s sensor polling success",
		    table_idx, sensor_num,
		    ((operation == DISABLE_SENSOR_POLLING) ? "disable" : "enable"));
	return;
}
