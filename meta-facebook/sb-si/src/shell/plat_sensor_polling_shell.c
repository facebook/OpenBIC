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

#include <zephyr.h>
#include <stdlib.h>
#include <shell/shell.h>
#include "plat_pldm_sensor.h"
#include "plat_sensor_polling_shell.h"

void cmd_set_plat_sensor_polling_all(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: set_sensor_polling all <value>");
		return;
	}
	int value = strtol(argv[1], NULL, 10);
	if (value != 0 && value != 1) {
		shell_warn(shell, "Help: set_sensor_polling all value should only accept 0 or 1");
		return;
	}

	set_plat_sensor_polling_enable_flag(value);
	shell_print(shell, "set_sensor_polling all -> %d ,success!", value);
	shell_print(shell, "Flag: all -> %d , adc-> %d , vr-> %d , temp-> %d",
		    get_plat_sensor_polling_enable_flag(),
		    get_plat_sensor_adc_polling_enable_flag(),
		    get_plat_sensor_vr_polling_enable_flag(),
		    get_plat_sensor_temp_polling_enable_flag());
	return;
}

void cmd_set_plat_sensor_polling_adc(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: set_sensor_polling adc <value>");
		return;
	}
	int value = strtol(argv[1], NULL, 10);
	if (value != 0 && value != 1) {
		shell_warn(shell, "Help: set_sensor_polling adc value should only accept 0 or 1");
		return;
	}

	set_plat_sensor_adc_polling_enable_flag(value);
	shell_print(shell, "set_sensor_polling adc -> %d ,success!", value);
	shell_print(shell, "Flag: all -> %d , adc -> %d, vr -> %d, temp -> %d",
		    get_plat_sensor_polling_enable_flag(),
		    get_plat_sensor_adc_polling_enable_flag(),
		    get_plat_sensor_vr_polling_enable_flag(),
		    get_plat_sensor_temp_polling_enable_flag());
	return;
}

void cmd_set_plat_sensor_polling_vr(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: set_sensor_polling vr <value>");
		return;
	}
	int value = strtol(argv[1], NULL, 10);
	if (value != 0 && value != 1) {
		shell_warn(shell, "Help: set_sensor_polling vr value should only accept 0 or 1");
		return;
	}

	set_plat_sensor_vr_polling_enable_flag(value);
	shell_print(shell, "set_sensor_polling vr -> %d ,success!", value);
	shell_print(shell, "Flag: all -> %d , adc -> %d, vr -> %d, temp -> %d",
		    get_plat_sensor_polling_enable_flag(),
		    get_plat_sensor_adc_polling_enable_flag(),
		    get_plat_sensor_vr_polling_enable_flag(),
		    get_plat_sensor_temp_polling_enable_flag());
	return;
}

void cmd_set_plat_sensor_polling_temp(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: set_sensor_polling temp <value>");
		return;
	}
	int value = strtol(argv[1], NULL, 10);
	if (value != 0 && value != 1) {
		shell_warn(shell, "Help: set_sensor_polling temp value should only accept 0 or 1");
		return;
	}

	set_plat_sensor_temp_polling_enable_flag(value);
	shell_print(shell, "set_sensor_polling temp -> %d ,success!", value);
	shell_print(shell, "Flag: all -> %d , adc -> %d, vr -> %d, temp -> %d",
		    get_plat_sensor_polling_enable_flag(),
		    get_plat_sensor_adc_polling_enable_flag(),
		    get_plat_sensor_vr_polling_enable_flag(),
		    get_plat_sensor_temp_polling_enable_flag());
	return;
}

void cmd_get_plat_sensor_polling_all(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "get_sensor_polling all -> %d , adc -> %d, vr -> %d, temp -> %d ",
		    get_plat_sensor_polling_enable_flag(),
		    get_plat_sensor_adc_polling_enable_flag(),
		    get_plat_sensor_vr_polling_enable_flag(),
		    get_plat_sensor_temp_polling_enable_flag());
	return;
}
