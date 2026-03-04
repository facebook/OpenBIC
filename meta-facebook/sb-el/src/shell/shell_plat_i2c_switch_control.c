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
#include <string.h>
#include <logging/log.h>
#include "plat_gpio.h"
#include "plat_pldm_sensor.h"

LOG_MODULE_REGISTER(plat_i2c_switch_control_shell, LOG_LEVEL_INF);

static int cmd_i2c_switch_get_all(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "I2C Switch GPIO status:");

	shell_print(shell, "HAMSA_SW_EN  : %d", gpio_get(HAMSA_SW_EN));
	shell_print(shell, "NUWA0_SW_EN : %d", gpio_get(NUWA0_SW_EN));
	shell_print(shell, "NUWA1_SW_EN : %d", gpio_get(NUWA1_SW_EN));

	return 0;
}

static int cmd_i2c_switch_set_all(const struct shell *shell, size_t argc, char **argv)
{
	/* Usage: i2c_switch_control set all <0|1> */
	if (argc != 2) {
		shell_print(shell, "Usage: i2c_switch_control set all <0|1>");
		return -EINVAL;
	}

	long set_val = strtol(argv[1], NULL, 10);
	if (set_val != 0 && set_val != 1) {
		shell_error(shell, "Value must be 0 or 1");
		return -EINVAL;
	}

	if (set_val == 1) {
		/* stop VR polling before enabling I2C switches */
		if (get_plat_sensor_vr_polling_enable_flag()) {
			LOG_INF("Disable VR polling before I2C switch enable");
			set_plat_sensor_vr_polling_enable_flag(false);
		}

		gpio_set(HAMSA_SW_EN, 1);
		gpio_set(NUWA0_SW_EN, 1);
		gpio_set(NUWA1_SW_EN, 1);

		shell_print(shell, "Set all I2C switch GPIOs to 1 (VR polling disabled)");
		return 0;
	}

	/* set_val == 0 */
	gpio_set(HAMSA_SW_EN, 0);
	gpio_set(NUWA0_SW_EN, 0);
	gpio_set(NUWA1_SW_EN, 0);

	/* restore VR polling after disabling I2C switches */
	if (!get_plat_sensor_vr_polling_enable_flag()) {
		LOG_INF("Enable VR polling after I2C switch disable");
		set_plat_sensor_vr_polling_enable_flag(true);
	}

	shell_print(shell, "Set all I2C switch GPIOs to 0 (VR polling enabled)");
	return 0;
}

/* level 2 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_get_cmds,
			       SHELL_CMD(all, NULL, "get all", cmd_i2c_switch_get_all),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_set_cmds,
			       SHELL_CMD(all, NULL, "set all <0|1>", cmd_i2c_switch_set_all),
			       SHELL_SUBCMD_SET_END);

/* level 1 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_i2c_switch_control_cmds,
			       SHELL_CMD(get, &sub_get_cmds, "get all", NULL),
			       SHELL_CMD(set, &sub_set_cmds, "set all <0|1>", NULL),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(i2c_switch_control, &sub_i2c_switch_control_cmds,
		   "I2C switch control (HAMSA/NUWA0/NUWA1)", NULL);
