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
#include "plat_pwm.h"
#include "plat_threshold.h"
#include "sensor.h"
#include "plat_sensor_table.h"
#include "plat_util.h"

LOG_MODULE_REGISTER(plat_shell);

// sensor polling flag
void cmd_sensor_polling(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t onoff = strtoul(argv[1], NULL, 10);
	if (onoff == 0) {
		plat_disable_sensor_poll();
	} else if (onoff == 1) {
		plat_enable_sensor_poll();
	} else {
		shell_warn(shell, "poll flag: %d", get_sensor_poll_enable_flag() ? 1 : 0);
	}
}

// pwm
void cmd_get_pump_duty(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t duty = 0xff;
	duty = get_pwm_group_cache(PWM_GROUP_E_PUMP);
	shell_warn(shell, "get pump duty: %d", duty);
}
void cmd_get_hex_fan_duty(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t duty = 0xff;
	duty = get_pwm_group_cache(PWM_GROUP_E_HEX_FAN);
	shell_warn(shell, "get hex fan duty: %d", duty);
}
void cmd_get_rpu_fan_duty(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t duty = 0xff;
	duty = get_pwm_group_cache(PWM_GROUP_E_RPU_FAN);
	shell_warn(shell, "get rpu fan duty: %d", duty);
}
void cmd_get_fan_duty(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t idx = strtoul(argv[1], NULL, 10);
	uint8_t duty = 0xff;

	duty = get_pwm_cache(idx);
	shell_warn(shell, "get fan %d duty: %d", idx, duty);
}

void cmd_set_pump_duty(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t duty = strtoul(argv[1], NULL, 10);

	shell_warn(shell, "set pump duty: %d", duty);
	set_pwm_group(PWM_GROUP_E_PUMP, duty);
}
void cmd_set_hex_fan_duty(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t duty = strtoul(argv[1], NULL, 10);

	shell_warn(shell, "set hex fan duty: %d", duty);
	set_pwm_group(PWM_GROUP_E_HEX_FAN, duty);
}
void cmd_set_rpu_fan_duty(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t duty = strtoul(argv[1], NULL, 10);

	shell_warn(shell, "set rpu fan duty: %d", duty);
	set_pwm_group(PWM_GROUP_E_RPU_FAN, duty);
}
void cmd_set_fan_duty(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t idx = strtoul(argv[1], NULL, 10);
	uint8_t duty = strtoul(argv[2], NULL, 10);

	shell_warn(shell, "set fan %d duty: %d", idx, duty);
	plat_pwm_ctrl(idx, duty);
}

// test command
void cmd_test(const struct shell *shell, size_t argc, char **argv)
{
	// test code
}

/* Sub-command Level 3 of command test */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_get_pwm_cmd,
			       SHELL_CMD(pump, NULL, "get pump duty", cmd_get_pump_duty),
			       SHELL_CMD(hex_fan, NULL, "get hex fan duty", cmd_get_hex_fan_duty),
			       SHELL_CMD(rpu_fan, NULL, "get rpu fan duty", cmd_get_rpu_fan_duty),
			       SHELL_CMD(single, NULL, "get fan device duty", cmd_get_fan_duty),
			       SHELL_SUBCMD_SET_END);
SHELL_STATIC_SUBCMD_SET_CREATE(sub_set_pwm_cmd,
			       SHELL_CMD(pump, NULL, "set pump duty", cmd_set_pump_duty),
			       SHELL_CMD(hex_fan, NULL, "set hex fan duty", cmd_set_hex_fan_duty),
			       SHELL_CMD(rpu_fan, NULL, "set rpu fan duty", cmd_set_rpu_fan_duty),
			       SHELL_CMD(single, NULL, "set fan device duty", cmd_set_fan_duty),
			       SHELL_SUBCMD_SET_END);

/* Sub-command Level 2 of command test */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_pwm_cmd, SHELL_CMD(get, &sub_get_pwm_cmd, "get duty", NULL),
			       SHELL_CMD(set, &sub_set_pwm_cmd, "set duty", NULL),
			       SHELL_SUBCMD_SET_END);

/* Sub-command Level 1 of command test */
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_test_cmds, SHELL_CMD(pwm, &sub_pwm_cmd, "set/get pwm command", NULL),
	SHELL_CMD(poll, NULL, "enable/disable sensor polling", cmd_sensor_polling),
	SHELL_CMD(test, NULL, "test command", cmd_test), SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(test, &sub_test_cmds, "Test commands for AALC", NULL);
