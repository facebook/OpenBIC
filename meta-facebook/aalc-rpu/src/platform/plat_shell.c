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
#include "common_i2c_mux.h"
#include "nct7363.h"

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

// mux
void cmd_switch_fb_mux(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t idx = strtoul(argv[1], NULL, 10);
	if ((idx < 1) || (idx > 14)) {
		shell_warn(shell, "wrong fb idx(1-14)");
		return;
	}

	uint8_t sensor_num = SENSOR_NUM_FB_1_FAN_TACH_RPM + idx - 1; // 1 base
	sensor_cfg *cfg = get_common_sensor_cfg_info(sensor_num);
	mux_config *pre_args = (mux_config *)cfg->pre_sensor_read_args;

	switch_sensor_mux(cfg);
	shell_warn(shell, "switch fb %d, bus: %d, mux_addr: 0x%02x, port: %d", idx, cfg->port,
		   (pre_args->target_addr << 1), pre_args->channel);
}
void cmd_switch_pb_mux(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t idx = strtoul(argv[1], NULL, 10);
	if ((idx < 1) || (idx > 3)) {
		shell_warn(shell, "wrong pb idx(1-3)");
		return;
	}

	uint8_t sensor_num = (idx == 1) ? SENSOR_NUM_PB_1_PUMP_TACH_RPM :
			     (idx == 2) ? SENSOR_NUM_PB_2_PUMP_TACH_RPM :
			     (idx == 3) ? SENSOR_NUM_PB_3_PUMP_TACH_RPM :
					  0xFF; // 1 base
	sensor_cfg *cfg = get_common_sensor_cfg_info(sensor_num);
	mux_config *pre_args = (mux_config *)cfg->pre_sensor_read_args;

	switch_sensor_mux(cfg);
	shell_warn(shell, "switch fb %d, bus: %d, mux_addr: 0x%02x, port: %d", idx, cfg->port,
		   (pre_args->target_addr << 1), pre_args->channel);
}
void cmd_switch_sb_mux(const struct shell *shell, size_t argc, char **argv)
{
	sensor_cfg *cfg = get_common_sensor_cfg_info(SENSOR_NUM_SB_TTV_COOLANT_LEAKAGE_1_VOLT_V);
	mux_config *pre_args = (mux_config *)cfg->pre_sensor_read_args;

	switch_sensor_mux(cfg);
	shell_warn(shell, "switch sb, bus: %d, mux_addr: 0x%02x, port: %d", cfg->port,
		   (pre_args->target_addr << 1), pre_args->channel);
}
void cmd_switch_pdb_mux(const struct shell *shell, size_t argc, char **argv)
{
	sensor_cfg *cfg = get_common_sensor_cfg_info(SENSOR_NUM_PDB_HDC1080DMBR_TEMP_C);
	mux_config *pre_args = (mux_config *)cfg->pre_sensor_read_args;

	switch_sensor_mux(cfg);
	shell_warn(shell, "switch pdb, bus: %d, mux_addr: 0x%02x, port: %d", cfg->port,
		   (pre_args->target_addr << 1), pre_args->channel);
}

// nct7363
void cmd_nct7363_fb(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t idx = strtoul(argv[1], NULL, 10);
	uint8_t offset = strtoul(argv[2], NULL, 16);
	uint8_t data = 0;

	if ((idx < 1) || (idx > 14)) {
		shell_warn(shell, "wrong fb idx(1-14)");
		return;
	}

	uint8_t sensor_num = SENSOR_NUM_FB_1_FAN_TACH_RPM + idx - 1; // 1 base
	sensor_cfg *cfg = get_common_sensor_cfg_info(sensor_num);
	data = nct7363_read_back_data(cfg, offset);

	shell_warn(shell, "nct7363 debug fb %d, bus: %d, addr: 0x%02x, val: 0x%02x", idx, cfg->port,
		   (cfg->target_addr << 1), data);
}
void cmd_nct7363_pb(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t idx = strtoul(argv[1], NULL, 10);
	uint8_t offset = strtoul(argv[2], NULL, 16);
	uint8_t data = 0;

	if ((idx < 1) || (idx > 3)) {
		shell_warn(shell, "wrong pb idx(1-3)");
		return;
	}

	uint8_t sensor_num = (idx == 1) ? SENSOR_NUM_PB_1_PUMP_TACH_RPM :
			     (idx == 2) ? SENSOR_NUM_PB_2_PUMP_TACH_RPM :
			     (idx == 3) ? SENSOR_NUM_PB_3_PUMP_TACH_RPM :
					  0xFF; // 1 base
	sensor_cfg *cfg = get_common_sensor_cfg_info(sensor_num);
	data = nct7363_read_back_data(cfg, offset);

	shell_warn(shell, "nct7363 debug pb %d, bus: %d, addr: 0x%02x, val: 0x%02x", idx, cfg->port,
		   (cfg->target_addr << 1), data);
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
// mux
SHELL_STATIC_SUBCMD_SET_CREATE(sub_mux_cmd,
			       SHELL_CMD(fb, NULL, "switch fan board mux", cmd_switch_fb_mux),
			       SHELL_CMD(pb, NULL, "switch pump board mux", cmd_switch_pb_mux),
			       SHELL_CMD(sb, NULL, "switch sensor board mux", cmd_switch_sb_mux),
			       SHELL_CMD(pdb, NULL, "switch pdb mux", cmd_switch_pdb_mux),
			       SHELL_SUBCMD_SET_END);

// nct7363
SHELL_STATIC_SUBCMD_SET_CREATE(sub_nct7363_cmd,
			       SHELL_CMD(fb, NULL, "nct7363 debug for fan board", cmd_nct7363_fb),
			       SHELL_CMD(pb, NULL, "nct7363 debug for pump board", cmd_nct7363_pb),
			       SHELL_SUBCMD_SET_END);

/* Sub-command Level 1 of command test */
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_test_cmds, SHELL_CMD(pwm, &sub_pwm_cmd, "set/get pwm command", NULL),
	SHELL_CMD(poll, NULL, "enable/disable sensor polling", cmd_sensor_polling),
	SHELL_CMD(mux, &sub_mux_cmd, "switch mux from sensor cfg", NULL),
	SHELL_CMD(nct7363, &sub_nct7363_cmd, "nct7363 debug command", NULL),
	SHELL_CMD(test, NULL, "test command", cmd_test), SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(test, &sub_test_cmds, "Test commands for AALC", NULL);
