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
#include "plat_status.h"
#include "plat_fsc.h"
#include "plat_hwmon.h"
#include "plat_fru.h"

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

	if (get_manual_pwm_flag(MANUAL_PWM_E_PUMP)) {
		shell_warn(shell, "pump is in manual mode");
		duty = get_manual_pwm_cache(MANUAL_PWM_E_PUMP);
		shell_warn(shell, "get pump group duty: %d", duty);
		duty = get_manual_pwm_cache(MANUAL_PWM_E_PUMP_1);
		shell_warn(shell, "get pump1 duty: %d", duty);
		duty = get_manual_pwm_cache(MANUAL_PWM_E_PUMP_2);
		shell_warn(shell, "get pump2 duty: %d", duty);
		duty = get_manual_pwm_cache(MANUAL_PWM_E_PUMP_3);
		shell_warn(shell, "get pump3 duty: %d", duty);
	} else {
		shell_warn(shell, "pump is in auto mode");
		duty = get_pwm_group_cache(PWM_GROUP_E_PUMP);
		shell_warn(shell, "get pump group duty: %d", duty);
	}
}
void cmd_get_hex_fan_duty(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t duty = 0xff;

	if (get_manual_pwm_flag(MANUAL_PWM_E_HEX_FAN)) {
		shell_warn(shell, "hex fan is in manual mode");
		duty = get_manual_pwm_cache(MANUAL_PWM_E_HEX_FAN);
		shell_warn(shell, "get hex fan duty: %d", duty);
	} else {
		shell_warn(shell, "hex fan is not in manual mode");
		duty = get_pwm_group_cache(PWM_GROUP_E_HEX_FAN);
		shell_warn(shell, "get hex fan duty: %d", duty);
	}
}
void cmd_get_rpu_fan_duty(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t duty = 0xff;

	if (get_manual_pwm_flag(MANUAL_PWM_E_RPU_FAN)) {
		shell_warn(shell, "rpu fan is in manual mode");
		duty = get_manual_pwm_cache(MANUAL_PWM_E_RPU_FAN);
		shell_warn(shell, "get rpu fan group duty: %d", duty);
		duty = get_manual_pwm_cache(MANUAL_PWM_E_PUMP_FAN_1);
		shell_warn(shell, "get pump fan 1 duty: %d", duty);
		duty = get_manual_pwm_cache(MANUAL_PWM_E_PUMP_FAN_2);
		shell_warn(shell, "get pump fan 2 duty: %d", duty);
		duty = get_manual_pwm_cache(MANUAL_PWM_E_PUMP_FAN_3);
		shell_warn(shell, "get pump fan 3 duty: %d", duty);
		duty = get_manual_pwm_cache(MANUAL_PWM_E_RPU_PCB_FAN);
		shell_warn(shell, "get rpu internal fan duty: %d", duty);
	} else {
		shell_warn(shell, "rpu fan is in auto mode");
		duty = get_pwm_group_cache(PWM_GROUP_E_RPU_FAN);
		shell_warn(shell, "get rpu fan group duty: %d", duty);
	}
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
	set_manual_pwm_flag(MANUAL_PWM_E_PUMP, 1);
	set_manual_pwm_cache(MANUAL_PWM_E_PUMP, duty);
	set_manual_pwm_cache(MANUAL_PWM_E_PUMP_1, duty);
	set_manual_pwm_cache(MANUAL_PWM_E_PUMP_2, duty);
	set_manual_pwm_cache(MANUAL_PWM_E_PUMP_3, duty);
}
void cmd_set_hex_fan_duty(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t duty = strtoul(argv[1], NULL, 10);

	shell_warn(shell, "set hex fan duty: %d", duty);
	set_manual_pwm_flag(MANUAL_PWM_E_HEX_FAN, 1);
	set_manual_pwm_cache(MANUAL_PWM_E_HEX_FAN, duty);
}
void cmd_set_rpu_fan_duty(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t duty = strtoul(argv[1], NULL, 10);

	shell_warn(shell, "set rpu fan duty: %d", duty);
	set_manual_pwm_flag(MANUAL_PWM_E_RPU_FAN, 1);
	set_manual_pwm_cache(MANUAL_PWM_E_RPU_FAN, duty);
	set_manual_pwm_cache(MANUAL_PWM_E_PUMP_FAN_1, duty);
	set_manual_pwm_cache(MANUAL_PWM_E_PUMP_FAN_2, duty);
	set_manual_pwm_cache(MANUAL_PWM_E_PUMP_FAN_3, duty);
	set_manual_pwm_cache(MANUAL_PWM_E_RPU_PCB_FAN, duty);
}
void cmd_set_fan_duty(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t idx = strtoul(argv[1], NULL, 10);
	uint8_t duty = strtoul(argv[2], NULL, 10);

	shell_warn(shell, "set fan %d duty: %d", idx, duty);
	plat_pwm_ctrl(idx, duty);
}

void cmd_set_pwm_manual_mode(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t onoff = strtoul(argv[1], NULL, 10);

	shell_warn(shell, "%s all pwm device manual mode", onoff ? "enable" : "disable");
	shell_warn(shell, "set all pwm device manual duty to default");
	for (uint8_t i = MANUAL_PWM_E_HEX_FAN; i < MANUAL_PWM_E_MAX; i++)
		set_manual_pwm_flag(i, (onoff ? 1 : 0));

	set_manual_pwm_cache_to_default();
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
					  SENSOR_NUM_PB_3_PUMP_TACH_RPM; // 1 base

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
	nct7363_read_back_data(cfg, offset, &data);

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
					  SENSOR_NUM_PB_3_PUMP_TACH_RPM; // 1 base

	sensor_cfg *cfg = get_common_sensor_cfg_info(sensor_num);
	nct7363_read_back_data(cfg, offset, &data);

	shell_warn(shell, "nct7363 debug pb %d, bus: %d, addr: 0x%02x, val: 0x%02x", idx, cfg->port,
		   (cfg->target_addr << 1), data);
}

// threshold
void cmd_threshold_tbl_get(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t sensor_num = strtoul(argv[1], NULL, 16);

	sensor_threshold *p = find_threshold_tbl_entry(sensor_num);
	if (p)
		shell_warn(
			shell,
			"threshold entry 0x%02x, type: %d, lcr: %02f, ucr: %02f, last_status: %d, last_val: %d",
			p->sensor_num, p->type, p->lcr, p->ucr, p->last_status, p->last_value);
	else
		shell_warn(shell, "threshold entry 0x%02x not found!", sensor_num);
}

void cmd_threshold_tbl_set(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t sensor_num = strtoul(argv[1], NULL, 16);
	uint8_t type = strtoul(argv[2], NULL, 10);
	uint16_t lcr = strtoul(argv[3], NULL, 10);
	uint16_t ucr = strtoul(argv[4], NULL, 10);

	sensor_threshold *p = find_threshold_tbl_entry(sensor_num);
	if (p)
		shell_warn(
			shell,
			"threshold entry 0x%02x, type: %d, lcr: %02f, ucr: %02f, last_status: %d, last_val: %d",
			p->sensor_num, p->type, p->lcr, p->ucr, p->last_status, p->last_value);
	else
		shell_warn(shell, "threshold entry 0x%02x not found!", sensor_num);

	p->type = type;
	p->lcr = lcr;
	p->ucr = ucr;
}

void cmd_threshold_pump_test(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t disable_dynamic = strtoul(argv[1], NULL, 10);

	set_status_flag(STATUS_FLAG_SPECIAL_MODE, SPECIAL_MODE_PUMP_THRESHOLD_DEBUG,
			(disable_dynamic ? 1 : 0));
}

// status
void cmd_status_leak_get(const struct shell *shell, size_t argc, char **argv)
{
	shell_warn(shell, "get leak status flag: %x", get_status_flag(STATUS_FLAG_LEAK));
}
void cmd_status_leak_set(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_warn(shell, "test status set leak [bit] [val]");
		return;
	}

	uint8_t bit = strtoul(argv[1], NULL, 10);
	uint8_t val = strtoul(argv[2], NULL, 10);

	set_status_flag(STATUS_FLAG_LEAK, bit, val);
	shell_warn(shell, "set leak status flag bit %d to %d", bit, val);
}
void cmd_status_failure_get(const struct shell *shell, size_t argc, char **argv)
{
	shell_warn(shell, "get failure status flag: %x", get_status_flag(STATUS_FLAG_FAILURE));
}
void cmd_status_failure_set(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_warn(shell, "test status set failure [bit] [val]");
		return;
	}

	uint8_t bit = strtoul(argv[1], NULL, 10);
	uint8_t val = strtoul(argv[2], NULL, 10);

	set_status_flag(STATUS_FLAG_FAILURE, bit, val);
	shell_warn(shell, "set failure status flag bit %d to %d", bit, val);
}
void cmd_status_auto_tune_get(const struct shell *shell, size_t argc, char **argv)
{
	shell_warn(shell, "get auto tune flag: %x", get_status_flag(STATUS_FLAG_AUTO_TUNE));
}
void cmd_status_auto_tune_set(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "test status set auto_tune [val]");
		return;
	}

	uint8_t val = strtoul(argv[1], NULL, 10);

	if (val) {
		for (uint8_t i = MANUAL_PWM_E_HEX_FAN; i <= MANUAL_PWM_E_RPU_FAN; i++) {
			if (get_manual_pwm_flag(i)) {
				shell_warn(shell, "turn off manual pwm control %d", i);
				set_manual_pwm_flag(i, 0);
			}
		}
	}

	set_status_flag(STATUS_FLAG_AUTO_TUNE, 0, val);

	shell_warn(shell, "set auto tune flag to %d", val);
}
void cmd_status_pump_redundant_get(const struct shell *shell, size_t argc, char **argv)
{
	shell_warn(shell, "get pump redundant flag: %x",
		   get_status_flag(STATUS_FLAG_PUMP_REDUNDANT));
}
void cmd_status_pump_redundant_set(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "test status set pump_redundant [val]");
		return;
	}

	uint8_t val = strtoul(argv[1], NULL, 10);

	set_status_flag(STATUS_FLAG_PUMP_REDUNDANT, 0xFF, val);

	shell_warn(shell, "set pump redundant flag to %d", val);
}

// fsc
static void cmd_fsc_debug_enable(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(shell);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const uint8_t enable = strtoul(argv[1], NULL, 16);
	shell_warn(shell, "fsc debug enable: %d", enable);

	fsc_debug_set(enable);
}
static void cmd_fsc_tbl_enable(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(shell);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const uint8_t enable = strtoul(argv[1], NULL, 10);
	shell_warn(shell, "fsc table enable: %d", enable);

	set_fsc_tbl_enable(enable);
}

// pump_redundant
static void cmd_pump_redundant_enable(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t enable = strtoul(argv[1], NULL, 10);
	shell_warn(shell, "pump redundant enable: %d", enable);

	pump_redundant_enable(enable);
}
static void cmd_pump_redundant_switch_day_set(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t type = strtoul(argv[1], NULL, 10);
	uint8_t time = strtoul(argv[2], NULL, 10);

	set_pump_redundant_switch_time_type(type);
	set_pump_redundant_switch_time(time);
	shell_warn(shell, "set pump redundant to %d %s", time, (type ? "minute" : "day"));
}

// modbus
static void cmd_modbus_write(const struct shell *shell, size_t argc, char **argv)
{
	uint16_t addr = strtoul(argv[1], NULL, 16);
	modbus_command_mapping *p = ptr_to_modbus_table(addr);
	if (!p) {
		shell_warn(shell, "cannot find modbus command 0x%04x", addr);
		return;
	}
	if (!p->wr_fn) {
		shell_warn(shell, "modbus command 0x%04x without write function", addr);
		return;
	}

	uint8_t num = strtoul(argv[2], NULL, 10);
	if (num > p->cmd_size) {
		shell_warn(shell, "modbus write command 0x%04x out of size %d", addr, num);
		return;
	}
	if (!num)
		num = 1;

	switch (addr) {
	case MODBUS_ENABLE_ABR_ADDR:
	case MODBUS_GET_SET_2ND_BOOT_UPDATE_FLAG_ADDR:
	case MODBUS_SET_FMC_WDT_ADDR:
	case MODBUS_DISABLE_ABR_ADDR:
		shell_warn(shell, "modbus write command 0x%04x not allowed, please use wedge400",
			   addr);
		return;
		break;
	default:
		break;
	}

	uint16_t data[num];
	memset(data, 0, num);
	for (uint16_t i = 0; i < num; i++)
		data[i] = strtoul(argv[3 + i], NULL, 10);

	p->start_addr = addr;
	p->data_len = num;
	memcpy(p->data, data, sizeof(uint16_t) * p->data_len);

	p->wr_fn(p);
}
static void cmd_modbus_read(const struct shell *shell, size_t argc, char **argv)
{
	uint16_t addr = strtoul(argv[1], NULL, 16);
	modbus_command_mapping *p = ptr_to_modbus_table(addr);
	if (!p) {
		shell_warn(shell, "cannot find modbus command 0x%04x", addr);
		return;
	}
	if (!p->rd_fn) {
		shell_warn(shell, "modbus command 0x%04x without read function", addr);
		return;
	}

	uint8_t num = strtoul(argv[2], NULL, 10);
	if (num > p->cmd_size) {
		shell_warn(shell, "modbus command read 0x%04x out of size %d", addr, num);
		return;
	}
	if (!num)
		num = 1;

	p->start_addr = addr;
	p->data_len = num;

	p->rd_fn(p);

	for (uint16_t i = 0; i < num; i++)
		shell_warn(shell, " 0x%04x", p->data[i]);
}

// test command
void cmd_test(const struct shell *shell, size_t argc, char **argv)
{
	// test code
}

void fru_print_cmd(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t board_fru_id = strtoul(argv[1], NULL, 10);
	print_fru_info(board_fru_id);
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
			       SHELL_CMD(manual, NULL, "set pwm device manual mode",
					 cmd_set_pwm_manual_mode),
			       SHELL_SUBCMD_SET_END);
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_get_status_cmd, SHELL_CMD(leak, NULL, "get leak status flag", cmd_status_leak_get),
	SHELL_CMD(failure, NULL, "get failure status flag", cmd_status_failure_get),
	SHELL_CMD(auto_tune, NULL, "get auto tune flag", cmd_status_auto_tune_get),
	SHELL_CMD(pump_redundant, NULL, "get pump redundant flag", cmd_status_pump_redundant_get),
	SHELL_SUBCMD_SET_END);
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_set_status_cmd, SHELL_CMD(leak, NULL, "set leak status flag", cmd_status_leak_set),
	SHELL_CMD(failure, NULL, "set failure status flag", cmd_status_failure_set),
	SHELL_CMD(auto_tune, NULL, "set auto tune flag", cmd_status_auto_tune_set),
	SHELL_CMD(pump_redundant, NULL, "set pump redundant flag", cmd_status_pump_redundant_set),
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

// threshold
SHELL_STATIC_SUBCMD_SET_CREATE(sub_threshold_cmd,
			       SHELL_CMD(get, NULL, "get threshold table", cmd_threshold_tbl_get),
			       SHELL_CMD(set, NULL, "set threshold table", cmd_threshold_tbl_set),
			       SHELL_CMD(pump_test, NULL, "disable pump dynamic threshold",
					 cmd_threshold_pump_test),
			       SHELL_SUBCMD_SET_END);

// status
SHELL_STATIC_SUBCMD_SET_CREATE(sub_status_cmd,
			       SHELL_CMD(get, &sub_get_status_cmd, "get status flag", NULL),
			       SHELL_CMD(set, &sub_set_status_cmd, "set status flag", NULL),
			       SHELL_SUBCMD_SET_END);

// fsc
SHELL_STATIC_SUBCMD_SET_CREATE(sub_fsc_cmd,
			       SHELL_CMD(debug, NULL, "fsc debug message", cmd_fsc_debug_enable),
			       SHELL_CMD(table, NULL, "fsc table enable", cmd_fsc_tbl_enable),
			       SHELL_SUBCMD_SET_END);

// pump redundant
SHELL_STATIC_SUBCMD_SET_CREATE(sub_pump_redundant_cmd,
			       SHELL_CMD(enable, NULL, "pump redundant enable/disable",
					 cmd_pump_redundant_enable),
			       SHELL_CMD(switch_time, NULL, "set pump redundant days",
					 cmd_pump_redundant_switch_day_set),
			       SHELL_SUBCMD_SET_END);

// modbus
SHELL_STATIC_SUBCMD_SET_CREATE(sub_modbus_cmd,
			       SHELL_CMD(write, NULL, "modbus write command", cmd_modbus_write),
			       SHELL_CMD(read, NULL, "modbus read command", cmd_modbus_read),
			       SHELL_SUBCMD_SET_END);

/* Sub-command Level 1 of command test */
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_test_cmds, SHELL_CMD(pwm, &sub_pwm_cmd, "set/get pwm command", NULL),
	SHELL_CMD(poll, NULL, "enable/disable sensor polling", cmd_sensor_polling),
	SHELL_CMD(mux, &sub_mux_cmd, "switch mux from sensor cfg", NULL),
	SHELL_CMD(nct7363, &sub_nct7363_cmd, "nct7363 debug command", NULL),
	SHELL_CMD(threshold, &sub_threshold_cmd, "threshold test command", NULL),
	SHELL_CMD(status, &sub_status_cmd, "status test command", NULL),
	SHELL_CMD(fsc, &sub_fsc_cmd, "fan speed control command", NULL),
	SHELL_CMD(pump_redundant, &sub_pump_redundant_cmd, "pump redundant command", NULL),
	SHELL_CMD(modbus, &sub_modbus_cmd, "modbus test command", NULL),
	SHELL_CMD(test, NULL, "test command", cmd_test),
	SHELL_CMD(fru, NULL, "fru command", fru_print_cmd), SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(test, &sub_test_cmds, "Test commands for AALC", NULL);
