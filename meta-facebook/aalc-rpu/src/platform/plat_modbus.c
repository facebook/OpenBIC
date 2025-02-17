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
#include <sys/util.h>
#include <sys/byteorder.h>
#include <drivers/gpio.h>
#include <modbus/modbus.h>
#include <time.h>
#include <logging/log.h>
#include "sensor.h"
#include "modbus_server.h"
#include "fru.h"
#include "eeprom.h"
#include "libutil.h"
#include "plat_modbus.h"
#include "plat_sensor_table.h"
#include "plat_fru.h"
#include "hal_gpio.h"
#include "plat_gpio.h"
#include <modbus_internal.h>
#include "plat_util.h"
#include "plat_pwm.h"
#include "util_sys.h"
#include "util_spi.h"
#include "plat_version.h"
#include "plat_hwmon.h"
#include "plat_log.h"
#include "plat_i2c.h"
#include "plat_threshold.h"
#include "plat_fsc.h"
#include "plat_led.h"
#include "plat_status.h"
#include "plat_fru_date.h"
#include "plat_isr.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_modbus);

#define FW_UPDATE_SWITCH_FC 0x64
#define FW_UPDATE_SWITCH_ADDR 0x0119
#define FW_UPDATE_ENABLE_DATA 0x0101
#define FW_UPDATE_DISABLE_DATA 0x0100

#define UPADTE_FW_DATA_LENGTH_MIN 3 // contain 2 regs(offeset)+ 1 reg(length) at least
#define LOG_BEGIN_MODBUS_ADDR MODBUS_EVENT_1_ERROR_LOG_ADDR //Event 1 Error log Modbus Addr

#define TIMER_SINCE_LAST_ON_LENGTH 4 // timer_since_last_on is 4 bytes
#define BOARD_MFG_DATA_READ_START_OFFSET 3

#define IPMI_FRU_DATA_LENGTH_MASK BIT_MASK(6)
//{ DT_PROP(DT_INST(0, zephyr_modbus_serial), label) }

typedef struct {
	char *iface_name;
	bool is_custom_fc64;
	uint8_t addr;
} modbus_server;
modbus_server modbus_server_config[] = {
	// iface_name, is_custom_fc64
	{ "MODBUS0", true },
	{ "MODBUS1", false },
	{ "MODBUS2", false },
};

/*
	arg0: sensor number
	arg1: m
	arg2: r

	actual_val =  raw_val * m * (10 ^ r)
*/
uint8_t modbus_get_senser_reading(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	float val = 0;
	uint8_t status = get_sensor_reading_to_real_val(cmd->arg0, &val);

	/* bic update workaround */
	if (cmd->addr == MODBUS_BPB_RPU_COOLANT_FLOW_RATE_LPM_ADDR &&
	    status == SENSOR_UNSPECIFIED_ERROR) {
		cmd->data[0] = 0xFFFF; // error
		return MODBUS_EXC_NONE;
	}

	if (status == SENSOR_READ_4BYTE_ACUR_SUCCESS) {
		float r = pow_of_10(cmd->arg2);
		int16_t byte_val = 0;
		// capacity scale kW to W
		if (cmd->addr == MODBUS_AALC_COOLING_CAPACITY_W_ADDR) {
			int32_t scaled_val = (int32_t)(val * 1000 / cmd->arg1 / r);
			byte_val = scaled_val & 0xFFFF;
		} else if (cmd->addr == MODBUS_AALC_COOLING_CAPACITY_W_EXT_ADDR) {
			int32_t scaled_val = (int32_t)(val * 1000 / cmd->arg1 / r);
			byte_val = (scaled_val >> 16) & 0xFFFF;
		} else {
			byte_val = val / cmd->arg1 / r; // scale
		}

		memcpy(cmd->data, &byte_val, sizeof(uint16_t) * cmd->cmd_size);
		return MODBUS_EXC_NONE;
	}

	return MODBUS_EXC_SERVER_DEVICE_FAILURE;
}

uint8_t modbus_read_fruid_data(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint8_t status;
	EEPROM_ENTRY fru_entry;

	fru_entry.config.dev_id = cmd->arg0; //fru id
	fru_entry.offset = (cmd->start_addr - cmd->addr) * 2;
	fru_entry.data_len = (cmd->data_len) * 2;

	status = FRU_read(&fru_entry);
	if (status != FRU_READ_SUCCESS)
		return MODBUS_EXC_SERVER_DEVICE_FAILURE;

	memcpy(cmd->data, &fru_entry.data[0], fru_entry.data_len);

	regs_reverse(cmd->data_len, cmd->data);

	return MODBUS_EXC_NONE;
}

uint8_t modbus_write_fruid_data(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint8_t status;
	EEPROM_ENTRY fru_entry;

	fru_entry.config.dev_id = cmd->arg0; //fru id
	fru_entry.offset = (cmd->start_addr - cmd->addr) * 2;
	fru_entry.data_len = (cmd->data_len) * 2;

	regs_reverse(cmd->data_len, cmd->data);

	memcpy(&fru_entry.data[0], cmd->data, fru_entry.data_len);
	status = FRU_write(&fru_entry);
	if (status != FRU_WRITE_SUCCESS)
		return MODBUS_EXC_SERVER_DEVICE_FAILURE;

	return MODBUS_EXC_NONE;
}

uint8_t modbus_get_fw_reversion(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint8_t ver[4] = { BIC_FW_YEAR_MSB, BIC_FW_YEAR_LSB, BIC_FW_WEEK, BIC_FW_VER };

	for (uint8_t i = 0; i < cmd->cmd_size; i++) {
		char tmp[3] = { 0 };
		sprintf(tmp, "%02x", ver[i]);
		cmd->data[i] = tmp[1] << 8 | tmp[0];
	}

	return MODBUS_EXC_NONE;
}

uint8_t modbus_fw_download(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint32_t offset = cmd->data[0] << 16 | cmd->data[1]; // offset
	uint16_t msg_len = cmd->data[2] & 0x7FFF; // length
	uint8_t flag = (cmd->data[2] & BIT(15)) ? (SECTOR_END_FLAG | NO_RESET_FLAG) : 0;

	if (cmd->data_len <= UPADTE_FW_DATA_LENGTH_MIN)
		return MODBUS_EXC_ILLEGAL_DATA_VAL;

	if (msg_len != ((cmd->data_len - UPADTE_FW_DATA_LENGTH_MIN) * 2))
		return MODBUS_EXC_ILLEGAL_DATA_VAL;

	regs_reverse(cmd->data_len, cmd->data);

	return fw_update(offset, msg_len, (uint8_t *)&cmd->data[UPADTE_FW_DATA_LENGTH_MIN], flag,
			 DEVSPI_FMC_CS0);
}

uint8_t modbus_command_i2c_master_write_read_response(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	modbus_i2c_master_write_read_response(cmd->data);

	return MODBUS_EXC_NONE;
}

uint8_t modbus_command_i2c_master_write_read(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	if (!modbus_i2c_master_write_read(cmd->data, cmd->data_len))
		return MODBUS_EXC_NONE;
	return MODBUS_EXC_SERVER_DEVICE_FAILURE;
}

static uint8_t i2c_scan_bus = 0;
uint8_t modbus_command_i2c_scan_bus_set(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	if (cmd->data[0] >= I2C_BUS_MAX_NUM)
		return MODBUS_EXC_ILLEGAL_DATA_VAL;

	i2c_scan_bus = (uint8_t)cmd->data[0];
	return MODBUS_EXC_NONE;
}

uint8_t modbus_command_i2c_scan(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint8_t addr[cmd->data_len], len;
	i2c_scan(i2c_scan_bus, addr, &len);

	memset(cmd->data, 0xFF, sizeof(uint16_t) * cmd->data_len);
	for (uint8_t i = 0; i < len; i++)
		cmd->data[i] = addr[i];

	return MODBUS_EXC_NONE;
}

uint8_t modbus_write_hmi_version(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint8_t pre_version[EEPROM_HMI_VERSION_SIZE] = { 0 };
	if (!plat_eeprom_read(EEPROM_HMI_VERSION_OFFSET, pre_version, EEPROM_HMI_VERSION_SIZE)) {
		LOG_ERR("read hmi version fail!");
		return MODBUS_EXC_SERVER_DEVICE_FAILURE;
	}

	LOG_HEXDUMP_INF(pre_version, EEPROM_HMI_VERSION_SIZE, "read version");
	LOG_HEXDUMP_INF(cmd->data, EEPROM_HMI_VERSION_SIZE, "write version");

	if (memcmp(pre_version, cmd->data, EEPROM_HMI_VERSION_SIZE)) {
		if (!plat_eeprom_write(EEPROM_HMI_VERSION_OFFSET, (uint8_t *)cmd->data,
				       EEPROM_HMI_VERSION_SIZE)) {
			LOG_ERR("write hmi version fail!");
			return MODBUS_EXC_SERVER_DEVICE_FAILURE;
		}
	}

	return MODBUS_EXC_NONE;
}

uint8_t modbus_read_hmi_version(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint8_t version[EEPROM_HMI_VERSION_SIZE] = { 0 };
	if (!plat_eeprom_read(EEPROM_HMI_VERSION_OFFSET, version, EEPROM_HMI_VERSION_SIZE)) {
		LOG_ERR("read hmi version fail!");
		return MODBUS_EXC_SERVER_DEVICE_FAILURE;
	}

	memcpy(cmd->data, version, EEPROM_HMI_VERSION_SIZE);
	regs_reverse(cmd->data_len, cmd->data);

	return MODBUS_EXC_NONE;
}

uint8_t modbus_leakage_status_read(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint8_t leakage_status_table[] = { IT_LEAK_ALERT0_R, IT_LEAK_ALERT1_R, IT_LEAK_ALERT2_R,
					   IT_LEAK_ALERT3_R };

	uint16_t state = 0;

	for (uint8_t i = 0; i < ARRAY_SIZE(leakage_status_table); i++) {
		int leakage_val = gpio_get(leakage_status_table[i]);

		if (leakage_val == 0 || leakage_val == 1) {
			WRITE_BIT(state, i, (bool)leakage_val);
		} else {
			LOG_ERR("read leakage status fail!");
			return MODBUS_EXC_SERVER_DEVICE_FAILURE;
		}
	}

	WRITE_BIT(state, 4, (get_status_flag(STATUS_FLAG_LEAK) >> AALC_STATUS_CDU_LEAKAGE) & 0x01);
	WRITE_BIT(state, 12,
		  (get_status_flag(STATUS_FLAG_LEAK) >> AALC_STATUS_RACK_LEAKAGE) & 0x01);

	cmd->data[0] = state;

	return MODBUS_EXC_NONE;
}

pump_reset_struct modbus_pump_setting_table[] = {
	{ PUMP_REDUNDENT_SWITCHED, pump_setting_set_pump_redundant, 0 },
	{ MANUAL_CONTROL_PUMP, pump_setting_set_manual_flag, 0 },
	{ MANUAL_CONTROL_FAN, pump_setting_set_manual_flag, 0 },
	{ AUTOTUNE_FLOW_CONTROL, pump_setting_set_auto_tune_flag, 0 },
	{ AUTOTUNE_PRESSURE_BALANCE_CONTROL, modbus_pump_setting_unsupport_function, 0 },
	{ SYSTEM_STOP, close_pump, 0 },
	{ RPU_REMOTE_POWER_CYCLE, rpu_remote_power_cycle_function, 0 },
	{ MANUAL_CONTROL, modbus_pump_setting_unsupport_function, 0 },
	{ CLEAR_PUMP_RUNNING_TIME, modbus_clear_pump_running_time_function, 0 },
	{ CLEAR_LOG, clear_log_for_modbus_pump_setting, 0 },
	{ PUMP_1_RESET, pump_setting_pump1_reset, 0 },
	{ PUMP_2_RESET, pump_setting_pump2_reset, 0 },
	{ PUMP_3_RESET, pump_setting_pump3_reset, 0 },
};

static uint16_t pump_setting;
uint8_t modbus_pump_setting_get(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);
	cmd->data[0] = pump_setting;
	WRITE_BIT(cmd->data[0], 1, get_manual_pwm_flag(MANUAL_PWM_E_PUMP));
	return MODBUS_EXC_NONE;
}
uint8_t modbus_pump_setting(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);
	uint16_t check_error_flag = 0;
	for (int i = 0; i < ARRAY_SIZE(modbus_pump_setting_table); i++) {
		uint8_t func_idx = modbus_pump_setting_table[i].function_index;
		// check bit value is 0 or 1
		uint8_t input_bit_value = (cmd->data[0] & BIT(func_idx)) ? 1 : 0;
		// check auto tune enable
		if (input_bit_value) {
			if (func_idx == MANUAL_CONTROL_PUMP || func_idx == MANUAL_CONTROL_FAN) {
				if (cmd->data[0] & 0x08) // check auto tune enable
					input_bit_value = 2; // error input val
			}
		}

		bool result_status = modbus_pump_setting_table[i].fn(&modbus_pump_setting_table[i],
								     input_bit_value);
		if (!result_status) {
			LOG_ERR("modbus 0x9410 setting %d-bit error\n", func_idx);
			WRITE_BIT(check_error_flag, func_idx, 1);
		}
	}

	if (check_error_flag) {
		LOG_ERR("modbus 0x9410 setting error flag: 0x%x\n", check_error_flag);
		return MODBUS_EXC_ILLEGAL_DATA_VAL;
	}

	pump_setting = cmd->data[0] & 0x23F;

	return MODBUS_EXC_NONE;
}

uint8_t modbus_error_log_count(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	cmd->data[0] = error_log_count();
	return MODBUS_EXC_NONE;
}

uint8_t modbus_error_log_event(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint16_t order = 1 + ((cmd->start_addr - MODBUS_EVENT_1_ERROR_LOG_ADDR) / cmd->cmd_size);

	log_transfer_to_modbus_data(cmd->data, cmd->cmd_size, order);

	memmove(cmd->data, cmd->data + cmd->start_addr - cmd->addr,
		cmd->data_len * sizeof(uint16_t));

	return MODBUS_EXC_NONE;
}

uint8_t modbus_get_manual_pwm(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint8_t idx = cmd->arg0;

	cmd->data[0] = get_manual_pwm_cache(idx);

	return MODBUS_EXC_NONE;
}

uint8_t modbus_set_manual_pwm(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint8_t idx = cmd->arg0;
	uint8_t duty = (uint8_t)cmd->data[0];

	switch (idx) {
	case MANUAL_PWM_E_PUMP:
		set_manual_pwm_cache(MANUAL_PWM_E_PUMP_1, duty);
		set_manual_pwm_cache(MANUAL_PWM_E_PUMP_2, duty);
		set_manual_pwm_cache(MANUAL_PWM_E_PUMP_3, duty);
		break;
	case MANUAL_PWM_E_RPU_FAN:
		set_manual_pwm_cache(MANUAL_PWM_E_PUMP_FAN_1, duty);
		set_manual_pwm_cache(MANUAL_PWM_E_PUMP_FAN_2, duty);
		set_manual_pwm_cache(MANUAL_PWM_E_PUMP_FAN_3, duty);
		set_manual_pwm_cache(MANUAL_PWM_E_RPU_PCB_FAN, duty);
		break;
	}

	set_manual_pwm_cache(idx, duty);

	return MODBUS_EXC_NONE;
}

uint8_t modbus_get_pwm(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint8_t idx = cmd->arg0;

	switch (idx) {
	case PWM_GROUP_E_HEX_FAN:
		cmd->data[0] = get_pwm_group_cache(PWM_GROUP_E_HEX_FAN);
		break;
	case PWM_GROUP_E_PUMP:
		if (get_manual_pwm_flag(MANUAL_PWM_E_PUMP))
			cmd->data[0] = get_manual_pwm_cache(MANUAL_PWM_E_PUMP);
		else
			cmd->data[0] = get_pwm_group_cache(PWM_GROUP_E_PUMP);
		break;
	default:
		LOG_ERR("illegal parameter in modbus_get_pwm");
		return MODBUS_EXC_ILLEGAL_DATA_VAL;
	}

	return MODBUS_EXC_NONE;
}

uint8_t modbus_set_sticky_sensor_status(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);
	if (cmd->data[0] != 0 && cmd->data[0] != 1)
		return MODBUS_EXC_SERVER_DEVICE_FAILURE;

	uint8_t status_num = cmd->arg0;
	if (set_sticky_sensor_status(status_num, cmd->data[0]))
		return MODBUS_EXC_NONE;
	else
		return MODBUS_EXC_SERVER_DEVICE_FAILURE;
}

uint8_t modbus_get_sticky_sensor_status(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint8_t status_num = cmd->arg0;
	cmd->data[0] = get_sticky_sensor_status(status_num);

	return MODBUS_EXC_NONE;
}

uint8_t modbus_get_manual_flag(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint8_t idx = cmd->arg0;

	cmd->data[0] = get_manual_pwm_flag(idx);

	return MODBUS_EXC_NONE;
}

uint8_t modbus_set_manual_flag(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint8_t idx = cmd->arg0;
	uint8_t force = cmd->arg1; // 1: force setting
	uint8_t val = cmd->data[0];

	if (val && !force) {
		if (get_status_flag(STATUS_FLAG_AUTO_TUNE))
			return MODBUS_EXC_SERVER_DEVICE_BUSY;
	}

	set_manual_pwm_flag(idx, val);

	return MODBUS_EXC_NONE;
}

uint8_t modbus_get_aalc_sensor_status(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	cmd->data[0] = get_sensor_status_for_modbus_cmd(cmd->arg0);

	return MODBUS_EXC_NONE;
}

uint8_t modbus_get_led_status(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint16_t val = 0;

	val = (get_led_status(LED_IDX_E_COOLANT) == LED_TURN_OFF)    ? 0 :
	      (get_led_status(LED_IDX_E_COOLANT) == LED_TURN_ON)     ? 1 :
	      (get_led_status(LED_IDX_E_COOLANT) == LED_START_BLINK) ? 3 :
								       0;
	WRITE_BIT(val, 2, (get_led_status(LED_IDX_E_LEAK) == LED_TURN_OFF) ? 0 : 1);
	WRITE_BIT(val, 5, gpio_get(get_led_pin(LED_IDX_E_FAULT)));
	WRITE_BIT(val, 4, gpio_get(get_led_pin(LED_IDX_E_POWER)));

	cmd->data[0] = val;

	return MODBUS_EXC_NONE;
}

uint8_t modbus_sensor_poll_get(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	cmd->data[0] = get_sensor_poll_enable_flag() ? 1 : 0;
	return MODBUS_EXC_NONE;
}

uint8_t modbus_sensor_poll_set(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	cmd->data[0] ? plat_enable_sensor_poll() : plat_disable_sensor_poll();
	return MODBUS_EXC_NONE;
}

uint8_t modbus_set_status_flag_config(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint8_t idx = (uint8_t)cmd->data[0];
	uint32_t val = ((cmd->data[1] << 16) | cmd->data[2]);

	set_status_flag_config(idx, val);
	return MODBUS_EXC_NONE;
}

uint8_t modbus_status_flag_get(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);
	uint8_t idx = 0;
	uint32_t val = 0;
	get_status_flag_config(&idx, &val);

	cmd->data[0] = (uint16_t)((get_status_flag(idx) >> 16) & 0xFFFF);
	cmd->data[1] = (uint16_t)(get_status_flag(idx) & 0xFFFF);

	return MODBUS_EXC_NONE;
}

uint8_t modbus_status_flag_set(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint8_t idx = 0;
	uint32_t val = 0;
	get_status_flag_config(&idx, &val);

	set_status_flag(idx, 0xFF, val);
	return MODBUS_EXC_NONE;
}

uint8_t modbus_get_rpu_addr(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	cmd->data[0] = modbus_server_config[0].addr;

	return MODBUS_EXC_NONE;
}

uint8_t modbus_set_rpu_addr(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint8_t addr = (uint8_t)cmd->data[0];

	uint8_t pre_addr = MODBUS_UART_NODE_ADDR;
	if (!plat_eeprom_read(EEPROM_RPU_ADDR_OFFSET, &pre_addr, EEPROM_RPU_ADDR_VERSION_SIZE)) {
		LOG_ERR("read rpu addr fail!");
		return MODBUS_EXC_SERVER_DEVICE_FAILURE;
	}

	if (memcmp(&pre_addr, &addr, EEPROM_RPU_ADDR_VERSION_SIZE)) {
		if (!plat_eeprom_write(EEPROM_RPU_ADDR_OFFSET, &addr,
				       EEPROM_RPU_ADDR_VERSION_SIZE)) {
			LOG_ERR("write rpu addr fail!");
			return MODBUS_EXC_SERVER_DEVICE_FAILURE;
		}

		if (change_modbus_slave_addr(0, addr))
			return MODBUS_EXC_SERVER_DEVICE_FAILURE;
	}

	return MODBUS_EXC_NONE;
}

static uint8_t modbus_to_do_get(modbus_command_mapping *cmd)
{
	// wait to do

	memset(cmd->data, 0, sizeof(uint16_t) * cmd->cmd_size);

	return MODBUS_EXC_NONE;
}

static uint8_t modbus_to_do_set(modbus_command_mapping *cmd)
{
	// wait to do

	return MODBUS_EXC_NONE;
}

uint8_t modbus_read_uptime(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint8_t uptime[EEPROM_UPTIME_SIZE] = { 0 };
	if (!plat_eeprom_read(EEPROM_UPTIME_OFFSET, uptime, EEPROM_UPTIME_SIZE)) {
		LOG_ERR("read uptime fail!");
		return MODBUS_EXC_SERVER_DEVICE_FAILURE;
	}

	memcpy(cmd->data, uptime, EEPROM_UPTIME_SIZE);
	regs_reverse(cmd->data_len, cmd->data);

	return MODBUS_EXC_NONE;
}

uint8_t modbus_read_time_since_last_on(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint32_t get_time_since_last_on_mins =
		(uint32_t)(k_uptime_get_32() / 60000); // mins(60 *1000 ms)

	// swap high and low byte

	cmd->data[0] = get_time_since_last_on_mins >> 16;
	cmd->data[1] = get_time_since_last_on_mins & 0xffff;

	return MODBUS_EXC_NONE;
}

uint8_t modbus_read_pump_running_time(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);
	uint32_t pump_uptime_sec = 0;
	if (get_pump_uptime_secs(cmd->arg0, &pump_uptime_sec)) {
		cmd->data[0] = pump_uptime_sec >> 16;
		cmd->data[1] = pump_uptime_sec & 0xffff;
	} else {
		LOG_ERR("read pump running time fail!");
		return MODBUS_EXC_SERVER_DEVICE_FAILURE;
	}

	return MODBUS_EXC_NONE;
}
uint8_t modbus_set_log_level(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);
	/*	LOG_LEVEL_NONE 0U
		LOG_LEVEL_ERR  1U
		LOG_LEVEL_WRN  2U
		LOG_LEVEL_INF  3U
		LOG_LEVEL_DBG  4U
	*/
	if (!set_log_level(cmd->data[0]))
		return MODBUS_EXC_ILLEGAL_DATA_VAL;

	return MODBUS_EXC_NONE;
}
/* set point relate */
uint8_t modbus_get_setpoint(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	cmd->data[0] = get_fsc_setpoint(cmd->arg0);

	return MODBUS_EXC_NONE;
}
uint8_t modbus_set_setpoint(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	set_fsc_setpoint(cmd->arg0, (uint8_t)cmd->data[0]);
	uint8_t idx = cmd->arg0;
	set_fsc_setpoint(idx, (uint8_t)cmd->data[0]);
	if ((get_status_flag(STATUS_FLAG_SETPOINT_FLAG) && BIT(idx))) {
		switch (idx) {
		case SETPOINT_FLAG_LPM:
			change_lpm_setpoint(1);
			break;
		case SETPOINT_FLAG_OUTLET_TEMP:
			change_temp_setpoint(1);
			break;
		default:
			return MODBUS_EXC_ILLEGAL_DATA_VAL;
		};
	}

	return MODBUS_EXC_NONE;
}
uint8_t modbus_get_setpoint_enable(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint8_t idx = cmd->arg0;
	cmd->data[0] = (uint16_t)((get_status_flag(STATUS_FLAG_SETPOINT_FLAG) >> idx) & 0x01);

	return MODBUS_EXC_NONE;
}
uint8_t modbus_set_setpoint_enable(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint8_t idx = cmd->arg0;
	uint8_t enable = (uint8_t)cmd->data[0];

	set_status_flag(STATUS_FLAG_SETPOINT_FLAG, idx, enable);

	switch (idx) {
	case SETPOINT_FLAG_LPM:
		change_lpm_setpoint(enable);
		break;
	case SETPOINT_FLAG_OUTLET_TEMP:
		change_temp_setpoint(enable);
		break;
	default:
		return MODBUS_EXC_ILLEGAL_DATA_VAL;
	};

	return MODBUS_EXC_NONE;
}

uint8_t modbus_get_board_stage(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	cmd->data[0] = get_board_stage();

	return MODBUS_EXC_NONE;
}

uint8_t modbus_get_fan_table_revision(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	cmd->data[0] = FAN_TABLE_REVISION;

	return MODBUS_EXC_NONE;
}

modbus_command_mapping modbus_command_table[] = {
	// addr, write_fn, read_fn, arg0, arg1, arg2, size
	{ MODBUS_BPB_RPU_COOLANT_FLOW_RATE_LPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_RPU_COOLANT_FLOW_RATE_LPM, 1, -1, 1 },
	{ MODBUS_BPB_RPU_COOLANT_OUTLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C, 1, -1, 1 },
	{ MODBUS_BPB_RPU_COOLANT_INLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_RPU_COOLANT_INLET_TEMP_C, 1, -1, 1 },
	{ MODBUS_BPB_RPU_COOLANT_OUTLET_P_KPA_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_P_KPA, 1, -1, 1 },
	{ MODBUS_BPB_RPU_COOLANT_INLET_P_KPA_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_RPU_COOLANT_INLET_P_KPA, 1, -1, 1 },
	{ MODBUS_RPU_PWR_W_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_RPU_PWR_W, 1, -1, 1 },
	{ MODBUS_AALC_TOTAL_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_AALC_TOTAL_PWR_W, 1, 0, 1 },
	{ MODBUS_RPU_INPUT_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_HSC_P48V_VIN_VOLT_V, 1, -1, 1 },
	{ MODBUS_MB_RPU_AIR_INLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_MB_RPU_AIR_INLET_TEMP_C, 1, -1, 1 },
	{ MODBUS_RPU_PUMP_PWM_TACH_PCT_ADDR, NULL, modbus_get_pwm, PWM_GROUP_E_PUMP, 0, 0, 1 },
	{ MODBUS_PB_1_PUMP_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_1_PUMP_TACH_RPM, 1, 0, 1 },
	{ MODBUS_PB_2_PUMP_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_2_PUMP_TACH_RPM, 1, 0, 1 },
	{ MODBUS_PB_3_PUMP_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_3_PUMP_TACH_RPM, 1, 0, 1 },
	{ MODBUS_RPU_FAN_STATUS_ADDR, NULL, modbus_get_aalc_sensor_status, RPU_FAN_STATUS, 0, 0,
	  1 },
	{ MODBUS_MB_FAN1_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_MB_FAN1_TACH_RPM, 1, 0, 1 },
	{ MODBUS_MB_FAN2_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_MB_FAN2_TACH_RPM, 1, 0, 1 },
	{ MODBUS_AALC_COOLING_CAPACITY_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_COOLING_CAPACITY, 1, -1, 1 },
	{ MODBUS_AALC_COOLING_CAPACITY_W_EXT_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_COOLING_CAPACITY, 1, -1, 1 },
	{ MODBUS_RPU_PUMP1_STATUS_ADDR, NULL, modbus_get_aalc_sensor_status, RPU_PUMP1_STATUS, 0, 0,
	  1 },
	{ MODBUS_RPU_PUMP2_STATUS_ADDR, NULL, modbus_get_aalc_sensor_status, RPU_PUMP2_STATUS, 0, 0,
	  1 },
	{ MODBUS_RPU_PUMP3_STATUS_ADDR, NULL, modbus_get_aalc_sensor_status, RPU_PUMP3_STATUS, 0, 0,
	  1 },
	{ MODBUS_RPU_RESERVOIR_STATUS_ADDR, NULL, modbus_get_aalc_sensor_status,
	  RPU_RESERVOIR_STATUS, 0, 0, 1 },
	{ MODBUS_RPU_LED_STATUS_ADDR, NULL, modbus_get_led_status, 0, 0, 0, 1 },
	{ MODBUS_RPU_PUMP_STATUS_ADDR, NULL, modbus_get_aalc_sensor_status, ALL_PUMP_STATUS, 0, 0,
	  1 },
	{ MODBUS_RPU_INTERNAL_FAN_STATUS_ADDR, NULL, modbus_get_aalc_sensor_status,
	  ALL_RPU_INTERNAL_FAN_STATUS, 0, 0, 1 },
	{ MODBUS_BB_TMP75_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_BB_TMP75_TEMP_C, 1,
	  0, 1 },
	{ MODBUS_BPB_RPU_OUTLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_RPU_OUTLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_PDB_HDC1080DMBR_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PDB_HDC1080DMBR_TEMP_C, 1, 0, 1 },
	{ MODBUS_BB_HSC_P48V_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BB_HSC_P48V_TEMP_C, 1, 0, 1 },
	{ MODBUS_BPB_HSC_P48V_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_HSC_P48V_TEMP_C, 1, 0, 1 },
	{ MODBUS_PB_1_HDC1080DMBR_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_1_HDC1080DMBR_TEMP_C, 1, 0, 1 },
	{ MODBUS_PB_2_HDC1080DMBR_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_2_HDC1080DMBR_TEMP_C, 1, 0, 1 },
	{ MODBUS_PB_3_HDC1080DMBR_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_3_HDC1080DMBR_TEMP_C, 1, 0, 1 },
	{ MODBUS_PB_1_HSC_P48V_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_1_HSC_P48V_TEMP_C, 1, 0, 1 },
	{ MODBUS_PB_2_HSC_P48V_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_2_HSC_P48V_TEMP_C, 1, 0, 1 },
	{ MODBUS_PB_3_HSC_P48V_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_3_HSC_P48V_TEMP_C, 1, 0, 1 },
	{ MODBUS_PB_1_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_1_HSC_P48V_VIN_VOLT_V, 1, -2, 1 },
	{ MODBUS_PB_2_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_2_HSC_P48V_VIN_VOLT_V, 1, -2, 1 },
	{ MODBUS_PB_3_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_3_HSC_P48V_VIN_VOLT_V, 1, -2, 1 },
	{ MODBUS_BB_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BB_HSC_P48V_VIN_VOLT_V, 1, -2, 1 },
	{ MODBUS_BPB_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_HSC_P48V_VIN_VOLT_V, 1, -2, 1 },
	{ MODBUS_BB_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BB_HSC_P48V_IOUT_CURR_A, 1, -1, 1 },
	{ MODBUS_BPB_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_HSC_P48V_IOUT_CURR_A, 1, -1, 1 },
	{ MODBUS_PB_1_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_1_HSC_P48V_IOUT_CURR_A, 1, -1, 1 },
	{ MODBUS_PB_2_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_2_HSC_P48V_IOUT_CURR_A, 1, -1, 1 },
	{ MODBUS_PB_3_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_3_HSC_P48V_IOUT_CURR_A, 1, -1, 1 },
	{ MODBUS_BB_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BB_HSC_P48V_PIN_PWR_W, 1, -1, 1 },
	{ MODBUS_PUMP_1_RUNNING_ADDR, NULL, modbus_read_pump_running_time, PUMP_1_UPTIME, 0, 0, 2 },
	{ MODBUS_PUMP_2_RUNNING_ADDR, NULL, modbus_read_pump_running_time, PUMP_2_UPTIME, 0, 0, 2 },
	{ MODBUS_PUMP_3_RUNNING_ADDR, NULL, modbus_read_pump_running_time, PUMP_3_UPTIME, 0, 0, 2 },
	{ MODBUS_BPB_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_HSC_P48V_PIN_PWR_W, 1, -1, 1 },
	{ MODBUS_PB_1_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_1_HSC_P48V_PIN_PWR_W, 1, -1, 1 },
	{ MODBUS_PB_2_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_2_HSC_P48V_PIN_PWR_W, 1, -1, 1 },
	{ MODBUS_PB_3_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_3_HSC_P48V_PIN_PWR_W, 1, -1, 1 },
	{ MODBUS_PB_1_FAN_OUTLET_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_1_FAN_1_TACH_RPM, 1, 0, 1 },
	{ MODBUS_PB_1_FAN_INLET_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_1_FAN_2_TACH_RPM, 1, 0, 1 },
	{ MODBUS_PB_2_FAN_OUTLET_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_2_FAN_1_TACH_RPM, 1, 0, 1 },
	{ MODBUS_PB_2_FAN_INLET_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_2_FAN_2_TACH_RPM, 1, 0, 1 },
	{ MODBUS_PB_3_FAN_OUTLET_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_3_FAN_1_TACH_RPM, 1, 0, 1 },
	{ MODBUS_PB_3_FAN_INLET_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_3_FAN_2_TACH_RPM, 1, 0, 1 },
	{ MODBUS_BPB_RACK_FILTER_INLET_PRESSURE_P_KPA_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_RACK_PRESSURE_3_P_KPA, 1, -1, 1 },
	{ MODBUS_BPB_RACK_FILTER_OUTLET_PRESSURE_P_KPA_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_RACK_PRESSURE_4_P_KPA, 1, -1, 1 },
	{ MODBUS_BPB_RACK_LEVEL_1_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_RACK_LEVEL_1, 1, 0, 1 },
	{ MODBUS_BPB_RACK_LEVEL_2_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_RACK_LEVEL_2, 1, 0, 1 },
	{ MODBUS_BPB_RPU_LEVEL_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_BPB_RPU_LEVEL, 1,
	  0, 1 },
	{ MODBUS_MB_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_MB_HUM_PCT_RH, 1,
	  0, 1 },
	{ MODBUS_PDB_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_PDB_HUM_PCT_RH, 1,
	  0, 1 },
	{ MODBUS_PB_1_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_PB_1_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_PB_2_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_PB_2_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_PB_3_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_PB_3_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_HEX_FAN_PWM_TACH_PCT_ADDR, NULL, modbus_get_pwm, PWM_GROUP_E_HEX_FAN, 0, 0, 1 },
	{ MODBUS_HEX_PWR_W_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_HEX_PWR_W, 1, -1, 1 },
	{ MODBUS_HEX_INPUT_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_1_HSC_P48V_VIN_VOLT_V, 1, 0, 1 },
	{ MODBUS_HEX_INPUT_CURRENT_A_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_HEX_CURR_A,
	  1, -1, 1 },
	{ MODBUS_FB_1_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_1_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_2_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_2_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_3_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_3_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_4_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_4_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_5_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_5_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_6_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_6_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_7_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_7_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_8_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_8_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_9_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_9_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_10_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_10_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_SB_HEX_AIR_INLET_1_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_SB_HEX_AIR_INLET_1_TEMP_C, 1, -1, 1 },
	{ MODBUS_SB_HEX_AIR_INLET_2_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_SB_HEX_AIR_INLET_2_TEMP_C, 1, -1, 1 },
	{ MODBUS_FB_1_HEX_OUTLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_1_HEX_OUTLET_TEMP_C, 1, -1, 1 },
	{ MODBUS_FB_2_HEX_OUTLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_2_HEX_OUTLET_TEMP_C, 1, -1, 1 },
	{ MODBUS_HEX_WATER_INLET_TEMP_C_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_HEX_WATER_INLET_TEMP_C, 1, -1, 1 },
	{ MODBUS_HEX_BLADDER_LEVEL_STATUS_ADDR, NULL, modbus_get_aalc_sensor_status,
	  HEX_BLADDER_LEVEL_STATUS, 0, 0, 1 },
	{ MODBUS_HEX_EXTERNAL_Y_FILTER_PRESSURE_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_HEX_EXTERNAL_Y_FILTER, 1, -1, 1 },
	//{ MODBUS_HEX_STATIC_PRESSURE_ADDR, NULL, modbus_to_do, 0, 0, 0, 1 },
	//{ MODBUS_HEX_VERTICAL_BLADDER_ADDR, NULL, modbus_to_do, 0, 0, 0, 1 },
	{ MODBUS_SB_HEX_AIR_INLET_3_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_SB_HEX_AIR_INLET_3_TEMP_C, 1, 0, 1 },
	{ MODBUS_SB_HEX_AIR_INLET_4_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_SB_HEX_AIR_INLET_4_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_3_HEX_OUTLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_3_HEX_OUTLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_4_HEX_OUTLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_4_HEX_OUTLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_5_HEX_OUTLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_5_HEX_OUTLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_6_HEX_OUTLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_6_HEX_OUTLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_7_HEX_OUTLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_7_HEX_OUTLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_8_HEX_OUTLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_8_HEX_OUTLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_9_HEX_OUTLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_9_HEX_OUTLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_10_HEX_OUTLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_10_HEX_OUTLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_11_HEX_OUTLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_11_HEX_OUTLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_12_HEX_OUTLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_12_HEX_OUTLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_13_HEX_OUTLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_13_HEX_OUTLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_14_HEX_OUTLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_14_HEX_OUTLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_1_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_1_HSC_TEMP_C, 1,
	  0, 1 },
	{ MODBUS_FB_2_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_2_HSC_TEMP_C, 1,
	  0, 1 },
	{ MODBUS_FB_3_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_3_HSC_TEMP_C, 1,
	  0, 1 },
	{ MODBUS_FB_4_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_4_HSC_TEMP_C, 1,
	  0, 1 },
	{ MODBUS_FB_5_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_5_HSC_TEMP_C, 1,
	  0, 1 },
	{ MODBUS_FB_6_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_6_HSC_TEMP_C, 1,
	  0, 1 },
	{ MODBUS_FB_7_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_7_HSC_TEMP_C, 1,
	  0, 1 },
	{ MODBUS_FB_8_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_8_HSC_TEMP_C, 1,
	  0, 1 },
	{ MODBUS_FB_9_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_9_HSC_TEMP_C, 1,
	  0, 1 },
	{ MODBUS_FB_10_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_10_HSC_TEMP_C,
	  1, 0, 1 },
	{ MODBUS_FB_11_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_11_HSC_TEMP_C,
	  1, 0, 1 },
	{ MODBUS_FB_12_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_12_HSC_TEMP_C,
	  1, 0, 1 },
	{ MODBUS_FB_13_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_13_HSC_TEMP_C,
	  1, 0, 1 },
	{ MODBUS_FB_14_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_14_HSC_TEMP_C,
	  1, 0, 1 },
	{ MODBUS_FB_1_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_1_HSC_P48V_VIN_VOLT_V, 1, -2, 1 },
	{ MODBUS_FB_2_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_2_HSC_P48V_VIN_VOLT_V, 1, -2, 1 },
	{ MODBUS_FB_3_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_3_HSC_P48V_VIN_VOLT_V, 1, -2, 1 },
	{ MODBUS_FB_4_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_4_HSC_P48V_VIN_VOLT_V, 1, -2, 1 },
	{ MODBUS_FB_5_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_5_HSC_P48V_VIN_VOLT_V, 1, -2, 1 },
	{ MODBUS_FB_6_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_6_HSC_P48V_VIN_VOLT_V, 1, -2, 1 },
	{ MODBUS_FB_7_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_7_HSC_P48V_VIN_VOLT_V, 1, -2, 1 },
	{ MODBUS_FB_8_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_8_HSC_P48V_VIN_VOLT_V, 1, -2, 1 },
	{ MODBUS_FB_9_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_9_HSC_P48V_VIN_VOLT_V, 1, -2, 1 },
	{ MODBUS_FB_10_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_10_HSC_P48V_VIN_VOLT_V, 1, -2, 1 },
	{ MODBUS_FB_11_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_11_HSC_P48V_VIN_VOLT_V, 1, -2, 1 },
	{ MODBUS_FB_12_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_12_HSC_P48V_VIN_VOLT_V, 1, -2, 1 },
	{ MODBUS_FB_13_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_13_HSC_P48V_VIN_VOLT_V, 1, -2, 1 },
	{ MODBUS_FB_14_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_14_HSC_P48V_VIN_VOLT_V, 1, -2, 1 },
	{ MODBUS_FB_1_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_1_HSC_P48V_IOUT_CURR_A, 1, -1, 1 },
	{ MODBUS_FB_2_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_2_HSC_P48V_IOUT_CURR_A, 1, -1, 1 },
	{ MODBUS_FB_3_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_3_HSC_P48V_IOUT_CURR_A, 1, -1, 1 },
	{ MODBUS_FB_4_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_4_HSC_P48V_IOUT_CURR_A, 1, -1, 1 },
	{ MODBUS_FB_5_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_5_HSC_P48V_IOUT_CURR_A, 1, -1, 1 },
	{ MODBUS_FB_6_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_6_HSC_P48V_IOUT_CURR_A, 1, -1, 1 },
	{ MODBUS_FB_7_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_7_HSC_P48V_IOUT_CURR_A, 1, -1, 1 },
	{ MODBUS_FB_8_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_8_HSC_P48V_IOUT_CURR_A, 1, -1, 1 },
	{ MODBUS_FB_9_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_9_HSC_P48V_IOUT_CURR_A, 1, -1, 1 },
	{ MODBUS_FB_10_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_10_HSC_P48V_IOUT_CURR_A, 1, -1, 1 },
	{ MODBUS_FB_11_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_11_HSC_P48V_IOUT_CURR_A, 1, -1, 1 },
	{ MODBUS_FB_12_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_12_HSC_P48V_IOUT_CURR_A, 1, -1, 1 },
	{ MODBUS_FB_13_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_13_HSC_P48V_IOUT_CURR_A, 1, -1, 1 },
	{ MODBUS_FB_14_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_14_HSC_P48V_IOUT_CURR_A, 1, -1, 1 },
	{ MODBUS_FB_1_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_1_HSC_P48V_PIN_PWR_W, 1, -1, 1 },
	{ MODBUS_FB_2_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_2_HSC_P48V_PIN_PWR_W, 1, -1, 1 },
	{ MODBUS_FB_3_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_3_HSC_P48V_PIN_PWR_W, 1, -1, 1 },
	{ MODBUS_FB_4_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_4_HSC_P48V_PIN_PWR_W, 1, -1, 1 },
	{ MODBUS_FB_5_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_5_HSC_P48V_PIN_PWR_W, 1, -1, 1 },
	{ MODBUS_FB_6_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_6_HSC_P48V_PIN_PWR_W, 1, -1, 1 },
	{ MODBUS_FB_7_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_7_HSC_P48V_PIN_PWR_W, 1, -1, 1 },
	{ MODBUS_FB_8_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_8_HSC_P48V_PIN_PWR_W, 1, -1, 1 },
	{ MODBUS_FB_9_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_9_HSC_P48V_PIN_PWR_W, 1, -1, 1 },
	{ MODBUS_FB_10_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_10_HSC_P48V_PIN_PWR_W, 1, -1, 1 },
	{ MODBUS_FB_11_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_11_HSC_P48V_PIN_PWR_W, 1, -1, 1 },
	{ MODBUS_FB_12_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_12_HSC_P48V_PIN_PWR_W, 1, -1, 1 },
	{ MODBUS_FB_13_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_13_HSC_P48V_PIN_PWR_W, 1, -1, 1 },
	{ MODBUS_FB_14_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_14_HSC_P48V_PIN_PWR_W, 1, -1, 1 },
	{ MODBUS_FB_11_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_11_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_12_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_12_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_13_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_13_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_14_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_14_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_SB_HEX_INLET_PRESSURE_P_KPA_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_SB_HEX_PRESSURE_1_P_KPA, 1, -1, 1 },
	{ MODBUS_SB_HEX_OUTLET_PRESSURE_P_KPA_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_SB_HEX_PRESSURE_2_P_KPA, 1, -1, 1 },
	{ MODBUS_FB_1_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_1_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_FB_2_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_2_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_FB_3_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_3_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_FB_4_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_4_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_FB_5_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_5_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_FB_6_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_6_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_FB_7_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_7_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_FB_8_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_8_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_FB_9_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_9_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_FB_10_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_10_HUM_PCT_RH, 1, 0, 1 },
	{ MODBUS_FB_11_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_11_HUM_PCT_RH, 1, 0, 1 },
	{ MODBUS_FB_12_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_12_HUM_PCT_RH, 1, 0, 1 },
	{ MODBUS_FB_13_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_13_HUM_PCT_RH, 1, 0, 1 },
	{ MODBUS_FB_14_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_14_HUM_PCT_RH, 1, 0, 1 },
	{ MODBUS_RPU_PDB_48V_SENSE_DIFF_POS_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PDB_48V_SENSE_DIFF_POS_VOLT_V, 1, -2, 1 },
	{ MODBUS_RPU_PDB_48V_SENSE_DIFF_NEG_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PDB_48V_SENSE_DIFF_NEG_VOLT_V, 1, -2, 1 },
	{ MODBUS_BPB_CDU_COOLANT_LEAKAGE_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_CDU_COOLANT_LEAKAGE_VOLT_V, 1, -2, 1 },
	{ MODBUS_BPB_RACK_COOLANT_LEAKAGE_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_RACK_COOLANT_LEAKAGE_VOLT_V, 1, -2, 1 },
	{ MODBUS_AALC_TOTAL_PWR_EXT_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_AALC_TOTAL_PWR_W, 1, 0, 1 },
	{ MODBUS_PUMP_FAN_STATUS_ADDR, NULL, modbus_get_aalc_sensor_status, PUMP_FAN_STATUS, 0, 0,
	  1 },
	{ MODBUS_HEX_AIR_THERMOMETER_STATUS_ADDR, NULL, modbus_get_aalc_sensor_status,
	  HEX_AIR_THERMOMETER_STATUS, 0, 0, 1 },
	// ADC
	{ MODBUS_V_12_AUX_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_V_12_AUX, 1, -2, 1 },
	{ MODBUS_V_5_AUX_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_V_5_AUX, 1, -2, 1 },
	{ MODBUS_V_3_3_AUX_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_V_3_3_AUX, 1, -2, 1 },
	{ MODBUS_V_1_2_AUX_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_V_1_2_AUX, 1, -2, 1 },
	{ MODBUS_V_5_USB_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_V_5_USB, 1, -2, 1 },
	// TTV
	{ MODBUS_SB_TTV_COOLANT_LEAKAGE_1_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_SB_TTV_COOLANT_LEAKAGE_1_VOLT_V, 1, -2, 1 },
	{ MODBUS_SB_TTV_COOLANT_LEAKAGE_2_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_SB_TTV_COOLANT_LEAKAGE_2_VOLT_V, 1, -2, 1 },
	{ MODBUS_SB_TTV_COOLANT_LEAKAGE_3_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_SB_TTV_COOLANT_LEAKAGE_3_VOLT_V, 1, -2, 1 },
	//FW UPDATE
	{ MODBUS_FW_DOWNLOAD_ADDR, modbus_fw_download, NULL, 0, 0, 0, 103 },
	// i2c master write read
	{ MODBUS_MASTER_I2C_WRITE_READ_ADDR, modbus_command_i2c_master_write_read, NULL, 0, 0, 0,
	  16 },
	{ MODBUS_MASTER_I2C_WRITE_READ_RESPONSE_ADDR, NULL,
	  modbus_command_i2c_master_write_read_response, 0, 0, 0, 16 },
	{ MODBUS_MASTER_I2C_SCAN_BUS_SET_ADDR, modbus_command_i2c_scan_bus_set, NULL, 0, 0, 0, 1 },
	{ MODBUS_MASTER_I2C_SCAN_ADDR, NULL, modbus_command_i2c_scan, 0, 0, 0, 31 },

	// System Alarm
	{ MODBUS_AALC_SENSOR_ALARM_ADDR, NULL, modbus_get_aalc_sensor_status, AALC_SENSOR_ALARM, 0,
	  0, 1 },
	{ MODBUS_AALC_STATUS_ALARM_ADDR, NULL, modbus_get_aalc_sensor_status, AALC_STATUS_ALARM, 0,
	  0, 1 },
	{ MODBUS_LEAKAGE_STATUS_ADDR, NULL, modbus_leakage_status_read, 0, 0, 0, 1 },
	{ MODBUS_HEX_FAN_ALARM_1_ADDR, NULL, modbus_get_aalc_sensor_status, HEX_FAN_ALARM_1, 0, 0,
	  1 },
	{ MODBUS_HEX_FAN_ALARM_2_ADDR, NULL, modbus_get_aalc_sensor_status, HEX_FAN_ALARM_2, 0, 0,
	  1 },
	{ MODBUS_HEX_FAN_COMMS_ALARM_ADDR, NULL, modbus_get_aalc_sensor_status, HEX_FAN_COMMS_ALARM,
	  0, 0, 1 },
	{ MODBUS_HSC_POWER_STATUS_ADDR, NULL, modbus_get_aalc_sensor_status, HSC_POWER_STATUS, 0, 0,
	  1 },
	{ MODBUS_FB_HSC_POWER_STATUS_ADDR, NULL, modbus_get_aalc_sensor_status, FB_HSC_POWER_STATUS,
	  0, 0, 1 },
	{ MODBUS_HSC_COMMS_STATUS_ADDR, NULL, modbus_get_aalc_sensor_status, HSC_COMMS_STATUS, 0, 0,
	  1 },
	{ MODBUS_FB_HSC_COMMS_STATUS_ADDR, NULL, modbus_get_aalc_sensor_status, FB_HSC_COMMS_STATUS,
	  0, 0, 1 },
	{ MODBUS_MODBUS_ADDR_PATH_WITH_WEDGE400_ADDR, modbus_set_rpu_addr, modbus_get_rpu_addr, 0,
	  0, 0, 1 },
	{ MODBUS_MANUAL_CONTROL_RPU_FAN_ON_OFF_ADDR, modbus_set_manual_flag, modbus_get_manual_flag,
	  MANUAL_PWM_E_RPU_FAN, 0, 0, 1 },
	{ MODBUS_FORCE_MANUAL_CONTROL_PUMP_ADDR, modbus_set_manual_flag, modbus_get_manual_flag,
	  MANUAL_PWM_E_PUMP, 1, 0, 1 },
	{ MODBUS_SET_LPM_SETPOINT_ADDR, modbus_set_setpoint_enable, modbus_get_setpoint_enable,
	  SETPOINT_FLAG_LPM, 0, 0, 1 },
	{ MODBUS_SET_OUTLET_TEMP_SETPOINT_ADDR, modbus_set_setpoint_enable,
	  modbus_get_setpoint_enable, SETPOINT_FLAG_OUTLET_TEMP, 0, 0, 1 },
	// Control
	{ MODBUS_AUTO_TUNE_COOLANT_FLOW_RATE_TARGET_SET_ADDR, modbus_set_setpoint,
	  modbus_get_setpoint, SETPOINT_FLAG_LPM, 0, 0, 1 },
	{ MODBUS_AUTO_TUNE_COOLANT_OUTLET_TEMPERATURE_TARGET_SET_ADDR, modbus_set_setpoint,
	  modbus_get_setpoint, SETPOINT_FLAG_OUTLET_TEMP, 0, 0, 1 },
	{ MODBUS_PUMP_REDUNDANT_SWITCHED_INTERVAL_ADDR, modbus_to_do_set, modbus_to_do_get, 0, 0, 0,
	  1 },
	{ MODBUS_MANUAL_CONTROL_PUMP_DUTY_SET_ADDR, modbus_set_manual_pwm, modbus_get_manual_pwm,
	  MANUAL_PWM_E_PUMP, 0, 0, 1 },
	{ MODBUS_MANUAL_CONTROL_FAN_DUTY_SET_ADDR, modbus_set_manual_pwm, modbus_get_manual_pwm,
	  MANUAL_PWM_E_HEX_FAN, 0, 0, 1 },
	{ MODBUS_MANUAL_CONTROL_RPU_FAN_DUTY_SET_ADDR, modbus_set_manual_pwm, modbus_get_manual_pwm,
	  MANUAL_PWM_E_RPU_FAN, 0, 0, 1 },
	{ MODBUS_MANUAL_CONTROL_PUMP1_DUTY_SET_ADDR, modbus_set_manual_pwm, modbus_get_manual_pwm,
	  MANUAL_PWM_E_PUMP_1, 0, 0, 1 },
	{ MODBUS_MANUAL_CONTROL_PUMP2_DUTY_SET_ADDR, modbus_set_manual_pwm, modbus_get_manual_pwm,
	  MANUAL_PWM_E_PUMP_2, 0, 0, 1 },
	{ MODBUS_MANUAL_CONTROL_PUMP3_DUTY_SET_ADDR, modbus_set_manual_pwm, modbus_get_manual_pwm,
	  MANUAL_PWM_E_PUMP_3, 0, 0, 1 },
	{ MODBUS_MANUAL_CONTROL_PUMP_FAN_1_DUTY_SET_ADDR, modbus_set_manual_pwm,
	  modbus_get_manual_pwm, MANUAL_PWM_E_PUMP_FAN_1, 0, 0, 1 },
	{ MODBUS_MANUAL_CONTROL_PUMP_FAN_2_DUTY_SET_ADDR, modbus_set_manual_pwm,
	  modbus_get_manual_pwm, MANUAL_PWM_E_PUMP_FAN_2, 0, 0, 1 },
	{ MODBUS_MANUAL_CONTROL_PUMP_FAN_3_DUTY_SET_ADDR, modbus_set_manual_pwm,
	  modbus_get_manual_pwm, MANUAL_PWM_E_PUMP_FAN_3, 0, 0, 1 },
	{ MODBUS_MANUAL_CONTROL_RPU_PCB_FAN_DUTY_SET_ADDR, modbus_set_manual_pwm,
	  modbus_get_manual_pwm, MANUAL_PWM_E_RPU_PCB_FAN, 0, 0, 1 },
	{ MODBUS_PUMP_SETTING_ADDR, modbus_pump_setting, modbus_pump_setting_get, 0, 0, 0, 1 },
	{ MODBUS_LEAKAGE_SETTING_ON_ADDR, modbus_to_do_set, modbus_to_do_get, 0, 0, 0, 1 },
	// Leakage Black Box
	{ MODBUS_STICKY_ITRACK_CHASSIS0_LEAKAGE_ADDR, modbus_set_sticky_sensor_status,
	  modbus_get_sticky_sensor_status, STICKY_ITRACK_CHASSIS0_LEAKAGE, 0, 0, 1 },
	{ MODBUS_STICKY_ITRACK_CHASSIS1_LEAKAGE_ADDR, modbus_set_sticky_sensor_status,
	  modbus_get_sticky_sensor_status, STICKY_ITRACK_CHASSIS1_LEAKAGE, 0, 0, 1 },
	{ MODBUS_STICKY_ITRACK_CHASSIS2_LEAKAGE_ADDR, modbus_set_sticky_sensor_status,
	  modbus_get_sticky_sensor_status, STICKY_ITRACK_CHASSIS2_LEAKAGE, 0, 0, 1 },
	{ MODBUS_STICKY_ITRACK_CHASSIS3_LEAKAGE_ADDR, modbus_set_sticky_sensor_status,
	  modbus_get_sticky_sensor_status, STICKY_ITRACK_CHASSIS3_LEAKAGE, 0, 0, 1 },
	{ MODBUS_STICKY_RPU_INTERNAL_LEAKAGE_ABNORMAL_ADDR, modbus_set_sticky_sensor_status,
	  modbus_get_sticky_sensor_status, STICKY_RPU_INTERNAL_LEAKAGE_ABNORMAL, 0, 0, 1 },
	// { MODBUS_STICKY_RPU_EXTERNAL_LEAKAGE_ABNORMAL_ADDR, NULL, modbus_get_sensor_status,
	//   STICKY_RPU_EXTERNAL_LEAKAGE_ABNORMAL, 0, 0, 1 },
	// { MODBUS_STICKY_RPU_OPT_EXTERNAL_LEAKAGE1_ABNORMAL_ADDR, NULL, modbus_get_sensor_status,
	//   STICKY_RPU_OPT_EXTERNAL_LEAKAGE1_ABNORMAL, 0, 0, 1 },
	// { MODBUS_STICKY_RPU_OPT_EXTERNAL_LEAKAGE2_ABNORMAL_ADDR, NULL, modbus_get_sensor_status,
	//   STICKY_RPU_OPT_EXTERNAL_LEAKAGE2_ABNORMAL, 0, 0, 1 },
	{ MODBUS_STICKY_HEX_RACK_PAN_LEAKAGE_ADDR, modbus_set_sticky_sensor_status,
	  modbus_get_sticky_sensor_status, STICKY_HEX_RACK_PAN_LEAKAGE, 0, 0, 1 },
	// { MODBUS_STICKY_HEX_RACK_FLOOR_LEAKAGE_ADDR, NULL, modbus_get_sensor_status,
	//   STICKY_HEX_RACK_FLOOR_LEAKAGE, 0, 0, 1 },
	// { MODBUS_STICKY_HEX_RACK_PAN_LEAKAGE_RELAY_ADDR, NULL, modbus_get_sensor_status,
	//   STICKY_HEX_RACK_PAN_LEAKAGE_RELAY, 0, 0, 1 },
	// { MODBUS_STICKY_HEX_RACK_FLOOR_LEAKAGE_RELAY_ADDR, NULL, modbus_get_sensor_status,
	//   STICKY_HEX_RACK_FLOOR_LEAKAGE_RELAY, 0, 0, 1 },
	// Error Log
	{ MODBUS_ERROR_LOG_COUNT_ADDR, NULL, modbus_error_log_count, 0, 0, 0, 1 },
	{ MODBUS_EVENT_1_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_2_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_3_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_4_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_5_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_6_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_7_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_8_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_9_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_10_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_11_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_12_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_13_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_14_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_15_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_16_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_17_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_18_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_19_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_20_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_21_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_22_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_23_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_24_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_25_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_26_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_27_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_28_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_29_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	{ MODBUS_EVENT_30_ERROR_LOG_ADDR, NULL, modbus_error_log_event, 0, 0, 0, 10 },
	// FRU write read
	{ MODBUS_MB_FRU_ADDR, modbus_write_fruid_data, modbus_read_fruid_data, MB_FRU_ID, 0, 0,
	  256 },
	{ MODBUS_BB_FRU_ADDR, modbus_write_fruid_data, modbus_read_fruid_data, BB_FRU_ID, 0, 0,
	  256 },
	{ MODBUS_BPB_FRU_ADDR, modbus_write_fruid_data, modbus_read_fruid_data, BPB_FRU_ID, 0, 0,
	  256 },
	{ MODBUS_PDB_FRU_ADDR, modbus_write_fruid_data, modbus_read_fruid_data, PDB_FRU_ID, 0, 0,
	  256 },
	{ MODBUS_SB_FRU_ADDR, modbus_write_fruid_data, modbus_read_fruid_data, SB_FRU_ID, 0, 0,
	  256 },
	{ MODBUS_PB_1_FRU_ADDR, modbus_write_fruid_data, modbus_read_fruid_data, PB_1_FRU_ID, 0, 0,
	  256 },
	{ MODBUS_PB_2_FRU_ADDR, modbus_write_fruid_data, modbus_read_fruid_data, PB_2_FRU_ID, 0, 0,
	  256 },
	{ MODBUS_PB_3_FRU_ADDR, modbus_write_fruid_data, modbus_read_fruid_data, PB_3_FRU_ID, 0, 0,
	  256 },
	{ MODBUS_FB_1_FRU_ADDR, modbus_write_fruid_data, modbus_read_fruid_data, FB_1_FRU_ID, 0, 0,
	  256 },
	{ MODBUS_FB_2_FRU_ADDR, modbus_write_fruid_data, modbus_read_fruid_data, FB_2_FRU_ID, 0, 0,
	  256 },
	{ MODBUS_FB_3_FRU_ADDR, modbus_write_fruid_data, modbus_read_fruid_data, FB_3_FRU_ID, 0, 0,
	  256 },
	{ MODBUS_FB_4_FRU_ADDR, modbus_write_fruid_data, modbus_read_fruid_data, FB_4_FRU_ID, 0, 0,
	  256 },
	{ MODBUS_FB_5_FRU_ADDR, modbus_write_fruid_data, modbus_read_fruid_data, FB_5_FRU_ID, 0, 0,
	  256 },
	{ MODBUS_FB_6_FRU_ADDR, modbus_write_fruid_data, modbus_read_fruid_data, FB_6_FRU_ID, 0, 0,
	  256 },
	{ MODBUS_FB_7_FRU_ADDR, modbus_write_fruid_data, modbus_read_fruid_data, FB_7_FRU_ID, 0, 0,
	  256 },
	{ MODBUS_FB_8_FRU_ADDR, modbus_write_fruid_data, modbus_read_fruid_data, FB_8_FRU_ID, 0, 0,
	  256 },
	{ MODBUS_FB_9_FRU_ADDR, modbus_write_fruid_data, modbus_read_fruid_data, FB_9_FRU_ID, 0, 0,
	  256 },
	{ MODBUS_FB_10_FRU_ADDR, modbus_write_fruid_data, modbus_read_fruid_data, FB_10_FRU_ID, 0,
	  0, 256 },
	{ MODBUS_FB_11_FRU_ADDR, modbus_write_fruid_data, modbus_read_fruid_data, FB_11_FRU_ID, 0,
	  0, 256 },
	{ MODBUS_FB_12_FRU_ADDR, modbus_write_fruid_data, modbus_read_fruid_data, FB_12_FRU_ID, 0,
	  0, 256 },
	{ MODBUS_FB_13_FRU_ADDR, modbus_write_fruid_data, modbus_read_fruid_data, FB_13_FRU_ID, 0,
	  0, 256 },
	{ MODBUS_FB_14_FRU_ADDR, modbus_write_fruid_data, modbus_read_fruid_data, FB_14_FRU_ID, 0,
	  0, 256 },
	{ MODBUS_FIO_FRU_ADDR, modbus_write_fruid_data, modbus_read_fruid_data, FIO_FRU_ID, 0, 0,
	  256 },
	// sensor poll
	{ MODBUS_GET_SET_SENSOR_POLL_ADDR, modbus_sensor_poll_set, modbus_sensor_poll_get, 0, 0, 0,
	  1 },
	// RPU FRU
	{ MODBUS_RPU_FBPN_ADDR, NULL, NULL, 0, 0, 0, 8 },
	{ MODBUS_RPU_MFR_MODEL_ADDR, NULL, NULL, 0, 0, 0, 8 },
	{ MODBUS_RPU_MFR_DATE_ADDR, NULL, NULL, 0, 0, 0, 4 },
	{ MODBUS_RPU_MFR_SERIAL_ADDR, NULL, NULL, 0, 0, 0, 8 },
	{ MODBUS_RPU_WORKORDER_ADDR, NULL, NULL, 0, 0, 0, 4 },
	{ MODBUS_RPU_HW_REVISION_ADDR, NULL, NULL, 0, 0, 0, 4 },
	{ MODBUS_RPU_PLC_FW_REVISION_ADDR, NULL, modbus_get_fw_reversion, 0, 0, 0, 4 },
	{ MODBUS_TOTAL_UP_TIME_ADDR, NULL, modbus_read_uptime, 0, 0, 0, 2 },
	{ MODBUS_TIME_SINCE_LAST_ON_ADDR, NULL, modbus_read_time_since_last_on, 0, 0, 0, 2 },
	{ MODBUS_RPU_HMI_FW_REVISION_ADDR, modbus_write_hmi_version, modbus_read_hmi_version, 0, 0,
	  0, 4 },
	{ MODBUS_RPU_HEX_FW_REVISION_ADDR, NULL, NULL, 0, 0, 0, 2 },
	{ MODBUS_RPU_NOAHS_ARK_CONFIGURATION_ADDR, NULL, NULL, 0, 0, 0, 4 },
	{ MODBUS_RPU_RESERVIOR_AND_PUMPING_UNIT_FBPN_ADDR, NULL, NULL, 0, 0, 0, 4 },
	{ MODBUS_HEAT_EXCHANGER_CONTROL_BOX_FBPN_ADDR, NULL, NULL, 0, 0, 0, 4 },
	{ MODBUS_HEAT_EXCHANGER_FANS_FBPN_ADDR, NULL, NULL, 0, 0, 0, 4 },
	{ MODBUS_HEAT_EXCHANGER_FAN_CONTROL_BOX_FBPN_ADDR, NULL, NULL, 0, 0, 0, 4 },
	// set log level
	{ MODBUS_SET_LOG_LEVEL_ADDR, modbus_set_log_level, NULL, 0, 0, 0, 1 },
	// get board stage
	{ MODBUS_GET_BOARD_STAGE_ADDR, NULL, modbus_get_board_stage, 0, 0, 0, 1 },
	// get fan table revision
	{ MODBUS_GET_FAN_TABLE_REVISION_ADDR, NULL, modbus_get_fan_table_revision, 0, 0, 0, 1 },
	// failure status
	{ MODBUS_STATUS_FALG_SET_CFG_ADDR, modbus_set_status_flag_config, NULL, 0, 0, 0, 1 },
	{ MODBUS_GET_SET_STATUS_FALG_ADDR, modbus_status_flag_set, modbus_status_flag_get, 0, 0, 0,
	  2 },
};

modbus_command_mapping *ptr_to_modbus_table(uint16_t addr)
{
	for (uint16_t i = 0; i < ARRAY_SIZE(modbus_command_table); i++) {
		if ((addr >= modbus_command_table[i].addr) &&
		    (addr < (modbus_command_table[i].addr + modbus_command_table[i].cmd_size)))
			return &modbus_command_table[i];
	}

	return NULL;
}

static void free_modbus_command_table_memory(void)
{
	for (uint16_t i = 0; i < ARRAY_SIZE(modbus_command_table); i++)
		if (modbus_command_table[i].data)
			SAFE_FREE(modbus_command_table[i].data);
}

void init_modbus_command_table(void)
{
	for (uint16_t i = 0; i < ARRAY_SIZE(modbus_command_table); i++) {
		if (modbus_command_table[i].data)
			SAFE_FREE(modbus_command_table[i].data);

		modbus_command_table[i].data =
			(uint16_t *)malloc(modbus_command_table[i].cmd_size * sizeof(uint16_t));
		if (modbus_command_table[i].data == NULL) {
			LOG_ERR("modbus_command_mapping[%i] malloc fail", i);
			goto init_fail;
		}
	}

	return;

init_fail:
	free_modbus_command_table_memory();
}

static int coil_rd(uint16_t addr, bool *state)
{
	CHECK_NULL_ARG_WITH_RETURN(state, MODBUS_EXC_ILLEGAL_DATA_VAL);

	if (addr < plat_gpio_cfg_size()) { //GPIO number
		int val = gpio_get((uint8_t)addr);
		if (val == 0 || val == 1)
			*state = (bool)val;
		else
			return MODBUS_EXC_SERVER_DEVICE_FAILURE;
	} else {
		return MODBUS_EXC_ILLEGAL_DATA_ADDR;
	}

	return MODBUS_EXC_NONE;
}

static int coil_wr(uint16_t addr, bool state)
{
	if (addr < plat_gpio_cfg_size()) { //GPIO number
		gpio_set((uint8_t)addr, (uint8_t)state);
		return MODBUS_EXC_NONE;
	} else if (addr == MODBUS_RPU_RUN_ADDR) { // FW update: Set RPU Stop/Run
		if (!state) { //Set RPU Stop
			controlFSC(FSC_DISABLE);
			pump_redundant_enable(0);
			if (ctl_all_pwm_dev(100))
				LOG_ERR("Set full status failed for all pumps.");
			//return MODBUS_EXC_SERVER_DEVICE_FAILURE;

			disable_sensor_poll();

			/* disable modbus except MODBUS0(BMC) */
			for (uint8_t i = 0; i < ARRAY_SIZE(modbus_server_config); i++) {
				if (strcmp(modbus_server_config[i].iface_name, "MODBUS0") != 0) {
					int server_iface = modbus_iface_get_by_name(
						modbus_server_config[i].iface_name);
					modbus_disable(server_iface);
				}
			}
		} else {
			shutdown_save_uptime_action();
			submit_bic_warm_reset();
		}
		// return success for Setting RPU RUN
		return MODBUS_EXC_NONE;
	} else if (addr == MODBUS_SYNAX_CHECK_ADDR) { // FW update: Synax Check
		if (state) {
			if (!get_sensor_poll_enable_flag())
				return MODBUS_EXC_NONE;
			else
				return MODBUS_EXC_SERVER_DEVICE_FAILURE;
		}
		return MODBUS_EXC_ILLEGAL_DATA_VAL;
	} else {
		return MODBUS_EXC_ILLEGAL_DATA_ADDR;
	}
}

static int holding_reg_multi_wr(char *iface_name, uint16_t addr, uint16_t *reg, uint16_t num_regs)
{
	modbus_command_mapping *ptr = ptr_to_modbus_table(addr);
	if (!ptr) {
		LOG_ERR("modbus write command 0x%x not find!\n", addr);
		return MODBUS_EXC_ILLEGAL_DATA_ADDR;
	}

	if (!ptr->wr_fn) {
		LOG_ERR("modbus write function 0x%x not set!\n", addr);
		return MODBUS_EXC_ILLEGAL_DATA_VAL;
	}

	ptr->start_addr = addr;
	ptr->data_len = num_regs;
	memcpy(ptr->data, reg, sizeof(uint16_t) * ptr->data_len);

	return ptr->wr_fn(ptr);
}

static int holding_reg_multi_rd(char *iface_name, uint16_t addr, uint16_t *reg, uint16_t num_regs)
{
	CHECK_NULL_ARG_WITH_RETURN(reg, MODBUS_EXC_ILLEGAL_DATA_VAL);

	modbus_command_mapping *ptr = ptr_to_modbus_table(addr);
	if (!ptr) {
		LOG_ERR("modbus read command 0x%x not find!\n", addr);
		return MODBUS_EXC_ILLEGAL_DATA_ADDR;
	}

	if (!ptr->rd_fn) {
		LOG_ERR("modbus read function 0x%x not set!\n", addr);
		return MODBUS_EXC_ILLEGAL_DATA_VAL;
	}

	int ret = MODBUS_EXC_NONE;
	ptr->start_addr = addr;
	ptr->data_len = num_regs;

	ret = ptr->rd_fn(ptr);

	memcpy(reg, ptr->data, sizeof(uint16_t) * ptr->data_len);

	return ret;
}

static struct modbus_user_callbacks mbs_cbs = {
	.coil_rd = coil_rd,
	.coil_wr = coil_wr,
	.holding_reg_multi_rd = holding_reg_multi_rd,
	.holding_reg_multi_wr = holding_reg_multi_wr,
};

static struct modbus_iface_param server_param = {
	.mode = MODBUS_MODE_RTU,
	.server = {
		.user_cb = &mbs_cbs,
		.unit_id = MODBUS_UART_NODE_ADDR,
	},
	.serial = {
		.baud = MODBUS_UART_BAUDRATE_LOW,
		.parity = MODBUS_UART_PARITY,
		.parity_none_1_stop_bit = true,
	}
};

static bool custom_handler_fc64(const int iface, const struct modbus_adu *rx_adu,
				struct modbus_adu *tx_adu, uint8_t *const excep_code,
				void *const user_data)
{
	static uint8_t req_len = 4;
	static uint8_t res_len = 4;

	if (rx_adu->length != req_len) {
		*excep_code = MODBUS_EXC_ILLEGAL_DATA_VAL;
		return true;
	}
	uint16_t addr = (rx_adu->data[0] << 8) | rx_adu->data[1];
	if (addr == FW_UPDATE_SWITCH_ADDR) {
		uint16_t data = (rx_adu->data[2] << 8) | rx_adu->data[3];
		if (data == FW_UPDATE_ENABLE_DATA || data == FW_UPDATE_DISABLE_DATA) {
			memcpy(&tx_adu->data, &rx_adu->data, req_len);
			tx_adu->length = res_len;
		} else {
			*excep_code = MODBUS_EXC_ILLEGAL_DATA_VAL;
		}
	} else {
		*excep_code = MODBUS_EXC_ILLEGAL_DATA_ADDR;
	}
	return true;
}

MODBUS_CUSTOM_FC_DEFINE(custom_fc64, custom_handler_fc64, FW_UPDATE_SWITCH_FC, NULL);

int init_custom_modbus_server(void)
{
	LOG_INF("init_custom_modbus_server");
	int server_iface;
	int ret = 0; // 0: success
	for (uint8_t i = 0; i < ARRAY_SIZE(modbus_server_config); i++) {
		uint8_t addr = MODBUS_UART_NODE_ADDR;
		// modbus 0 read addr from eeprom
		if (i == 0) {
			if (!plat_eeprom_read(EEPROM_RPU_ADDR_OFFSET, &addr,
					      EEPROM_HMI_VERSION_SIZE)) {
				LOG_ERR("read modbus0 addr fail!");
				return -ENODEV;
			}

			// default eeprom val
			if (addr == 0xFF)
				addr = MODBUS_UART_NODE_ADDR;
		}

		server_param.server.unit_id = addr;

		server_iface = modbus_iface_get_by_name(modbus_server_config[i].iface_name);

		if (server_iface < 0) {
			LOG_ERR("Failed to get iface index for %s",
				modbus_server_config[i].iface_name);
			return -ENODEV;
		}

		int err = modbus_init_server(server_iface, server_param);

		if (err < 0) {
			LOG_ERR("modbus_init_server fail %d\n", i);
			return err;
		}

		if (modbus_server_config[i].is_custom_fc64) {
			ret = modbus_register_user_fc(server_iface, &modbus_cfg_custom_fc64);
			if (ret)
				return ret;
		}

		modbus_server_config[i].addr = server_param.server.unit_id;
	}

	return ret;
}

int change_modbus_slave_addr(uint8_t idx, uint8_t addr)
{
	server_param.server.unit_id = addr;

	int server_iface = modbus_iface_get_by_name(modbus_server_config[idx].iface_name);
	modbus_disable(server_iface);

	if (server_iface < 0) {
		LOG_ERR("Failed to get iface index for %s", modbus_server_config[idx].iface_name);
		return -ENODEV;
	}

	int err = modbus_init_server(server_iface, server_param);
	int ret = 0;

	if (err < 0) {
		LOG_ERR("modbus_init_server fail %d\n", idx);
		return err;
	}

	if (modbus_server_config[idx].is_custom_fc64) {
		ret = modbus_register_user_fc(server_iface, &modbus_cfg_custom_fc64);
		if (ret)
			return ret;
	}

	modbus_server_config[idx].addr = addr;

	return ret;
}
