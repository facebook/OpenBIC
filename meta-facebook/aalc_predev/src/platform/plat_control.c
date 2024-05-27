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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "plat_i2c.h"
#include "sensor.h"
#include <logging/log.h>
#include <sys/util.h>
#include "plat_sensor_table.h"
#include "plat_modbus.h"
#include "modbus_server.h"
#include "plat_control.h"
#include "adm1272.h"
//#include "plat_log.h"

LOG_MODULE_REGISTER(plat_modbus_funtion);

static sensor_cfg *get_sensor_config_data(uint8_t sensor_num)
{
	// Check sensor information in sensor config table
	sensor_cfg *cfg = NULL;
	cfg = find_sensor_cfg_via_sensor_num(sensor_config, sensor_config_count, sensor_num);
	if (cfg == NULL)
		LOG_ERR("Fail to find sensor info in config table, sensor_num: 0x%x, cfg count: 0x%x",
			sensor_num, sensor_config_count);
	return cfg;
}

bool modbus_pump_setting_unsupport_function(pump_reset_struct *data)
{
	return true;
}

bool clear_log_for_modbus_pump_setting(pump_reset_struct *data)
{
	if (data->bit_value == 0) // do nothing
		return true;

	//bool clear_log_status = modbus_clear_log();
	//return clear_log_status;

	return false;
}

bool pump_reset(pump_reset_struct *data)
{
	if (data == NULL)
		return false;

	if (data->bit_value == 0) // do nothing
		return true;

	// Check sensor information in sensor config table
	sensor_cfg *cfg = get_sensor_config_data(data->senser_num);
	if (cfg == NULL)
		return false;
	//uint8_t bus,uint8_t addr, bool enable_flag
	uint8_t bus = cfg->port;
	uint8_t addr = cfg->target_addr;
	// 1 enable, 0 disable, stop pump first
	if (enable_adm1272_hsc(bus, addr, false)) {
		// check pump is already enable
		k_msleep(500);
		// enable pump
		if (enable_adm1272_hsc(bus, addr, true)) {
			return true;
		} else {
			LOG_ERR("Fail when start the pump.");
			return false;
		}
	} else {
		LOG_ERR("Fail when stop the pump.");
		return false;
	}
}

pump_reset_struct modbus_pump_setting_table[] = {
	{ PUMP_REDUNDENT_SWITCHED, 0, 0, 0 },
	{ MANUAL_CONTROL_PUMP, 0, modbus_pump_setting_unsupport_function, 0 },
	{ MANUAL_CONTROL_FAN, 0, modbus_pump_setting_unsupport_function, 0 },
	{ AUTOTUNE_FLOW_CONTROL, 0, modbus_pump_setting_unsupport_function, 0 },
	{ AUTOTUNE_PRESSURE_BALANCE_CONTROL, 0, modbus_pump_setting_unsupport_function, 0 },
	{ SYSTEM_STOP, 0, modbus_pump_setting_unsupport_function, 0 },
	{ RPU_REMOTE_POWER_CYCLE, 0, modbus_pump_setting_unsupport_function, 0 },
	{ RESERVED_0, 0, 0, 0 },
	{ RESERVED_1, 0, 0, 0 },
	{ MANUAL_CONTROL, 0, modbus_pump_setting_unsupport_function, 0 },
	{ CLEAR_PUMP_RUNNING_TIME, 0, modbus_pump_setting_unsupport_function, 0 },
	{ CLEAR_LOG, 0, clear_log_for_modbus_pump_setting, 0 },
	{ PUMP_1_RESET, 0, pump_reset, SENSOR_NUM_PB_1_HSC_P48V_PIN_PWR_W },
	{ PUMP_2_RESET, 0, pump_reset, SENSOR_NUM_PB_2_HSC_P48V_PIN_PWR_W },
	{ PUMP_3_RESET, 0, pump_reset, SENSOR_NUM_PB_3_HSC_P48V_PIN_PWR_W },
};

uint8_t modbus_pump_setting(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);
	uint16_t check_error_flag = 0;
	for (int i = 0; i < MAX_STATE; i++) {
		// if bit is 1
		if (cmd->data[0] & BIT(modbus_pump_setting_table[i].function_index))
			modbus_pump_setting_table[i].bit_value = 1;

		bool result_status = modbus_pump_setting_table[i].fn(&modbus_pump_setting_table[i]);
		if (result_status == false) {
			LOG_ERR("modebus 0x9410 setting %d-bit error\n",
				modbus_pump_setting_table[i].function_index);
			WRITE_BIT(check_error_flag, modbus_pump_setting_table[i].function_index, 1);
		}
	}

	if (check_error_flag) {
		LOG_ERR("modebus 0x9410 setting error flag: 0x%x\n", check_error_flag);
		return MODBUS_EXC_ILLEGAL_DATA_VAL;
	} else {
		return MODBUS_EXC_NONE;
	}
}
