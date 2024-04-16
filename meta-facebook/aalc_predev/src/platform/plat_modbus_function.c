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
#include <string.h>
#include "plat_i2c.h"
#include "sensor.h"
#include <logging/log.h>
#include "plat_modbus.h"
#include "plat_modbus_function.h"
#include <sys/util.h>
#include "plat_sensor_table.h"

LOG_MODULE_REGISTER(plat_modbus_funtion);

enum pump_state {
	PUMP_REDUNDENT_SWITCHED,
	MANUAL_CONTROL_PUMP,
	MANUAL_CONTROL_FAN,
	AUTOTUNE_FLOW_CONTROL,
	AUTOTUNE_PRESSURE_BALANCE_CONTROL,
	SYSTEM_STOP,
	RPU_REMOTE_POWER_CYCLE,
	SET_PUMP_GROUP_1,
	SET_PUMP_GROUP_2,
	MANUAL_CONTROL,
	CLEAR_PUMP_RUNNING_TIME,
	CLEAR_LOG,
	PUMP_1_RESET,
	PUMP_2_RESET,
	PUMP_3_RESET,
	PUMP_4_RESET,
};
static sensor_cfg *get_sensor_config_data(uint8_t sensor_num)
{
	// Check sensor information in sensor config table
	sensor_cfg *cfg = NULL;
	cfg = find_sensor_cfg_via_sensor_num(sensor_config, sensor_config_count, sensor_num);
	if (cfg == NULL) {
		LOG_ERR("Fail to find sensor info in config table, sensor_num: 0x%x, cfg count: 0x%x",
			sensor_num, sensor_config_count);
		return NULL;
	}
	return cfg;
}

uint16_t modbus_pump_reset(uint8_t sensor_num)
{
	// Check sensor information in sensor config table
	sensor_cfg *cfg = get_sensor_config_data(sensor_num);
	//uint8_t bus,uint8_t addr, bool enable_flag
	uint8_t bus = cfg->port;
	uint8_t addr = cfg->target_addr;
	bool enable_flag = false;
	// 1 enable, 0 disable, stop pump first
	if (enable_adm1272_hsc(bus, addr, enable_flag) == true) {
		// check pump is already enable
		k_msleep(3000);
		// enable pump
		if (enable_adm1272_hsc(bus, addr, 1) == true) {
			return MODBUS_EXC_NONE;
		} else {
			return MODBUS_EXC_GW_TARGET_FAILED_TO_RESP;
		}
	} else {
		return MODBUS_EXC_GW_TARGET_FAILED_TO_RESP;
	}
}

uint8_t modbus_pump_setting(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	switch ((int)cmd->data) {
	case PUMP_REDUNDENT_SWITCHED:
		break;
	case MANUAL_CONTROL_PUMP:
		break;
	case MANUAL_CONTROL_FAN:
		break;
	case AUTOTUNE_FLOW_CONTROL:
		break;
	case AUTOTUNE_PRESSURE_BALANCE_CONTROL:
		break;
	case SYSTEM_STOP:
		break;
	case RPU_REMOTE_POWER_CYCLE:
		break;
	case SET_PUMP_GROUP_1:
		break;
	case SET_PUMP_GROUP_2:
		break;
	case MANUAL_CONTROL:
		break;
	case CLEAR_PUMP_RUNNING_TIME:
		break;
	case CLEAR_LOG:
		break;
	case PUMP_1_RESET:
		return modbus_pump_reset(SENSOR_NUM_PB_1_HSC_P48V_PIN_PWR_W);
		break;
	case PUMP_2_RESET:
		return modbus_pump_reset(SENSOR_NUM_PB_2_HSC_P48V_PIN_PWR_W);
		break;
	case PUMP_3_RESET:
		return modbus_pump_reset(SENSOR_NUM_PB_3_HSC_P48V_PIN_PWR_W);
		break;
	case PUMP_4_RESET:
		break;
	default:
		LOG_ERR("invalid pump setting");
		return MODBUS_EXC_ILLEGAL_DATA_VAL;
	}
	return MODBUS_EXC_NONE;
}