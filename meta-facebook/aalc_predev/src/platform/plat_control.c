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

bool pump_reset(uint8_t sensor_num)
{
	// Check sensor information in sensor config table
	sensor_cfg *cfg = get_sensor_config_data(sensor_num);
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
		if (enable_adm1272_hsc(bus, addr, true)){
			return true;
		}
		else{
			LOG_ERR("Fail when start the pump.");
			return false;
		}
	} else {
		LOG_ERR("Fail when stop the pump.");
		return false;
	}
}
uint8_t modbus_pump_setting(modbus_command_mapping *cmd)
{
		CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

		switch (cmd->data[0]) {
		case PUMP_REDUNDENT_SWITCHED:
		case MANUAL_CONTROL_PUMP:
		case MANUAL_CONTROL_FAN:
		case AUTOTUNE_FLOW_CONTROL:
		case AUTOTUNE_PRESSURE_BALANCE_CONTROL:
		case SYSTEM_STOP:
		case RPU_REMOTE_POWER_CYCLE:
		case MANUAL_CONTROL:
		case CLEAR_PUMP_RUNNING_TIME:
		case CLEAR_LOG:
		case PUMP_1_RESET:
			if (pump_reset(SENSOR_NUM_PB_1_HSC_P48V_PIN_PWR_W))
				break;
			return MODBUS_EXC_SERVER_DEVICE_FAILURE;
		case PUMP_2_RESET:
			if (pump_reset(SENSOR_NUM_PB_2_HSC_P48V_PIN_PWR_W))
				break;
			return MODBUS_EXC_SERVER_DEVICE_FAILURE;	
		case PUMP_3_RESET:
			if (pump_reset(SENSOR_NUM_PB_3_HSC_P48V_PIN_PWR_W))
				break;
			return MODBUS_EXC_SERVER_DEVICE_FAILURE;
		default:
			LOG_ERR("invalid pump setting");
			return MODBUS_EXC_ILLEGAL_DATA_VAL;
		}
		return MODBUS_EXC_NONE;
}

