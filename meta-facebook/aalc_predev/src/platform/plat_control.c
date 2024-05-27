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
	uint16_t check_error_flag = 0;

	// PUMP_REDUNDENT_SWITCHED
	if (cmd->data[0] & BIT(PUMP_REDUNDENT_SWITCHED)){ // if bit0 != 0
		//if (<PUMP_REDUNDENT_SWITCHED function>){
			LOG_ERR("modebus 0x9410 setting %d-bit error\n", PUMP_REDUNDENT_SWITCHED);
			WRITE_BIT(check_error_flag, PUMP_REDUNDENT_SWITCHED, 1);
		//}
	}
	else {
		//if (<PUMP_REDUNDENT_SWITCHED function>){
			LOG_ERR("modebus 0x9410 setting %d-bit error\n", PUMP_REDUNDENT_SWITCHED);
			WRITE_BIT(check_error_flag, PUMP_REDUNDENT_SWITCHED, 1);
		//}
	}

	// MANUAL_CONTROL_PUMP:
	if (cmd->data[0] & BIT(MANUAL_CONTROL_PUMP)){
		//if (<MANUAL_CONTROL_PUMP function>){
			LOG_ERR("modebus 0x9410 setting %d-bit error\n", MANUAL_CONTROL_PUMP);
			WRITE_BIT(check_error_flag, MANUAL_CONTROL_PUMP, 1);
		//}
	}
	else {
		//if (<MANUAL_CONTROL_PUMP function>){
			LOG_ERR("modebus 0x9410 setting %d-bit error\n", MANUAL_CONTROL_PUMP);
			WRITE_BIT(check_error_flag, MANUAL_CONTROL_PUMP, 1);
		//}	
	}

	// MANUAL_CONTROL_FAN:
	if (cmd->data[0] & BIT(MANUAL_CONTROL_FAN)){
		//if (<MANUAL_CONTROL_FAN function>){
			LOG_ERR("modebus 0x9410 setting %d-bit error\n", MANUAL_CONTROL_FAN);
			WRITE_BIT(check_error_flag, MANUAL_CONTROL_FAN, 1);
		//}
	}
	else {
		//if (<MANUAL_CONTROL_FAN function>){
			LOG_ERR("modebus 0x9410 setting %d-bit error\n", MANUAL_CONTROL_FAN);
			WRITE_BIT(check_error_flag, MANUAL_CONTROL_FAN, 1);
		//}	
	}

	// AUTOTUNE_FLOW_CONTROL:
	if (cmd->data[0] & BIT(AUTOTUNE_FLOW_CONTROL)){
		//if (<AUTOTUNE_FLOW_CONTROL function>){
			LOG_ERR("modebus 0x9410 setting %d-bit error\n", AUTOTUNE_FLOW_CONTROL);
			WRITE_BIT(check_error_flag, AUTOTUNE_FLOW_CONTROL, 1);
		//}	
	}
	else {
		//if (<AUTOTUNE_FLOW_CONTROL function>){
			LOG_ERR("modebus 0x9410 setting %d-bit error\n", AUTOTUNE_FLOW_CONTROL);
			WRITE_BIT(check_error_flag, AUTOTUNE_FLOW_CONTROL, 1);
		//}		
	}

	// AUTOTUNE_PRESSURE_BALANCE_CONTROL:
	if (cmd->data[0] & BIT(AUTOTUNE_PRESSURE_BALANCE_CONTROL)){
		//if (<AUTOTUNE_PRESSURE_BALANCE_CONTROL function>){
			LOG_ERR("modebus 0x9410 setting %d-bit error\n", AUTOTUNE_PRESSURE_BALANCE_CONTROL);
			WRITE_BIT(check_error_flag, AUTOTUNE_PRESSURE_BALANCE_CONTROL, 1);
		//}	
	}
	else {
		//if (<AUTOTUNE_PRESSURE_BALANCE_CONTROL function>){
			LOG_ERR("modebus 0x9410 setting %d-bit error\n", AUTOTUNE_PRESSURE_BALANCE_CONTROL);
			WRITE_BIT(check_error_flag, AUTOTUNE_PRESSURE_BALANCE_CONTROL, 1);
		//}	
	}

	// SYSTEM_STOP:
	if (cmd->data[0] & BIT(SYSTEM_STOP)){
		//if (<SYSTEM_STOP function>){
			LOG_ERR("modebus 0x9410 setting %d-bit error\n", SYSTEM_STOP);
			WRITE_BIT(check_error_flag, SYSTEM_STOP, 1);
		//}	
	}
	else {
		//if (<SYSTEM_STOP function>){
			LOG_ERR("modebus 0x9410 setting %d-bit error\n", SYSTEM_STOP);
			WRITE_BIT(check_error_flag, SYSTEM_STOP, 1);
		//}
	}

	// RPU_REMOTE_POWER_CYCLE:
	if (cmd->data[0] & BIT(RPU_REMOTE_POWER_CYCLE)){
		//if (<RPU_REMOTE_POWER_CYCLE function>){
			LOG_ERR("modebus 0x9410 setting %d-bit error\n", RPU_REMOTE_POWER_CYCLE);
			WRITE_BIT(check_error_flag, RPU_REMOTE_POWER_CYCLE, 1);
		//}
	}
	else {
		//if (<RPU_REMOTE_POWER_CYCLE function>){
			LOG_ERR("modebus 0x9410 setting %d-bit error\n", RPU_REMOTE_POWER_CYCLE);
			WRITE_BIT(check_error_flag, RPU_REMOTE_POWER_CYCLE, 1);
		//}
	}

	// MANUAL_CONTROL:
	if (cmd->data[0] & BIT(MANUAL_CONTROL)){
		//if (<MANUAL_CONTROL function>){
			LOG_ERR("modebus 0x9410 setting %d-bit error\n", MANUAL_CONTROL);
			WRITE_BIT(check_error_flag, MANUAL_CONTROL, 1);
		//}
	}
	else {
		//if (<MANUAL_CONTROL function>){
			LOG_ERR("modebus 0x9410 setting %d-bit error\n", MANUAL_CONTROL);
			WRITE_BIT(check_error_flag, MANUAL_CONTROL, 1);
		//}		
	}

	// CLEAR_PUMP_RUNNING_TIME:
	if (cmd->data[0] & BIT(CLEAR_PUMP_RUNNING_TIME)){
		//if (<CLEAR_PUMP_RUNNING_TIME function>){
			LOG_ERR("modebus 0x9410 setting %d-bit error\n", CLEAR_PUMP_RUNNING_TIME);
			WRITE_BIT(check_error_flag, CLEAR_PUMP_RUNNING_TIME, 1);
		//}
	}
	else {
		//if (<CLEAR_PUMP_RUNNING_TIME function>){
			LOG_ERR("modebus 0x9410 setting %d-bit error\n", CLEAR_PUMP_RUNNING_TIME);
			WRITE_BIT(check_error_flag, CLEAR_PUMP_RUNNING_TIME, 1);
		//}
	}

	// CLEAR_LOG:
	if (cmd->data[0] & BIT(CLEAR_LOG)){
		//if (!modbus_clear_log())
			LOG_ERR("modebus 0x9410 setting %d-bit error\n", CLEAR_LOG);
			WRITE_BIT(check_error_flag, CLEAR_LOG, 1);
	}

	// PUMP_1_RESET
	if (cmd->data[0] & BIT(PUMP_1_RESET)){
		if (!pump_reset(SENSOR_NUM_PB_1_HSC_P48V_PIN_PWR_W)){
			LOG_ERR("modebus 0x9410 setting %d-bit error\n", PUMP_1_RESET);
			WRITE_BIT(check_error_flag, PUMP_1_RESET, 1);
		}
	}

	// PUMP_2_RESET
	if (cmd->data[0] & BIT(PUMP_2_RESET)){
		if (!pump_reset(SENSOR_NUM_PB_2_HSC_P48V_PIN_PWR_W)){
			LOG_ERR("modebus 0x9410 setting %d-bit error\n", PUMP_2_RESET);
			WRITE_BIT(check_error_flag, PUMP_2_RESET, 1);
		}
	}

	// PUMP_3_RESET
	if (cmd->data[0] & BIT(PUMP_3_RESET)){
		if (!pump_reset(SENSOR_NUM_PB_3_HSC_P48V_PIN_PWR_W)){
			LOG_ERR("modebus 0x9410 setting %d-bit error\n", PUMP_3_RESET);
			WRITE_BIT(check_error_flag, PUMP_3_RESET, 1);	
		}
	}

	if (check_error_flag){
		LOG_ERR("modebus 0x9410 setting error flag: 0x%x\n", check_error_flag);
		return MODBUS_EXC_ILLEGAL_DATA_VAL;
	}
	else{
		return MODBUS_EXC_NONE;
	}
   
}

