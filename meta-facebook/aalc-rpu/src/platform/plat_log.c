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

#include <kernel.h>
#include <stdlib.h>
#include "plat_log.h"
#include "modbus_server.h"
#include <logging/log.h>
#include <libutil.h>
#include "plat_modbus.h"
#include "plat_util.h"
#include "plat_pwm.h"
#include "plat_sensor_table.h"
#include "fru.h"
#include "plat_fru.h"

LOG_MODULE_REGISTER(plat_log);

#define LOG_MAX_INDEX 0x0FFF //recount when log index > 0x0FFF
#define LOG_MAX_NUM 30 // total log amount: 30
#define AALC_FRU_LOG_START 0x4000 //log offset: 16KB
#define AALC_FRU_LOG_SIZE                                                                          \
	(LOG_MAX_NUM * sizeof(modbus_err_log_mapping)) // 30 logs(1 log = 20 bytes)

static modbus_err_log_mapping err_log_data[LOG_MAX_NUM];

const err_sensor_mapping sensor_err_codes[] = {
	{ LEAK_CHASSIS_0, SENSOR_NUM_IT_LEAK_0_GPIO },
	{ LEAK_CHASSIS_1, SENSOR_NUM_IT_LEAK_1_GPIO },
	{ LEAK_CHASSIS_2, SENSOR_NUM_IT_LEAK_2_GPIO },
	{ LEAK_CHASSIS_3, SENSOR_NUM_IT_LEAK_3_GPIO },
	{ LEAK_RPU_INT, SENSOR_NUM_BPB_CDU_COOLANT_LEAKAGE_VOLT_V },
	{ LEAK_MAN_PAN_GPO, SENSOR_NUM_BPB_RACK_COOLANT_LEAKAGE_VOLT_V },
	{ LOW_WATER_LEVEL, SENSOR_NUM_BPB_RACK_LEVEL_2 },
	{ PUMP_1_SPEED_ABNORMAL, SENSOR_NUM_PB_1_PUMP_TACH_RPM },
	{ PUMP_2_SPEED_ABNORMAL, SENSOR_NUM_PB_2_PUMP_TACH_RPM },
	{ PUMP_3_SPEED_ABNORMAL, SENSOR_NUM_PB_3_PUMP_TACH_RPM },
	{ HIGH_PRESS_DETECTED, SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_P_KPA },
	{ FLOW_RATE_SENSOR_TRIGGERED, SENSOR_NUM_BPB_RPU_COOLANT_FLOW_RATE_LPM },
	{ EMERGENCY_BUTTON_TRIGGERED, SENSOR_NUM_EMERGENCY_BUTTON_TRIGGERED },
	{ LOG_ERR_HEX_EXTERNAL_Y_FILTER, SENSOR_NUM_HEX_EXTERNAL_Y_FILTER },
	{ LOG_ERR_BPB_RPU_COOLANT_INLET_P_KPA, SENSOR_NUM_BPB_RPU_COOLANT_INLET_P_KPA },
	{ LOG_ERR_BPB_RACK_PRESSURE_3_P_KPA, SENSOR_NUM_BPB_RACK_PRESSURE_3_P_KPA },
	{ LOG_ERR_BPB_RACK_PRESSURE_4_P_KPA, SENSOR_NUM_BPB_RACK_PRESSURE_4_P_KPA },
	{ LOG_ERR_SB_HEX_PRESSURE_1_P_KPA, SENSOR_NUM_SB_HEX_PRESSURE_1_P_KPA },
	{ LOG_ERR_SB_HEX_PRESSURE_2_P_KPA, SENSOR_NUM_SB_HEX_PRESSURE_2_P_KPA },
	{ LOG_ERR_BPB_RPU_COOLANT_INLET_TEMP_C, SENSOR_NUM_BPB_RPU_COOLANT_INLET_TEMP_C },
	{ LOG_ERR_BPB_RPU_COOLANT_OUTLET_TEMP_C, SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C },
	{ LOG_ERR_BPB_HEX_WATER_INLET_TEMP_C, SENSOR_NUM_BPB_HEX_WATER_INLET_TEMP_C },
	{ LOG_ERR_SB_HEX_AIR_INLET_AVG_TEMP_C, SENSOR_NUM_SB_HEX_AIR_INLET_AVG_TEMP_C },
	{ LOG_ERR_FB_1_FAN_TACH_RPM, SENSOR_NUM_FB_1_FAN_TACH_RPM },
	{ LOG_ERR_FB_2_FAN_TACH_RPM, SENSOR_NUM_FB_2_FAN_TACH_RPM },
	{ LOG_ERR_FB_3_FAN_TACH_RPM, SENSOR_NUM_FB_3_FAN_TACH_RPM },
	{ LOG_ERR_FB_4_FAN_TACH_RPM, SENSOR_NUM_FB_4_FAN_TACH_RPM },
	{ LOG_ERR_FB_5_FAN_TACH_RPM, SENSOR_NUM_FB_5_FAN_TACH_RPM },
	{ LOG_ERR_FB_6_FAN_TACH_RPM, SENSOR_NUM_FB_6_FAN_TACH_RPM },
	{ LOG_ERR_FB_7_FAN_TACH_RPM, SENSOR_NUM_FB_7_FAN_TACH_RPM },
	{ LOG_ERR_FB_8_FAN_TACH_RPM, SENSOR_NUM_FB_8_FAN_TACH_RPM },
	{ LOG_ERR_FB_9_FAN_TACH_RPM, SENSOR_NUM_FB_9_FAN_TACH_RPM },
	{ LOG_ERR_FB_10_FAN_TACH_RPM, SENSOR_NUM_FB_10_FAN_TACH_RPM },
	{ LOG_ERR_FB_11_FAN_TACH_RPM, SENSOR_NUM_FB_11_FAN_TACH_RPM },
	{ LOG_ERR_FB_12_FAN_TACH_RPM, SENSOR_NUM_FB_12_FAN_TACH_RPM },
	{ LOG_ERR_FB_13_FAN_TACH_RPM, SENSOR_NUM_FB_13_FAN_TACH_RPM },
	{ LOG_ERR_FB_14_FAN_TACH_RPM, SENSOR_NUM_FB_14_FAN_TACH_RPM },
	{ LOG_ERR_BPB_HSC, SENSOR_NUM_BPB_HSC_FAIL },
};

const err_sensor_mapping sensor_normal_codes[] = {
	{ PUMP_1_SPEED_RECOVER, SENSOR_NUM_PB_1_PUMP_TACH_RPM },
	{ PUMP_2_SPEED_RECOVER, SENSOR_NUM_PB_2_PUMP_TACH_RPM },
	{ PUMP_3_SPEED_RECOVER, SENSOR_NUM_PB_3_PUMP_TACH_RPM },
};

uint16_t error_log_count(void)
{
	for (uint16_t i = 0; i < LOG_MAX_NUM; i++) {
		if (err_log_data[i].index == 0xFFFF)
			return i;
	}
	return (uint16_t)LOG_MAX_NUM;
}

/* 
(order = N of "the Nth newest event")
input order to get the position in log array(err_log_data)
*/
static uint16_t get_log_position_by_time_order(uint8_t order)
{
	if (order > LOG_MAX_NUM) {
		LOG_ERR("Invalid LOG count  %d", order);
		return 0;
	}

	uint16_t i = 0;

	for (i = 0; i < LOG_MAX_NUM - 1; i++) {
		/* the index is continuous */
		if (err_log_data[i + 1].index == (err_log_data[i].index + 1))
			continue;

		/* reset the index after LOG_MAX_INDEX */
		if (err_log_data[i + 1].index == 1 && err_log_data[i].index == LOG_MAX_INDEX)
			continue;

		break;
	}

	return (i + LOG_MAX_NUM - (order - 1)) % LOG_MAX_NUM;
}
void log_transfer_to_modbus_data(uint16_t *modbus_data, uint8_t cmd_size, uint16_t order)
{
	CHECK_NULL_ARG(modbus_data);

	memcpy(modbus_data, &err_log_data[get_log_position_by_time_order(order)],
	       sizeof(uint16_t) * cmd_size);

	modbus_err_log_mapping *p = (modbus_err_log_mapping *)modbus_data;
	if (p->index == 0xffff)
		memset(modbus_data, 0x00, sizeof(uint16_t) * cmd_size);
}

void modbus_clear_log()
{
	memset(err_log_data, 0xFF, sizeof(err_log_data));

	for (uint8_t i = 0; i < LOG_MAX_NUM; i++) {
		if (!plat_eeprom_write(AALC_FRU_LOG_START + sizeof(modbus_err_log_mapping) * i,
				       (uint8_t *)err_log_data, sizeof(modbus_err_log_mapping))) {
			LOG_ERR("Clear EEPROM Log failed");
		}
		k_msleep(10); // the eeprom max write time is 10 ms
	}
}

uint32_t get_uptime_secs(void)
{
	return (uint32_t)((k_uptime_get() / 1000) % INT32_MAX);
}

void error_log_event(uint8_t sensor_num, bool val_normal)
{
	uint16_t err_code = 0;

	if (val_normal) {
		for (uint8_t j = 0; j < ARRAY_SIZE(sensor_normal_codes); j++) {
			if (sensor_num == sensor_normal_codes[j].sen_num) {
				err_code = sensor_normal_codes[j].err_code;
				break;
			}
		}
	} else {
		for (uint8_t j = 0; j < ARRAY_SIZE(sensor_err_codes); j++) {
			if (sensor_num == sensor_err_codes[j].sen_num) {
				err_code = sensor_err_codes[j].err_code;
				break;
			}
		}
	}

	// do nothing, if not found
	if (!err_code)
		return;

	uint16_t newest_count = get_log_position_by_time_order(1);
	uint16_t fru_count = (err_log_data[newest_count].index == 0xFFFF) ?
				     newest_count :
				     ((newest_count + 1) % LOG_MAX_NUM);

	err_log_data[fru_count].index = (err_log_data[newest_count].index == LOG_MAX_INDEX ||
					 err_log_data[newest_count].index == 0xFFFF) ?
						1 :
						(err_log_data[newest_count].index + 1);
	err_log_data[fru_count].err_code = err_code;
	err_log_data[fru_count].sys_time = get_uptime_secs();
	err_log_data[fru_count].pump_duty =
		(uint16_t)(get_manual_pwm_flag(MANUAL_PWM_E_PUMP) ?
				   get_manual_pwm_cache(MANUAL_PWM_E_PUMP) :
				   get_pwm_group_cache(PWM_GROUP_E_PUMP));
	err_log_data[fru_count].fan_duty =
		(uint16_t)(get_manual_pwm_flag(MANUAL_PWM_E_HEX_FAN) ?
				   get_manual_pwm_cache(MANUAL_PWM_E_HEX_FAN) :
				   get_pwm_group_cache(PWM_GROUP_E_HEX_FAN));

	err_log_data[fru_count].outlet_temp =
		get_sensor_reading_to_modbus_val(SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C, -1, 1);
	err_log_data[fru_count].outlet_press =
		get_sensor_reading_to_modbus_val(SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_P_KPA, -1, 1);
	err_log_data[fru_count].flow_rate =
		get_sensor_reading_to_modbus_val(SENSOR_NUM_BPB_RPU_COOLANT_FLOW_RATE_LPM, -1, 1);
	err_log_data[fru_count].volt =
		get_sensor_reading_to_modbus_val(SENSOR_NUM_BPB_HSC_P48V_VIN_VOLT_V, -1, 1);

	if (!plat_eeprom_write((AALC_FRU_LOG_START + fru_count * sizeof(modbus_err_log_mapping)),
			       (uint8_t *)&err_log_data[fru_count], sizeof(modbus_err_log_mapping)))
		LOG_ERR("Write Log failed with Error code: %02x", err_code);
	else
		k_msleep(5); // wait 5ms write eeprom
}

void init_load_eeprom_log(void)
{
	// read/write data length has to be less EEPROM_WRITE_SIZE(32)
	memset(err_log_data, 0xFF, sizeof(err_log_data));
	uint16_t log_len = sizeof(modbus_err_log_mapping);
	for (uint8_t i = 0; i < LOG_MAX_NUM; i++) {
		if (!plat_eeprom_read(AALC_FRU_LOG_START + i * log_len, (uint8_t *)&err_log_data[i],
				      log_len)) {
			LOG_ERR("READ Event %d failed from EEPROM", i + 1);
		}
	}
}
