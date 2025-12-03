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
#include "plat_modbus.h"

#define IS_NORMAL_VAL true
#define IS_ABNORMAL_VAL false

uint16_t error_log_count(void);
void log_transfer_to_modbus_data(uint16_t *modbus_data, uint8_t cmd_size, uint16_t order);
void error_log_event(uint8_t sensor_num, bool val_normal);
void init_load_eeprom_log(void);
void modbus_clear_log();

typedef struct _modbus_err_log_mapping {
	uint16_t index;
	uint16_t err_code;
	uint32_t sys_time;
	uint16_t pump_duty;
	uint16_t fan_duty;
	uint16_t outlet_temp;
	uint16_t outlet_press;
	uint16_t flow_rate;
	uint16_t volt;
	uint8_t reserved[12];
} modbus_err_log_mapping;

enum LOG_ERROR_CODE {
	LEAK_CHASSIS_0 = 0x0A,
	LEAK_CHASSIS_1 = 0x0B,
	LEAK_CHASSIS_2 = 0x0C,
	LEAK_CHASSIS_3 = 0x0D,
	LEAK_RPU_INT = 0x0E,
	LEAK_RPU_EXT = 0x0F,
	LEAK_MAN_HOT = 0x10,
	LEAK_MAN_COLD = 0x11,
	LEAK_MAN_PAN_GPO = 0x6E,
	LEAK_MAN_FLOOR_GPO = 0x6F,
	LEAK_MAN_PAN_RELAY = 0x70,
	LEAK_MAN_FLOOR_RELAY = 0x71,
	LOW_WATER_LEVEL = 0x02,
	PUMP_1_SPEED_ABNORMAL = 0x1F,
	PUMP_2_SPEED_ABNORMAL = 0x20,
	PUMP_3_SPEED_ABNORMAL = 0x21,
	PUMP_1_SPEED_RECOVER = 0x29,
	PUMP_2_SPEED_RECOVER = 0x2A,
	PUMP_3_SPEED_RECOVER = 0x2B,
	HIGH_PRESS_DETECTED = 0xA0,
	FLOW_RATE_SENSOR_TRIGGERED = 0xA1,
	EMERGENCY_BUTTON_TRIGGERED = 0xA2,
	LOG_ERR_HEX_EXTERNAL_Y_FILTER = 0xA3,
	LOG_ERR_BPB_RPU_COOLANT_INLET_P_KPA = 0xA4,
	LOG_ERR_BPB_RACK_PRESSURE_3_P_KPA = 0xA5,
	LOG_ERR_BPB_RACK_PRESSURE_4_P_KPA = 0xA6,
	LOG_ERR_SB_HEX_PRESSURE_1_P_KPA = 0xA7,
	LOG_ERR_SB_HEX_PRESSURE_2_P_KPA = 0xA8,
	LOG_ERR_BPB_RPU_COOLANT_INLET_TEMP_C = 0xA9,
	LOG_ERR_BPB_RPU_COOLANT_OUTLET_TEMP_C = 0xAA,
	LOG_ERR_BPB_HEX_WATER_INLET_TEMP_C = 0xAB,
	LOG_ERR_SB_HEX_AIR_INLET_AVG_TEMP_C = 0xAC,
	LOG_ERR_FB_1_FAN_TACH_RPM = 0xAD,
	LOG_ERR_FB_2_FAN_TACH_RPM = 0xAE,
	LOG_ERR_FB_3_FAN_TACH_RPM = 0xAF,
	LOG_ERR_FB_4_FAN_TACH_RPM = 0xB0,
	LOG_ERR_FB_5_FAN_TACH_RPM = 0xB1,
	LOG_ERR_FB_6_FAN_TACH_RPM = 0xB2,
	LOG_ERR_FB_7_FAN_TACH_RPM = 0xB3,
	LOG_ERR_FB_8_FAN_TACH_RPM = 0xB4,
	LOG_ERR_FB_9_FAN_TACH_RPM = 0xB5,
	LOG_ERR_FB_10_FAN_TACH_RPM = 0xB6,
	LOG_ERR_FB_11_FAN_TACH_RPM = 0xB7,
	LOG_ERR_FB_12_FAN_TACH_RPM = 0xB8,
	LOG_ERR_FB_13_FAN_TACH_RPM = 0xB9,
	LOG_ERR_FB_14_FAN_TACH_RPM = 0xBA,
	LOG_ERR_BPB_HSC = 0xBB,
	LOG_ERR_MB_RPU_AIR_INLET_TEMP_C = 0xBC,
	PUMP_1_SPEED_ABNORMAL_UCR = 0xBD,
	PUMP_2_SPEED_ABNORMAL_UCR = 0xBE,
	PUMP_3_SPEED_ABNORMAL_UCR = 0xBF,
	PUMP_1_SPEED_NOT_ACCESS = 0xC0,
	PUMP_2_SPEED_NOT_ACCESS = 0xC1,
	PUMP_3_SPEED_NOT_ACCESS = 0xC2,
};

typedef struct _err_sensor_mapping {
	uint16_t err_code;
	uint8_t sen_num;
} err_sensor_mapping;
