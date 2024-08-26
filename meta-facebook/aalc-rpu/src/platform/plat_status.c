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

#include <stdbool.h>
#include <logging/log.h>
#include "plat_status.h"
#include "plat_sensor_table.h"
#include "plat_fru.h"

#define AALC_STICKY_STATUS_START 0x6000 //log offset: 24KB
#define STICKY_STATUS_SIZE 2 // 1 register

LOG_MODULE_REGISTER(plat_status);

static uint8_t leak_flag;

uint8_t get_leak_status()
{
	return leak_flag;
}

void set_leak_status(uint8_t idx, uint8_t val)
{
	WRITE_BIT(leak_flag, idx, val);
}

uint16_t get_sticky_sensor_status(uint8_t idx)
{
	uint16_t val;

	if (idx >= STICKY_STATUS_MAX) {
		LOG_ERR("get Sticky status failed");
		return 0;
	}

	if (!plat_eeprom_read(AALC_STICKY_STATUS_START + (idx * STICKY_STATUS_SIZE),
			      (uint8_t *)&val, STICKY_STATUS_SIZE))
		LOG_ERR("get Sticky status failed");

	return (val == 0xFFFF) ? 0 : val;
}

bool set_sticky_sensor_status(uint8_t idx, uint16_t val)
{
	if (idx >= STICKY_STATUS_MAX) {
		LOG_ERR("Set Sticky status failed");
		return false;
	}
	uint16_t eeprom_val = (val == 0) ? 0xFFFF : val;

	uint16_t sticky_status = get_sticky_sensor_status(idx);
	if (sticky_status != val) {
		if (!plat_eeprom_write(AALC_STICKY_STATUS_START + (idx * STICKY_STATUS_SIZE),
				       (uint8_t *)&eeprom_val, STICKY_STATUS_SIZE)) {
			LOG_ERR("Write Sticky status failed");
			return false;
		}
	}
	return true;
}

uint16_t get_sensor_status_for_modbus_cmd(uint8_t status)
{
	uint16_t val = 0;

	switch (status) {
	case RPU_FAN_STATUS:
		WRITE_BIT(val, 0, (get_threshold_status(SENSOR_NUM_MB_FAN1_TACH_RPM)) ? 1 : 0);
		WRITE_BIT(val, 1, (get_threshold_status(SENSOR_NUM_MB_FAN2_TACH_RPM)) ? 1 : 0);
		break;
	case RPU_PUMP1_STATUS:
		val = (get_threshold_status(SENSOR_NUM_PB_1_PUMP_TACH_RPM)) ? PUMP_STATUS_ABNORMAL :
									      PUMP_STATUS_ENABLE;
		break;
	case RPU_PUMP2_STATUS:
		val = (get_threshold_status(SENSOR_NUM_PB_2_PUMP_TACH_RPM)) ? PUMP_STATUS_ABNORMAL :
									      PUMP_STATUS_ENABLE;
		break;
	case RPU_PUMP3_STATUS:
		val = (get_threshold_status(SENSOR_NUM_PB_3_PUMP_TACH_RPM)) ? PUMP_STATUS_ABNORMAL :
									      PUMP_STATUS_ENABLE;
		break;
	case RPU_RESERVOIR_STATUS:
		// 0: low, 1: intermediate, 3: high
		WRITE_BIT(val, 0, (get_threshold_status(SENSOR_NUM_BPB_RACK_LEVEL_2)) ? 0 : 1);
		WRITE_BIT(val, 1, (get_threshold_status(SENSOR_NUM_BPB_RACK_LEVEL_1)) ? 0 : 1);
		break;
	case ALL_PUMP_STATUS:
		val = 3;
		if (get_threshold_status(SENSOR_NUM_PB_1_PUMP_TACH_RPM))
			val--;
		if (get_threshold_status(SENSOR_NUM_PB_2_PUMP_TACH_RPM))
			val--;
		if (get_threshold_status(SENSOR_NUM_PB_3_PUMP_TACH_RPM))
			val--;
		break;
	case ALL_RPU_INTERNAL_FAN_STATUS:
		val = 2;
		if (get_threshold_status(SENSOR_NUM_MB_FAN1_TACH_RPM))
			val--;
		if (get_threshold_status(SENSOR_NUM_MB_FAN2_TACH_RPM))
			val--;
		break;
	case PUMP_FAN_STATUS:
		WRITE_BIT(val, 0, (get_threshold_status(SENSOR_NUM_PB_1_FAN_1_TACH_RPM)) ? 1 : 0);
		WRITE_BIT(val, 1, (get_threshold_status(SENSOR_NUM_PB_1_FAN_2_TACH_RPM)) ? 1 : 0);
		WRITE_BIT(val, 2, (get_threshold_status(SENSOR_NUM_PB_2_FAN_1_TACH_RPM)) ? 1 : 0);
		WRITE_BIT(val, 3, (get_threshold_status(SENSOR_NUM_PB_2_FAN_2_TACH_RPM)) ? 1 : 0);
		WRITE_BIT(val, 4, (get_threshold_status(SENSOR_NUM_PB_3_FAN_1_TACH_RPM)) ? 1 : 0);
		WRITE_BIT(val, 5, (get_threshold_status(SENSOR_NUM_PB_3_FAN_2_TACH_RPM)) ? 1 : 0);
		break;
	case HEX_BLADDER_LEVEL_STATUS:
		val = (get_threshold_status(SENSOR_NUM_BPB_RACK_LEVEL_2)) ? 1 : 0;
		break;
	case AALC_SENSOR_ALARM:
		WRITE_BIT(val, 0,
			  (get_threshold_status(SENSOR_NUM_BPB_RPU_COOLANT_INLET_TEMP_C)) ? 1 : 0);
		WRITE_BIT(val, 1,
			  (get_threshold_status(SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C)) ? 1 : 0);
		WRITE_BIT(val, 2,
			  (get_threshold_status(SENSOR_NUM_BPB_RPU_COOLANT_INLET_P_KPA)) ? 1 : 0);
		WRITE_BIT(val, 3,
			  (get_threshold_status(SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_P_KPA)) ? 1 : 0);
		WRITE_BIT(val, 4,
			  (get_threshold_status(SENSOR_NUM_BPB_RPU_COOLANT_FLOW_RATE_LPM)) ? 1 : 0);
		WRITE_BIT(val, 5,
			  (get_threshold_status(SENSOR_NUM_FB_1_HEX_OUTLET_TEMP_C)) ? 1 : 0);
		WRITE_BIT(val, 6,
			  (get_threshold_status(SENSOR_NUM_FB_2_HEX_OUTLET_TEMP_C)) ? 1 : 0);
		WRITE_BIT(val, 7,
			  (get_threshold_status(SENSOR_NUM_SB_HEX_AIR_INLET_1_TEMP_C)) ? 1 : 0);
		WRITE_BIT(val, 8,
			  (get_threshold_status(SENSOR_NUM_SB_HEX_AIR_INLET_2_TEMP_C)) ? 1 : 0);
		WRITE_BIT(val, 9,
			  (get_threshold_status(SENSOR_NUM_BPB_HEX_WATER_INLET_TEMP_C)) ? 1 : 0);
		WRITE_BIT(val, 15, (get_threshold_status(SENSOR_NUM_BPB_RACK_LEVEL_2)) ? 1 : 0);
		break;
	case HEX_AIR_THERMOMETER_STATUS:
		WRITE_BIT(val, 0,
			  (get_threshold_status(SENSOR_NUM_FB_3_HEX_OUTLET_TEMP_C)) ? 1 : 0);
		WRITE_BIT(val, 1,
			  (get_threshold_status(SENSOR_NUM_FB_4_HEX_OUTLET_TEMP_C)) ? 1 : 0);
		WRITE_BIT(val, 2,
			  (get_threshold_status(SENSOR_NUM_FB_5_HEX_OUTLET_TEMP_C)) ? 1 : 0);
		WRITE_BIT(val, 3,
			  (get_threshold_status(SENSOR_NUM_FB_6_HEX_OUTLET_TEMP_C)) ? 1 : 0);
		WRITE_BIT(val, 4,
			  (get_threshold_status(SENSOR_NUM_FB_7_HEX_OUTLET_TEMP_C)) ? 1 : 0);
		WRITE_BIT(val, 5,
			  (get_threshold_status(SENSOR_NUM_FB_8_HEX_OUTLET_TEMP_C)) ? 1 : 0);
		WRITE_BIT(val, 6,
			  (get_threshold_status(SENSOR_NUM_FB_9_HEX_OUTLET_TEMP_C)) ? 1 : 0);
		WRITE_BIT(val, 7,
			  (get_threshold_status(SENSOR_NUM_FB_10_HEX_OUTLET_TEMP_C)) ? 1 : 0);
		WRITE_BIT(val, 8,
			  (get_threshold_status(SENSOR_NUM_FB_11_HEX_OUTLET_TEMP_C)) ? 1 : 0);
		WRITE_BIT(val, 9,
			  (get_threshold_status(SENSOR_NUM_FB_12_HEX_OUTLET_TEMP_C)) ? 1 : 0);
		WRITE_BIT(val, 10,
			  (get_threshold_status(SENSOR_NUM_FB_13_HEX_OUTLET_TEMP_C)) ? 1 : 0);
		WRITE_BIT(val, 11,
			  (get_threshold_status(SENSOR_NUM_FB_14_HEX_OUTLET_TEMP_C)) ? 1 : 0);
		WRITE_BIT(val, 12,
			  (get_threshold_status(SENSOR_NUM_SB_HEX_AIR_INLET_3_TEMP_C)) ? 1 : 0);
		WRITE_BIT(val, 13,
			  (get_threshold_status(SENSOR_NUM_SB_HEX_AIR_INLET_4_TEMP_C)) ? 1 : 0);
		break;
	case AALC_STATUS_ALARM:
		WRITE_BIT(val, 0, (get_threshold_status(SENSOR_NUM_BPB_RACK_LEVEL_2)) ? 1 : 0);
		WRITE_BIT(val, 1,
			  (get_threshold_status(SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C)) ? 1 : 0);
		WRITE_BIT(val, 2,
			  (get_threshold_status(SENSOR_NUM_BPB_RPU_COOLANT_INLET_TEMP_C)) ? 1 : 0);
		WRITE_BIT(val, 3,
			  (get_threshold_status(SENSOR_NUM_MB_RPU_AIR_INLET_TEMP_C)) ? 1 : 0);
		WRITE_BIT(val, 4,
			  (get_threshold_status(SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_P_KPA)) ? 1 : 0);
		WRITE_BIT(val, 5,
			  ((get_threshold_status(SENSOR_NUM_SB_HEX_AIR_INLET_1_TEMP_C)) ||
			   (get_threshold_status(SENSOR_NUM_SB_HEX_AIR_INLET_2_TEMP_C)) ||
			   (get_threshold_status(SENSOR_NUM_SB_HEX_AIR_INLET_3_TEMP_C)) ||
			   (get_threshold_status(SENSOR_NUM_SB_HEX_AIR_INLET_4_TEMP_C))) ?
				  1 :
				  0);
		WRITE_BIT(val, 6,
			  ((get_threshold_status(SENSOR_NUM_PB_1_PUMP_TACH_RPM)) ||
			   (get_threshold_status(SENSOR_NUM_PB_2_PUMP_TACH_RPM)) ||
			   (get_threshold_status(SENSOR_NUM_PB_3_PUMP_TACH_RPM))) ?
				  1 :
				  0);
		WRITE_BIT(val, 9, (get_rpu_ready_pin_status()) ? 1 : 0);
		break;
	case HEX_FAN_ALARM_1:
		WRITE_BIT(val, 0, (get_threshold_status(SENSOR_NUM_FB_1_FAN_TACH_RPM)) ? 1 : 0);
		WRITE_BIT(val, 1, (get_threshold_status(SENSOR_NUM_FB_2_FAN_TACH_RPM)) ? 1 : 0);
		WRITE_BIT(val, 2, (get_threshold_status(SENSOR_NUM_FB_3_FAN_TACH_RPM)) ? 1 : 0);
		WRITE_BIT(val, 3, (get_threshold_status(SENSOR_NUM_FB_4_FAN_TACH_RPM)) ? 1 : 0);
		WRITE_BIT(val, 4, (get_threshold_status(SENSOR_NUM_FB_5_FAN_TACH_RPM)) ? 1 : 0);
		WRITE_BIT(val, 5, (get_threshold_status(SENSOR_NUM_FB_6_FAN_TACH_RPM)) ? 1 : 0);
		WRITE_BIT(val, 6, (get_threshold_status(SENSOR_NUM_FB_7_FAN_TACH_RPM)) ? 1 : 0);
		WRITE_BIT(val, 7, (get_threshold_status(SENSOR_NUM_FB_8_FAN_TACH_RPM)) ? 1 : 0);
		WRITE_BIT(val, 8, (get_threshold_status(SENSOR_NUM_FB_9_FAN_TACH_RPM)) ? 1 : 0);
		WRITE_BIT(val, 9, (get_threshold_status(SENSOR_NUM_FB_10_FAN_TACH_RPM)) ? 1 : 0);
		break;
	case HEX_FAN_ALARM_2:
		WRITE_BIT(val, 0, (get_threshold_status(SENSOR_NUM_FB_11_FAN_TACH_RPM)) ? 1 : 0);
		WRITE_BIT(val, 1, (get_threshold_status(SENSOR_NUM_FB_12_FAN_TACH_RPM)) ? 1 : 0);
		WRITE_BIT(val, 2, (get_threshold_status(SENSOR_NUM_FB_13_FAN_TACH_RPM)) ? 1 : 0);
		WRITE_BIT(val, 3, (get_threshold_status(SENSOR_NUM_FB_14_FAN_TACH_RPM)) ? 1 : 0);
		break;
	};

	return val;
}