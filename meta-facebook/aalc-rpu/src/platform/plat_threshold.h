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

enum AALC_SENSOR_STATUS_E {
	// RPU FAN STATUS
	RPU_FAN1_STATUS,
	RPU_FAN2_STATUS,
	// RPU PUMP STATUS
	RPU_PUMP1_STATUS,
	RPU_PUMP2_STATUS,
	RPU_PUMP3_STATUS,
	// RPU LED STATUS
	RPU_RESERVOIR_STATUS,
	RPU_LED_RESERVOIR_STATUS,
	RPU_LED_LEAKAGE_STATUS,
	RPU_LED_FAULT_STATUS,
	RPU_LED_POWER_STATUS,
	// RPU STATUS
	RPU_PUMP_STATUS,
	RPU_INTERNAL_FAN_STATUS,
	// HEX STATUS
	HEX_BLADDER_LEVEL_STATUS,
	// PUMP FAN STATUS 0xa080
	PUMP_FAN_STATUS,
	// AALC Sensor Alarm 0xa202
	HEX_AIR_THERMOMETER_STATUS,
	// Leakage 0x9202, 0xa200
	SB_TTV_COOLANT_LEAKAGE,
	// AALC Sensor Alarm 0x9200
	AALC_SENSOR_ALARM,
	// Y_FILTER_SENSOR_STATUS 0x91ff
	Y_FILTER_SENSOR_STATUS,
	// AALC Status Alarm 0x9201
	AALC_STATUS_ALARM,
	// HEX Alarm 0x9203, 0xa201
	HEX_FAN_ALARM_1,
	HEX_FAN_ALARM_2,
	// Leakage Black Box 0x19a1~0x19ac, 0xa300~0xa302
	STICKY_ITRACK_CHASSIS0_LEAKAGE,
	STICKY_ITRACK_CHASSIS1_LEAKAGE,
	STICKY_ITRACK_CHASSIS2_LEAKAGE,
	STICKY_ITRACK_CHASSIS3_LEAKAGE,
	STICKY_RPU_INTERNAL_LEAKAGE_ABNORMAL,
	STICKY_RPU_EXTERNAL_LEAKAGE_ABNORMAL,
	STICKY_RPU_OPT_EXTERNAL_LEAKAGE1_ABNORMAL,
	STICKY_RPU_OPT_EXTERNAL_LEAKAGE2_ABNORMAL,
	STICKY_HEX_RACK_PAN_LEAKAGE,
	STICKY_HEX_RACK_FLOOR_LEAKAGE,
	STICKY_HEX_RACK_PAN_LEAKAGE_RELAY,
	STICKY_HEX_RACK_FLOOR_LEAKAGE_RELAY,
	MAX_NUM_OF_AALC_STATUS
};

void set_threshold_poll_enable_flag(bool flag);
bool get_threshold_poll_enable_flag();
void threshold_poll_init();
void fan_pump_pwrgd();
uint16_t read_sensor_status(uint8_t sensor_status_num);