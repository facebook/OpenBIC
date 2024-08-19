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

#ifndef PLAT_STATUS_H
#define PLAT_STATUS_H

#include <stdint.h>
#include "plat_util.h"
#include "plat_threshold.h"

enum AALC_MODBUS_SENSOR_STATUS_E {
	RPU_FAN_STATUS, // 0x900E
	RPU_PUMP1_STATUS, // 0x9012
	RPU_PUMP2_STATUS, // 0x9013
	RPU_PUMP3_STATUS, // 0x9014
	RPU_RESERVOIR_STATUS, // 0x9016
	ALL_PUMP_STATUS, // 0x9018
	ALL_RPU_INTERNAL_FAN_STATUS, // 0x9019
	PUMP_FAN_STATUS, // 0xA080
	HEX_BLADDER_LEVEL_STATUS, // 0x9113
	AALC_SENSOR_ALARM, // 0x9200
	HEX_AIR_THERMOMETER_STATUS, // 0xA202
	AALC_STATUS_ALARM, // 0x9201
	HEX_FAN_ALARM_1, // 0x9203
	HEX_FAN_ALARM_2, // 0xA201
};

enum AALC_STICKY_STATUS_E {
	STICKY_ITRACK_CHASSIS0_LEAKAGE, // 0x19A1
	STICKY_ITRACK_CHASSIS1_LEAKAGE, // 0x19A2
	STICKY_ITRACK_CHASSIS2_LEAKAGE, // 0x19A3
	STICKY_ITRACK_CHASSIS3_LEAKAGE, // 0x19A4
	STICKY_RPU_INTERNAL_LEAKAGE_ABNORMAL, // 0x19A5
	STICKY_RPU_EXTERNAL_LEAKAGE_ABNORMAL, // 0x19A6
	STICKY_RPU_OPT_EXTERNAL_LEAKAGE1_ABNORMAL, // 0x19A7
	STICKY_RPU_OPT_EXTERNAL_LEAKAGE2_ABNORMAL, // 0x19A8
	STICKY_HEX_RACK_PAN_LEAKAGE, // 0x19A9
	STICKY_HEX_RACK_FLOOR_LEAKAGE, // 0x19AA
	STICKY_HEX_RACK_PAN_LEAKAGE_RELAY, // 0x19AB
	STICKY_HEX_RACK_FLOOR_LEAKAGE_RELAY, // 0x19AC
	STICKY_STATUS_MAX,
};

enum PUMP_STATUS_E {
	PUMP_STATUS_DISABLE = 0,
	PUMP_STATUS_REDAUNDANT = 1,
	PUMP_STATUS_ENABLE = 2,
	PUMP_STATUS_MAINTAIN = 3,
	PUMP_STATUS_ABNORMAL = 4,
};

enum AALC_STATUS_LEAK_E {
	AALC_STATUS_IT_LEAK_0 = 0,
	AALC_STATUS_IT_LEAK_1 = 1,
	AALC_STATUS_IT_LEAK_2 = 2,
	AALC_STATUS_IT_LEAK_3 = 3,
	AALC_STATUS_CDU_LEAKAGE = 4,
	AALC_STATUS_RACK_LEAKAGE = 5,
	AALC_STATUS_LEAK_E_MAX,
};

uint8_t get_leak_status();
void set_leak_status(uint8_t idx, uint8_t val);
uint16_t get_sticky_sensor_status(uint8_t idx);
bool set_sticky_sensor_status(uint8_t idx, uint16_t val);
uint16_t get_sensor_status_for_modbus_cmd(uint8_t status);

#endif //PLAT_STATUS_H
