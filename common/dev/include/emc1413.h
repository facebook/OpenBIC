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

#ifndef EMC1413_H
#define EMC1413_H

#include "sensor.h"

bool emc1413_get_temp_status(sensor_cfg *cfg, uint8_t *temp_status);
bool emc1413_clear_temp_status(sensor_cfg *cfg);

enum EMC1413_CHANNELS {
	EMC1413_LOCAL_TEMPERATRUE,
	EMC1413_REMOTE_TEMPERATRUE_1,
	EMC1413_REMOTE_TEMPERATRUE_2,
};

enum EMC1413_REIGSTER_MAP {
	INTERNAL_DIODE_HIGH_BYTE = 0x00,
	EXTERNAL_DIODE_1_HIGH_BYTE = 0x01,
	EXTERNAL_DIODE_2_HIGH_BYTE = 0x23,
	INTERNAL_DIODE_LOW_BYTE = 0x29,
	EXTERNAL_DIODE_1_LOW_BYTE = 0x10,
	EXTERNAL_DIODE_2_LOW_BYTE = 0x24,

	/* EMC1413 Limit Registers */
	EMC1413_INTERNAL_HIGH_LIMIT_REG = 0x05,
	EMC1413_INTERNAL_LOW_LIMIT_REG = 0x06,
	EMC1413_EXTERNAL_1_HIGH_LIMIT_HIGH_BYTE_REG = 0x07,
	EMC1413_EXTERNAL_1_HIGH_LIMIT_LOW_BYTE_REG = 0x13,
	EMC1413_EXTERNAL_1_LOW_LIMIT_HIGH_BYTE_REG = 0x08,
	EMC1413_EXTERNAL_1_LOW_LIMIT_LOW_BYTE_REG = 0x14,
	EMC1413_EXTERNAL_2_HIGH_LIMIT_HIGH_BYTE_REG = 0x15,
	EMC1413_EXTERNAL_2_HIGH_LIMIT_LOW_BYTE_REG = 0x17,
	EMC1413_EXTERNAL_2_LOW_LIMIT_HIGH_BYTE_REG = 0x16,
	EMC1413_EXTERNAL_2_LOW_LIMIT_LOW_BYTE_REG = 0x18,
	EMC1413_INTERNAL_THERM_LIMIT_REG = 0x20,
	EMC1413_EXTERNAL_1_THERM_LIMIT_REG = 0x19,
	EMC1413_EXTERNAL_2_THERM_LIMIT_REG = 0x1A,
	EMC1413_REG_MAX,
};

bool emc1413_get_temp_threshold(sensor_cfg *cfg, uint8_t temp_threshold_index,
				uint32_t *millidegree_celsius);
bool emc1413_set_temp_threshold(sensor_cfg *cfg, uint8_t temp_threshold_index,
				uint32_t *millidegree_celsius);
bool emc1413_set_comparator_mode(sensor_cfg *cfg);
bool emc1413_set_therm_hysteresis(sensor_cfg *cfg, uint8_t temp_therm_hysteresis_val);

#endif
