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

#ifndef TMP431_H
#define TMP431_H

#include "sensor.h"

bool tmp432_get_temp_status(sensor_cfg *cfg, uint8_t *temp_status);
bool tmp432_clear_temp_status(sensor_cfg *cfg);

enum TMP431_CHANNELS {
	TMP431_LOCAL_TEMPERATRUE,
	TMP431_REMOTE_TEMPERATRUE,
	TMP432_REMOTE_TEMPERATRUE_1,
	TMP432_REMOTE_TEMPERATRUE_2,
};

enum TMP431_REIGSTER_MAP {
	LOCAL_TEMPERATURE_HIGH_BYTE = 0x00,
	REMOTE_TEMPERATURE_HIGH_BYTE = 0x01,
	CONFIGURATION_REGISTER_1 = 0x03,
	REMOTE_TEMPERATURE_LOW_BYTE = 0x10,
	LOCAL_TEMPERATURE_LOW_BYTE = 0x15,
	REMOTE_TEMPERATURE_2_HIGH_BYTE = 0x23,
	REMOTE_TEMPERATURE_2_LOW_BYTE = 0x24,

	/* TMP432 Limit Registers */
	TMP432_LOCAL_HIGH_LIMIT_HIGH_BYTE_READ_REG = 0x05,
	TMP432_LOCAL_HIGH_LIMIT_HIGH_BYTE_WRITE_REG = 0x0B,

	TMP432_LOCAL_LOW_LIMIT_HIGH_BYTE_READ_REG = 0x06,
	TMP432_LOCAL_LOW_LIMIT_HIGH_BYTE_WRITE_REG = 0x0C,

	TMP432_REMOTE_1_HIGH_LIMIT_HIGH_BYTE_READ_REG = 0x07,
	TMP432_REMOTE_1_HIGH_LIMIT_HIGH_BYTE_WRITE_REG = 0x0D,

	TMP432_REMOTE_1_LOW_LIMIT_HIGH_BYTE_READ_REG = 0x08,
	TMP432_REMOTE_1_LOW_LIMIT_HIGH_BYTE_WRITE_REG = 0x0E,

	TMP432_REMOTE_2_HIGH_LIMIT_HIGH_BYTE_READ_REG = 0x15,
	TMP432_REMOTE_2_HIGH_LIMIT_HIGH_BYTE_WRITE_REG = 0x15,

	TMP432_REMOTE_2_LOW_LIMIT_HIGH_BYTE_READ_REG = 0x16,
	TMP432_REMOTE_2_LOW_LIMIT_HIGH_BYTE_WRITE_REG = 0x16,

	TMP432_LOCAL_THERM_LIMIT_REG = 0x20,
	TMP432_REMOTE_1_THERM_LIMIT_REG = 0x19,
	TMP432_REMOTE_2_THERM_LIMIT_REG = 0x1A,
};

bool tmp432_get_temp_threshold(sensor_cfg *cfg, uint8_t temp_threshold_index,
			       uint32_t *millidegree_celsius);
bool tmp432_set_temp_threshold(sensor_cfg *cfg, uint8_t temp_threshold_index,
			       uint32_t *millidegree_celsius);
bool tmp432_set_thermal_mode(sensor_cfg *cfg);
bool tmp432_set_therm_hysteresis(sensor_cfg *cfg, uint8_t temp_therm_hysteresis_val);

#endif
