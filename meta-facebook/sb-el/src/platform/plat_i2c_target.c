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

/*
  NAME: I2C TARGET INIT
  FILE: plat_i2c_target.c
  DESCRIPTION: Provide i2c target EN/CFG table "I2C_TARGET_EN_TABLE[]/I2C_TARGET_CFG_TABLE[]" for init target config.
  AUTHOR: MouchenHung
  DATE/VERSION: 2021.11.26 - v1.1
  Note: 
    (1) "plat_i2c_target.h" is included by "hal_i2c_target.h"
*/

#include <zephyr.h>
#include <stdio.h>
#include <stdlib.h>
#include "pldm_sensor.h"
#include "plat_i2c_target.h"

#define DATA_TABLE_LENGTH_4 4

plat_sensor_reading *sensor_reading_table[DATA_TABLE_LENGTH_4] = { NULL };

/* I2C target init-enable table */
const bool I2C_TARGET_ENABLE_TABLE[MAX_TARGET_NUM] = {
	TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE,
	TARGET_DISABLE, TARGET_ENABLE,	TARGET_ENABLE, TARGET_DISABLE,
	TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE,
};

// static bool command_reply_data_handle(void *arg)
// {
// 	/*TODO: put board telemetry here*/

// 	return false;
// }

/* I2C target init-config table */
const struct _i2c_target_config I2C_TARGET_CONFIG_TABLE[MAX_TARGET_NUM] = {
	{ 0xFF, 0xA }, { 0xFF, 0xA }, { 0xFF, 0xA },
	{ 0xFF, 0xA }, { 0xFF, 0xA }, { 0x42, 0xA },
	{ 0x40, 0xA }, { 0xFF, 0xA }, { 0xFF, 0xA },
	{ 0xFF, 0xA }, { 0xFF, 0xA }, { 0xFF, 0xA },
};
#define SENSOR_READING_PDR_INDEX_MAX 50


void update_sensor_reading_by_sensor_number(uint8_t sensor_number)
{
	// sensor number is 1-base but index is 0-base
	uint8_t sensor_index_offset = sensor_number - 1;
	uint8_t table_index = sensor_index_offset / SENSOR_READING_PDR_INDEX_MAX;
	uint8_t sensor_index = sensor_index_offset % SENSOR_READING_PDR_INDEX_MAX;
	plat_sensor_reading *sensor_data = sensor_reading_table[table_index];

	uint8_t status = SENSOR_UNAVAILABLE;
	int reading = 0;
	uint8_t sensor_operational_state = PLDM_SENSOR_STATUSUNKOWN;

	status = pldm_sensor_get_reading_from_cache(sensor_number, &reading,
						    &sensor_operational_state);

	sensor_data->sensor_entries[sensor_index].sensor_value =
		(status == SENSOR_READ_SUCCESS) ? reading : 0xFFFFFFFF;
}

