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

#include "tmp431.h"

#include <stdio.h>
#include <stdlib.h>

#include "sensor.h"
#include "hal_i2c.h"
#include "libutil.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(dev_tmp431);

#define I2C_DATA_SIZE 5
#define RANGE_0_127 0
#define RANGE_m55_150 1

static uint8_t temperature_range = 0xFF;

uint8_t tmp431_read(uint8_t sensor_num, int *reading)
{
	if (!reading || (sensor_num > SENSOR_NUM_MAX)) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5, temperature_high_byte = 0xFF, temperature_low_byte = 0xFF;
	I2C_MSG msg = { 0 };

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	uint8_t offset = sensor_config[sensor_config_index_map[sensor_num]].offset;

	if (offset == TMP431_LOCAL_TEMPERATRUE) {
		msg.data[0] = LOCAL_TEMPERATURE_HIGH_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_high_byte = msg.data[0];
		msg.data[0] = LOCAL_TEMPERATURE_LOW_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_low_byte = msg.data[0];
	} else if (offset == TMP431_REMOTE_TEMPERATRUE) {
		msg.data[0] = REMOTE_TEMPERATURE_HIGH_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_high_byte = msg.data[0];
		msg.data[0] = REMOTE_TEMPERATURE_LOW_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_low_byte = msg.data[0];
	} else {
		LOG_ERR("Unknown offset(%d)", offset);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	float val;
	if (temperature_range == RANGE_0_127) {
		val = temperature_high_byte + ((temperature_low_byte >> 4) * 0.0625);
	} else if (temperature_range == RANGE_m55_150) {
		val = (temperature_high_byte - 64) + ((temperature_low_byte >> 4) * 0.0625);
	} else {
		LOG_ERR("Unknown temperature range(%d)", temperature_range);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int32_t)val;
	sval->fraction = (int32_t)(val * 1000) % 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t tmp431_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	I2C_MSG msg;

	char *data = (uint8_t *)malloc(I2C_DATA_SIZE * sizeof(uint8_t));
	if (data == NULL) {
		LOG_ERR("Memory allocation failed!");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	// Get the temperature range from chip
	uint8_t bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	uint8_t target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	uint8_t tx_len = 1;
	uint8_t rx_len = 1;

	data[0] = CONFIGURATION_REGISTER_1;
	msg = construct_i2c_message(bus, target_addr, tx_len, data, rx_len);
	if (i2c_master_read(&msg, retry) != 0) {
		LOG_ERR("Failed to read TMP431 register(0x%x)", data[0]);
		goto cleanup;
	}
	temperature_range = (msg.data[0] & BIT(2)) == BIT(2) ? RANGE_m55_150 : RANGE_0_127;

cleanup:
	SAFE_FREE(data);

	sensor_config[sensor_config_index_map[sensor_num]].read = tmp431_read;
	return SENSOR_INIT_SUCCESS;
}
