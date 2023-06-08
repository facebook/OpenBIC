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
#include <stdlib.h>
#include <logging/log.h>
#include "libutil.h"
#include "sensor.h"
#include "tmp461.h"

LOG_MODULE_REGISTER(dev_tmp461);

static uint8_t temperature_range = 0xFF;

uint8_t tmp461_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5, temperature_high_byte = 0xFF, temperature_low_byte = 0xFF;
	I2C_MSG msg = { 0 };

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	uint8_t offset = cfg->offset;

	switch (offset) {
	case TMP461_LOCAL_TEMPERATRUE:
		msg.data[0] = OFFSET_LOCAL_TEMPERATURE_HIGH_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_high_byte = msg.data[0];

		msg.data[0] = OFFSET_LOCAL_TEMPERATURE_LOW_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_low_byte = msg.data[0];
		break;
	case TMP461_REMOTE_TEMPERATRUE:
		msg.data[0] = OFFSET_REMOTE_TEMPERATURE_HIGH_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_high_byte = msg.data[0];

		msg.data[0] = OFFSET_REMOTE_TEMPERATURE_LOW_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_low_byte = msg.data[0];
		break;
	default:
		LOG_ERR("Unknown register offset(%d)", offset);
		break;
	}

	float val = 0;
	switch (temperature_range) {
	case TEMP_RANGE_M40_127:
		// Negative numbers are represented in twos complement format
		if (GETBIT(temperature_high_byte, 7)) {
			temperature_high_byte = ~temperature_high_byte + 1;
			val = -temperature_high_byte - ((temperature_low_byte >> 4) * 0.0625);
		} else {
			val = temperature_high_byte + ((temperature_low_byte >> 4) * 0.0625);
		}
		break;
	case TEMP_RANGE_M64_191:
		// All values are unsigned with a –64°C offset
		val = (temperature_high_byte - 64) + ((temperature_low_byte >> 4) * 0.0625);
		break;
	default:
		LOG_ERR("Unknown temperature range(%d)", temperature_range);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int32_t)val;
	sval->fraction = (int32_t)(val * 1000) % 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t tmp461_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	I2C_MSG *i2c_msg = (I2C_MSG *)malloc(sizeof(I2C_MSG));

	if (i2c_msg == NULL) {
		LOG_ERR("Failed to allocate memory");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	// Get the temperature range from chip
	i2c_msg->bus = cfg->port;
	i2c_msg->target_addr = cfg->target_addr;
	i2c_msg->tx_len = 1;
	i2c_msg->rx_len = 1;
	i2c_msg->data[0] = OFFSET_CONFIGURATION;
	if (i2c_master_read(i2c_msg, retry) != 0) {
		LOG_ERR("Failed to read TMP461 register(0x%x)", OFFSET_CONFIGURATION);
		goto exit;
	}

	temperature_range = GETBIT(i2c_msg->data[0], 2) ? TEMP_RANGE_M40_127 : TEMP_RANGE_M64_191;

exit:
	SAFE_FREE(i2c_msg);

	cfg->read = tmp461_read;
	return SENSOR_INIT_SUCCESS;
}
