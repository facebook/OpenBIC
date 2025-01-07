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
#include <string.h>
#include <logging/log.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "emc1413.h"

LOG_MODULE_REGISTER(dev_emc1413);

#define EMC1413_DEFAULT_RESOLUTION 0.125
#define EMC1413_TEMP_SHIFT_BIT 5

uint8_t emc1413_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5, temperature_high_byte = 0x00, temperature_low_byte = 0x00;
	uint16_t val = 0;
	I2C_MSG msg = { 0 };

	/* Read temperature msb register */
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	uint8_t offset = cfg->offset;

	if (offset == EMC1413_LOCAL_TEMPERATRUE) {
		msg.data[0] = INTERNAL_DIODE_HIGH_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_high_byte = msg.data[0];
		msg.data[0] = INTERNAL_DIODE_LOW_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_low_byte = msg.data[0];
	} else if (offset == EMC1413_REMOTE_TEMPERATRUE_1) {
		msg.data[0] = EXTERNAL_DIODE_1_HIGH_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_high_byte = msg.data[0];
		msg.data[0] = EXTERNAL_DIODE_1_LOW_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_low_byte = msg.data[0];
	} else if (offset == EMC1413_REMOTE_TEMPERATRUE_2) {
		msg.data[0] = EXTERNAL_DIODE_2_HIGH_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_high_byte = msg.data[0];
		msg.data[0] = EXTERNAL_DIODE_2_LOW_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_low_byte = msg.data[0];
	} else {
		LOG_ERR("Unknown offset(%d)", offset);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	val = (temperature_high_byte << 8) | temperature_low_byte;

	/* Temperature data is high 11-bit value */
	val = val >> EMC1413_TEMP_SHIFT_BIT;
	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (uint16_t)(val * EMC1413_DEFAULT_RESOLUTION);
	sval->fraction = ((val * EMC1413_DEFAULT_RESOLUTION) - sval->integer) * 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t emc1413_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = emc1413_read;
	return SENSOR_INIT_SUCCESS;
}
