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
#include "libutil.h"
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"
#include "util_pmbus.h"

#define RAA228238_READ_TEMPERATURE_1_RESOLUTION 1
#define RAA228238_READ_VOUT_RESOLUTION 0.001
#define RAA228238_READ_IOUT_RESOLUTION 0.1
#define RAA228238_READ_POUT_RESOLUTION 1

LOG_MODULE_REGISTER(raa228238);

uint8_t raa228238_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	sensor_val *sval = (sensor_val *)reading;
	I2C_MSG msg;
	memset(sval, 0, sizeof(sensor_val));

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = cfg->offset;

	if (i2c_master_read(&msg, retry)) {
		LOG_WRN("I2C read failed");
		return SENSOR_FAIL_TO_ACCESS;
	}

	float val;
	if (cfg->offset == PMBUS_READ_VOUT) {
		/* Unsigned integer */
		uint16_t read_value = (msg.data[1] << 8) | msg.data[0];
		val = read_value * RAA228238_READ_VOUT_RESOLUTION;

	} else if (cfg->offset == PMBUS_READ_TEMPERATURE_1 || cfg->offset == PMBUS_READ_POUT) {
		/* 2's complement */
		int16_t read_value = (msg.data[1] << 8) | msg.data[0];
		val = read_value;

	} else if (cfg->offset == PMBUS_READ_IOUT) {
		/* 2's complement */
		int16_t read_value = (msg.data[1] << 8) | msg.data[0];
		val = read_value * RAA228238_READ_IOUT_RESOLUTION;

	} else {
		return SENSOR_FAIL_TO_ACCESS;
	}

	sval->integer = (int)val;
	sval->fraction = (val - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t raa228238_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = raa228238_read;
	return SENSOR_INIT_SUCCESS;
}
