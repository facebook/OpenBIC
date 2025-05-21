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

#define BMR316_READ_VOUT_EXP_VALUE (1.0 / (1 << 11)) //2^(-11)

LOG_MODULE_REGISTER(bmr316);

uint8_t bmr316_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	float val;

	uint8_t retry = 5;
	sensor_val *sval = (sensor_val *)reading;
	I2C_MSG msg;
	memset(sval, 0, sizeof(sensor_val));
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = cfg->offset;

	if (i2c_master_read(&msg, retry))
		return SENSOR_FAIL_TO_ACCESS;

	if (cfg->offset == PMBUS_READ_VOUT) {
		uint16_t read_value = (msg.data[1] << 8) | msg.data[0];
		val = read_value * BMR316_READ_VOUT_EXP_VALUE;

	} else if (cfg->offset == PMBUS_READ_TEMPERATURE_1 || cfg->offset == PMBUS_READ_VIN ||
		   cfg->offset == PMBUS_READ_IOUT) {
		uint16_t read_value = (msg.data[1] << 8) | msg.data[0];
		val = slinear11_to_float(read_value);
	} else if (cfg->offset == PMBUS_READ_POUT) {
		msg.data[0] = PMBUS_READ_VOUT;
		if (i2c_master_read(&msg, retry))
			return SENSOR_FAIL_TO_ACCESS;
		uint16_t read_value = (msg.data[1] << 8) | msg.data[0];
		float vout_val = read_value * BMR316_READ_VOUT_EXP_VALUE;
		msg.data[0] = PMBUS_READ_IOUT;
		if (i2c_master_read(&msg, retry))
			return SENSOR_FAIL_TO_ACCESS;
		read_value = (msg.data[1] << 8) | msg.data[0];
		float iout_val = slinear11_to_float(read_value);
		val = vout_val * iout_val;
	} else {
		return SENSOR_FAIL_TO_ACCESS;
	}

	sval->integer = val;
	sval->fraction = (val - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t bmr316_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = bmr316_read;
	return SENSOR_INIT_SUCCESS;
}
