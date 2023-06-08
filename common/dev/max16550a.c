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
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"

#include <logging/log.h>

LOG_MODULE_REGISTER(max16550a);

uint8_t max16550a_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	*reading = 0;
	uint8_t retry = 5;
	uint8_t offset = cfg->offset;
	int ret = 0;
	float val = 0, r_load = 0;
	I2C_MSG msg;
	memset(&msg, 0, sizeof(I2C_MSG));

	max16550a_init_arg *init_arg = (max16550a_init_arg *)cfg->init_args;
	// R_load is the value of resistance connected to EFUSE , and EFUSE would adjust the reading accuracy according to r_load
	r_load = init_arg->r_load;

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = offset;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("i2c read fail  sensor number 0x%x  ret: %d", cfg->num, ret);
		return SENSOR_FAIL_TO_ACCESS;
	}

	switch (offset) {
	case PMBUS_READ_VIN:
		// m = +7578, b = +0, R = -2
		val = (float)((msg.data[1] << 8) | msg.data[0]) * 100 / 7578;
		break;
	case PMBUS_READ_VOUT:
		// m = +7578, b = +0, R = -2
		val = (float)((msg.data[1] << 8) | msg.data[0]) * 100 / 7578;
		break;
	case PMBUS_READ_IOUT:
		// m = +3.824 * Rload, b = -4300, R = -3
		val = (float)(((msg.data[1] << 8) | msg.data[0]) * 1000 + 4300) / (3.824 * r_load);
		break;
	case PMBUS_READ_TEMPERATURE_1:
		// m = +199, b = +7046, R = -2
		val = (float)(((msg.data[1] << 8) | msg.data[0]) * 100 - 7046) / 199;
		break;
	case PMBUS_READ_PIN:
		// m = +0.895 * Rload, b = -9100, R = -2
		val = (float)(((msg.data[1] << 8) | msg.data[0]) * 100 + 9100) / (0.895 * r_load);
		break;
	default:
		LOG_ERR("Not support sensor number 0x%x  offset: 0x%x", cfg->num, offset);
		return SENSOR_NOT_FOUND;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t max16550a_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = max16550a_read;
	return SENSOR_INIT_SUCCESS;
}
