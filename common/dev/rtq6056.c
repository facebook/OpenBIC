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
#include <stdint.h>
#include <string.h>
#include <logging/log.h>
#include "sensor.h"
#include "pmbus.h"
#include "hal_i2c.h"
#include "rtq6056.h"

#define RTQ6056_BUS_VOLTAGE_LSB 0.00125 // 1.25 mV.

LOG_MODULE_REGISTER(dev_rtq6056);

uint8_t RTQ6056_DEVICE_ID[2] = { 0x12, 0x14 };

uint8_t rtq6056_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	rtq6056_init_arg *init_arg = (rtq6056_init_arg *)cfg->init_args;
	if (init_arg->is_init != true) {
		LOG_ERR("device isn't initialized");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	int ret = 0;
	int16_t val = 0;
	float parameter = 0;
	I2C_MSG msg;
	memset(&msg, 0, sizeof(I2C_MSG));
	*reading = 0;
	uint8_t offset = cfg->offset;
	sensor_val *sval = (sensor_val *)reading;

	switch (cfg->offset) {
	case PMBUS_READ_VOUT:
		offset = RTQ6056_BUS_VOL_OFFSET;
		parameter = RTQ6056_BUS_VOLTAGE_LSB;
		break;
	case PMBUS_READ_IOUT:
		offset = RTQ6056_CUR_OFFSET;
		parameter = init_arg->current_lsb;
		break;
	case PMBUS_READ_POUT:
		offset = RTQ6056_PWR_OFFSET;
		parameter = init_arg->current_lsb * 25;
		break;
	default:
		LOG_ERR("Offset not supported: 0x%x", offset);
		return SENSOR_FAIL_TO_ACCESS;
		break;
	}

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = offset;
	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("i2c read fail ret: %d", ret);
		return SENSOR_FAIL_TO_ACCESS;
	}

	val = (msg.data[0] << 8) | msg.data[1];
	if (offset == RTQ6056_CUR_OFFSET) {
		if (GETBIT(msg.data[0], 7)) {
			// If raw value is negative, set it zero.
			val = 0;
		}
	}

	sval->integer = val * parameter;
	sval->fraction = ((val * parameter) - sval->integer) * 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t rtq6056_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	int ret = 0;
	int retry = 5;
	I2C_MSG msg = { 0 };

	rtq6056_init_arg *init_arg = (rtq6056_init_arg *)cfg->init_args;

	if (init_arg->is_init != true) {
		memset(&msg, 0, sizeof(I2C_MSG));
		uint16_t calibration = 0;

		msg.bus = cfg->port;
		msg.target_addr = cfg->target_addr;
		msg.tx_len = 3;
		msg.data[0] = RTQ6056_CALIBRATION_OFFSET;

		// Calibration formula = (0.00512 / (current_lsb * r_shunt))
		calibration = (uint16_t)(5120 /
					 (1000 * init_arg->current_lsb * 1000 * init_arg->r_shunt));
		msg.data[1] = (calibration >> 8) & 0xFF;
		msg.data[2] = calibration & 0xFF;

		ret = i2c_master_write(&msg, retry);
		if (ret != 0) {
			LOG_ERR("i2c write CALIBRATION register fail ret: %d, sensor num: 0x%x",
				ret, cfg->num);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}
		init_arg->is_init = true;
	}

	cfg->read = rtq6056_read;
	return SENSOR_INIT_SUCCESS;
}
