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
#include <logging/log.h>
#include "sensor.h"
#include "hal_i2c.h"

#define TS0_LID 0b0010
#define TS1_LID 0b0110
#define TS_SENSE_OFFSET 0x31

LOG_MODULE_REGISTER(dev_ddr5_temp);

uint8_t ddr5_temp_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	ddr5_init_temp_arg *init_arg = (ddr5_init_temp_arg *)cfg->init_args;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = cfg->port;
	msg.target_addr = ((TS0_LID << 3) | (init_arg->HID_code & 0x07));
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = TS_SENSE_OFFSET;

	if (i2c_master_read(&msg, retry)) {
		LOG_WRN("I2C read failed.");
		return SENSOR_FAIL_TO_ACCESS;
	}

	float tmp;
	// 0.25 degree C/LSB
	if (msg.data[1] & BIT(4)) { // negative
		tmp = -0.25 * (((~((msg.data[1] << 8) | msg.data[0]) >> 2) + 1) & 0x3FF);
	} else {
		tmp = 0.25 * ((((msg.data[1] << 8) | msg.data[0]) >> 2) & 0x3FF);
	}
	init_arg->ts0_temp = tmp;

	msg.bus = cfg->port;
	msg.target_addr = ((TS1_LID << 3) | (init_arg->HID_code & 0x07));
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = TS_SENSE_OFFSET;

	if (i2c_master_read(&msg, retry)) {
		LOG_WRN("I2C read failed.");
		return SENSOR_FAIL_TO_ACCESS;
	}

	if (msg.data[1] & BIT(4)) { // negative
		tmp = -0.25 * (((~((msg.data[1] << 8) | msg.data[0]) >> 2) + 1) & 0x3FF);
	} else {
		tmp = 0.25 * ((((msg.data[1] << 8) | msg.data[0]) >> 2) & 0x3FF);
	}
	init_arg->ts1_temp = tmp;

	float val =
		(init_arg->ts0_temp > init_arg->ts1_temp) ? init_arg->ts0_temp : init_arg->ts1_temp;
	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int32_t)val;
	sval->fraction = (int32_t)(val * 1000) % 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t ddr5_temp_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = ddr5_temp_read;
	return SENSOR_INIT_SUCCESS;
}
