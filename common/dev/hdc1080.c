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

#include "hdc1080.h"

LOG_MODULE_REGISTER(dev_hdc1080);

#define RAW2TEMP(raw) (raw * 165 / 65536 - 40)
#define RAW2HUM(raw) (raw * 100 / 65536)

#define HDC1080_DELAY 10

static bool hdc1080_read_val(sensor_cfg *cfg, float *val)
{
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.data[0] = cfg->offset;

	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("hdc1080 write offset 0x%02x fail", cfg->offset);
		return false;
	}

	k_msleep(HDC1080_DELAY);

	msg.tx_len = 0;
	msg.rx_len = 2;
	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("hdc1080 read fail");
		return false;
	}

	switch (cfg->offset) {
	case HDC1080_TEMP_OFFSET:
		*val = RAW2TEMP((msg.data[0] << 8 | msg.data[1]));
		break;
	case HDC1080_HUM_OFFSET:
		*val = RAW2HUM((msg.data[0] << 8 | msg.data[1]));
		break;
	default:
		LOG_ERR("0x%02x offset %d read fail", cfg->num, cfg->offset);
		break;
	}

	return true;
}

uint8_t hdc1080_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor 0x%x input parameter is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	float val = 0.0;
	if (!hdc1080_read_val(cfg, &val)) {
		LOG_ERR("sensor 0x%02x read 0x%02x fail!", cfg->num, cfg->offset);
		return SENSOR_FAIL_TO_ACCESS;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t hdc1080_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	hdc1080_init_arg *init_arg = (hdc1080_init_arg *)cfg->init_args;
	if (init_arg->is_init)
		goto skip_init;

	// write cfg reg
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 3;

	msg.data[0] = HDC1080_CONFIGURE_OFFSET;
	msg.data[1] = 0; // TEMP_OR_HUM mode
	msg.data[2] = 0; //Reserved

	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("Write config reg failed");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	init_arg->is_init = true;

skip_init:
	cfg->read = hdc1080_read;
	return SENSOR_INIT_SUCCESS;
}