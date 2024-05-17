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
#include "ads1015.h"

#define MSB_MASK BIT(15)
#define CONVERSION_VAL_MASK GENMASK(15, 4)
#define DEFAULT_CONFIG_VAL 0x8583
#define VALUE_MSB_MASK GENMASK(15, 8)
#define VALUE_LSB_MASK GENMASK(7, 0)

LOG_MODULE_REGISTER(dev_ads1015);

static uint16_t twoscomplement_to_decimal(uint16_t twoscomplement_val)
{
	if (twoscomplement_val & MSB_MASK) { // Check if MSB is 1 (negative number)
		twoscomplement_val =
			~twoscomplement_val + 1; // Two's complement operation for negative numbers
		return -twoscomplement_val; // Return negative number
	}

	return twoscomplement_val; // Return positive number as is
}

uint8_t ads1015_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = CONVERSION_REG;

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Failed to read ads1015 conversion register.");
		return SENSOR_FAIL_TO_ACCESS;
	}

	uint16_t read_val =
		twoscomplement_to_decimal(msg.data[0] << 4 | msg.data[1]) & CONVERSION_VAL_MASK;

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int)read_val & 0xFFFF;
	sval->fraction = (read_val - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t ads1015_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	ina238_init_arg *init_args = (ina238_init_arg *)cfg->init_args;

	if (init_args->is_init)
		goto skip_init;

	uint16_t config_val = DEFAULT_CONFIG_VAL;
	if (init_args->device_operation_mode == CONTINUOUS_MODE)
		WRITE_BIT(config_val, 8, 0);

	if (init_args->alert_latch == ENABLE_LATCH)
		WRITE_BIT(config_val, 2, 1);

	uint8_t retry = 5;
	I2C_MSG msg;
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	// set config reg
	msg.tx_len = 3;
	msg.data[0] = CONFIG_REG;
	msg.data[1] = config_val & VALUE_MSB_MASK;
	msg.data[2] = config_val & VALUE_LSB_MASK;
	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("Failed to write ads1015 config register.");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	init_args->is_init = true;

skip_init:
	cfg->read = ads1015_read;
	return SENSOR_INIT_SUCCESS;
}
