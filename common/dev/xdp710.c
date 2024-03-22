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

#include "pmbus.h"
#include "sensor.h"
#include <logging/log.h>
#include "xdp710.h"

LOG_MODULE_REGISTER(dev_xdp710);

uint8_t xdp710_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("Sensor number out of range.");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	xdp710_init_arg *init_arg = (xdp710_init_arg *)cfg->init_args;
	if (init_arg->is_init == false) {
		LOG_ERR("device isn't initialized");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	if (!init_arg->r_sense) {
		LOG_ERR("Rsense not provided");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	float Rsense = init_arg->r_sense;
	float val;
	uint8_t retry = 5;
	I2C_MSG msg;
	uint8_t offset = cfg->offset;

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.data[0] = offset;
	msg.rx_len = 2;

	if (i2c_master_read(&msg, retry))
		return SENSOR_FAIL_TO_ACCESS;

	switch (offset) {
	case PMBUS_READ_TEMPERATURE_1:
		val = (float)(((msg.data[1] << 8) | msg.data[0]) * 10 - 14321) / 52;
		break;
	case PMBUS_READ_VIN:
		val = (float)((msg.data[1] << 8) | msg.data[0]) * 100 / 4653;
		break;
	case PMBUS_READ_IOUT:
		val = (float)(((msg.data[1] << 8) | msg.data[0]) * 100) / (23165 * Rsense);
		break;
	case PMBUS_READ_PIN:
		val = (float)(((msg.data[1] << 8) | msg.data[0]) * 100) / (4211 * Rsense);
		break;

	default:
		LOG_WRN("Invalid offset 0x%x", offset);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;
	return SENSOR_READ_SUCCESS;
}

/*
	TO DO: VTLM_RNG and VSNS_CS setting
*/
uint8_t xdp710_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("Sensor number out of range.");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	xdp710_init_arg *init_args = (xdp710_init_arg *)cfg->init_args;
	if (init_args->is_init) {
		LOG_DBG("xdp710 already initialized.");
		goto skip_init;
	}

	init_args->is_init = true;

skip_init:
	cfg->read = xdp710_read;
	return SENSOR_INIT_SUCCESS;
}
