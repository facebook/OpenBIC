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
#include <stdlib.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "libutil.h"
#include "pmbus.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(tps25990);

/* RIMON = 2 kohm*/
#define TPS25990_RIMON_OHM 2000

uint8_t tps25990_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	double val;
	I2C_MSG msg = { 0 };

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = cfg->offset;

	if (i2c_master_read(&msg, retry))
		return SENSOR_FAIL_TO_ACCESS;

	switch (cfg->offset) {
	case PMBUS_READ_VIN:
		/* spec: m = 5251, b = 0, R = -2 */
		val = (float)((msg.data[1] << 8) | msg.data[0]) * 100 / 5251;
		break;
	case PMBUS_READ_IIN:
		/* spec: m = 9.538 x RIMON, b = 0, R = -3 */
		val = (float)((msg.data[1] << 8) | msg.data[0]) * 1000 /
		      (9.538 * TPS25990_RIMON_OHM);
		break;
	case PMBUS_READ_PIN:
		/* spec: m = 4.901 x RIMON, b = 0, R = -4 */
		val = (float)((msg.data[1] << 8) | msg.data[0]) * 10000 /
		      (4.901 * TPS25990_RIMON_OHM);
		break;
	case PMBUS_READ_TEMPERATURE_1:
		/* spec: m = 140, b = 32100, R = -2 */
		val = (((msg.data[1] << 8) | msg.data[0]) * 100 - 32100) / 140;
		break;
	default:
		return SENSOR_NOT_FOUND;
	}

	sensor_val *sval = (sensor_val *)reading;
	memset(sval, 0, sizeof(*sval));

	sval->integer = (int32_t)val;
	sval->fraction = (int32_t)(val * 1000) % 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t tps25990_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX)
		return SENSOR_INIT_UNSPECIFIED_ERROR;

	tps25990_init_arg *init_args = (tps25990_init_arg *)cfg->init_args;

	if (init_args->is_init)
		goto skip_init;

	init_args->is_init = true;

skip_init:
	cfg->read = tps25990_read;
	return SENSOR_INIT_SUCCESS;
}