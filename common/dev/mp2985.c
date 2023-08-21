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
#include <string.h>
#include <logging/log.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"
#include "mp2985.h"

LOG_MODULE_REGISTER(mp2985);

#define MP2985_GET_USER_DATA 0xB8

bool mp2985_get_checksum(uint8_t bus, uint8_t addr, uint8_t *checksum)
{
	CHECK_NULL_ARG_WITH_RETURN(checksum, false);

	uint8_t retry = 3;
	I2C_MSG i2c_msg = { 0 };

	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 2;
	i2c_msg.data[0] = PMBUS_PAGE;
	i2c_msg.data[1] = 0x0;
	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Fail to set page to 0, bus: 0x%x, addr: 0x%x", bus, addr);
		return false;
	}

	memset(&i2c_msg, 0, sizeof(I2C_MSG));
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 5;
	i2c_msg.data[0] = MP2985_GET_USER_DATA;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Fail to read checksum, bus: 0x%x, addr: 0x%x", bus, addr);
		return false;
	}

	checksum[0] = i2c_msg.data[4];
	checksum[1] = i2c_msg.data[3];
	checksum[2] = i2c_msg.data[2];
	checksum[3] = i2c_msg.data[1];

	return true;
}

uint8_t mp2985_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	mp2985_init_arg *init_arg = (mp2985_init_arg *)cfg->init_args;
	if (!init_arg->is_init) {
		LOG_ERR("device isn't initialized");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t i2c_max_retry = 5;
	float val = 0;
	sensor_val *sval = (sensor_val *)reading;
	I2C_MSG msg;
	memset(sval, 0, sizeof(sensor_val));

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = cfg->offset;

	if (i2c_master_read(&msg, i2c_max_retry)) {
		/* read fail */
		return SENSOR_FAIL_TO_ACCESS;
	}

	float pow2_n_iout = 0.0f;
	int y_iout = 0;
	switch (cfg->offset) {
	case PMBUS_READ_TEMPERATURE_1:
		/* TEMP[15:11] are reserved.
		 * TEMP[10:0] are two’s complement integer.
		 * BIC reports 0 degeee if the temperature is negative.
		 */
		if (GETBIT(msg.data[1], 2)) {
			val = 0;
		} else {
			val = ((msg.data[1] & 0x3) << 8) | msg.data[0];
		}
		break;
	case PMBUS_READ_VOUT:
		/* The VOUT mode can be set into offset 20h.(VOUT_MODE)
		 * VOUT mode is set as direct format and the resolution is 1mV/LSB in initialization stage.
		 * VOUT[15:12] are reserved.
		 * VOUT[11:0] are output voltage.
		 */
		val = ((msg.data[1] & 0x7) << 8) | msg.data[0];
		val *= 0.001;
		break;
	case PMBUS_READ_IOUT:
		/* The IOUT calculation format is linear-11.
		 * Refer to PMBus spec 7.3, the linear-11 formula is
		 * X = Y * (2 ^N)
		 * Where, as described above:
		 * X: real world value
		 * Y: 11-bit(IOUT[10:0]), two’s complement integer
		 * N: 5-bit(IOUT[15:11]), two’s complement integer
		 */
		if (GETBIT(msg.data[1], 7)) {
			pow2_n_iout = 1 / (float)(1 << ((~(msg.data[1] >> 3) + 1) & 0x1F));
		} else {
			pow2_n_iout = 1 / (float)(1 << (msg.data[1] >> 3));
		}

		if (GETBIT(msg.data[1], 2)) {
			y_iout = ~(((msg.data[1] & 0x7) << 8) | msg.data[0]) + 1;
		} else {
			y_iout = ((msg.data[1] & 0x7) << 8) | msg.data[0];
		}

		val = y_iout * pow2_n_iout;
		break;
	case PMBUS_READ_POUT:
		/* Output power = POUT[10:0] * 0.25W/LSB */
		val = ((msg.data[1] & 0x7) << 8) | msg.data[0];
		val *= 0.25;
		break;
	default:
		LOG_ERR("unsupported offset 0x%x reading", cfg->offset);
		break;
	}

	sval->integer = (int32_t)val;
	sval->fraction = (int32_t)(val * 1000) % 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t mp2985_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	mp2985_init_arg *init_args = (mp2985_init_arg *)cfg->init_args;
	if (init_args->is_init)
		goto skip_init;

	uint8_t retry = 5;
	I2C_MSG msg;
	memset(&msg, 0, sizeof(msg));
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;

	/* Set Page0 */
	msg.tx_len = 2;
	msg.data[0] = PMBUS_PAGE;
	msg.data[1] = 0x0;
	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("failed to set page0");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	/* Set VOUT mode as direct format, 1mV/LSB
	 * Offset 20h VOUT_MODE[7:0]:
	 * 8'h40: direct format, 1mV/LSB
	 * 8'h17: linear format, 1/512 V/LSB
	 * 8'h21: VID format
	 */
	msg.tx_len = 2;
	msg.data[0] = PMBUS_VOUT_MODE;
	msg.data[1] = 0x40;
	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("failed to vout mode");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	init_args->is_init = true;

skip_init:
	cfg->read = mp2985_read;
	return SENSOR_INIT_SUCCESS;
}
