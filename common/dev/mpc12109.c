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

LOG_MODULE_REGISTER(mpc12109);

#define MP2891_READ_TEMPERATURE_1_RESOLUTION 1
#define MP2891_READ_VIN_RESOLUTION 0.125
#define MP2891_READ_VOUT_RESOLUTION 0.0625
#define MP2891_READ_IOUT_RESOLUTION 0.25
#define MP2891_READ_POUT_RESOLUTION 1

#define MP2891_READ_TEMPERATURE_1_MASK GENMASK(7, 0)
#define MP2891_READ_VIN_MASK GENMASK(9, 0)
#define MP2891_READ_VOUT_MASK GENMASK(8, 0)
#define MP2891_READ_IOUT_MASK GENMASK(9, 0)
#define MP2891_READ_POUT_MASK GENMASK(10, 0)

bool mpc12109_i2c_read(uint8_t bus, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	memset(data, 0, len);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = len;
	i2c_msg.data[0] = reg;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read mp29816a, bus: %d, addr: 0x%x, reg: 0x%x", bus, addr, reg);
		return false;
	}

	memcpy(data, i2c_msg.data, len);
	return true;
}

bool mpc12109_get_vr_status(sensor_cfg *cfg, uint8_t rail, uint8_t vr_status_rail,
			    uint16_t *vr_status)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(vr_status, false);

	uint16_t val = 0;

	switch (vr_status_rail) {
	case PMBUS_STATUS_BYTE: {
		val = 0xFFFF;
	} break;
	case PMBUS_STATUS_WORD: {
		uint8_t data[2] = { 0 };
		if (!mpc12109_i2c_read(cfg->port, cfg->target_addr, PMBUS_STATUS_WORD, data,
				       sizeof(data))) {
			return false;
		}
		val = data[0] | (data[1] << 8);
	} break;
	case PMBUS_STATUS_VOUT: {
		uint8_t data[1] = { 0 };
		if (!mpc12109_i2c_read(cfg->port, cfg->target_addr, PMBUS_STATUS_VOUT, data,
				       sizeof(data))) {
			return false;
		}
		val = (uint16_t)data[0];
	} break;
	case PMBUS_STATUS_IOUT: {
		uint8_t data[1] = { 0 };
		if (!mpc12109_i2c_read(cfg->port, cfg->target_addr, PMBUS_STATUS_IOUT, data,
				       sizeof(data))) {
			return false;
		}
		val = (uint16_t)data[0];
	} break;
	case PMBUS_STATUS_INPUT: {
		uint8_t data[1] = { 0 };
		if (!mpc12109_i2c_read(cfg->port, cfg->target_addr, PMBUS_STATUS_INPUT, data,
				       sizeof(data))) {
			return false;
		}
		val = (uint16_t)data[0];
	} break;
	case PMBUS_STATUS_TEMPERATURE: {
		uint8_t data[1] = { 0 };
		if (!mpc12109_i2c_read(cfg->port, cfg->target_addr, PMBUS_STATUS_TEMPERATURE, data,
				       sizeof(data))) {
			return false;
		}
		val = (uint16_t)data[0];
	} break;
	case PMBUS_STATUS_CML: {
		uint8_t data[1] = { 0 };
		if (!mpc12109_i2c_read(cfg->port, cfg->target_addr, PMBUS_STATUS_CML, data,
				       sizeof(data))) {
			return false;
		}
		val = (uint16_t)data[0];
	} break;
	default:
		LOG_ERR("VR[0x%x] not support vr status:0x%x.", cfg->num, vr_status_rail);
		return false;
		break;
	}
	*vr_status = val;
	return true;
}

bool mpc12109_clear_vr_status(sensor_cfg *cfg, uint8_t rail)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = cfg->port;
	i2c_msg.target_addr = cfg->target_addr;
	i2c_msg.tx_len = 1;
	i2c_msg.data[0] = PMBUS_CLEAR_FAULTS;

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("VR[0x%x] clear fault failed.", cfg->num);
		return false;
	}

	return true;
}

uint8_t mpc12109_read(sensor_cfg *cfg, int *reading)
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

	float iout_resolution = MP2891_READ_IOUT_RESOLUTION;
	float pout_resolution = MP2891_READ_POUT_RESOLUTION;

	if ((cfg->offset == PMBUS_READ_IOUT) || (cfg->offset == PMBUS_READ_POUT)) {
		if (cfg->init_args != NULL) {
			const mpc12109_init_arg *init_arg = (mpc12109_init_arg *)cfg->init_args;
			iout_resolution = init_arg->iout_lsb;
			pout_resolution = init_arg->pout_lsb;
		}
	}

	float val;
	if (cfg->offset == PMBUS_READ_TEMPERATURE_1) {
		uint16_t read_value =
			((msg.data[1] << 8) | msg.data[0]) & MP2891_READ_TEMPERATURE_1_MASK;
		val = (float)read_value * MP2891_READ_TEMPERATURE_1_RESOLUTION;
	} else if (cfg->offset == PMBUS_READ_VIN) {
		uint16_t read_value = ((msg.data[1] << 8) | msg.data[0]) & MP2891_READ_VIN_MASK;
		val = (float)read_value * MP2891_READ_VIN_RESOLUTION;
	} else if (cfg->offset == PMBUS_READ_VOUT) {
		uint16_t read_value = ((msg.data[1] << 8) | msg.data[0]) & MP2891_READ_VOUT_MASK;
		val = (float)read_value * MP2891_READ_VOUT_RESOLUTION;
	} else if (cfg->offset == PMBUS_READ_IOUT) {
		uint16_t read_value = ((msg.data[1] << 8) | msg.data[0]) & MP2891_READ_IOUT_MASK;
		val = (float)read_value * iout_resolution;
	} else if (cfg->offset == PMBUS_READ_POUT) {
		uint16_t read_value = ((msg.data[1] << 8) | msg.data[0]) & MP2891_READ_POUT_MASK;
		val = (float)read_value * pout_resolution;
	} else {
		return SENSOR_FAIL_TO_ACCESS;
	}

	sval->integer = val;
	sval->fraction = (val - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t mpc12109_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = mpc12109_read;
	return SENSOR_INIT_SUCCESS;
}
