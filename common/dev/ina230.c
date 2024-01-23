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

#include "ina230.h"
#include "sensor.h"
#include "hal_i2c.h"
#include <logging/log.h>

#define I2C_RETRY 5

#define INA230_BUS_VOLTAGE_LSB 0.00125 // 1.25 mV.
#define INA230_VSH_VOLTAGE_LSB 0.0000025 // 2.5 uV.

#define INA230_ALT_MASK 0xF800
#define INA230_ALT_SOL_OFFSET 0x8000
#define INA230_ALT_SUL_OFFSET 0x4000
#define INA230_ALT_BOL_OFFSET 0x2000
#define INA230_ALT_BUL_OFFSET 0x1000
#define INA230_ALT_POL_OFFSET 0x0800

LOG_MODULE_REGISTER(dev_ina230);

uint8_t ina230_reset_alt(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	I2C_MSG msg = { 0 };
	ina230_init_arg *init_args = (ina230_init_arg *)cfg->init_args;
	if (init_args->is_init == false) {
		LOG_ERR("Device isn't initialized");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.rx_len = 2;
	msg.tx_len = 1;
	msg.data[0] = INA230_MSK_OFFSET;

	if (i2c_master_read(&msg, I2C_RETRY)) {
		LOG_ERR("Failed to access sensor");
		return SENSOR_FAIL_TO_ACCESS;
	}

	return SENSOR_READ_SUCCESS;
}

uint8_t ina230_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	double val = 0.0;
	int16_t signed_reg_val = 0;
	uint16_t reg_val = 0;
	I2C_MSG msg = { 0 };

	ina230_init_arg *init_args = (ina230_init_arg *)cfg->init_args;
	if (init_args->is_init == false) {
		LOG_ERR("Device isn't initialized");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = cfg->offset;

	if (i2c_master_read(&msg, I2C_RETRY)) {
		LOG_ERR("Failed to access sensor");
		return SENSOR_FAIL_TO_ACCESS;
	}

	reg_val = (msg.data[0] << 8) | msg.data[1];

	switch (cfg->offset) {
	case INA230_BUS_VOL_OFFSET:
		val = (double)(reg_val)*INA230_BUS_VOLTAGE_LSB;
		break;
	case INA230_PWR_OFFSET:
		val = (double)(reg_val)*init_args->pwr_lsb;
		break;
	case INA230_CUR_OFFSET:
		if (GETBIT(msg.data[0], 7)) {
			// If raw value is negative, set it zero.
			reg_val = 0;
		}
		signed_reg_val = (int16_t)reg_val;
		val = (double)(signed_reg_val)*init_args->cur_lsb;
		break;
	default:
		return SENSOR_NOT_FOUND;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t ina230_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	uint16_t calibration = 0, alt_reg = 0;
	I2C_MSG msg = { 0 };

	ina230_init_arg *init_args = (ina230_init_arg *)cfg->init_args;

	if (init_args->r_shunt <= 0.0 || init_args->i_max <= 0.0) {
		LOG_ERR("INA230 has invalid initial arguments");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	if (init_args->is_init) {
		goto skip_init;
	}

	/* Calibration = 0.00512 / (cur_lsb * r_shunt)
	 * - 0.00512 : The fixed value in ina230 used to ensure scaling is maintained properly.
	 * - cur_lsb : Maximum Expected Current(i_max) / 2^15.
	 * - r_shunt : Value of the shunt resistor.
	 * Ref: https://www.ti.com/product/INA230
	 */
	init_args->cur_lsb = init_args->i_max / 32768.0;
	calibration = (uint16_t)(0.00512 / (init_args->cur_lsb * init_args->r_shunt));

	// The power LSB is internally programmed to equal 25 times the current LSB.
	init_args->pwr_lsb = init_args->cur_lsb * 25.0;

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 3;
	msg.data[0] = INA230_CFG_OFFSET;
	msg.data[1] = (init_args->config.value >> 8) & 0xFF;
	msg.data[2] = init_args->config.value & 0xFF;

	if (i2c_master_write(&msg, I2C_RETRY)) {
		LOG_ERR("Failed to set INA230(%.2d-%.2x) configuration.", msg.bus, msg.target_addr);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	// The size of calibration is 15 bits, and the MSB is unused.
	if (calibration & 0x8000) {
		LOG_ERR("Failed to set INA230(%.2d-%.2x) calibration due to"
			" the data overflowed (CAL: 0x%.2X)",
			msg.bus, msg.target_addr, calibration);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	memset(msg.data, 0, I2C_BUFF_SIZE);
	msg.data[0] = INA230_CAL_OFFSET;
	msg.data[1] = (calibration >> 8) & 0xFF;
	msg.data[2] = calibration & 0xFF;

	if (i2c_master_write(&msg, I2C_RETRY)) {
		LOG_ERR("Failed to set INA230(%.2d-%.2x) calibration.", msg.bus, msg.target_addr);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	if (init_args->alt_cfg.value & INA230_ALT_MASK) {
		if (init_args->alert_value == 0.0 ||
		    ((init_args->alt_cfg.BOL || init_args->alt_cfg.BUL || init_args->alt_cfg.POL) &&
		     init_args->alert_value < 0.0)) {
			LOG_ERR("INA230(%.2d-%.2x) has invalid alert function config.", msg.bus,
				msg.target_addr);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}

		switch (init_args->alt_cfg.value & INA230_ALT_MASK) {
		case INA230_ALT_SOL_OFFSET:
		case INA230_ALT_SUL_OFFSET:
			alt_reg = (uint16_t)(init_args->alert_value / INA230_VSH_VOLTAGE_LSB);
			break;
		case INA230_ALT_BOL_OFFSET:
		case INA230_ALT_BUL_OFFSET:
			alt_reg = (uint16_t)(init_args->alert_value / INA230_BUS_VOLTAGE_LSB);
			break;
		case INA230_ALT_POL_OFFSET:
			alt_reg = (uint16_t)(init_args->alert_value / init_args->pwr_lsb);
			break;
		default:
			LOG_ERR("INA230(%.2d-%.2x) has invalid alert function config.", msg.bus,
				msg.target_addr);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}

		memset(msg.data, 0, I2C_BUFF_SIZE);
		msg.data[0] = INA230_ALT_OFFSET;
		msg.data[1] = (alt_reg >> 8) & 0xFF;
		msg.data[2] = alt_reg & 0xFF;

		if (i2c_master_write(&msg, I2C_RETRY)) {
			LOG_ERR("Failed to set INA230(%.2d-%.2x) alert value.", msg.bus,
				msg.target_addr);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}

		memset(msg.data, 0, I2C_BUFF_SIZE);
		msg.data[0] = INA230_MSK_OFFSET;
		msg.data[1] = (init_args->alt_cfg.value >> 8) & 0xFF;
		msg.data[2] = init_args->alt_cfg.value & 0xFF;

		if (i2c_master_write(&msg, I2C_RETRY)) {
			LOG_ERR("Failed to set INA230(%.2d-%.2x) mask/enable.", msg.bus,
				msg.target_addr);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}
	}

	init_args->is_init = true;

skip_init:
	cfg->read = ina230_read;
	return SENSOR_INIT_SUCCESS;
}
