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
#include "libutil.h"
#include "sensor.h"
#include "sq52205.h"
#include "hal_i2c.h"

LOG_MODULE_REGISTER(dev_sq52205);

#define SQ52205_CONFIG_REG_OFFSET 0x00
#define SQ52205_CALIBRATION_OFFSET 0x05
#define SQ52205_MASK_ENABLE_OFFSET 0x06
#define SQ52205_ALERT_LIMIT_OFFSET 0x07
#define SQ52205_ACCUM_CONFIG_OFFSET 0x0D
#define SQ52205_SHUNT_LSB 0.0000025
#define SQ52205_EIN_POWER_CNT_MAX 0x10000

#define SQ52205_ALERT_MASK_SHIFT_OFFSET 11

static int sq52205_set_alert_threshold(sensor_cfg *cfg, uint8_t alert_mask, uint16_t set_val)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, -1);

	int ret = 0;
	uint8_t retry = 5;
	uint16_t mask = 0;

	I2C_MSG msg = { 0 };

	/* Set alert threshold */
	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 3;
	msg.data[0] = SQ52205_ALERT_LIMIT_OFFSET;
	msg.data[1] = (set_val >> 8) & 0xFF;
	msg.data[2] = set_val & 0xFF;

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("i2c set alert threshold fail ret: %d, sensor num: 0x%x, set val: 0x%x",
			ret, cfg->num, set_val);
		return -1;
	}

	/* Read enable_mask register */
	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = SQ52205_MASK_ENABLE_OFFSET;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("i2c read mask fail ret: %d, sensor num: 0x%x, alert mask: 0x%x", ret,
			cfg->num, alert_mask);
		return -1;
	}

	mask = (((msg.data[0] << 8) | msg.data[1]) |
		(alert_mask << SQ52205_ALERT_MASK_SHIFT_OFFSET));

	/* Set alert mask */
	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 3;
	msg.data[0] = SQ52205_MASK_ENABLE_OFFSET;
	msg.data[1] = (mask >> 8) & 0xFF;
	msg.data[2] = mask & 0xFF;

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("i2c set alert mask fail ret: %d, sensor num: 0x%x, mask: 0x%x", ret,
			cfg->num, mask);
		return -1;
	}

	return 0;
}

static int sq52205_calculate_ein(uint8_t *data_buf, float *val)
{
	CHECK_NULL_ARG_WITH_RETURN(data_buf, -1);
	CHECK_NULL_ARG_WITH_RETURN(val, -1);

	uint32_t power = 0;
	uint32_t rollover = 0;
	uint32_t sample = 0;

	// Record the current data
	power = (data_buf[4] << 8) | data_buf[5];
	rollover = data_buf[3];
	sample = (data_buf[0] << 16) | (data_buf[1] << 8) | data_buf[2];

	if (sample == 0) {
		LOG_ERR("Sample is zero");
		return -1;
	}

	power = rollover * SQ52205_EIN_POWER_CNT_MAX + power;

	*val = (float)(power / sample);
	return 0;
}

uint8_t sq52205_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	sq52205_init_arg *init_arg = (sq52205_init_arg *)cfg->init_args;

	if (init_arg->is_init != true) {
		LOG_ERR("device isn't initialized");
		return SENSOR_UNSPECIFIED_ERROR;
	}
	uint8_t retry = 5;
	int ret = 0;
	float val = 0;
	I2C_MSG msg = { 0 };

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = ((cfg->offset == SQ52205_READ_EIN_OFFSET) ? 6 : 2);
	msg.data[0] = cfg->offset;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("i2c read fail ret: %d", ret);
		return SENSOR_FAIL_TO_ACCESS;
	}

	val = (msg.data[0] << 8) | msg.data[1];
	sensor_val *sval = (sensor_val *)reading;
	switch (cfg->offset) {
	case SQ52205_READ_VOL_OFFSET:
		// 1.25 mV/LSB
		val = val * 1.25 / 1000;
		break;
	case SQ52205_READ_CUR_OFFSET:
		if (GETBIT(msg.data[0], 7)) {
			// If raw value is negative, set it zero.
			val = 0;
		}
		// current_lsb mA/LSB
		val = val * init_arg->current_lsb;
		break;
	case SQ52205_READ_PWR_OFFSET:
		// (25 * current_lsb) mW/LSB
		val = val * (25 * init_arg->current_lsb);
		break;
	case SQ52205_READ_EIN_OFFSET:
		ret = sq52205_calculate_ein(&msg.data[0], &val);
		if (ret < 0) {
			LOG_ERR("SQ52205 calculate EIN fail, sensor num: 0x%x", cfg->num);
			return SENSOR_UNSPECIFIED_ERROR;
		}
		val = val * (25 * init_arg->current_lsb);
		break;
	default:
		LOG_ERR("Offset not supported: 0x%x", cfg->offset);
		return SENSOR_PARAMETER_NOT_VALID;
		break;
	}

	sval->integer = (int16_t)(val);
	sval->fraction = (int16_t)((val - sval->integer) * 1000);
	return SENSOR_READ_SUCCESS;
}

uint8_t sq52205_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sq52205_init_arg *init_arg = (sq52205_init_arg *)cfg->init_args;

	if (init_arg->is_init != true) {
		int ret = 0, retry = 5;
		uint16_t calibration = 0;
		I2C_MSG msg = { 0 };

		msg.bus = cfg->port;
		msg.target_addr = cfg->target_addr;
		msg.tx_len = 3;
		msg.data[0] = SQ52205_CONFIG_REG_OFFSET;
		msg.data[1] = (init_arg->config.value >> 8) & 0xFF;
		msg.data[2] = init_arg->config.value & 0xFF;

		ret = i2c_master_write(&msg, retry);
		if (ret != 0) {
			LOG_ERR("i2c write config fail ret: %d, sensor num: 0x%x", ret, cfg->num);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}

		memset(&msg, 0, sizeof(I2C_MSG));
		msg.bus = cfg->port;
		msg.target_addr = cfg->target_addr;
		msg.tx_len = 3;
		msg.data[0] = SQ52205_CALIBRATION_OFFSET;

		// Calibration formula = ((2048 * shunt_lsb) / (current_lsb * r_shunt) round it up
		calibration = (uint16_t)(((2048 * SQ52205_SHUNT_LSB) /
					  (init_arg->current_lsb * init_arg->r_shunt)) +
					 0.5);
		msg.data[1] = (calibration >> 8) & 0xFF;
		msg.data[2] = calibration & 0xFF;

		ret = i2c_master_write(&msg, retry);
		if (ret != 0) {
			LOG_ERR("i2c write calibration fail ret: %d, sensor num: 0x%x", ret,
				cfg->num);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}

		if (init_arg->is_need_accum_config_init != false) {
			memset(&msg, 0, sizeof(I2C_MSG));

			msg.bus = cfg->port;
			msg.target_addr = cfg->target_addr;
			msg.tx_len = 3;
			msg.data[0] = SQ52205_ACCUM_CONFIG_OFFSET;
			msg.data[1] = init_arg->accum_config.value & 0xFF;
			msg.data[2] = (init_arg->accum_config.value >> 8) & 0xFF;

			ret = i2c_master_write(&msg, retry);
			if (ret != 0) {
				LOG_ERR("i2c write accum config fail ret: %d, sensor num: 0x%x",
					ret, cfg->num);
				return SENSOR_INIT_UNSPECIFIED_ERROR;
			}
		}

		if (init_arg->is_need_set_alert_threshold != false) {
			ret = sq52205_set_alert_threshold(cfg, init_arg->alert_mask_config.value,
							  init_arg->alert_threshold);
			if (ret != 0) {
				LOG_ERR("i2c set alert threshold fail, sensor num: 0x%x", cfg->num);
				return SENSOR_INIT_UNSPECIFIED_ERROR;
			}
		}

		init_arg->is_init = true;
	}

	cfg->read = sq52205_read;
	return SENSOR_INIT_SUCCESS;
}
