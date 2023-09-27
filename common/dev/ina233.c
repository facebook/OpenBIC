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
#include "ina233.h"

LOG_MODULE_REGISTER(dev_ina233);

#define INA233_CALIBRATION_OFFSET 0xD4
#define INA233_MFR_ADC_CONFIG 0xD0
#define INA233_MFR_DEVICE_CONFIG 0xD5
#define INA233_MFR_ALERT_MASK 0xD2

#define INA233_IOUT_OC_WARN_LIMIT_REG 0x4A
#define INA233_VIN_OV_WARN_LIMIT_REG 0x57
#define INA233_VIN_UV_WARN_LIMIT_REG 0x58
#define INA233_PIN_OP_WARN_LIMIT_REG 0x6B

#define INA233_EIN_POWER_CNT_MAX 0x10000

uint8_t INA233_DEVICE_ID[3] = { 0x02, 0x54, 0x49 };

static int ina233_set_alert_threshold(sensor_cfg *cfg, uint8_t offset, int set_val)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, -1);

	if (set_val == INA233_NO_SET_ALERT_THRESHOLD) {
		return 0;
	}

	int ret = 0;
	uint8_t retry = 5;

	I2C_MSG msg = { 0 };
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 3;
	msg.data[0] = offset;
	msg.data[1] = set_val & 0xFF;
	msg.data[2] = (set_val >> 8) & 0xFF;

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("i2c set alert threshold fail ret: %d, sensor num: 0x%x, offset: 0x%x, set val: 0x%x",
			ret, cfg->num, offset, set_val);
		return -1;
	}

	return 0;
}

int ina233_read_ein(sensor_cfg *cfg, int16_t *val)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, -1);
	CHECK_NULL_ARG_WITH_RETURN(val, -1);

	int ret = 0;
	uint8_t retry = 5;
	uint32_t power = 0;
	uint32_t rollover = 0;
	uint32_t sample = 0;
	I2C_MSG msg = { 0 };

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 7;
	msg.data[0] = cfg->offset;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("i2c read fail ret: %d, sensor num: 0x%x, offset: 0x%x", ret, cfg->num,
			cfg->offset);
		return SENSOR_FAIL_TO_ACCESS;
	}

	// Record the current data
	power = (msg.data[2] << 8) | msg.data[1];
	rollover = msg.data[3];
	sample = (msg.data[6] << 16) | (msg.data[5] << 8) | msg.data[4];

	if (sample == 0) {
		LOG_ERR("Sample is zero, sensor num: 0x%x", cfg->num);
		return -1;
	}

	power = rollover * INA233_EIN_POWER_CNT_MAX + power;

	*val = (int16_t)(power / sample);
	return 0;
}

uint8_t ina233_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	ina233_init_arg *init_arg = (ina233_init_arg *)cfg->init_args;
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

	if (cfg->offset != PMBUS_READ_EIN) {
		msg.bus = cfg->port;
		msg.target_addr = cfg->target_addr;
		msg.tx_len = 1;
		msg.rx_len = 2;
		msg.data[0] = cfg->offset;

		ret = i2c_master_read(&msg, retry);
		if (ret != 0) {
			LOG_ERR("i2c read fail ret: %d", ret);
			return SENSOR_FAIL_TO_ACCESS;
		}
	}

	uint8_t offset = cfg->offset;
	val = (msg.data[1] << 8) | msg.data[0];
	sensor_val *sval = (sensor_val *)reading;
	switch (offset) {
	case PMBUS_READ_VOUT:
		// 1 mV/LSB, unsigned integer
		// m = 8 , b = 0 , r = 2
		// voltage convert formula = ((val / 100) - 0) / 8
		parameter = 800;
		break;
	case PMBUS_READ_IOUT:
		if (GETBIT(msg.data[1], 7)) {
			// If raw value is negative, set it zero.
			val = 0;
		}
		// 1 mA/LSB, 2's complement
		// current convert formula = val / (1 / current_lsb)
		parameter = (1 / init_arg->current_lsb);
		break;
	case PMBUS_READ_POUT:
		// 1 Watt/LSB, 2's complement
		// power convert formula = val / (1 / (current_lsb * 25))
		parameter = (1 / (init_arg->current_lsb * 25));
		break;
	case PMBUS_READ_EIN:
		// 1 Watt/LSB, 2's complement
		// average power convert formula = val / (1 / (current_lsb * 25))
		ret = ina233_read_ein(cfg, &val);
		if (ret != 0) {
			return SENSOR_UNSPECIFIED_ERROR;
		}
		parameter = (1 / (init_arg->current_lsb * 25));
		break;
	default:
		LOG_ERR("Offset not supported: 0x%x", offset);
		return SENSOR_FAIL_TO_ACCESS;
		break;
	}

	sval->integer = val / parameter;
	sval->fraction = ((val / parameter) - sval->integer) * 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t ina233_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	int ret = 0;
	int retry = 5;
	I2C_MSG msg = { 0 };

	ina233_init_arg *init_arg = (ina233_init_arg *)cfg->init_args;
	if (init_arg->mfr_config_init == true) {
		memset(&msg, 0, sizeof(I2C_MSG));

		msg.bus = cfg->port;
		msg.target_addr = cfg->target_addr;
		msg.tx_len = 3;
		msg.data[0] = INA233_MFR_ADC_CONFIG;
		msg.data[1] = init_arg->mfr_config.value & 0xFF;
		msg.data[2] = init_arg->mfr_config.value >> 8;
		ret = i2c_master_write(&msg, retry);
		if (ret != 0) {
			LOG_ERR("i2c write MFR_ADC_CONFIG fail ret: %d, sensor num: 0x%x", ret,
				cfg->num);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}
		init_arg->mfr_config_init = false;
	}

	if (init_arg->is_need_mfr_device_config_init == true) {
		memset(&msg, 0, sizeof(I2C_MSG));

		msg.bus = cfg->port;
		msg.target_addr = cfg->target_addr;
		msg.tx_len = 2;
		msg.data[0] = INA233_MFR_DEVICE_CONFIG;
		msg.data[1] = init_arg->mfr_device_config.value;
		ret = i2c_master_write(&msg, retry);
		if (ret != 0) {
			LOG_ERR("i2c write MFR_DEVICE_CONFIG fail ret: %d, sensor num: 0x%x", ret,
				cfg->num);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}
	}

	if (init_arg->is_need_set_alert_threshold) {
		ret = ina233_set_alert_threshold(cfg, INA233_IOUT_OC_WARN_LIMIT_REG,
						 init_arg->iout_oc_warn_limit);
		if (ret != 0) {
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}

		ret = ina233_set_alert_threshold(cfg, INA233_VIN_OV_WARN_LIMIT_REG,
						 init_arg->vin_ov_warn_limit);
		if (ret != 0) {
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}

		ret = ina233_set_alert_threshold(cfg, INA233_VIN_UV_WARN_LIMIT_REG,
						 init_arg->vin_uv_warn_limit);
		if (ret != 0) {
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}

		ret = ina233_set_alert_threshold(cfg, INA233_PIN_OP_WARN_LIMIT_REG,
						 init_arg->pin_op_warn_limit);
		if (ret != 0) {
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}
	}

	if (init_arg->is_init != true) {
		memset(&msg, 0, sizeof(I2C_MSG));
		uint16_t calibration = 0;

		msg.bus = cfg->port;
		msg.target_addr = cfg->target_addr;
		msg.tx_len = 3;
		msg.data[0] = INA233_CALIBRATION_OFFSET;

		// Calibration formula = (0.00512 / (current_lsb * r_shunt))
		calibration = (uint16_t)(0.00512 / (init_arg->current_lsb * init_arg->r_shunt));
		memcpy(&msg.data[1], &calibration, sizeof(uint16_t));

		ret = i2c_master_write(&msg, retry);
		if (ret != 0) {
			LOG_ERR("i2c write CALIBRATION register fail ret: %d, sensor num: 0x%x",
				ret, cfg->num);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}
		init_arg->is_init = true;
	}

	cfg->read = ina233_read;
	return SENSOR_INIT_SUCCESS;
}
