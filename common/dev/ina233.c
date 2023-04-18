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

uint8_t INA233_DEVICE_ID[3] = { 0x02, 0x54, 0x49 };

uint8_t ina233_read(uint8_t sensor_num, int *reading)
{
	if (reading == NULL) {
		LOG_ERR("input parameter reading is NULL");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	if (sensor_num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor 0x%x input parameter is invalid", sensor_num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	ina233_init_arg *init_arg =
		(ina233_init_arg *)sensor_config[sensor_config_index_map[sensor_num]].init_args;
	if (init_arg == NULL) {
		LOG_ERR("input initial pointer is NULL");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

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

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = sensor_config[sensor_config_index_map[sensor_num]].offset;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("i2c read fail ret: %d", ret);
		return SENSOR_FAIL_TO_ACCESS;
	}

	uint8_t offset = sensor_config[sensor_config_index_map[sensor_num]].offset;
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
	default:
		LOG_ERR("Offset not supported: 0x%x", offset);
		return SENSOR_FAIL_TO_ACCESS;
		break;
	}

	sval->integer = val / parameter;
	sval->fraction = ((val / parameter) - sval->integer) * 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t ina233_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		LOG_ERR("input sensor number is invalid");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	ina233_init_arg *init_arg =
		(ina233_init_arg *)sensor_config[sensor_config_index_map[sensor_num]].init_args;
	if (init_arg == NULL) {
		LOG_ERR("input initial pointer is NULL");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	if (init_arg->mfr_config_init == true) {
		int ret = 0, retry = 5;
		I2C_MSG msg = { 0 };

		msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
		msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
		msg.tx_len = 3;
		msg.data[0] = INA233_MFR_ADC_CONFIG;
		msg.data[1] = init_arg->mfr_config.value & 0xFF;
		msg.data[2] = init_arg->mfr_config.value >> 8;
		ret = i2c_master_write(&msg, retry);
		if (ret != 0) {
			LOG_ERR("i2c write fail ret: %d", ret);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}
		init_arg->mfr_config_init = false;
	}

	if (init_arg->is_init != true) {
		int ret = 0, retry = 5;
		uint16_t calibration = 0;
		I2C_MSG msg = { 0 };

		msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
		msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
		msg.tx_len = 3;
		msg.data[0] = INA233_CALIBRATION_OFFSET;

		// Calibration formula = (0.00512 / (current_lsb * r_shunt))
		calibration = (uint16_t)(0.00512 / (init_arg->current_lsb * init_arg->r_shunt));
		memcpy(&msg.data[1], &calibration, sizeof(uint16_t));

		ret = i2c_master_write(&msg, retry);
		if (ret != 0) {
			LOG_ERR("i2c write fail ret: %d", ret);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}
		init_arg->is_init = true;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = ina233_read;
	return SENSOR_INIT_SUCCESS;
}
