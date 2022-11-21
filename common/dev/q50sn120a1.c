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
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"

LOG_MODULE_REGISTER(dev_q50sn120a1);

/** Q50SN120A1 VOUT scale is exponent -12 base 2 **/
#define Q50SN120A1_VOUT_SCALE 0.000244140625

/** Q50SN120A1 IOUT/Temperature scale is exponent -2 base 2 **/
#define Q50SN120A1_IOUT_SCALE 0.25
#define Q50SN120A1_TEMPERATURE_SCALE 0.25

int q50sn120a1_read_pout(uint8_t sensor_num, float *pout_value)
{
	CHECK_NULL_ARG_WITH_RETURN(pout_value, -1)

	uint8_t retry = 5;
	float vout = 0, iout = 0;
	int ret = 0;
	I2C_MSG msg = { 0 };

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = PMBUS_READ_VOUT;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("I2c read vout fail, ret: %d", ret);
		return -1;
	}
	vout = ((msg.data[1] << 8) | msg.data[0]) * Q50SN120A1_VOUT_SCALE;

	memset(&msg, 0, sizeof(msg));
	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = PMBUS_READ_IOUT;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("I2c read iout fail, ret: %d", ret);
		return -1;
	}
	iout = ((msg.data[1] << 8) | msg.data[0]) * Q50SN120A1_IOUT_SCALE;

	*pout_value = vout * iout;
	return 0;
}

uint8_t q50sn120a1_read(uint8_t sensor_num, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (sensor_num > SENSOR_NUM_MAX) {
		LOG_ERR("Sensor 0x%x input parameter is invalid", sensor_num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	uint8_t offset = sensor_config[sensor_config_index_map[sensor_num]].offset;
	float val = 0;
	int ret = 0;
	I2C_MSG msg = { 0 };

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = offset;

	if (offset != PMBUS_READ_POUT) {
		ret = i2c_master_read(&msg, retry);
		if (ret != 0) {
			LOG_ERR("I2c read fail  ret: %d", ret);
			return SENSOR_FAIL_TO_ACCESS;
		}
		val = (msg.data[1] << 8) | msg.data[0];
	}

	sensor_val *sval = (sensor_val *)reading;
	switch (offset) {
	case PMBUS_READ_VOUT:
		val = val * Q50SN120A1_VOUT_SCALE;
		break;
	case PMBUS_READ_IOUT:
		val = val * Q50SN120A1_IOUT_SCALE;
		break;
	case PMBUS_READ_TEMPERATURE_1:
		val = val * Q50SN120A1_TEMPERATURE_SCALE;
		break;
	case PMBUS_READ_POUT:
		ret = q50sn120a1_read_pout(sensor_num, &val);
		if (ret != 0) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		break;
	default:
		LOG_ERR("Offset not supported: 0x%x", offset);
		return SENSOR_FAIL_TO_ACCESS;
		break;
	}

	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t q50sn120a1_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		LOG_ERR("Input sensor number is invalid");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = q50sn120a1_read;
	return SENSOR_INIT_SUCCESS;
}
