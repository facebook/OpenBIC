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
#include "util_pmbus.h"

LOG_MODULE_REGISTER(dev_q50sn120a1);

int q50sn120a1_read_pout(uint8_t sensor_num, float *pout_value)
{
	CHECK_NULL_ARG_WITH_RETURN(pout_value, -1)

	int ret = 0;
	float vout = 0;
	float iout = 0;
	float exponent = 1;
	uint16_t tmp = 0;

	/* Read Vout */
	if (get_exponent_from_vout_mode(sensor_num, &exponent) == false) {
		LOG_ERR("Fail to get exponent from VOUT mode command");
		return -1;
	}

	ret = pmbus_read_command(sensor_num, PMBUS_READ_VOUT, &tmp);
	if (ret != 0) {
		return -1;
	}
	vout = (float)tmp * exponent;

	/* Read Iout */
	ret = pmbus_read_command(sensor_num, PMBUS_READ_IOUT, &tmp);
	if (ret != 0) {
		return -1;
	}
	iout = slinear11_to_float(tmp);

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

	int ret = 0;
	float val = 0;
	float exponent = 1;
	uint16_t tmp = 0;
	uint8_t offset = sensor_config[sensor_config_index_map[sensor_num]].offset;

	switch (offset) {
	case PMBUS_READ_VOUT:
		if (get_exponent_from_vout_mode(sensor_num, &exponent) == false) {
			LOG_ERR("Fail to get exponent from VOUT mode command");
			return SENSOR_UNSPECIFIED_ERROR;
		}

		ret = pmbus_read_command(sensor_num, offset, &tmp);
		if (ret != 0) {
			return SENSOR_UNSPECIFIED_ERROR;
		}
		val = (float)tmp * exponent;
		break;
	case PMBUS_READ_IOUT:
	case PMBUS_READ_TEMPERATURE_1:
		ret = pmbus_read_command(sensor_num, offset, &tmp);
		if (ret != 0) {
			return SENSOR_UNSPECIFIED_ERROR;
		}
		val = slinear11_to_float(tmp);
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

	sensor_val *sval = (sensor_val *)reading;
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
