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
#include <logging/log.h>
#include "libutil.h"
#include "sensor.h"
#include "hal_i2c.h"

#define LM75BD118_TEMP_REG 0x00
#define LM75BD118_TEMP_DATA_UNIT 0.125
#define LM75BD118_TEMP_INTEGER_INDEX 0
#define LM75BD118_TEMP_FRACTION_INDEX 1
#define LM75BD118_TEMP_DATA_OFFSET 5

LOG_MODULE_REGISTER(dev_lm75bd118);

uint8_t lm75bd118_read(uint8_t sensor_num, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (sensor_num > SENSOR_NUM_MAX) {
		LOG_ERR("Sensor 0x%x input parameter is invalid", sensor_num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	sensor_cfg cfg = sensor_config[sensor_config_index_map[sensor_num]];

	if (cfg.offset != LM75BD118_TEMP_REG) {
		LOG_ERR("Invalid offset: 0x%x", cfg.offset);
		return SENSOR_PARAMETER_NOT_VALID;
	}

	int ret = 0;
	int8_t val = 0;
	uint8_t fraction = 0;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = cfg.port;
	msg.target_addr = cfg.target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = cfg.offset;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("i2c read fail ret: %d", ret);
		return SENSOR_FAIL_TO_ACCESS;
	}

	val = msg.data[LM75BD118_TEMP_INTEGER_INDEX];
	fraction = msg.data[LM75BD118_TEMP_FRACTION_INDEX] >> LM75BD118_TEMP_DATA_OFFSET;

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = val;
	sval->fraction = (fraction * LM75BD118_TEMP_DATA_UNIT) * 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t lm75bd118_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		LOG_ERR("Sensor 0x%x input parameter is invalid", sensor_num);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = lm75bd118_read;
	return SENSOR_INIT_SUCCESS;
}
