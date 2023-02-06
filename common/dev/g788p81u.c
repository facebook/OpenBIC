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
#include "sensor.h"
#include "hal_i2c.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(dev_g788p81u);

uint8_t g788p81u_read(uint8_t sensor_num, int *reading)
{
	if (!reading || (sensor_num > SENSOR_NUM_MAX)) {
		if (sensor_num > SENSOR_NUM_MAX){
			LOG_ERR("Invalid sensor num");
		}
		else {
			LOG_ERR("reading pointer is NULL");
		}
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	I2C_MSG msg = { 0 };
	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = cfg->offset;

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Failed to access sensor");
		return SENSOR_FAIL_TO_ACCESS;
	}

	float val = 0;

	switch (cfg->offset) {
	case G788P81U_LOCAL_TEMP_OFFSET:
		val = (int8_t)msg.data[0];
		break;
	case G788P81U_REMOTE_TEMP_OFFSET:
		val = (int8_t)msg.data[0];

		msg.bus = cfg->port;
		msg.target_addr = cfg->target_addr;
		msg.tx_len = 1;
		msg.rx_len = 1;
		msg.data[0] = G788P81U_REMOTE_TEMP_EXT_OFFSET;
		if (i2c_master_read(&msg, retry)) {
			LOG_ERR("Failed to access sensor");
			return SENSOR_FAIL_TO_ACCESS;
		}
		val += 0.125 * (msg.data[0] >> 5);
		break;

	default:
		break;
	}

	sensor_val *sval = (sensor_val *)reading;
	memset(sval, 0, sizeof(sensor_val));
	sval->integer = (int32_t)val;
	sval->fraction = (int32_t)(val * 1000) % 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t g788p81u_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		LOG_ERR("Invalid sensor num: %d", sensor_num);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = g788p81u_read;
	return SENSOR_INIT_SUCCESS;
}
