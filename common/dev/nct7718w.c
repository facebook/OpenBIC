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

uint8_t nct7718w_read(uint8_t sensor_num, int *reading)
{
	if (!reading || (sensor_num > SENSOR_NUM_MAX)) {
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
		return SENSOR_FAIL_TO_ACCESS;
	}

	float read_out = 0;

	switch (cfg->offset) {
	case NCT7718W_LOCAL_TEMP_OFFSET:
		read_out = (int8_t)msg.data[0];
		break;
	case NCT7718W_REMOTE_TEMP_MSB_OFFSET:
		read_out = (int8_t)msg.data[0];

		msg.bus = cfg->port;
		msg.target_addr = cfg->target_addr;
		msg.tx_len = 1;
		msg.rx_len = 1;
		msg.data[0] = NCT7718W_REMOTE_TEMP_LSB_OFFSET;

		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}

		read_out += 0.125 * (msg.data[0] >> 5);
		break;

	default:
		break;
	}

	sensor_val *sval = (sensor_val *)reading;
	memset(sval, 0, sizeof(sensor_val));
	sval->integer = (int)read_out;
	sval->fraction = (int)(read_out * 1000) % 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t nct7718w_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = nct7718w_read;
	return SENSOR_INIT_SUCCESS;
}
