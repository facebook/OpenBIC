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
#include "sensor.h"
#include "hal_i2c.h"

#define NVMe_NOT_AVAILABLE 0x80
#define NVMe_TMP_SENSOR_FAILURE 0x81

uint8_t nvme_read(uint8_t sensor_num, int *reading)
{
	if (!reading || (sensor_num > SENSOR_NUM_MAX)) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	int val;
	bool is_drive_ready;
	I2C_MSG msg;

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.data[0] = sensor_config[sensor_config_index_map[sensor_num]].offset;
	msg.tx_len = 1;
	msg.rx_len = 4;

	if (!i2c_master_read(&msg, retry)) {
		/* Check SSD drive ready */
		is_drive_ready = ((msg.data[1] & 0x40) == 0 ? true : false);
		if (!is_drive_ready)
			return SENSOR_NOT_ACCESSIBLE;

		/* Check reading value */
		val = msg.data[3];
		if (val == NVMe_NOT_AVAILABLE) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		if (val == NVMe_TMP_SENSOR_FAILURE) {
			return SENSOR_UNSPECIFIED_ERROR;
		}
	} else {
		return SENSOR_FAIL_TO_ACCESS;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = val & 0xFF;
	sval->fraction = 0;

	return SENSOR_READ_SUCCESS;
}

uint8_t nvme_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = nvme_read;
	return SENSOR_INIT_SUCCESS;
}
