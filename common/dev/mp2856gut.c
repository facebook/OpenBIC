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
#include "pmbus.h"
#include "util_pmbus.h"

#define MFR_VR_CONFIG2_VOUT_MODE_BIT BIT(11)
#define MFR_VR_CONFIG2 0x5E

uint8_t mp2856gut_read(uint8_t sensor_num, int *reading)
{
	if (reading == NULL || (sensor_num > SENSOR_NUM_MAX)) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	sensor_val *sval = (sensor_val *)reading;
	I2C_MSG msg;
	memset(sval, 0, sizeof(sensor_val));
	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = cfg->offset;

	if (i2c_master_read(&msg, retry)) {
		return SENSOR_FAIL_TO_ACCESS;
	}

	float val;
	if (cfg->offset == PMBUS_READ_VOUT) {
		val = (msg.data[1] << 8) | msg.data[0];

		msg.bus = cfg->port;
		msg.target_addr = cfg->target_addr;
		msg.tx_len = 1;
		msg.rx_len = 2;
		msg.data[0] = MFR_VR_CONFIG2;

		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}

		uint16_t mfg_vr_config2_data = (msg.data[1] << 8) | msg.data[0];
		bool vout_mode = mfg_vr_config2_data && MFR_VR_CONFIG2_VOUT_MODE_BIT;

		if (vout_mode) {
			val *= 0.00390625;
		} else {
			val *= 0.005;
		}
	} else if (cfg->offset == PMBUS_READ_IOUT || cfg->offset == PMBUS_READ_POUT) {
		uint16_t read_value = (msg.data[1] << 8) | msg.data[0];
		val = slinear11_to_float(read_value);
	} else if (cfg->offset == PMBUS_READ_TEMPERATURE_1) {
		if (msg.data[1] == 0x07) {
			val = -(((~msg.data[0]) & 0xFF) + 1);
		} else {
			val = msg.data[0];
		}
	} else {
		return SENSOR_FAIL_TO_ACCESS;
	}
	sval->integer = val;
	sval->fraction = (val - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t mp2856gut_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = mp2856gut_read;
	return SENSOR_INIT_SUCCESS;
}
