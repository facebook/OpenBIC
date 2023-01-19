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
#include "emc1412.h"

LOG_MODULE_REGISTER(dev_emc1412);

#define EMC1412_DEFAULT_TEMP_MSB_REG 0x00
#define EMC1412_DEFAULT_TEMP_LSB_REG 0x29
#define EMC1412_DEFAULT_RESOLUTION 0.125
#define EMC1412_TEMP_SHIFT_BIT 5

uint8_t emc1412_read(uint8_t sensor_num, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (sensor_num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor 0x%x input parameter is invalid", sensor_num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	int ret = -1;
	uint8_t retry = 5;
	uint16_t val = 0;
	I2C_MSG msg = { 0 };

	sensor_cfg cfg = sensor_config[sensor_config_index_map[sensor_num]];
	if (cfg.offset != EMC1412_READ_TEMP) {
		LOG_ERR("Invalid offset: 0x%x", cfg.offset);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	/* Read temperature msb register */
	msg.bus = cfg.port;
	msg.target_addr = cfg.target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = EMC1412_DEFAULT_TEMP_MSB_REG;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("i2c write fail ret: %d", ret);
		return SENSOR_FAIL_TO_ACCESS;
	}

	val = (msg.data[0] << 8);

	/* Read temperature lsb register */
	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = cfg.port;
	msg.target_addr = cfg.target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = EMC1412_DEFAULT_TEMP_LSB_REG;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("i2c write fail ret: %d", ret);
		return SENSOR_FAIL_TO_ACCESS;
	}

	val = (val | msg.data[0]);

	/* Temperature data is high 11-bit value */
	val = val >> EMC1412_TEMP_SHIFT_BIT;
	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (uint16_t)(val * EMC1412_DEFAULT_RESOLUTION);
	sval->fraction = ((val * EMC1412_DEFAULT_RESOLUTION) - sval->integer) * 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t emc1412_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		LOG_ERR("input sensor number is invalid");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = emc1412_read;
	return SENSOR_INIT_SUCCESS;
}
