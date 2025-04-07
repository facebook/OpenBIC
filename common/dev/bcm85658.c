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
#include "bcm85658.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(bcm85658);

K_MUTEX_DEFINE(bcm85658_mutex);

bool bcm85658_get_fw_version(I2C_MSG *msg, uint8_t *version)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	CHECK_NULL_ARG_WITH_RETURN(version, false);
	uint8_t retry = 3;

	if (k_mutex_lock(&bcm85658_mutex, K_MSEC(BCM85658_MUTEX_LOCK_MS))) {
		LOG_ERR("bcm85658 mutex lock failed");
		return false;
	}

	msg->tx_len = 6;
	msg->rx_len = 9;
	msg->data[0] = SMBUS_PROCESS_BLOCK_CMD;
	msg->data[1] = 0x04; //byte count
	msg->data[2] = BCM85658_VERSION_OFFSET & 0xff; //version lower offset
	msg->data[3] = (BCM85658_VERSION_OFFSET >> 8) & 0xff; 
	msg->data[4] = (BCM85658_VERSION_OFFSET >> 16) & 0xff;
	msg->data[5] = (BCM85658_VERSION_OFFSET >> 24) & 0xff;

	if (i2c_master_read(msg, retry)) {
		LOG_ERR("Failed to get PCIE RETIMER version offset");
		
		if (k_mutex_unlock(&bcm85658_mutex)) {
			LOG_ERR("bcm85658 mutex unlock failed");
		}
		return false;
	}

	for (int i = 0; i < 9; i++)
	{
		LOG_ERR("version data[%d]: 0x%x", i, msg->data[i]);
	}
	

	memcpy(version, &(msg->data[5]), 4);

	if (k_mutex_unlock(&bcm85658_mutex)) {
		LOG_ERR("bcm85658 mutex unlock failed");
		return false;
	}

	return true;
}

uint8_t bcm85658_read_temp(I2C_MSG *msg, double *avg_temperature)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, SENSOR_UNSPECIFIED_ERROR);
	int temp_raw_data = 0;
	uint8_t ret = SENSOR_UNSPECIFIED_ERROR;
	uint8_t retry = 3;

	if (k_mutex_lock(&bcm85658_mutex, K_MSEC(BCM85658_MUTEX_LOCK_MS))) {
		LOG_ERR("bcm85658 mutex lock failed");
		return ret;
	}

	msg->tx_len = 6;
	msg->rx_len = 9;
	msg->data[0] = SMBUS_PROCESS_BLOCK_CMD;
	msg->data[1] = 0x04; //byte count
	msg->data[2] = BCM85658_TEMP_OFFSET_32B & 0xff;
	msg->data[3] = (BCM85658_TEMP_OFFSET_32B >> 8) & 0xff; 
	msg->data[4] = (BCM85658_TEMP_OFFSET_32B >> 16) & 0xff;
	msg->data[5] = (BCM85658_TEMP_OFFSET_32B >> 24) & 0xff;

	if (i2c_master_read(msg, retry)) {
		LOG_ERR("Failed to set RETIMER BCM85658_TEMP_OFFSET_32B");
		goto unlock_exit;
	}

	temp_raw_data = (msg->data[8] << 24) + (msg->data[7] << 16) + (msg->data[6] << 8) +
			   msg->data[5];

	if(IS_BCM85658_TEMP_VAILD(temp_raw_data)) {
		*avg_temperature = (-0.30654 * BCM85658_TEMP_SENSOR_DATA_GET(temp_raw_data)) + 469.27;
		ret = SENSOR_READ_SUCCESS;
	} else {
		LOG_ERR("Temperature data is not valid");
		ret = SENSOR_NOT_ACCESSIBLE;
	}

unlock_exit:
	if (k_mutex_unlock(&bcm85658_mutex)) {
		LOG_ERR("bcm85658 mutex unlock failed");
	}

	return ret;
}

uint8_t bcm85658_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	I2C_MSG msg = { 0 };

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;

	double val = 0;
	uint8_t ret;

	switch (cfg->offset) {
	case BCM85658_TEMP_OFFSET:
		ret = bcm85658_read_temp(&msg, &val);
		if (ret != SENSOR_READ_SUCCESS) {
			return ret;
		}
		break;
	default:
		LOG_ERR("Invalid sensor 0x%x offset 0x%x", cfg->num, cfg->offset);
		return SENSOR_NOT_FOUND;
	}

	sensor_val *sval = (sensor_val *)reading;
	memset(sval, 0, sizeof(*sval));

	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t bcm85658_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("bcm85658_init fail 0x%x offset 0x%x", cfg->num, cfg->offset);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	
	cfg->read = bcm85658_read;
	return SENSOR_INIT_SUCCESS;
}
