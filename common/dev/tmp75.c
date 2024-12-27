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
#include "libutil.h"
#include <logging/log.h>
#include "tmp75.h"

LOG_MODULE_REGISTER(tmp75);

bool get_tmp75_two_byte_limit(sensor_cfg *cfg, uint8_t temp_threshold_index,
			      uint32_t *millidegree_celsius)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millidegree_celsius, false);

	uint8_t retry = 5;
	I2C_MSG msg = { 0 };
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = ((temp_threshold_index == LOCAL_HIGH_LIMIT) ? TMP75_LOCAL_HIGH_LIMIT_REG :
								    TMP75_LOCAL_LOW_LIMIT_REG);

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Failed to write TMP431 register(0x%x)", temp_threshold_index);
		return false;
	}

	float limit_byte_1_val = (float)msg.data[0] * 1000.0;
	float limit_byte_2_val = (float)(msg.data[1] >> 4) * 0.0625;

	*millidegree_celsius = (uint32_t)limit_byte_1_val + limit_byte_2_val;

	return true;
}

bool tmp75_get_temp_threshold(sensor_cfg *cfg, uint8_t temp_threshold_index,
			      uint32_t *millidegree_celsius)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millidegree_celsius, false);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return false;
	}

	if ((temp_threshold_index != LOCAL_HIGH_LIMIT) &&
	    (temp_threshold_index != LOCAL_LOW_LIMIT)) {
		LOG_ERR("temp_threshold_index: 0x%x is invalid", temp_threshold_index);
		return false;
	}

	if (!get_tmp75_two_byte_limit(cfg, temp_threshold_index, millidegree_celsius)) {
		return false;
	}

	return true;
}

bool set_tmp75_two_byte_limit(sensor_cfg *cfg, uint8_t temp_threshold_index,
			      uint32_t *millidegree_celsius)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millidegree_celsius, false);

	uint8_t retry = 5;
	I2C_MSG msg = { 0 };
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = ((temp_threshold_index == LOCAL_HIGH_LIMIT) ? TMP75_LOCAL_HIGH_LIMIT_REG :
								    TMP75_LOCAL_LOW_LIMIT_REG);
	msg.data[1] = (uint8_t)(*millidegree_celsius / 1000.0);
	uint32_t remainder = *millidegree_celsius % 1000;
	uint8_t low_byte_val = (uint8_t)((remainder * 16) / 1000);
	msg.data[2] = (low_byte_val << 4);

	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("Failed to write TMP431 register(0x%x)", temp_threshold_index);
		return false;
	}

	return true;
}

bool tmp75_set_temp_threshold(sensor_cfg *cfg, uint8_t temp_threshold_index,
			      uint32_t *millidegree_celsius)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millidegree_celsius, false);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return false;
	}

	if ((temp_threshold_index != LOCAL_HIGH_LIMIT) &&
	    (temp_threshold_index != LOCAL_LOW_LIMIT)) {
		LOG_ERR("temp_threshold_index: 0x%x is invalid", temp_threshold_index);
		return false;
	}

	if (!set_tmp75_two_byte_limit(cfg, temp_threshold_index, millidegree_celsius)) {
		return false;
	}

	return true;
}

uint8_t tmp75_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = cfg->offset;

	if (i2c_master_read(&msg, retry))
		return SENSOR_FAIL_TO_ACCESS;

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int8_t)msg.data[0];
	sval->fraction = 0;
	return SENSOR_READ_SUCCESS;
}

uint8_t tmp75_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = tmp75_read;
	return SENSOR_INIT_SUCCESS;
}
