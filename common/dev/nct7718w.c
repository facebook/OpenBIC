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

LOG_MODULE_REGISTER(dev_nct7718w);

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
	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	nct7718w_init_arg *init_arg = (nct7718w_init_arg *)cfg->init_args;
	if (init_arg == NULL) {
		LOG_DBG("Input initial pointer is NULL, skip initialization.");
		goto skip_init;
	}

	if (init_arg->is_init == true) {
		LOG_DBG("Already initialized.");
		goto skip_init;
	}

	int ret = 0;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = NCT7718W_RT1_HIGH_ALERT_TEMP_OFFSET;
	msg.data[1] = init_arg->rt1_high_alert_temp & 0xFF;
	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Failed to set RT1 High Alert temperature, ret: %d", ret);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	memset(&msg, 0, sizeof(msg));
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = NCT7718W_RT_FILTER_ALERT_MODE_OFFSET;
	msg.data[1] = init_arg->rt_filter_alert_mode & 0xFF;
	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Failed to set RT filter and Alert mode, ret: %d", ret);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	memset(&msg, 0, sizeof(msg));
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = NCT7718W_ALERT_MASK_OFFSET;
	msg.data[1] = init_arg->alert_mask & 0xFF;
	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Failed to set alert mask, ret: %d", ret);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	memset(&msg, 0, sizeof(msg));
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = NCT7718W_CONFIGURATION_OFFSET;
	msg.data[1] = init_arg->configuration & 0xFF;
	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Failed to set configuration, ret: %d", ret);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	memset(&msg, 0, sizeof(msg));
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = NCT7718W_RT1_CRITICAL_TEMP_OFFSET;
	msg.data[1] = init_arg->rt1_critical_temperature & 0xFF;
	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Failed to set RT1 critical temperature, ret: %d", ret);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	memset(&msg, 0, sizeof(msg));
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = NCT7718W_LT_CRITICAL_TEMP_OFFSET;
	msg.data[1] = init_arg->lt_critical_temperature & 0xFF;
	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Failed to set local critical temperature, ret: %d", ret);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	init_arg->is_init = true;

skip_init:
	cfg->read = nct7718w_read;
	return SENSOR_INIT_SUCCESS;
}
