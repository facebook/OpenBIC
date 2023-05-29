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

uint8_t g788p81u_read(sensor_cfg *cfg, int *reading)
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

uint8_t g788p81u_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	g788p81u_init_arg *init_arg = (g788p81u_init_arg *)cfg->init_args;
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
	msg.data[0] = G788P81U_REMOTE_THIGH_LIMIT_OFFSET;
	msg.data[1] = init_arg->remote_T_high_limit & 0xFF;
	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Failed to set Remote T_high limit, ret: %d", ret);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	memset(&msg, 0, sizeof(msg));
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = G788P81U_ALERT_MODE_OFFSET;
	msg.data[1] = init_arg->alert_mode & 0xFF;
	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Failed to set alert mode, ret: %d", ret);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	memset(&msg, 0, sizeof(msg));
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = G788P81U_ALERT_MASK_OFFSET;
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
	msg.data[0] = G788P81U_CONFIGURATION_OFFSET;
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
	msg.data[0] = G788P81U_REMOTE_TEMP_THERM_LIMIT_OFFSET;
	msg.data[1] = init_arg->remote_temp_therm_limit & 0xFF;
	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Failed to set remote temperature THERM limit, ret: %d", ret);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	memset(&msg, 0, sizeof(msg));
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = G788P81U_LOCAL_TEMP_THERM_LIMIT_OFFSET;
	msg.data[1] = init_arg->local_temp_therm_limit & 0xFF;
	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Failed to set local temperature THERM limit, ret: %d", ret);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	init_arg->is_init = true;

skip_init:
	cfg->read = g788p81u_read;
	return SENSOR_INIT_SUCCESS;
}
