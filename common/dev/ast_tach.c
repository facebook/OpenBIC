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

#include <drivers/pwm.h>
#include <drivers/sensor.h>
#include "sensor.h"
#include <logging/log.h>

#include "ast_tach.h"

LOG_MODULE_REGISTER(dev_ast_tach);

#undef DT_DRV_COMPAT
#if DT_HAS_COMPAT_STATUS_OKAY(aspeed_tach)
#define DT_DRV_COMPAT aspeed_tach
#endif

static const struct device *dev_tach[TACH_MAX_NUM];

static void init_tach_dev(void)
{
	for (uint8_t i = TACH_PORT0; i < TACH_MAX_NUM; i++) {
		char device_name[6];
		snprintf(device_name, sizeof(device_name), "FAN%d", i);
		dev_tach[i] = device_get_binding(device_name);
	}
}

static uint8_t get_fan_rpm(uint8_t port, int32_t *val)
{
	CHECK_NULL_ARG_WITH_RETURN(val, SENSOR_UNSPECIFIED_ERROR);
	struct sensor_value sensor_value;
	int ret = 0;

	if (port >= TACH_MAX_NUM) {
		LOG_ERR("port %d out of range", port);
		return SENSOR_PARAMETER_NOT_VALID;
	}

	if (dev_tach[port] == NULL) {
		LOG_ERR("tach dev %d NULL", port);
		return SENSOR_UNAVAILABLE;
	}

	ret = sensor_sample_fetch(dev_tach[port]);
	if (ret < 0) {
		LOG_ERR("Failed to read FAN%d due to sensor_sample_fetch failed, ret: %d", port,
			ret);
		return SENSOR_FAIL_TO_ACCESS;
	}

	ret = sensor_channel_get(dev_tach[port], SENSOR_CHAN_RPM, &sensor_value);
	if (ret < 0) {
		LOG_ERR("Failed to read FAN%d due to sensor_channel_get failed, ret: %d", port,
			ret);
		return SENSOR_FAIL_TO_ACCESS;
	}

	*val = sensor_value.val1;
	return SENSOR_READ_SUCCESS;
}

uint8_t ast_tach_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_DBG("Invalid sensor number");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	int32_t ret_val = 0;
	uint8_t ret = SENSOR_UNSPECIFIED_ERROR;

	switch (cfg->offset) {
	case AST_TACH_RPM:
		ret = get_fan_rpm(cfg->port, &ret_val);
		if (ret != SENSOR_READ_SUCCESS)
			LOG_ERR("get fan %d rpm fail", cfg->port);
		break;
	default:
		LOG_ERR("fan %d method undefined", cfg->port);
		break;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int16_t)ret_val;
	sval->fraction = 0;

	return ret;
}

uint8_t ast_tach_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_DBG("Invalid sensor number");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	ast_tach_init_arg *init_args = (ast_tach_init_arg *)cfg->init_args;
	if (init_args->is_init)
		goto skip_init;

	init_tach_dev();

	init_args->is_init = true;

skip_init:
	cfg->read = ast_tach_read;

	return SENSOR_INIT_SUCCESS;
}