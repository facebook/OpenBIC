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

#include "tmp431.h"

#include <stdio.h>
#include <stdlib.h>

#include "sensor.h"
#include "hal_i2c.h"
#include "libutil.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(dev_tmp431);

#define TMP432_STATUS_REG 0x02
#define TMP432_OPEN_STATUS_REG 0x1B
#define TMP432_HIGH_LIMIT_STATUS_REG 0x35
#define TMP432_LOW_LIMIT_STATUS_REG 0x36
#define I2C_DATA_SIZE 5
#define RANGE_0_127 0
#define RANGE_m55_150 1

static uint8_t temperature_range = 0xFF;

typedef struct _temp_mapping_register {
	uint8_t threshold_index;
	uint8_t read_byte;
	uint8_t write_byte;
} temp_mapping_register;

temp_mapping_register tmp432_temp_register_map[] = {
	{ LOCAL_HIGH_LIMIT, TMP432_LOCAL_HIGH_LIMIT_HIGH_BYTE_READ_REG,
	  TMP432_LOCAL_HIGH_LIMIT_HIGH_BYTE_WRITE_REG },
	{ LOCAL_LOW_LIMIT, TMP432_LOCAL_LOW_LIMIT_HIGH_BYTE_READ_REG,
	  TMP432_LOCAL_LOW_LIMIT_HIGH_BYTE_WRITE_REG },
	{ REMOTE_1_HIGH_LIMIT, TMP432_REMOTE_1_HIGH_LIMIT_HIGH_BYTE_READ_REG,
	  TMP432_REMOTE_1_HIGH_LIMIT_HIGH_BYTE_WRITE_REG },
	{ REMOTE_1_LOW_LIMIT, TMP432_REMOTE_1_LOW_LIMIT_HIGH_BYTE_READ_REG,
	  TMP432_REMOTE_1_LOW_LIMIT_HIGH_BYTE_WRITE_REG },
	{ REMOTE_2_HIGH_LIMIT, TMP432_REMOTE_2_HIGH_LIMIT_HIGH_BYTE_READ_REG,
	  TMP432_REMOTE_2_HIGH_LIMIT_HIGH_BYTE_WRITE_REG },
	{ REMOTE_2_LOW_LIMIT, TMP432_REMOTE_2_LOW_LIMIT_HIGH_BYTE_READ_REG,
	  TMP432_REMOTE_2_LOW_LIMIT_HIGH_BYTE_WRITE_REG },
	{ LOCAL_THERM_LIMIT, TMP432_LOCAL_THERM_LIMIT_REG, TMP432_LOCAL_THERM_LIMIT_REG },
	{ REMOTE_1_THERM_LIMIT, TMP432_REMOTE_1_THERM_LIMIT_REG, TMP432_REMOTE_1_THERM_LIMIT_REG },
	{ REMOTE_2_THERM_LIMIT, TMP432_REMOTE_2_THERM_LIMIT_REG, TMP432_REMOTE_2_THERM_LIMIT_REG },
};

bool get_tmp432_two_byte_limit(sensor_cfg *cfg, uint8_t temp_threshold_index,
			       uint32_t *millidegree_celsius)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millidegree_celsius, false);

	uint8_t register_high = tmp432_temp_register_map[temp_threshold_index].read_byte;

	uint8_t retry = 5;
	I2C_MSG msg = { 0 };
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = register_high;

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Failed to write TMP431 register(0x%x)", temp_threshold_index);
		return false;
	}

	uint32_t limit_high_byte_val = (uint32_t)msg.data[0] * 1000;
	uint32_t limit_low_byte_val = ((msg.data[1] >> 4) * 625) / 10;
	*millidegree_celsius = limit_high_byte_val + limit_low_byte_val;

	return true;
}

bool get_tmp432_one_byte_limit(sensor_cfg *cfg, uint8_t temp_threshold_index,
			       uint32_t *millidegree_celsius)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millidegree_celsius, false);

	uint8_t register_high = tmp432_temp_register_map[temp_threshold_index].read_byte;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = register_high;

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Failed to write TMP431 register(0x%x)", temp_threshold_index);
		return false;
	}

	uint32_t limit_high_byte_val = (uint32_t)msg.data[0] * 1000;

	*millidegree_celsius = limit_high_byte_val;

	return true;
}

bool tmp432_get_temp_threshold(sensor_cfg *cfg, uint8_t temp_threshold_index,
			       uint32_t *millidegree_celsius)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millidegree_celsius, false);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return false;
	}

	if (temp_threshold_index >= TEMP_THRESHOLD_TYPE_E_MAX) {
		LOG_ERR("temp_threshold_index: 0x%x is invalid", temp_threshold_index);
		return false;
	}

	switch (temp_threshold_index) {
	case LOCAL_HIGH_LIMIT:
	case LOCAL_LOW_LIMIT:
	case REMOTE_1_HIGH_LIMIT:
	case REMOTE_1_LOW_LIMIT:
	case REMOTE_2_HIGH_LIMIT:
	case REMOTE_2_LOW_LIMIT:
		if (!get_tmp432_two_byte_limit(cfg, temp_threshold_index, millidegree_celsius)) {
			return false;
		} else {
			return true;
		}
		break;
	case LOCAL_THERM_LIMIT:
	case REMOTE_1_THERM_LIMIT:
	case REMOTE_2_THERM_LIMIT:
		if (!get_tmp432_one_byte_limit(cfg, temp_threshold_index, millidegree_celsius)) {
			return false;
		} else {
			return true;
		}
		break;
	default:
		LOG_ERR("temp_threshold_index: 0x%x is invalid", temp_threshold_index);
		return false;
	}

	return false;
}

bool set_tmp432_two_byte_limit(sensor_cfg *cfg, uint8_t temp_threshold_index,
			       uint32_t *millidegree_celsius)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millidegree_celsius, false);

	uint8_t register_high = tmp432_temp_register_map[temp_threshold_index].write_byte;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 3;
	msg.data[0] = register_high;
	msg.data[1] = (uint8_t)(*millidegree_celsius / 1000);

	uint32_t remainder = *millidegree_celsius % 1000;
	uint8_t low_byte_val = (uint8_t)((remainder * 16) / 1000);
	msg.data[2] = (low_byte_val << 4);

	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("Failed to write TMP431 register(0x%x)", temp_threshold_index);
		return false;
	}

	return true;
}

bool set_tmp432_one_byte_limit(sensor_cfg *cfg, uint8_t temp_threshold_index,
			       uint32_t *millidegree_celsius)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millidegree_celsius, false);

	uint8_t register_high = tmp432_temp_register_map[temp_threshold_index].write_byte;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = register_high;
	msg.data[1] = (uint8_t)(*millidegree_celsius / 1000);
	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("Failed to write TMP431 register(0x%x)", temp_threshold_index);
		return false;
	}

	return true;
}

bool tmp432_set_temp_threshold(sensor_cfg *cfg, uint8_t temp_threshold_index,
			       uint32_t *millidegree_celsius)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millidegree_celsius, false);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return false;
	}

	if (temp_threshold_index >= TEMP_THRESHOLD_TYPE_E_MAX) {
		LOG_ERR("temp_threshold_index: 0x%x is invalid", temp_threshold_index);
		return false;
	}

	switch (temp_threshold_index) {
	case LOCAL_HIGH_LIMIT:
	case LOCAL_LOW_LIMIT:
	case REMOTE_1_HIGH_LIMIT:
	case REMOTE_1_LOW_LIMIT:
	case REMOTE_2_HIGH_LIMIT:
	case REMOTE_2_LOW_LIMIT:
		if (!set_tmp432_two_byte_limit(cfg, temp_threshold_index, millidegree_celsius)) {
			return false;
		} else {
			return true;
		}
		break;
	case LOCAL_THERM_LIMIT:
	case REMOTE_1_THERM_LIMIT:
	case REMOTE_2_THERM_LIMIT:
		if (!set_tmp432_one_byte_limit(cfg, temp_threshold_index, millidegree_celsius)) {
			return false;
		} else {
			return true;
		}
		break;
	default:
		LOG_ERR("temp_threshold_index: 0x%x is invalid", temp_threshold_index);
		return false;
	}

	return false;
}

uint8_t tmp431_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5, temperature_high_byte = 0xFF, temperature_low_byte = 0xFF;
	I2C_MSG msg = { 0 };

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	uint8_t offset = cfg->offset;

	if (offset == TMP431_LOCAL_TEMPERATRUE) {
		msg.data[0] = LOCAL_TEMPERATURE_HIGH_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_high_byte = msg.data[0];
		msg.data[0] = LOCAL_TEMPERATURE_LOW_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_low_byte = msg.data[0];
	} else if ((offset == TMP431_REMOTE_TEMPERATRUE) ||
		   (offset == TMP432_REMOTE_TEMPERATRUE_1)) {
		msg.data[0] = REMOTE_TEMPERATURE_HIGH_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_high_byte = msg.data[0];
		msg.data[0] = REMOTE_TEMPERATURE_LOW_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_low_byte = msg.data[0];
	} else if (offset == TMP432_REMOTE_TEMPERATRUE_2) {
		msg.data[0] = REMOTE_TEMPERATURE_2_HIGH_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_high_byte = msg.data[0];
		msg.data[0] = REMOTE_TEMPERATURE_2_LOW_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_low_byte = msg.data[0];
	} else {
		LOG_ERR("Unknown offset(%d)", offset);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	float val;
	if (temperature_range == RANGE_0_127) {
		val = temperature_high_byte + ((temperature_low_byte >> 4) * 0.0625);
	} else if (temperature_range == RANGE_m55_150) {
		val = (temperature_high_byte - 64) + ((temperature_low_byte >> 4) * 0.0625);
	} else {
		LOG_ERR("Unknown temperature range(%d)", temperature_range);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int32_t)val;
	sval->fraction = (int32_t)(val * 1000) % 1000;
	return SENSOR_READ_SUCCESS;
}

bool tmp432_get_temp_status(sensor_cfg *cfg, uint8_t *temp_status)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = cfg->port;
	i2c_msg.target_addr = cfg->target_addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = TMP432_STATUS_REG;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("TMP[0x%x] get status reg[0x%d] failed.", cfg->num, TMP432_STATUS_REG);
		return false;
	}
	*temp_status = i2c_msg.data[0];

	return true;
}

bool tmp432_clear_temp_status(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = cfg->port;
	i2c_msg.target_addr = cfg->target_addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = TMP432_OPEN_STATUS_REG;
	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("TMP[0x%x] clear status reg[0x%d] failed.", cfg->num,
			TMP432_OPEN_STATUS_REG);
		return false;
	}
	i2c_msg.data[0] = TMP432_HIGH_LIMIT_STATUS_REG;
	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("TMP[0x%x] clear status reg[0x%d] failed.", cfg->num,
			TMP432_HIGH_LIMIT_STATUS_REG);
		return false;
	}
	i2c_msg.data[0] = TMP432_LOW_LIMIT_STATUS_REG;
	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("TMP[0x%x] clear status reg[0x%d] failed.", cfg->num,
			TMP432_LOW_LIMIT_STATUS_REG);
		return false;
	}

	return true;
}

uint8_t tmp431_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	I2C_MSG msg;

	char *data = (uint8_t *)malloc(I2C_DATA_SIZE * sizeof(uint8_t));
	if (data == NULL) {
		LOG_ERR("Memory allocation failed!");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	// Get the temperature range from chip
	uint8_t bus = cfg->port;
	uint8_t target_addr = cfg->target_addr;
	uint8_t tx_len = 1;
	uint8_t rx_len = 1;

	data[0] = CONFIGURATION_REGISTER_1;
	msg = construct_i2c_message(bus, target_addr, tx_len, data, rx_len);
	if (i2c_master_read(&msg, retry) != 0) {
		LOG_ERR("Failed to read TMP431 register(0x%x)", data[0]);
		goto cleanup;
	}
	temperature_range = (msg.data[0] & BIT(2)) == BIT(2) ? RANGE_m55_150 : RANGE_0_127;

cleanup:
	SAFE_FREE(data);

	cfg->read = tmp431_read;
	return SENSOR_INIT_SUCCESS;
}
