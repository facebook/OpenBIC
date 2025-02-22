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
#include "emc1413.h"

LOG_MODULE_REGISTER(dev_emc1413);

#define EMC1413_STATUS_REG 0x02
#define EMC1413_EXTERNAL_DIODE_FAULT_REG 0x1B
#define EMC1413_HIGH_LIMIT_STATUS_REG 0x35
#define EMC1413_LOW_LIMIT_STATUS_REG 0x36
#define EMC1413_THERM_LIMIT_STATUS_REG 0x37
#define EMC1413_DEFAULT_RESOLUTION 0.125
#define EMC1413_TEMP_SHIFT_BIT 5

typedef struct _temp_mapping_register {
	uint8_t threshold_index;
	uint8_t high_byte;
	uint8_t low_byte;
} temp_mapping_register;

temp_mapping_register emc1413_temp_register_map[] = {
	{ LOCAL_HIGH_LIMIT, EMC1413_INTERNAL_HIGH_LIMIT_REG, 0xFF },
	{ LOCAL_LOW_LIMIT, EMC1413_INTERNAL_LOW_LIMIT_REG, 0xFF },
	{ REMOTE_1_HIGH_LIMIT, EMC1413_EXTERNAL_1_HIGH_LIMIT_HIGH_BYTE_REG,
	  EMC1413_EXTERNAL_1_HIGH_LIMIT_LOW_BYTE_REG },
	{ REMOTE_1_LOW_LIMIT, EMC1413_EXTERNAL_1_LOW_LIMIT_HIGH_BYTE_REG,
	  EMC1413_EXTERNAL_1_LOW_LIMIT_LOW_BYTE_REG },
	{ REMOTE_2_HIGH_LIMIT, EMC1413_EXTERNAL_2_HIGH_LIMIT_HIGH_BYTE_REG,
	  EMC1413_EXTERNAL_2_HIGH_LIMIT_LOW_BYTE_REG },
	{ REMOTE_2_LOW_LIMIT, EMC1413_EXTERNAL_2_LOW_LIMIT_HIGH_BYTE_REG,
	  EMC1413_EXTERNAL_2_LOW_LIMIT_LOW_BYTE_REG },
	{ LOCAL_THERM_LIMIT, EMC1413_INTERNAL_THERM_LIMIT_REG, 0xFF },
	{ REMOTE_1_THERM_LIMIT, EMC1413_EXTERNAL_1_THERM_LIMIT_REG, 0xFF },
	{ REMOTE_2_THERM_LIMIT, EMC1413_EXTERNAL_2_THERM_LIMIT_REG, 0xFF },
};

bool get_emc1413_two_byte_limit(sensor_cfg *cfg, uint8_t temp_threshold_index,
				uint32_t *millidegree_celsius)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millidegree_celsius, false);

	uint8_t register_high = emc1413_temp_register_map[temp_threshold_index].high_byte;
	uint8_t register_low = emc1413_temp_register_map[temp_threshold_index].low_byte;

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

	float limit_high_byte_val = (float)msg.data[0] * 1000;

	msg.data[0] = register_low;
	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Failed to write TMP431 register(0x%x)", temp_threshold_index);
		return false;
	}

	float limit_low_byte_val = (float)(msg.data[0] >> 5) * 125;

	*millidegree_celsius = (uint32_t)limit_high_byte_val + limit_low_byte_val;

	return true;
}

bool get_emc1413_one_byte_limit(sensor_cfg *cfg, uint8_t temp_threshold_index,
				uint32_t *millidegree_celsius)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millidegree_celsius, false);

	uint8_t register_high = emc1413_temp_register_map[temp_threshold_index].high_byte;
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

bool emc1413_get_temp_threshold(sensor_cfg *cfg, uint8_t temp_threshold_index,
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
	case REMOTE_1_HIGH_LIMIT:
	case REMOTE_1_LOW_LIMIT:
	case REMOTE_2_HIGH_LIMIT:
	case REMOTE_2_LOW_LIMIT:
		if (!get_emc1413_two_byte_limit(cfg, temp_threshold_index, millidegree_celsius)) {
			return false;
		} else {
			return true;
		}
		break;
	case LOCAL_HIGH_LIMIT:
	case LOCAL_LOW_LIMIT:
	case LOCAL_THERM_LIMIT:
	case REMOTE_1_THERM_LIMIT:
	case REMOTE_2_THERM_LIMIT:
		if (!get_emc1413_one_byte_limit(cfg, temp_threshold_index, millidegree_celsius)) {
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

bool set_emc1413_two_byte_limit(sensor_cfg *cfg, uint8_t temp_threshold_index,
				uint32_t *millidegree_celsius)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millidegree_celsius, false);

	uint8_t register_high = emc1413_temp_register_map[temp_threshold_index].high_byte;
	uint8_t register_low = emc1413_temp_register_map[temp_threshold_index].low_byte;
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

	msg.data[0] = register_low;
	msg.data[1] = (uint8_t)(((*millidegree_celsius % 1000) / 125) << 5);
	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("Failed to write TMP431 register(0x%x)", temp_threshold_index);
		return false;
	}

	return true;
}

bool set_emc1413_one_byte_limit(sensor_cfg *cfg, uint8_t temp_threshold_index,
				uint32_t *millidegree_celsius)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millidegree_celsius, false);

	uint8_t register_high = emc1413_temp_register_map[temp_threshold_index].high_byte;
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

bool emc1413_set_temp_threshold(sensor_cfg *cfg, uint8_t temp_threshold_index,
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
		if (!set_emc1413_two_byte_limit(cfg, temp_threshold_index, millidegree_celsius)) {
			return false;
		} else {
			return true;
		}
		break;
	case LOCAL_THERM_LIMIT:
	case REMOTE_1_THERM_LIMIT:
	case REMOTE_2_THERM_LIMIT:
		if (!set_emc1413_one_byte_limit(cfg, temp_threshold_index, millidegree_celsius)) {
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

uint8_t emc1413_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5, temperature_high_byte = 0x00, temperature_low_byte = 0x00;
	uint16_t val = 0;
	I2C_MSG msg = { 0 };

	/* Read temperature msb register */
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	uint8_t offset = cfg->offset;

	if (offset == EMC1413_LOCAL_TEMPERATRUE) {
		msg.data[0] = INTERNAL_DIODE_HIGH_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_high_byte = msg.data[0];
		msg.data[0] = INTERNAL_DIODE_LOW_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_low_byte = msg.data[0];
	} else if (offset == EMC1413_REMOTE_TEMPERATRUE_1) {
		msg.data[0] = EXTERNAL_DIODE_1_HIGH_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_high_byte = msg.data[0];
		msg.data[0] = EXTERNAL_DIODE_1_LOW_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_low_byte = msg.data[0];
	} else if (offset == EMC1413_REMOTE_TEMPERATRUE_2) {
		msg.data[0] = EXTERNAL_DIODE_2_HIGH_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_high_byte = msg.data[0];
		msg.data[0] = EXTERNAL_DIODE_2_LOW_BYTE;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		temperature_low_byte = msg.data[0];
	} else {
		LOG_ERR("Unknown offset(%d)", offset);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	val = (temperature_high_byte << 8) | temperature_low_byte;

	/* Temperature data is high 11-bit value */
	val = val >> EMC1413_TEMP_SHIFT_BIT;
	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (uint16_t)(val * EMC1413_DEFAULT_RESOLUTION);
	sval->fraction = ((val * EMC1413_DEFAULT_RESOLUTION) - sval->integer) * 1000;
	return SENSOR_READ_SUCCESS;
}

bool emc1413_get_temp_status(sensor_cfg *cfg, uint8_t *temp_status)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = cfg->port;
	i2c_msg.target_addr = cfg->target_addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = EMC1413_STATUS_REG;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("TMP[0x%x] get status reg[0x%d] failed.", cfg->num, EMC1413_STATUS_REG);
		return false;
	}
	*temp_status = i2c_msg.data[0];

	return true;
}

bool emc1413_clear_temp_status(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = cfg->port;
	i2c_msg.target_addr = cfg->target_addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = EMC1413_EXTERNAL_DIODE_FAULT_REG;
	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("TMP[0x%x] clear status reg[0x%d] failed.", cfg->num,
			EMC1413_EXTERNAL_DIODE_FAULT_REG);
		return false;
	}
	i2c_msg.data[0] = EMC1413_HIGH_LIMIT_STATUS_REG;
	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("TMP[0x%x] clear status reg[0x%d] failed.", cfg->num,
			EMC1413_HIGH_LIMIT_STATUS_REG);
		return false;
	}
	i2c_msg.data[0] = EMC1413_LOW_LIMIT_STATUS_REG;
	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("TMP[0x%x] clear status reg[0x%d] failed.", cfg->num,
			EMC1413_LOW_LIMIT_STATUS_REG);
		return false;
	}
	i2c_msg.data[0] = EMC1413_THERM_LIMIT_STATUS_REG;
	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("TMP[0x%x] clear status reg[0x%d] failed.", cfg->num,
			EMC1413_THERM_LIMIT_STATUS_REG);
		return false;
	}

	return true;
}

uint8_t emc1413_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = emc1413_read;
	return SENSOR_INIT_SUCCESS;
}
