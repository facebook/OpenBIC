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
#include <stdint.h>
#include <string.h>
#include <logging/log.h>
#include "libutil.h"
#include "sensor.h"
#include "ltc2991.h"
#include "hal_i2c.h"

LOG_MODULE_REGISTER(dev_ltc2991);

int ltc2991_read_optional_to_register(uint8_t read_optional, uint8_t *msb_register,
				      uint8_t *lsb_register, float *parameter, uint8_t *channel)
{
	CHECK_NULL_ARG_WITH_RETURN(msb_register, -1);
	CHECK_NULL_ARG_WITH_RETURN(lsb_register, -1);
	CHECK_NULL_ARG_WITH_RETURN(parameter, -1);
	CHECK_NULL_ARG_WITH_RETURN(channel, -1);

	switch (read_optional) {
	case LTC2991_READ_V1_VOLTAGE:
	case LTC2991_READ_V1_V2_TEMPERATURE:
		*msb_register = LTC2991_V1_MSB_REG;
		*lsb_register = LTC2991_V1_LSB_REG;
		*channel = LTC2991_ENABLE_CHANNEL_V1_V2;
		break;
	case LTC2991_READ_V2_VOLTAGE:
		*msb_register = LTC2991_V2_MSB_REG;
		*lsb_register = LTC2991_V2_LSB_REG;
		*channel = LTC2991_ENABLE_CHANNEL_V1_V2;
		break;
	case LTC2991_READ_V3_VOLTAGE:
	case LTC2991_READ_V3_V4_TEMPERATURE:
		*msb_register = LTC2991_V3_MSB_REG;
		*lsb_register = LTC2991_V3_LSB_REG;
		*channel = LTC2991_ENABLE_CHANNEL_V3_V4;
		break;
	case LTC2991_READ_V4_VOLTAGE:
		*msb_register = LTC2991_V4_MSB_REG;
		*lsb_register = LTC2991_V4_LSB_REG;
		*channel = LTC2991_ENABLE_CHANNEL_V3_V4;
		break;
	case LTC2991_READ_V5_VOLTAGE:
	case LTC2991_READ_V5_V6_TEMPERATURE:
		*msb_register = LTC2991_V5_MSB_REG;
		*lsb_register = LTC2991_V5_LSB_REG;
		*channel = LTC2991_ENABLE_CHANNEL_V5_V6;
		break;
	case LTC2991_READ_V6_VOLTAGE:
		*msb_register = LTC2991_V6_MSB_REG;
		*lsb_register = LTC2991_V6_LSB_REG;
		*channel = LTC2991_ENABLE_CHANNEL_V5_V6;
		break;
	case LTC2991_READ_V7_VOLTAGE:
	case LTC2991_READ_V7_V8_TEMPERATURE:
		*msb_register = LTC2991_V7_MSB_REG;
		*lsb_register = LTC2991_V7_LSB_REG;
		*channel = LTC2991_ENABLE_CHANNEL_V7_V8;
		break;
	case LTC2991_READ_V8_VOLTAGE:
		*msb_register = LTC2991_V8_MSB_REG;
		*lsb_register = LTC2991_V8_LSB_REG;
		*channel = LTC2991_ENABLE_CHANNEL_V7_V8;
		break;
	default:
		LOG_ERR("Read optional is invalid, read optional: %d", read_optional);
		return -1;
	}

	if ((read_optional & LTC2991_READ_TEMPERATURE) == 0) {
		*parameter = LTC2991_VOLTAGE_LSB;
	} else {
		*parameter = LTC2991_TEMPERATURE_LSB;
	}

	return 0;
}

uint8_t ltc2991_read(uint8_t sensor_num, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (sensor_num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor 0x%x input parameter is invalid", sensor_num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	sensor_cfg cfg = sensor_config[sensor_config_index_map[sensor_num]];

	int ret = 0;
	uint8_t retry = 5;
	uint8_t channel = 0;
	uint8_t msb_register = 0;
	uint8_t lsb_register = 0;
	uint8_t read_optional = cfg.offset;
	uint16_t temp = 0;
	int16_t val = 0;
	float parameter = 0;
	I2C_MSG msg = { 0 };

	ret = ltc2991_read_optional_to_register(read_optional, &msb_register, &lsb_register,
						&parameter, &channel);
	if (ret != 0) {
		return SENSOR_PARAMETER_NOT_VALID;
	}

	/* Enable voltage channel */
	msg.bus = cfg.port;
	msg.target_addr = cfg.target_addr;
	msg.tx_len = 2;
	msg.data[0] = LTC2991_ENABLE_CHANNEL_REG;
	msg.data[1] = channel;

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("i2c write fail ret: %d", ret);
		return false;
	}

	k_msleep(LTC2991_DATA_NOT_READY_DELAY_MS);

	/* Read MSB register */
	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = cfg.port;
	msg.target_addr = cfg.target_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = msb_register;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("i2c read fail ret: %d", ret);
		return SENSOR_FAIL_TO_ACCESS;
	}

	if ((msg.data[0] & LTC2991_DATA_VALID_BIT) == 0) {
		LOG_ERR("MSB data invalid");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	/* Skip data valid bit */
	temp = ((msg.data[0] & 0x7F) << 8);

	/* Read LSB register */
	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = cfg.port;
	msg.target_addr = cfg.target_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = lsb_register;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("i2c read fail ret: %d", ret);
		return SENSOR_FAIL_TO_ACCESS;
	}

	temp = (temp | msg.data[0]);
	if ((temp & (LTC2991_DATA_SIGN_BIT << 8)) == 0) {
		val = temp;
	} else {
		/* Convert to two's component value */
		/* Skip data sign bit */
		temp = (temp & 0x3FFF);
		val = -(~(temp) + 1);
	}

	if (cfg.arg1 == 0) {
		LOG_ERR("sensor config setting error, arg1 is 0");
		return SENSOR_PARAMETER_NOT_VALID;
	}

	parameter = parameter * cfg.arg0 / cfg.arg1;

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = val * parameter;
	sval->fraction = ((val * parameter) - sval->integer) * 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t ltc2991_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		LOG_ERR("input sensor number is invalid");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	ltc2991_init_arg *init_arg =
		(ltc2991_init_arg *)sensor_config[sensor_config_index_map[sensor_num]].init_args;
	if (init_arg == NULL) {
		LOG_ERR("input initial pointer is NULL");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	if (init_arg->is_init != true) {
		int ret = 0, retry = 5;
		I2C_MSG msg = { 0 };
		sensor_cfg cfg = sensor_config[sensor_config_index_map[sensor_num]];

		/* Set V1~V4 control register */
		if (init_arg->v1_v4_control_operation.value != LTC2991_KEEP_DEFAULT_SETTING) {
			memset(&msg, 0, sizeof(I2C_MSG));
			msg.bus = cfg.port;
			msg.target_addr = cfg.target_addr;
			msg.tx_len = 2;
			msg.data[0] = LTC2991_V1_V4_CONTROL_REG;
			msg.data[1] = init_arg->v1_v4_control_operation.value;

			ret = i2c_master_write(&msg, retry);
			if (ret != 0) {
				LOG_ERR("i2c write fail ret: %d", ret);
				return SENSOR_INIT_UNSPECIFIED_ERROR;
			}
		}

		/* Read V1~V4 control register */
		memset(&msg, 0, sizeof(I2C_MSG));
		msg.bus = cfg.port;
		msg.target_addr = cfg.target_addr;
		msg.tx_len = 1;
		msg.rx_len = 1;
		msg.data[0] = LTC2991_V1_V4_CONTROL_REG;

		ret = i2c_master_read(&msg, retry);
		if (ret != 0) {
			LOG_ERR("i2c read fail ret: %d", ret);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}

		init_arg->v1_v4_control_operation.value = msg.data[0];

		/* Set V5~V8 control register */
		if (init_arg->v5_v8_control_operation.value != LTC2991_KEEP_DEFAULT_SETTING) {
			memset(&msg, 0, sizeof(I2C_MSG));
			msg.bus = cfg.port;
			msg.target_addr = cfg.target_addr;
			msg.tx_len = 2;
			msg.data[0] = LTC2991_V5_V8_CONTROL_REG;
			msg.data[1] = init_arg->v5_v8_control_operation.value;

			ret = i2c_master_write(&msg, retry);
			if (ret != 0) {
				LOG_ERR("i2c write fail ret: %d", ret);
				return SENSOR_INIT_UNSPECIFIED_ERROR;
			}
		}

		/* Read V5~V8 control register */
		memset(&msg, 0, sizeof(I2C_MSG));
		msg.bus = cfg.port;
		msg.target_addr = cfg.target_addr;
		msg.tx_len = 1;
		msg.rx_len = 1;
		msg.data[0] = LTC2991_V5_V8_CONTROL_REG;

		ret = i2c_master_read(&msg, retry);
		if (ret != 0) {
			LOG_ERR("i2c read fail ret: %d", ret);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}

		init_arg->v5_v8_control_operation.value = msg.data[0];
		init_arg->is_init = true;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = ltc2991_read;
	return SENSOR_INIT_SUCCESS;
}
