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
#include <stdlib.h>
#include <string.h>
#include "sensor.h"
#include "hal_i2c.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(dev_ltc4282);

enum {
	SET_BIT = 0,
	CLEAR_BIT,
};

#define LTC4282_METER_HALT_BIT 5
#define LTC4282_METER_RESET_BIT 6

struct VOLTAGE_FULL_SCALE_RANGE_INFO {
	long unsigned int v_range_selection;
	float full_scale_output_voltage;
};

struct VOLTAGE_FULL_SCALE_RANGE_INFO FOLDBACK_MODE_TABLE[4] = { { 0x0, 5.547 },
								{ 0x1, 8.32 },
								{ 0x2, 16.64 },
								{ 0x3, 33.28 } };

static int set_clear_bit(I2C_MSG *msg, uint8_t reg, uint8_t bit, uint8_t op, uint8_t retry)
{
	if (msg == NULL) {
		LOG_ERR("NULL parameter");
		return -1;
	}

	msg->data[0] = reg;
	msg->tx_len = 1;
	msg->rx_len = 1;

	if (i2c_master_read(msg, retry) != 0) {
		return -1;
	}

	msg->data[1] = msg->data[0];
	msg->data[0] = reg;
	msg->tx_len = 2;

	if (op == SET_BIT) {
		msg->data[1] = SETBIT(msg->data[1], bit);
	} else if (op == CLEAR_BIT) {
		msg->data[1] = CLEARBIT(msg->data[1], bit);
	} else {
		return -1;
	}

	if (i2c_master_write(msg, retry) != 0) {
		return -1;
	}

	return 0;
}

static int ltc4282_read_ein(I2C_MSG *msg, double *val, uint8_t retry)
{
	int ret = -1;
	uint64_t energy = 0;
	uint32_t counter = 0;
	bool ticker_overflow = false;
	bool meter_overflow = false;

	if (msg == NULL || val == NULL) {
		LOG_ERR("NULL parameter");
		return -1;
	}

	// halt meter and tick counter
	if (set_clear_bit(msg, LTC4282_ADC_CONTROL_OFFSET, LTC4282_METER_HALT_BIT, SET_BIT, retry) <
	    0) {
		LOG_ERR("Failed to halt");
		return -1;
	}

	// check overflow
	msg->data[0] = LTC4282_STATUS_OFFSET_BYTE2;
	msg->tx_len = 1;
	msg->rx_len = 1;
	if (i2c_master_read(msg, retry) != 0) {
		goto exit;
	}

	meter_overflow = (msg->data[0] & BIT(0));
	ticker_overflow = (msg->data[0] & BIT(1));

	// tick counter or meter accumulator has overflowed, reset energy meter and counter
	if (meter_overflow || ticker_overflow) {
		LOG_ERR("Reset meter counter and status register");
		if (set_clear_bit(msg, LTC4282_ADC_CONTROL_OFFSET, LTC4282_METER_RESET_BIT, SET_BIT,
				  retry) < 0) {
			LOG_ERR("Failed to reset meter counter and status register");
		}
		if (set_clear_bit(msg, LTC4282_ADC_CONTROL_OFFSET, LTC4282_METER_RESET_BIT,
				  CLEAR_BIT, retry) < 0) {
			LOG_ERR("Failed to reset meter counter and status register");
		}
		goto exit;
	}

	// read energy and counter
	msg->data[0] = LTC4282_ENERGY_OFFSET;
	msg->tx_len = 1;
	msg->rx_len = 10;
	if (i2c_master_read(msg, retry) != 0) {
		goto exit;
	}

	energy = ((uint64_t)msg->data[0] << (uint64_t)40) |
		 ((uint64_t)msg->data[1] << (uint64_t)32) |
		 ((uint64_t)msg->data[2] << (uint64_t)24) |
		 ((uint64_t)msg->data[3] << (uint64_t)16) |
		 ((uint64_t)msg->data[4] << (uint64_t)8) | ((uint64_t)msg->data[5]);
	counter = ((uint32_t)msg->data[6] << (uint32_t)24) |
		  ((uint32_t)msg->data[7] << (uint32_t)16) |
		  ((uint32_t)msg->data[8] << (uint32_t)8) | ((uint32_t)msg->data[9]);

	if (counter == 0) {
		LOG_ERR("No available data");
		goto exit;
	}

	*val = (double)(energy / counter);
	ret = 0;
exit:
	// continue meter and tick counter
	if (set_clear_bit(msg, LTC4282_ADC_CONTROL_OFFSET, LTC4282_METER_HALT_BIT, CLEAR_BIT,
			  retry) < 0) {
		LOG_ERR("Failed to continue");
		return -1;
	}

	return ret;
}

uint8_t ltc4282_read(uint8_t sensor_num, int *reading)
{
	if ((reading == NULL) || (sensor_num > SENSOR_NUM_MAX) ||
	    (sensor_config[sensor_config_index_map[sensor_num]].init_args == NULL)) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	ltc4282_init_arg *init_arg =
		(ltc4282_init_arg *)sensor_config[sensor_config_index_map[sensor_num]].init_args;

	if (!init_arg->is_init) {
		LOG_ERR("Device isn't initialized");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	if (!init_arg->r_sense_mohm) {
		LOG_ERR("Rsense hasn't given");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	float rsense_mohm = init_arg->r_sense_mohm / 1000;
	float full_scale_output_voltage;
	for (uint8_t cnt = 0; cnt <= ARRAY_SIZE(FOLDBACK_MODE_TABLE); cnt++) {
		if (init_arg->ilim_adjust.fields.foldback_mode ==
		    FOLDBACK_MODE_TABLE[cnt].v_range_selection) {
			full_scale_output_voltage =
				FOLDBACK_MODE_TABLE[cnt].full_scale_output_voltage;
			break;
		}
		if (cnt == ARRAY_SIZE(FOLDBACK_MODE_TABLE)) {
			LOG_ERR("unknown voltage range, the foldback mode 0x%x",
				init_arg->ilim_adjust.fields.foldback_mode);
			return SENSOR_UNSPECIFIED_ERROR;
		}
	}

	uint8_t retry = 5;
	double val = 0;
	I2C_MSG msg = { 0 };

	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.data[0] = cfg->offset;
	msg.rx_len = 2;

	if (cfg->offset != LTC4282_ENERGY_OFFSET) {
		if (i2c_master_read(&msg, retry) != 0)
			return SENSOR_FAIL_TO_ACCESS;
	}
	// Refer to LTC4282 datasheet page 23.
	switch (cfg->offset) {
	case LTC4282_VSENSE_OFFSET:
		val = (((msg.data[0] << 8) | msg.data[1]) * 0.04 / 65535 / rsense_mohm);
		break;
	case LTC4282_POWER_OFFSET:
		val = (((msg.data[0] << 8) | msg.data[1]) * full_scale_output_voltage * 0.04 *
		       65536 / 65535 / 65535 / rsense_mohm);
		break;
	case LTC4282_VSOURCE_OFFSET:
		val = (((msg.data[0] << 8) | msg.data[1]) * full_scale_output_voltage / 65535);
		break;
	case LTC4282_ENERGY_OFFSET:
		if (ltc4282_read_ein(&msg, &val, retry) < 0) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		val = (val * 16.64 * 0.04 * 256 / 65535 / 65535 / rsense_mohm);
		break;
	default:
		LOG_ERR("Invalid sensor 0x%x offset 0x%x", sensor_num, cfg->offset);
		return SENSOR_NOT_FOUND;
	}

	sensor_val *sval = (sensor_val *)reading;
	memset(sval, 0, sizeof(*sval));

	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t ltc4282_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	I2C_MSG msg;
	uint8_t retry = 5;

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;

	ltc4282_init_arg *init_args =
		(ltc4282_init_arg *)sensor_config[sensor_config_index_map[sensor_num]].init_args;

	if (init_args->is_init) {
		goto exit;
	}

	if (init_args->is_register_setting_needed == 0x0) {
		goto init_param;
	}

	for (int bit = 0; bit < 8; ++bit) {
		switch (init_args->is_register_setting_needed & (BIT(bit))) {
		case BIT(0):
			// Set ILIM_ADJUST register
			msg.tx_len = 2;
			msg.rx_len = 0;
			msg.data[0] = LTC4282_ILIM_ADJUST_OFFSET;
			msg.data[1] = init_args->ilim_adjust.value;
			if (i2c_master_write(&msg, retry) != 0) {
				LOG_ERR("Failed to set LTC4282 register(0x%x)", msg.data[0]);
				goto error;
			}
			break;
		default:
			break;
		}
	}

init_param:
	// Read ILIM_ADJUST register value
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = LTC4282_ILIM_ADJUST_OFFSET;
	if (i2c_master_read(&msg, retry) != 0) {
		LOG_ERR("Failed to read LTC4282 register(0x%x)", msg.data[0]);
		init_args->is_init = false;
		goto error;
	}
	init_args->ilim_adjust.value = msg.data[0];

	init_args->is_init = true;

exit:
	sensor_config[sensor_config_index_map[sensor_num]].read = ltc4282_read;
	return SENSOR_INIT_SUCCESS;

error:
	return SENSOR_INIT_UNSPECIFIED_ERROR;
}
