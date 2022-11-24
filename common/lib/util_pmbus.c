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

#include <stdint.h>
#include <logging/log.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"

LOG_MODULE_REGISTER(util_pmbus);

const float slinear11_exponents[32] = { 1.0,
					2.0,
					4.0,
					8.0,
					16.0,
					32.0,
					64.0,
					128.0,
					256.0,
					512.0,
					1024.0,
					2048.0,
					4096.0,
					8192.0,
					16384.0,
					32768.0,
					0.0000152587890625,
					0.000030517578125,
					0.00006103515625,
					0.0001220703125,
					0.000244140625,
					0.00048828125,
					0.0009765625,
					0.001953125,
					0.00390625,
					0.0078125,
					0.015625,
					0.03125,
					0.0625,
					0.125,
					0.25,
					0.5 };

float slinear11_to_float(uint16_t read_value)
{
	uint8_t exponent;
	int16_t mantissa;
	float lsb;
	float ret;

	exponent = read_value >> 11;
	mantissa = read_value & 0x07FF;
	/* Sign extend Mantissa to 16 bits */
	if (mantissa > 0x03FF)
		mantissa |= 0xF800;

	lsb = slinear11_exponents[exponent];
	ret = mantissa * lsb;
	return ret;
}

bool get_exponent_from_vout_mode(uint8_t sensor_num, float *exponent)
{
	CHECK_NULL_ARG_WITH_RETURN(exponent, false);

	uint8_t retry = 5;
	I2C_MSG msg;

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = PMBUS_VOUT_MODE;

	if (i2c_master_read(&msg, retry)) {
		return false;
	}

	*exponent = slinear11_exponents[msg.data[0] & 0x1f];
	return true;
}

int pmbus_read_command(uint8_t sensor_num, uint8_t command, uint16_t *result)
{
	CHECK_NULL_ARG_WITH_RETURN(result, -1);

	int ret = 0;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = command;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("I2C read command: 0x%x fail, ret: %d", command, ret);
		return -1;
	}

	*result = ((msg.data[1] << 8) | msg.data[0]);
	return 0;
}
