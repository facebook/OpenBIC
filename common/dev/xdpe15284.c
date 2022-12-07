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
#include "libutil.h"
#include "hal_i2c.h"
#include "util_pmbus.h"
#include "pmbus.h"

LOG_MODULE_REGISTER(xdpe15284);

#define XDPE15284_MFR_FW_CMD_REG 0xFE
#define XDPE15284_MFR_FW_CMD_DATA_REG 0xFD
#define XDPE15284_MFR_FW_CMD_DATA_LEN 5
#define XDPE15284_NOP_CMD 0x00
#define XDPE15284_CALCULATE_CRC_CMD 0x2D
#define XDPE15284_CALCULATE_CRC_DELAY_MS 10

bool xdpe15284_get_checksum(uint8_t bus, uint8_t addr, uint8_t *checksum)
{
	CHECK_NULL_ARG_WITH_RETURN(checksum, false);

	uint8_t retry = 3;
	I2C_MSG i2c_msg = { 0 };

	/* Reset the MFR data register to default value */
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 2;
	i2c_msg.data[0] = XDPE15284_MFR_FW_CMD_REG;
	i2c_msg.data[1] = XDPE15284_NOP_CMD;

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Reset data register fail");
		return false;
	}

	/* Send the command to get firmware crc checksum */
	memset(&i2c_msg, 0, sizeof(I2C_MSG));
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 2;
	i2c_msg.data[0] = XDPE15284_MFR_FW_CMD_REG;
	i2c_msg.data[1] = XDPE15284_CALCULATE_CRC_CMD;

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Write calculate crc command to register fail");
		return false;
	}

	k_msleep(XDPE15284_CALCULATE_CRC_DELAY_MS);

	/* Read the firmware checksum in MFR firmware command data register */
	memset(&i2c_msg, 0, sizeof(I2C_MSG));
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = XDPE15284_MFR_FW_CMD_DATA_LEN;
	i2c_msg.data[0] = XDPE15284_MFR_FW_CMD_DATA_REG;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Read version from register fail");
		return false;
	}

	checksum[0] = i2c_msg.data[4];
	checksum[1] = i2c_msg.data[3];
	checksum[2] = i2c_msg.data[2];
	checksum[3] = i2c_msg.data[1];

	return true;
}

uint8_t xdpe15284_read(uint8_t sensor_num, int *reading)
{
	if (reading == NULL || (sensor_num > SENSOR_NUM_MAX)) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	sensor_val *sval = (sensor_val *)reading;
	I2C_MSG msg;
	memset(sval, 0, sizeof(sensor_val));

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = sensor_config[sensor_config_index_map[sensor_num]].offset;

	if (i2c_master_read(&msg, retry))
		return SENSOR_FAIL_TO_ACCESS;

	uint8_t offset = sensor_config[sensor_config_index_map[sensor_num]].offset;
	if (offset == PMBUS_READ_VOUT) {
		/* ULINEAR16, get exponent from VOUT_MODE */
		float exponent;
		if (!get_exponent_from_vout_mode(sensor_num, &exponent))
			return SENSOR_FAIL_TO_ACCESS;

		float actual_value = ((msg.data[1] << 8) | msg.data[0]) * exponent;
		sval->integer = actual_value;
		sval->fraction = (actual_value - sval->integer) * 1000;
	} else if (offset == PMBUS_READ_IOUT || offset == PMBUS_READ_TEMPERATURE_1 ||
		   offset == PMBUS_READ_POUT) {
		/* SLINEAR11 */
		uint16_t read_value = (msg.data[1] << 8) | msg.data[0];
		float actual_value = slinear11_to_float(read_value);
		if (offset == PMBUS_READ_IOUT && actual_value < 0) {
			/* In the case POUT is 0, IOUT may read small negative value, replace this case with 0 */
			sval->integer = 0;
			sval->fraction = 0;
		} else {
			sval->integer = actual_value;
			sval->fraction = (actual_value - sval->integer) * 1000;
		}
	} else {
		return SENSOR_FAIL_TO_ACCESS;
	}

	return SENSOR_READ_SUCCESS;
}

uint8_t xdpe15284_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = xdpe15284_read;
	return SENSOR_INIT_SUCCESS;
}
