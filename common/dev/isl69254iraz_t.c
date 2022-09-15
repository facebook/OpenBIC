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
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"
#include "isl69254iraz_t.h"

bool isl69254iraz_t_get_checksum(uint8_t bus, uint8_t target_addr, uint8_t *checksum)
{
	if (checksum == NULL) {
		printf("<error> isl69254iraz_t checksum is NULL\n");
		return false;
	}

	I2C_MSG i2c_msg;
	uint8_t retry = 3;

	i2c_msg.bus = bus;
	i2c_msg.target_addr = target_addr;
	i2c_msg.tx_len = 3;
	i2c_msg.data[0] = 0xC7; //DMAADDR command code
	i2c_msg.data[1] = 0x3F; //DMA register offset
	i2c_msg.data[2] = 0x00; //dummy data

	if (i2c_master_write(&i2c_msg, retry)) {
		printf("<error> isl69254iraz_t get checksum while i2c writing\n");
		return false;
	}

	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = isl69254iraz_t_checksum_length;
	i2c_msg.data[0] = 0xC5; //DMAFIX command code

	if (i2c_master_read(&i2c_msg, retry)) {
		printf("<error> isl69254iraz_t get checksum while i2c reading\n");
		return false;
	}

	memcpy(checksum, i2c_msg.data, isl69254iraz_t_checksum_length);
	reverse_array(checksum, isl69254iraz_t_checksum_length);

	return true;
}

bool isl69254iraz_t_get_remaining_write(uint8_t bus, uint8_t target_addr, uint8_t *remain_write)
{
	I2C_MSG i2c_msg;
	uint8_t retry = 3;

	i2c_msg.bus = bus;
	i2c_msg.target_addr = target_addr;
	i2c_msg.tx_len = 3;
	i2c_msg.data[0] = 0xC7; //DMAADDR command code
	i2c_msg.data[1] = 0xC2; //DMA register offset
	i2c_msg.data[2] = 0x00; //dummy data

	if (i2c_master_write(&i2c_msg, retry)) {
		printf("<error> isl69254iraz_t get remaining write while i2c writing\n");
		return false;
	}

	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = 0xC5; //DMAFIX command code

	if (i2c_master_read(&i2c_msg, retry)) {
		printf("<error> isl69254iraz_t get remaining write while i2c reading\n");
		return false;
	}

	*remain_write = i2c_msg.data[0];
	return true;
}

uint8_t isl69254iraz_t_read(uint8_t sensor_num, int *reading)
{
	if (reading == NULL) {
		printf("[%s] input parameter reading is NULL\n", __func__);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	if (sensor_num > SENSOR_NUM_MAX) {
		printf("[%s] sensor 0x%x input parameter is invalid\n", __func__, sensor_num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	int val = 0, ret = 0;
	I2C_MSG msg;
	memset(&msg, 0, sizeof(I2C_MSG));
	*reading = 0;

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = sensor_config[sensor_config_index_map[sensor_num]].offset;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		printf("[%s] i2c read fail  ret: %d\n", __func__, ret);
		return SENSOR_FAIL_TO_ACCESS;
	}

	uint8_t offset = sensor_config[sensor_config_index_map[sensor_num]].offset;
	val = (msg.data[1] << 8) | msg.data[0];
	sensor_val *sval = (sensor_val *)reading;
	switch (offset) {
	case PMBUS_READ_VOUT:
		// 1 mV/LSB, unsigned integer
		sval->integer = val / 1000;
		sval->fraction = val % 1000;
		break;
	case PMBUS_READ_IOUT:
		// 0.1 A/LSB, 2's complement
		sval->integer = (int16_t)val / 10;
		sval->fraction = (int16_t)(val - (sval->integer * 10)) * 100;
		break;
	case PMBUS_READ_TEMPERATURE_1:
		// 1 Degree C/LSB, 2's complement
		sval->integer = val;
		break;
	case PMBUS_READ_POUT:
		// 1 Watt/LSB, 2's complement
		sval->integer = val;
		break;
	default:
		printf("[%s] not support offset 0x%x\n", __func__, offset);
		return SENSOR_FAIL_TO_ACCESS;
		break;
	}

	return SENSOR_READ_SUCCESS;
}

uint8_t isl69254iraz_t_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		printf("[%s] input sensor number is invalid\n", __func__);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = isl69254iraz_t_read;
	return SENSOR_INIT_SUCCESS;
}
