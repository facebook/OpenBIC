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
#include <stdlib.h>
#include <sys/util.h>
#include "plat_modbus.h"
#include <modbus/modbus.h>
#include "modbus_server.h"
#include "hal_i2c.h"
#include <logging/log.h>
#include "libutil.h"
#include "plat_modbus_debug.h"

bool i2c_w_r_flag = 0;
uint8_t check_i2c_data[4] = { 0, 0, 0, 0 };
uint16_t temp_read_data[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t temp_reg = 0;

LOG_MODULE_REGISTER(plat_modbus_i2c_write_read);

uint8_t modbus_command_data_convert_to_8bit(uint16_t value)
{
	return value |= 0xff;
}

uint8_t modbus_command_i2c_write(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);
	// write data: bus(2Bytes), addr(2Bytes), reg(2Bytes), data(10Bytes)
	uint8_t retry = 5;
	I2C_MSG msg;
	uint8_t target_bus = modbus_command_data_convert_to_8bit(cmd->data[0]);
	uint8_t target_addr = modbus_command_data_convert_to_8bit(cmd->data[1]);
	uint8_t target_reg = modbus_command_data_convert_to_8bit(cmd->data[2]);
	uint8_t target_write_data = modbus_command_data_convert_to_8bit(cmd->data[3]);
	msg.bus = target_bus;
	msg.target_addr = target_addr;
	msg.tx_len = 2;
	msg.data[0] = target_reg;
	msg.data[1] = target_write_data;
	int result = i2c_master_write(&msg, retry);
	if (result != 0) {
		LOG_ERR("I2C write fail");
		return MODBUS_EXC_SERVER_DEVICE_FAILURE;
	}
	return MODBUS_EXC_NONE;
}

uint8_t modbus_command_i2c_write_for_read(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);
	// write data: bus(2Bytes), addr(2Bytes), reg(2Bytes), read length(2Bytes), data(8Bytes)
	if (i2c_w_r_flag == 0) {
		check_i2c_data[0] = modbus_command_data_convert_to_8bit(cmd->data[0]); // bus
		check_i2c_data[1] = modbus_command_data_convert_to_8bit(cmd->data[1]); // addr
		check_i2c_data[2] = modbus_command_data_convert_to_8bit(cmd->data[2]); // reg
		check_i2c_data[3] =
			modbus_command_data_convert_to_8bit(cmd->data[3]); // read length

		uint8_t target_bus = modbus_command_data_convert_to_8bit(cmd->data[0]);
		uint8_t target_addr = modbus_command_data_convert_to_8bit(cmd->data[1]);
		uint8_t target_reg = modbus_command_data_convert_to_8bit(cmd->data[2]);
		uint8_t target_read_length = modbus_command_data_convert_to_8bit(cmd->data[3]);
		uint8_t retry = 5;
		I2C_MSG msg;
		msg.bus = target_bus;
		msg.target_addr = target_addr;
		msg.tx_len = 2;
		msg.data[0] = target_reg;
		msg.rx_len = (int)target_read_length;
		int result = i2c_master_read(&msg, retry);
		if (result != 0) {
			LOG_ERR("I2C read fail");
			return MODBUS_EXC_SERVER_DEVICE_FAILURE;
		}
		for (int i = 0; i < target_read_length * 2; i++) {

			if (i / 2 == 0 && i > 1) {
				temp_read_data[(i / 2) - 1] = (uint16_t)(msg.data[i - 1] << 8 | msg.data[i]);
			} else {
				continue;
			}
		}
		i2c_w_r_flag = 1;
	} else {
		LOG_ERR("Read I2c fail, need to read register: 0x%x first", check_i2c_data[2]);
		return MODBUS_EXC_SERVER_DEVICE_FAILURE;
	}
	return MODBUS_EXC_NONE;
}

uint8_t modbus_command_i2c_read(modbus_command_mapping *cmd)
{
	// write data: bus(2Bytes), addr(2Bytes), reg(2Bytes), read length(2Bytes), data(8Bytes)
	if (i2c_w_r_flag == 1) {
		for (int i = 0; i < check_i2c_data[3]; i++) {
			uint16_t read_data = temp_read_data[i];
			memcpy(&cmd->data[i], &read_data, sizeof(uint16_t));
		}
		i2c_w_r_flag = 0;
		return MODBUS_EXC_NONE;
	} else {
		LOG_ERR("Please do modbus_command_i2c_write_for_read first");
		return MODBUS_EXC_SERVER_DEVICE_FAILURE;
	}
}
