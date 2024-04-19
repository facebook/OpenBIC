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

#include <modbus/modbus.h>
#include "modbus_server.h"
#include "hal_i2c.h"
#include <logging/log.h>
#include "libutil.h"
#include "plat_modbus.h"

static bool i2c_w_r_flag;
static uint8_t check_i2c_data[4];
static uint16_t temp_read_data[8];


LOG_MODULE_REGISTER(plat_modbus_i2c_write_read);

uint8_t modbus_command_i2c_master_write_read(modbus_command_mapping *cmd)
{
	//CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);
	// write data: bus(2Bytes), addr(2Bytes), reg(2Bytes), read length(2Bytes), data(8Bytes)
	// if it is only write,read length will be 0
	if (cmd->data[3] == 0) { // only write
		LOG_ERR("I2C write read in \n");
		uint8_t retry = 5;
		I2C_MSG msg;
		uint8_t target_bus = cmd->data[0] &= GENMASK(7,0);
		uint8_t target_addr = cmd->data[1] &= GENMASK(7,0);
		uint8_t target_reg = cmd->data[2] &= GENMASK(7,0);
		uint8_t target_write_data = cmd->data[4] &= GENMASK(7,0);
		msg.bus = target_bus;
		msg.target_addr = target_addr;
		msg.tx_len = 2;
		msg.data[0] = target_reg;
		msg.data[1] = target_write_data;
		int result = i2c_master_write(&msg, retry);
		if (result != 0) {
			LOG_ERR("I2C write fail \n");
			return false;
		}
		return true;
	}
	else {
		if (i2c_w_r_flag == 0) {
			LOG_ERR("I2C read in \n");
			//LOG_ERR(" cmd->data[0]: %x, cmd->data[1]: %x, cmd->data[2]: %x, cmd->data[3]: %x \n", cmd->data[0], cmd->data[1], cmd->data[2], cmd->data[3]);
			check_i2c_data[0] = cmd->data[0] &= GENMASK(7,0); // bus
			check_i2c_data[1] = cmd->data[1] &= GENMASK(7,0); // addr
			check_i2c_data[2] = cmd->data[2] &= GENMASK(7,0); // reg
			check_i2c_data[3] = cmd->data[3] &= GENMASK(7,0); // read length

			uint8_t target_bus = cmd->data[0] &= GENMASK(7,0);
			uint8_t target_addr = cmd->data[1] &= GENMASK(7,0);
			uint8_t target_reg = cmd->data[2] &= GENMASK(7,0);
			uint8_t target_read_length = cmd->data[3] &= GENMASK(7,0);
			uint8_t retry = 5;
			I2C_MSG msg;
			msg.bus = target_bus;
			msg.target_addr = target_addr;
			msg.tx_len = 2;
			msg.data[0] = target_reg;
			msg.rx_len = (int)target_read_length;
			int result = i2c_master_read(&msg, retry);
			//LOG_ERR("i2c read data: 0x%x, 0x%x, 0x%x, 0x%x \n", msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
			if (result != 0) {
				LOG_ERR("I2C read fail \n");
				return false;
			}
			int data_num = 0;
			for (int i = 0; i < target_read_length * 2; i++) {
				if ( i > 0) {
					if ((i+1) % 2 == 0){
						temp_read_data[data_num] = (uint16_t)(msg.data[i-1] << 8 | msg.data[i]);
						data_num += 1;
					}
				}
				
			}
			i2c_w_r_flag = 1;
		} else {
			LOG_ERR("Write or read I2c fail, please to read register: 0x%x first \n", check_i2c_data[2]);
			return false;
		}
	}
	return true;
}


uint8_t modbus_command_i2c_master_write_read_response(modbus_command_mapping *cmd)
{
	// write data: bus(2Bytes), addr(2Bytes), reg(2Bytes), read length(2Bytes), data(8Bytes)
	if (i2c_w_r_flag == 1) {
		for (int i = 0; i < check_i2c_data[3]; i++) {
			uint16_t read_data = temp_read_data[i];
			memcpy(&cmd->data[i], &read_data, sizeof(uint16_t));
		}
		i2c_w_r_flag = 0;
		return true;
	} else {
		LOG_ERR("Please do modbus_command_i2c_write_for_read first \n");
		return false;
	}
}