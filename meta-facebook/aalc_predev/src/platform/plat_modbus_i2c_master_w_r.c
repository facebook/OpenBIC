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
static uint8_t temp_i2c_reg;
static uint16_t temp_read_length;
static uint16_t temp_read_data[16]; //16 registers


LOG_MODULE_REGISTER(plat_modbus_i2c_write_read);

uint8_t modbus_command_i2c_master_write_read(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);
	// write data: bus(2Bytes), addr(2Bytes), reg(2Bytes), read length(2Bytes), data(24Bytes)
	// if it is only write,read length will be 0
	if (cmd->data[3] == 0) { // only write
		uint8_t retry = 5;
		I2C_MSG msg;
		uint8_t target_bus = cmd->data[0] &= BIT_MASK(7); // get 7:0 bit data
		uint8_t target_addr = cmd->data[1] &= BIT_MASK(7);
		uint8_t target_reg = cmd->data[2] &= BIT_MASK(7);

		msg.bus = target_bus;
		msg.target_addr = target_addr;
		msg.tx_len = 9;
		msg.data[0] = target_reg;
		for (int i = 0; i < 8; i++) {
			msg.data[i+1] = cmd->data[i+4] &= BIT_MASK(7);
		}
		int result = i2c_master_write(&msg, retry);
		if (result != 0) {
			LOG_ERR("I2C write fail \n");
			return false;
		}
		return true;
	}
	else {
		if (i2c_w_r_flag == false) {
			temp_read_length = cmd->data[3] &= BIT_MASK(7); // read length
			temp_i2c_reg = cmd->data[2] &= BIT_MASK(7); // reg
			uint8_t target_bus = cmd->data[0] &= BIT_MASK(7);
			uint8_t target_addr = cmd->data[1] &= BIT_MASK(7);
			uint8_t target_reg = cmd->data[2] &= BIT_MASK(7);
			uint8_t target_read_length = cmd->data[3] &= BIT_MASK(7);
			uint8_t retry = 5;
			I2C_MSG msg;
			msg.bus = target_bus;
			msg.target_addr = target_addr;
			msg.tx_len = 2;
			msg.data[0] = target_reg;
			msg.rx_len = (int)target_read_length;
			int result = i2c_master_read(&msg, retry);
			
			if (result != 0) {
				LOG_ERR("I2C read fail \n");
				return false;
			}
			for (int i = 0; i < 16; i++) {
				if (i < temp_read_length)
				{
					temp_read_data[i] = msg.data[i];
				}
				else
				{
					temp_read_data[i] = 0xff;
				}
							
			}
			i2c_w_r_flag = true;
		} else {
			LOG_ERR("Write or read I2c fail, please to read register: 0x%x first \n", temp_i2c_reg);
			return false;
		}
	}
	return true;
}


uint8_t modbus_command_i2c_master_write_read_response(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);
	// write data: bus(2Bytes), addr(2Bytes), reg(2Bytes), read length(2Bytes), data(24Bytes)
	if (i2c_w_r_flag == true) {
		for (int i = 0; i < 16; i++) {
			if (i < temp_read_length){
				cmd->data[i] = temp_read_data[i];
			}
			else{
				cmd->data[i] = 0xff;
			}
			
		}
		i2c_w_r_flag = false;
		return true;
	} else {
		LOG_ERR("Please do modbus_command_i2c_write_for_read first \n");
		return false;
	}
}