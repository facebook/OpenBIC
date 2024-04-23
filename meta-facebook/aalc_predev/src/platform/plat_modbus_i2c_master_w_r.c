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
#include "plat_modbus_i2c_master_w_r.h"

static bool i2c_w_r_flag;
static uint16_t temp_read_length;
static uint16_t temp_read_data[16] = {[0 ... 15] = 0xff}; //16 registers


LOG_MODULE_REGISTER(plat_modbus_i2c_write_read);

uint8_t modbus_command_i2c_master_write_read(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);
	// write data: bus(2Bytes), addr(2Bytes), read length(2Bytes), data(26Bytes)
	// if it is only write,read length will be 0
	uint8_t target_bus;
	uint8_t target_addr;
	uint8_t target_read_length;
	uint8_t write_length = 8; // need to set uint8_t data_len here;
	I2C_MSG msg;
	uint8_t retry = 5;
	if(write_length <= 3){ // check bus,addr,read length is not null
		return MODBUS_EXC_ILLEGAL_DATA_VAL;
	}
	target_bus = cmd->data[0] & BIT_MASK(8); // get 7:0 bit data, BIT_MASK(n): get 0~n-1 bit data
	target_addr = cmd->data[1] & BIT_MASK(8);
	target_read_length = cmd->data[2] & BIT_MASK(8);
	msg.bus = target_bus;
	msg.target_addr = target_addr;
	msg.tx_len = write_length - 3; // write length need to -3 (bus,addr,read length)
	for (int i = 0; i < write_length-3 ; i++) {
		msg.data[i] = cmd->data[i+3] & BIT_MASK(8);
	}
	if (target_read_length == 0) { // only write
		int result = i2c_master_write(&msg, retry);
		if (result != 0) {
			LOG_ERR("I2C write fail \n");
			return MODBUS_EXC_SERVER_DEVICE_FAILURE;
		}
		return MODBUS_EXC_NONE;
	}
	if (i2c_w_r_flag) {
		LOG_ERR("Write or read I2c fail, please to read data first \n");
		return MODBUS_EXC_SERVER_DEVICE_FAILURE;
	}
	temp_read_length = target_read_length; // read length
	msg.rx_len = (int)temp_read_length;
	int result = i2c_master_read(&msg, retry);
	
	if (result != 0) {
		LOG_ERR("I2C read fail \n");
		return MODBUS_EXC_SERVER_DEVICE_FAILURE;
	}
	for (int i = 0; i < temp_read_length; i++) {
		temp_read_data[i] = msg.data[i];
	}
	i2c_w_r_flag = true;
	return MODBUS_EXC_NONE;
}


uint8_t modbus_command_i2c_master_write_read_response(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);
	// write data: bus(2Bytes), addr(2Bytes), read length(2Bytes), data(reg:2Bytes+data:24Bytes)
	if (i2c_w_r_flag == false){
		LOG_ERR("Please do modbus_command_i2c_write_for_read first \n");
		return MODBUS_EXC_SERVER_DEVICE_FAILURE;
	}
	for (int i = 0; i < temp_read_length; i++) {
		cmd->data[i] = temp_read_data[i];
	}
	i2c_w_r_flag = false;
	return MODBUS_EXC_NONE;

}