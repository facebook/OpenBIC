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
#include <logging/log.h>
#include "util_spi.h"
#include "pt5161l.h"
#include "hal_i2c.h"
#include "libutil.h"
#include "sensor.h"

LOG_MODULE_REGISTER(dev_pt5161l);

K_MUTEX_DEFINE(pt5161l_mutex);

static bool is_update_ongoing = false;
uint8_t PT5161L_VENDOR_ID[7] = { 0x06, 0x04, 0x00, 0x01, 0x00, 0xFA, 0x1D };

bool pt5161l_get_vendor_id(I2C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	uint8_t retry = 5;

	msg->tx_len = 4;
	msg->data[0] = 0x02; //COMMAND CODE = 0x02 END=0, START=1, FUNC=3'b000, PEC=0
	msg->data[1] = 0x02; //byte count
	msg->data[2] = 0x04; //vendor id lower offset
	msg->data[3] = 0x00; //vendor id upper offset

	if (i2c_master_write(msg, retry)) {
		LOG_ERR("Failed to set PCIE RETIMER vendor id offset");
		return false;
	}

	memset(msg->data, 0, I2C_BUFF_SIZE);
	msg->tx_len = 1;
	msg->rx_len = 7;
	msg->data[0] = 0x01; //COMMAND CODE = 0x01 END=1, START=0, FUNC=3'b000, PEC=0
	if (i2c_master_read(msg, retry)) {
		LOG_ERR("Failed to read PCIE RETIMER vendor id");
		return false;
	}

	return true;
}

//Write multiple data bytes to retimer over I2C
bool pt5161l_write_block_data(I2C_MSG *msg, uint32_t address, uint8_t num_bytes, uint8_t *values)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	CHECK_NULL_ARG_WITH_RETURN(values, false);
	uint8_t cmd_code;
	uint8_t rsvd;
	uint8_t func_code;
	uint8_t start;
	uint8_t end;
	uint8_t upper_addr;
	uint8_t lower_addr;
	uint8_t retry = 3;

	// If Address more than 16 bit, return with an error code
	// Intel format only allows for 16 bit address
	if (address > 0xffff) {
		LOG_ERR("Address cannot be more than 16 bits in Intel format");
		return false;
	}

	// If byte count is greater than 4, perform multiple iterations
	// Hence keep track of remaining bytes each time we perform a
	// transaction
	uint8_t curr_bytes = 0;
	uint8_t remaining_bytes = num_bytes;
	while (remaining_bytes > 0) {
		if (remaining_bytes > 4) {
			curr_bytes = 4;
			remaining_bytes -= 4;
		} else {
			curr_bytes = remaining_bytes;
			remaining_bytes = 0;
		}

		rsvd = 0;
		func_code = 1;
		start = 1;
		end = 1;

		// Construct Command code
		cmd_code = (rsvd << 5) + (func_code << 2) + (start << 1) + (end << 0);
		upper_addr = (address & 0xff00) >> 8; // Get bits 16:8
		lower_addr = address & 0xff; // Get bits 7:0

		/* Set buffer length based on number of bytes being written
		 * 2 bytes of address in calculation
		 * 1 extra byte for num bytes in buffer
		 */
		int write_num_bytes = 2 + 1 + curr_bytes;
		msg->data[0] = cmd_code;
		msg->data[1] = write_num_bytes - 1;
		msg->data[2] = lower_addr;
		msg->data[3] = upper_addr;

		LOG_DBG("Write:");
		LOG_DBG("msg->data[0] = 0x%02x", msg->data[0]);
		LOG_DBG("msg->data[1] = 0x%02x", msg->data[1]);
		LOG_DBG("msg->data[2] = 0x%02x", msg->data[2]);
		LOG_DBG("msg->data[3] = 0x%02x", msg->data[3]);

		memcpy(&(msg->data[4]), values, curr_bytes);

		msg->tx_len = write_num_bytes + 1;

		if (i2c_master_write(msg, retry)) {
			LOG_ERR("pt5161l write block failed");
			return false;
		}

		// Increment iteration count
		values += curr_bytes;
		address += curr_bytes;
	}
	return true;
}

/*
 * Read multiple data bytes from Aries over I2C
 */
bool pt5161l_read_block_data(I2C_MSG *msg, uint32_t address, uint8_t num_bytes, uint8_t *values)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	CHECK_NULL_ARG_WITH_RETURN(values, false);
	uint8_t wr_cmd_code;
	uint8_t rd_cmd_code;
	uint8_t rsvd;
	uint8_t func_code;
	uint8_t start;
	uint8_t end;

	uint8_t upper_addr; // Get bits 16:8
	uint8_t lower_addr; // Get bits 7:0

	uint8_t retry = 3;

	// If Address more than 16 bit, return with an error code
	// Intel format only allows for 16 bit address
	if (address > 0xffff) {
		LOG_ERR("Address cannot be more than 16 bits in Intel format");
		return false;
	}

	uint8_t remaining_bytes = num_bytes;
	uint8_t curr_bytes = 0;
	while (remaining_bytes > 0) {
		if (remaining_bytes > 4) {
			curr_bytes = 4;
			remaining_bytes -= 4;
		} else {
			curr_bytes = remaining_bytes;
			remaining_bytes = 0;
		}

		rsvd = 0;
		func_code = 0;
		start = 1;
		end = 0;

		wr_cmd_code = (rsvd << 5) + (func_code << 2) + (start << 1) + (end << 0);

		upper_addr = (address & 0xff00) >> 8;
		lower_addr = address & 0xff;

		msg->data[0] = wr_cmd_code;
		msg->data[1] = 3 - 1; //write Numbers Bytes - 1 byte wr_cmd_code
		msg->data[2] = lower_addr;
		msg->data[3] = upper_addr;
		msg->tx_len = 4;

		LOG_DBG("Write:");
		LOG_DBG("cmd_code = 0x%02x", msg->data[0]);
		LOG_DBG("msg->data[1] = 0x%02x", msg->data[1]);
		LOG_DBG("msg->data[2] = 0x%02x", msg->data[2]);
		LOG_DBG("msg->data[3] = 0x%02x", msg->data[3]);

		// read buffer is 3 + data bytes always
		// First byte is num.bytes read
		// Second and third bytes are address
		// Bytes 4 onwards is data (4 bytes)
		uint8_t read_buf_len = 3 + 4;

		if (i2c_master_write(msg, retry)) {
			LOG_ERR("pt5161l write block failed");
			return false;
		}

		start = 0;
		end = 1;
		func_code = 0;

		rd_cmd_code = (rsvd << 5) + (func_code << 2) + (start << 1) + (end << 0);

		msg->data[0] = rd_cmd_code;
		msg->tx_len = 1;
		msg->rx_len = read_buf_len;

		LOG_DBG("Read:");
		LOG_DBG("cmd_code = 0x%02x", rd_cmd_code);

		if (i2c_master_read(msg, retry)) {
			LOG_ERR("pt5161l read failed");
			return false;
		}

		// Fill up user given array
		memcpy(values, &(msg->data[3]), curr_bytes);

		// Increment iteration count
		values += curr_bytes;
		address += curr_bytes;
	}
	return true;
}

/*
 * Read multiple (up to eight) data bytes from micro SRAM over I2C
 */
bool pt5161l_read_block_data_to_sram(I2C_MSG *msg, uint32_t micro_ind_struct_oft,
				     uint32_t eeprom_offset, uint8_t num_bytes, uint8_t *values)
{
	bool ret = false;
	int byte_index;
	uint8_t count;
	uint8_t MAX_RETRY = 5;
	uint8_t rdata;

	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	CHECK_NULL_ARG_WITH_RETURN(values, false);
	if (k_mutex_lock(&pt5161l_mutex, K_MSEC(PT5161L_MUTEX_LOCK_MS))) {
		LOG_ERR("pt5161l mutex lock failed");
		return false;
	}

	// No multi-byte indirect support here. Hence read a byte at a time
	for (byte_index = 0; byte_index < num_bytes; byte_index++) {
		// Write eeprom addr
		uint8_t eeprom_addr_bytes[3];
		int eeprom_acc_addr = eeprom_offset - PT5161L_MAIN_SRAM_DMEM_OFFSET + byte_index;
		eeprom_addr_bytes[0] = (eeprom_acc_addr) & 0xff;
		eeprom_addr_bytes[1] = (eeprom_acc_addr >> 8) & 0xff;
		eeprom_addr_bytes[2] = (eeprom_acc_addr >> 16) & 0xff;
		ret = pt5161l_write_block_data(msg, micro_ind_struct_oft, 3, eeprom_addr_bytes);
		if (!ret) {
			LOG_ERR("Write eeprom addr %d failed", eeprom_acc_addr);
			goto error_exit;
		}

		// Write eeprom cmd
		uint8_t eeprom_cmd_byte = PT5161L_TG_RD_LOC_IND_SRAM;
		ret = pt5161l_write_block_data(msg, (micro_ind_struct_oft + 4), 1,
					       &eeprom_cmd_byte);
		if (!ret) {
			LOG_ERR("Write eeprom cmd %x failed", eeprom_cmd_byte);
			goto error_exit;
		}

		// Test successfull access
		for (count = 0; count < MAX_RETRY; ++count) {
			ret = pt5161l_read_block_data(msg, (micro_ind_struct_oft + 4), 1, &rdata);
			if (ret == false) {
				LOG_ERR("eeprom test read failed");
				goto error_exit;
			}

			if (GETBIT(rdata, 1) == 0) {
				break;
			}
		}

		if (count == MAX_RETRY) {
			LOG_ERR("Tetimer test read eeprom failed");
			goto error_exit;
		} else {
			ret = pt5161l_read_block_data(msg, (micro_ind_struct_oft + 3), 1, &rdata);
			if (!ret) {
				LOG_ERR("pt5161l read eeprom byte index%d failed", byte_index);
				goto error_exit;
			}
			values[byte_index] = rdata;
		}
	}

error_exit:
	if (k_mutex_unlock(&pt5161l_mutex)) {
		LOG_ERR("pt5161l mutex unlock failed");
		return false;
	}

	return ret;
}

bool pt5161l_i2c_master_soft_reset(I2C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	bool ret = false;
	uint8_t data_byte;
	uint8_t i2c_init_ctrl;

	// Enable Bit bang mode
	data_byte = 3;
	ret = pt5161l_write_block_data(msg, PT5161L_I2C_MST_BB_OUTPUT_ADDRESS, 1, &data_byte);
	if (!ret) {
		LOG_ERR("pt5161l enable Bit bang mode failed");
		goto exit;
	}

	ret = pt5161l_read_block_data(msg, PT5161L_I2C_MST_INIT_CTRL_ADDRESS, 1, &i2c_init_ctrl);
	if (!ret) {
		LOG_ERR("pt5161l read I2C master init control failed");
		goto exit;
	}

	i2c_init_ctrl = PT5161L_I2C_MST_INIT_CTRL_BIT_BANG_MODE_EN_MODIFY(i2c_init_ctrl, 1);
	ret = pt5161l_write_block_data(msg, PT5161L_I2C_MST_INIT_CTRL_ADDRESS, 1, &i2c_init_ctrl);
	if (!ret) {
		LOG_ERR("pt5161l set I2C master init control failed");
		goto exit;
	}

	// Start Sequence
	// SDA = 1, SCL = 1
	data_byte = 3;
	ret = pt5161l_write_block_data(msg, PT5161L_I2C_MST_BB_OUTPUT_ADDRESS, 1, &data_byte);
	if (!ret) {
		LOG_ERR("pt5161l start sequence I2C master SDA = 1, SCL = 1 failed");
		goto exit;
	}

	// SDA = 0, SCL = 1
	data_byte = 1;
	ret = pt5161l_write_block_data(msg, PT5161L_I2C_MST_BB_OUTPUT_ADDRESS, 1, &data_byte);
	if (!ret) {
		LOG_ERR("pt5161l start sequence I2C master SDA = 0, SCL = 1 failed");
		goto exit;
	}

	// SDA = 0, SCL = 0
	data_byte = 0;
	ret = pt5161l_write_block_data(msg, PT5161L_I2C_MST_BB_OUTPUT_ADDRESS, 1, &data_byte);
	if (!ret) {
		LOG_ERR("pt5161l start sequence I2C master SDA = 0, SCL = 0 failed");
		goto exit;
	}

	// SDA = 1, SCL = 0
	data_byte = 2;
	ret = pt5161l_write_block_data(msg, PT5161L_I2C_MST_BB_OUTPUT_ADDRESS, 1, &data_byte);
	if (!ret) {
		LOG_ERR("pt5161l start sequence I2C master SDA = 1, SCL = 0 failed");
		goto exit;
	}

	int i = 0;
	for (i = 0; i < 9; i++) {
		// SDA = 1, SCL = 1
		data_byte = 3;
		ret = pt5161l_write_block_data(msg, PT5161L_I2C_MST_BB_OUTPUT_ADDRESS, 1,
					       &data_byte);
		if (!ret) {
			LOG_ERR("pt5161l set clock I2C master SDA = 1, SCL = 0 failed");
			goto exit;
		}

		// SDA = 1 SCL = 0
		data_byte = 2;
		ret = pt5161l_write_block_data(msg, PT5161L_I2C_MST_BB_OUTPUT_ADDRESS, 1,
					       &data_byte);
		if (!ret) {
			LOG_ERR("pt5161l set clock I2C master SDA = 1, SCL = 0 failed");
			goto exit;
		}
	}

	// Stop Sequence
	// SDA = 0, SCL = 0
	data_byte = 0;
	ret = pt5161l_write_block_data(msg, PT5161L_I2C_MST_BB_OUTPUT_ADDRESS, 1, &data_byte);
	if (!ret) {
		LOG_ERR("pt5161l stop sequence set I2C master SDA = 0, SCL = 0 failed");
		goto exit;
	}

	// SDA = 0, SCL = 1
	data_byte = 1;
	ret = pt5161l_write_block_data(msg, PT5161L_I2C_MST_BB_OUTPUT_ADDRESS, 1, &data_byte);
	if (!ret) {
		LOG_ERR("pt5161l stop sequence set I2C master SDA = 0, SCL = 1 failed");
		goto exit;
	}
	// SDA = 1, SCL = 1
	data_byte = 3;
	ret = pt5161l_write_block_data(msg, PT5161L_I2C_MST_BB_OUTPUT_ADDRESS, 1, &data_byte);
	if (!ret) {
		LOG_ERR("pt5161l stop sequence set I2C master SDA = 1, SCL = 1 failed");
		goto exit;
	}

	// Disable BB mode
	i2c_init_ctrl = PT5161L_I2C_MST_INIT_CTRL_BIT_BANG_MODE_EN_MODIFY(i2c_init_ctrl, 0);
	ret = pt5161l_write_block_data(msg, PT5161L_I2C_MST_INIT_CTRL_ADDRESS, 1, &i2c_init_ctrl);
	if (!ret) {
		LOG_ERR("pt5161l I2C master Disable BB mode failed");
		goto exit;
	}

exit:
	return ret;
}

bool pt5161l_i2c_master_write_ctrl_reg(I2C_MSG *msg, uint32_t offset, uint8_t length, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	bool ret = false;
	uint8_t oft = offset;
	uint8_t data_bytes[4] = { 0, 0, 0, 0 };
	uint16_t data_offset[4] = { PT5161L_I2C_MST_DATA0_ADDR, PT5161L_I2C_MST_DATA1_ADDR,
				    PT5161L_I2C_MST_DATA2_ADDR, PT5161L_I2C_MST_DATA3_ADDR };

	int i = 0;

	ret = pt5161l_write_block_data(msg, PT5161L_I2C_MST_IC_CMD_ADDR, 1, &oft);
	if (!ret) {
		LOG_ERR("PT5161L_I2C_MST_IC_CMD_ADDR write offset %x failed", offset);
		goto exit;
	}

	memcpy(data_bytes, data, length);

	for (i = 0; i < 4; i++) {
		ret = pt5161l_write_block_data(msg, data_offset[i], 1, &(data_bytes[i]));
		if (!ret) {
			LOG_ERR("PT5161L write %x data %x failed", data_offset[i], data_bytes[i]);
			goto exit;
		}
	}

	uint8_t cmd = PT5161L_SELF_WR_CSR_CMD;
	ret = pt5161l_write_block_data(msg, PT5161L_I2C_MST_CMD_ADDR, 1, &cmd);
	if (!ret) {
		LOG_ERR("PT5161L self wr csr cmd failed");
		goto exit;
	}

exit:
	return ret;
}

// I2C Master init for EEPROM Write-Thru
bool pt5161l_i2c_master_init(I2C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	uint8_t data[2];
	bool ret;

	data[0] = 0;
	ret = pt5161l_i2c_master_write_ctrl_reg(msg, 0x6c, 1, data);
	if (!ret) {
		LOG_ERR("pt5161l write 0x0 in to 0x6c failed");
		goto exit;
	}

	data[0] = 0xe5;
	data[1] = 0xf;
	ret = pt5161l_i2c_master_write_ctrl_reg(msg, 0, 2, data);
	if (!ret) {
		LOG_ERR("pt5161l write 0xe5 0x0f in to 0x0 failed");
		goto exit;
	}

	data[0] = 0x50;
	ret = pt5161l_i2c_master_write_ctrl_reg(msg, 0x04, 1, data);
	if (!ret) {
		LOG_ERR("pt5161l write 0x50 in to 0x04 failed");
		goto exit;
	}

	data[0] = 0;
	ret = pt5161l_i2c_master_write_ctrl_reg(msg, 0x38, 1, data);
	if (!ret) {
		LOG_ERR("pt5161l write 0x0 in to 0x38 failed");
		goto exit;
	}

	data[0] = 4;
	ret = pt5161l_i2c_master_write_ctrl_reg(msg, 0x3c, 1, data);
	if (!ret) {
		LOG_ERR("pt5161l write 0x4 in to 0x3c failed");
		goto exit;
	}

	data[0] = 1;
	ret = pt5161l_i2c_master_write_ctrl_reg(msg, 0x6c, 1, data);
	if (!ret) {
		LOG_ERR("pt5161l write 0x1 in to 0x6c failed");
		goto exit;
	}

exit:
	return ret;
}

//Set Page address in I2C Master
bool pt5161l_i2c_master_set_page(I2C_MSG *msg, int page)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	uint8_t target;
	uint8_t data;
	bool ret;

	// Power-on default value is 0x50
	target = 0x50 | (page & 3);

	data = 0;
	ret = pt5161l_i2c_master_write_ctrl_reg(msg, 0x6c, 1, &data);
	if (!ret) {
		LOG_ERR("pt5161l write 0x0 in to 0x6c failed");
		goto exit;
	}
	ret = pt5161l_i2c_master_write_ctrl_reg(msg, 0x04, 1, &target);
	if (!ret) {
		LOG_ERR("pt5161l write %x in to 0x04 failed", target);
		goto exit;
	}
	data = 1;
	ret = pt5161l_i2c_master_write_ctrl_reg(msg, 0x6c, 1, &data);
	if (!ret) {
		LOG_ERR("pt5161l write 0x1 in to 0x6c failed");
		goto exit;
	}

exit:
	return ret;
}

//Write multiple bytes to the I2C Master via Main Micro
bool pt5161l_i2c_master_multi_block_write(I2C_MSG *msg, uint16_t offset, int num_bytes,
					  uint8_t *values)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	CHECK_NULL_ARG_WITH_RETURN(values, false);
	// Data is arranged in four-byte chunks
	uint8_t data_bytes[4];
	uint8_t upper_oft;
	uint8_t lower_oft;
	bool ret;

	// IC Data command
	data_bytes[0] = 0x10;
	ret = pt5161l_write_block_data(msg, PT5161L_I2C_MST_IC_CMD_ADDR, 1, &(data_bytes[0]));
	if (!ret) {
		LOG_ERR("pt5161l write IC Data command 0x10 failed");
		return ret;
	}

	// Prepare Flag Byte
	data_bytes[0] = 0;
	ret = pt5161l_write_block_data(msg, PT5161L_I2C_MST_DATA1_ADDR, 1, &(data_bytes[0]));
	if (!ret) {
		LOG_ERR("pt5161l prepare Flag Byte failed");
		return ret;
	}

	// Send offset
	upper_oft = (offset >> 8) & 0xff;
	lower_oft = offset & 0xff;
	data_bytes[0] = upper_oft;
	ret = pt5161l_write_block_data(msg, PT5161L_I2C_MST_DATA0_ADDR, 1, &(data_bytes[0]));
	if (!ret) {
		LOG_ERR("pt5161l set upper offset %x failed", upper_oft);
		return ret;
	}

	data_bytes[0] = 1;
	ret = pt5161l_write_block_data(msg, PT5161L_I2C_MST_CMD_ADDR, 1, &(data_bytes[0]));
	if (!ret) {
		LOG_ERR("pt5161l trigger upper_oft failed");
		return ret;
	}

	data_bytes[0] = lower_oft;
	ret = pt5161l_write_block_data(msg, PT5161L_I2C_MST_DATA0_ADDR, 1, &(data_bytes[0]));
	if (!ret) {
		LOG_ERR("pt5161l set lower offset %x failed", lower_oft);
		return ret;
	}

	data_bytes[0] = 1;
	ret = pt5161l_write_block_data(msg, PT5161L_I2C_MST_CMD_ADDR, 1, &(data_bytes[0]));
	if (!ret) {
		LOG_ERR("pt5161l trigger lower_oft failed");
		return ret;
	}

	int num_iters = num_bytes / PT5161L_EEPROM_BLOCK_WRITE_SIZE;
	int iter_idx;
	int num_blocks = PT5161L_EEPROM_BLOCK_WRITE_SIZE / 4; // 4-byte blocks
	int block_idx;
	int oft = 0;
	uint8_t cmd;
	int try;
	int max_tries = 30;
	bool is_busy = false;

	for (iter_idx = 0; iter_idx < num_iters; iter_idx++) {
		// determine MM-assist command
		cmd = PT5161L_MM_EEPROM_WRITE_REG_CODE;
		if (iter_idx == (num_iters - 1)) {
			cmd = PT5161L_MM_EEPROM_WRITE_END_CODE;
		}
		cmd = cmd | PT5161L_EEPROM_BLOCK_CMD_MODIFIER;

		// Write data
		for (block_idx = 0; block_idx < num_blocks; block_idx++) {
			memcpy(data_bytes, &(values[(oft + block_idx * 4)]), 4);
			// write the data to Retimer holding registers
			ret = pt5161l_write_block_data(
				msg, PT5161L_EEPROM_BLOCK_BASE_ADDR + 4 * block_idx, 4, data_bytes);
			if (!ret) {
				LOG_ERR("pt5161l write the data to Retimer holding registers failed");
				return ret;
			}
		}

		// Write cmd
		data_bytes[0] = cmd;
		ret = pt5161l_write_block_data(msg, PT5161L_MM_EEPROM_ASSIST_CMD_ADDR, 1,
					       &(data_bytes[0]));
		if (!ret) {
			LOG_ERR("pt5161l write cmd %x failed", cmd);
			return ret;
		}

		// Verify Command returned back to zero
		is_busy = true;
		for (try = 0; try < max_tries; try++) {
			ret = pt5161l_read_block_data(msg, PT5161L_MM_EEPROM_ASSIST_CMD_ADDR, 1,
						      data_bytes);
			if (!ret) {
				LOG_ERR("pt5161l write cmd %x failed", cmd);
				return ret;
			}

			if (data_bytes[0] == 0) {
				is_busy = false;
				break;
			}
			k_msleep(PT5161L_MM_STATUS_TIME_5MS);
		}

		// If status not reset to 0, return BUSY error
		if (is_busy) {
			LOG_ERR("Main Micro busy writing data block to EEPROM. Did not commit write");
			return false;
		}

		oft += PT5161L_EEPROM_BLOCK_WRITE_SIZE;
	}

	return ret;
}

bool pt5161l_pre_update(I2C_MSG *msg)
{
	bool ret;

	// Deassert HW and SW resets
	uint8_t tmp_data[2];
	tmp_data[0] = 0;
	tmp_data[1] = 0;
	ret = pt5161l_write_block_data(msg, 0x600, 2, tmp_data); // hw_rst
	if (!ret) {
		LOG_ERR("set hw_rst failed");
		return ret;
	}
	ret = pt5161l_write_block_data(msg, 0x602, 2, tmp_data); // sw_rst
	if (!ret) {
		LOG_ERR("set sw_rst failed");
		return ret;
	}

	tmp_data[0] = 0;
	tmp_data[1] = 2;
	ret = pt5161l_write_block_data(msg, 0x602, 2, tmp_data); // sw_rst
	if (!ret) {
		LOG_ERR("set sw_rst data %x %x failed", tmp_data[0], tmp_data[1]);
		return ret;
	}

	tmp_data[0] = 0;
	tmp_data[1] = 0;
	ret = pt5161l_write_block_data(msg, 0x602, 2, tmp_data); // sw_rst
	if (!ret) {
		LOG_ERR("set sw_rst data %x %x failed", tmp_data[0], tmp_data[1]);
		return ret;
	}

	ret = pt5161l_i2c_master_soft_reset(msg);
	if (!ret) {
		LOG_ERR("pt5161l i2c master soft reset failed");
		return ret;
	}
	k_msleep(PT5161L_MM_STATUS_TIME_5MS);

	// Init I2C Master
	ret = pt5161l_i2c_master_init(msg);
	if (!ret) {
		LOG_ERR("pt5161l i2c master init failed");
		return ret;
	}

	return ret;
}

bool pt5161l_write_eeprom_image(I2C_MSG *msg, uint32_t offset, uint8_t *txbuf, uint16_t length,
				bool is_end)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	CHECK_NULL_ARG_WITH_RETURN(txbuf, false);

	int amend_len;
	int oft_msb;
	int oft_i2c;
	uint8_t data[PT5161L_EEPROM_PAGE_SIZE];
	bool ret = false;

	if (length > PT5161L_EEPROM_PAGE_SIZE) {
		LOG_ERR("pt5161l write eeprom image invalid length %d", length);
		return ret;
	}

	// Set MSB and local addresses for this page
	oft_msb = offset / SECTOR_SZ_64K;
	oft_i2c = offset % SECTOR_SZ_64K;

	// Set Page address
	ret = pt5161l_i2c_master_set_page(msg, oft_msb);
	if (!ret) {
		LOG_ERR("pt5161l i2c master set page %x failed", oft_msb);
		return false;
	}

	memcpy(data, txbuf, length);

	if (is_end && (length % PT5161L_EEPROM_BLOCK_WRITE_SIZE)) {
		amend_len = PT5161L_EEPROM_BLOCK_WRITE_SIZE -
			    (length % PT5161L_EEPROM_BLOCK_WRITE_SIZE);
		memset(&(data[length]), 0, amend_len);
	} else {
		amend_len = 0;
	}

	// Send a block of bytes to the EEPROM starting at address
	ret = pt5161l_i2c_master_multi_block_write(msg, oft_i2c, length + amend_len, data);
	if (!ret) {
		LOG_ERR("pt5161l Send a block of bytes to the EEPROM starting at address %x failed",
			offset);
		return ret;
	}

	k_msleep(PT5161L_MM_STATUS_TIME_10MS);

	return ret;
}

bool pt5161l_post_update(I2C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	bool ret = false;
	uint8_t tmp_data[2] = { 0 };

	// Assert HW resets for I2C master interface
	tmp_data[0] = 0x00;
	tmp_data[1] = 0x02;
	ret = pt5161l_write_block_data(msg, 0x600, 2, tmp_data); // hw_rst
	if (!ret) {
		LOG_ERR("post update failed to assert hw reset");
		return ret;
	}

	ret = pt5161l_write_block_data(msg, 0x602, 2, tmp_data); // sw_rst
	if (!ret) {
		LOG_ERR("post update failed to assert sw reset");
		return ret;
	}

	k_msleep(PT5161L_MM_STATUS_TIME_5MS);
	return ret;
}

uint8_t pt5161l_do_update(I2C_MSG *msg, uint32_t update_offset, uint8_t *txbuf, uint16_t length,
			  bool is_end)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, FWUPDATE_UPDATE_FAIL);
	CHECK_NULL_ARG_WITH_RETURN(txbuf, FWUPDATE_UPDATE_FAIL);

	if (update_offset == 0) {
		// Update device FW update progress state
		is_update_ongoing = true;
		if (!pt5161l_pre_update(msg)) {
			return FWUPDATE_UPDATE_FAIL;
		}
	}

	if (!pt5161l_write_eeprom_image(msg, update_offset, txbuf, length, is_end)) {
		LOG_ERR("Failed to write offset %x into eeprom", update_offset);
		return FWUPDATE_UPDATE_FAIL;
	}

	if (is_end) {
		if (!pt5161l_post_update(msg)) {
			LOG_ERR("Failed to reset retimer");
			return FWUPDATE_UPDATE_FAIL;
		}
		// Update device FW update progress state
		is_update_ongoing = false;
	}

	return FWUPDATE_SUCCESS;
}

uint8_t pcie_retimer_fw_update(I2C_MSG *msg, uint32_t offset, uint16_t msg_len, uint8_t *msg_buf,
			       uint8_t flag)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, FWUPDATE_UPDATE_FAIL);
	CHECK_NULL_ARG_WITH_RETURN(msg_buf, FWUPDATE_UPDATE_FAIL);
	static bool is_init = false;
	static uint8_t *txbuf = NULL;
	static uint32_t start_offset = 0, buf_offset = 0;
	uint32_t ret = 0;
	bool is_end = false;

	if (!is_init) {
		SAFE_FREE(txbuf);
		txbuf = (uint8_t *)malloc(SECTOR_SZ_256);
		if (txbuf == NULL) { // Retry alloc
			k_msleep(100);
			txbuf = (uint8_t *)malloc(SECTOR_SZ_256);
		}
		if (txbuf == NULL) {
			LOG_ERR("eeprom offset %x, failed to allocate txbuf.", offset);
			return FWUPDATE_OUT_OF_HEAP;
		}
		is_init = true;
		start_offset = offset;
		buf_offset = 0;
		k_msleep(10);
	}

	if (msg_len > SECTOR_SZ_256) {
		LOG_ERR("eeprom offset %x, recv data 0x%x over sector size 0x%x", offset,
			buf_offset + msg_len, SECTOR_SZ_256);
		SAFE_FREE(txbuf);
		k_msleep(10);
		is_init = false;
		return FWUPDATE_OVER_LENGTH;
	}

	LOG_DBG("update offset %x , msg_len %d, flag 0x%x, msg_buf: %2x %2x %2x %2x", offset,
		msg_len, flag, msg_buf[0], msg_buf[1], msg_buf[2], msg_buf[3]);

	memcpy(&txbuf[buf_offset], msg_buf, msg_len);
	buf_offset += msg_len;

	if ((buf_offset == SECTOR_SZ_256) || (flag & SECTOR_END_FLAG)) {
		if (flag & SECTOR_END_FLAG) {
			is_end = true;
		}

		if (k_mutex_lock(&pt5161l_mutex, K_FOREVER)) {
			LOG_ERR("pt5161l mutex lock failed");
			SAFE_FREE(txbuf);
			k_msleep(10);
			return FWUPDATE_UPDATE_FAIL;
		}

		ret = pt5161l_do_update(msg, start_offset, txbuf, buf_offset, is_end);

		if (k_mutex_unlock(&pt5161l_mutex)) {
			LOG_ERR("pt5161l mutex unlock failed");
			SAFE_FREE(txbuf);
			return FWUPDATE_UPDATE_FAIL;
		}

		SAFE_FREE(txbuf);
		k_msleep(10);

		if (ret) {
			LOG_ERR("Failed to update PCIE retimer eeprom, status %d", ret);
			return FWUPDATE_UPDATE_FAIL;
		} else {
			LOG_INF("PCIE retimer %x update success", start_offset);
		}

		is_init = false;

		return ret;
	}

	return FWUPDATE_SUCCESS;
}

bool get_retimer_fw_version(I2C_MSG *msg, uint8_t *version)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	CHECK_NULL_ARG_WITH_RETURN(version, false);

	bool ret = false;

	if (is_update_ongoing) {
		return false;
	}

	ret = pt5161l_read_block_data_to_sram(
		msg, PT5161L_MAIN_MICRO_INDIRECT,
		(PT5161L_MAIN_MICRO_FW_INFO + PT5161L_MM_FW_VERSION_MAJOR), 1, &(version[0]));
	if (!ret) {
		LOG_ERR("Read PT5161L main version failed");
		return ret;
	}

	ret = pt5161l_read_block_data_to_sram(
		msg, PT5161L_MAIN_MICRO_INDIRECT,
		(PT5161L_MAIN_MICRO_FW_INFO + PT5161L_MM_FW_VERSION_MINOR), 1, &(version[1]));
	if (!ret) {
		LOG_ERR("Read PT5161L minor version failed");
		return ret;
	}

	ret = pt5161l_read_block_data_to_sram(
		msg, PT5161L_MAIN_MICRO_INDIRECT,
		(PT5161L_MAIN_MICRO_FW_INFO + PT5161L_MM_FW_VERSION_BUILD), 2, &(version[2]));
	if (!ret) {
		LOG_ERR("Read PT5161L build version failed");
		return ret;
	}

	return ret;
}

//Get temp caliberation codes
bool get_temp_calibration_codes(I2C_MSG *msg, pt5161l_init_arg *init_args)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	CHECK_NULL_ARG_WITH_RETURN(init_args, false);
	bool ret = false;
	uint8_t data_byte[1];
	uint8_t data_bytes5[5];
	uint8_t invalid;
	uint8_t flag;
	uint8_t offset;
	uint8_t cal_code;

	if (k_mutex_lock(&pt5161l_mutex, K_MSEC(PT5161L_MUTEX_LOCK_MS))) {
		LOG_ERR("pt5161l mutex lock failed");
		return ret;
	}

	// eFuse read procedure
	// 1. Switch to refclk/8 clock for TCK
	// self.csr.misc.efuse_cntl.sms_clk_sel = 1
	ret = pt5161l_read_block_data(msg, 0x8ec, 5, data_bytes5);
	if (!ret) {
		LOG_ERR("Read refclk clock for TCK failed");
		goto unlock_exit;
	}
	// Assert bit 25
	data_bytes5[3] = SETBIT(data_bytes5[3], 1);
	ret = pt5161l_write_block_data(msg, 0x8ec, 5, data_bytes5);
	if (!ret) {
		LOG_ERR("Set refclk clock for TCK failed");
		goto unlock_exit;
	}
	// 2. Assert efuse_load
	// self.csr.misc.sms_efuse_cntl.sms_efuse_load = 1
	ret = pt5161l_read_block_data(msg, 0x8f6, 1, data_byte);
	if (!ret) {
		LOG_ERR("Read assert efuse_load failed");
		goto unlock_exit;
	}
	// Assert bit 7
	data_byte[0] = SETBIT(data_byte[0], 7);
	ret = pt5161l_write_block_data(msg, 0x8f6, 1, data_byte);
	if (!ret) {
		LOG_ERR("Set assert efuse_load failed");
		goto unlock_exit;
	}
	// 3. Assert smart_test
	// self.csr.misc.efuse_cntl.smart_test = 1
	ret = pt5161l_read_block_data(msg, 0x8ec, 5, data_bytes5);
	if (!ret) {
		LOG_ERR("Read assert smart_test failed");
		goto unlock_exit;
	}
	// Assert bit 24
	data_bytes5[3] = SETBIT(data_bytes5[3], 0);
	ret = pt5161l_write_block_data(msg, 0x8ec, 5, data_bytes5);
	if (!ret) {
		LOG_ERR("Set assert smart_test failed");
		goto unlock_exit;
	}
	// 4. De-assert smart_test
	// self.csr.misc.efuse_cntl.smart_test = 0
	ret = pt5161l_read_block_data(msg, 0x8ec, 5, data_bytes5);
	if (!ret) {
		LOG_ERR("Read De-assert smart_test failed");
		goto unlock_exit;
	}
	// De-assert bit 24
	data_bytes5[3] = CLEARBIT(data_bytes5[3], 0);
	ret = pt5161l_write_block_data(msg, 0x8ec, 5, data_bytes5);
	if (!ret) {
		LOG_ERR("Set De-assert smart_test failed");
		goto unlock_exit;
	}
	// 5. De-assert efuse_load
	// self.csr.misc.sms_efuse_cntl.sms_efuse_load = 0
	ret = pt5161l_read_block_data(msg, 0x8f6, 1, data_byte);
	if (!ret) {
		LOG_ERR("Read De-assert efuse_load failed");
		goto unlock_exit;
	}
	// De-assert bit 7
	data_byte[0] = CLEARBIT(data_byte[0], 7);
	ret = pt5161l_write_block_data(msg, 0x8f6, 1, data_byte);
	if (!ret) {
		LOG_ERR("Set De-assert efuse_load failed");
		goto unlock_exit;
	}

	// Read eFuse “primary page invalid” bit and adjust offset accordingly
	// Set address
	data_byte[0] = 63;
	ret = pt5161l_write_block_data(msg, 0x8f6, 1, data_byte);
	if (!ret) {
		LOG_ERR("Set eFuse primary page invalid bit failed");
		goto unlock_exit;
	}
	// Read data
	ret = pt5161l_read_block_data(msg, 0x8f7, 1, data_byte);
	if (!ret) {
		LOG_ERR("Read eFuse primary page invalid bit failed");
		goto unlock_exit;
	}
	invalid = data_byte[0];

	if (invalid & 0x80) {
		offset = 64;
	} else {
		offset = 0;
	}

	// Determine calibration codes
	// Set address
	data_byte[0] = 48 + offset;
	ret = pt5161l_write_block_data(msg, 0x8f6, 1, data_byte);
	if (!ret) {
		LOG_ERR("Set Determine calibration codes failed");
		goto unlock_exit;
	}
	// Read data
	ret = pt5161l_read_block_data(msg, 0x8f7, 1, data_byte);
	if (!ret) {
		LOG_ERR("Read Determine calibration codes failed");
		goto unlock_exit;
	}
	flag = data_byte[0];

	// Compute PMA A calibration codes
	int i;
	for (i = 0; i < 4; i++) {
		if (flag & 0x4) {
			data_byte[0] = 34 + (i * 4) + offset;
			ret = pt5161l_write_block_data(msg, 0x8f6, 1, data_byte);
			if (!ret) {
				LOG_ERR("Set PMA A[%d] calibration codes failed", i);
				goto unlock_exit;
			}
			ret = pt5161l_read_block_data(msg, 0x8f7, 1, data_byte);
			if (!ret) {
				LOG_ERR("Read PMA A[%d] calibration codes failed", i);
				goto unlock_exit;
			}
			cal_code = data_byte[0];
			if (cal_code == 0) {
				cal_code = 84;
			}
		} else {
			cal_code = 84;
		}
		init_args->temp_cal_code_pma_a[i] = cal_code;
	}

	// Compute PMA B calibration codes
	for (i = 0; i < 4; i++) {
		if (flag & 0x04) {
			data_byte[0] = 32 + (i * 4) + offset;
			ret = pt5161l_write_block_data(msg, 0x8f6, 1, data_byte);
			if (!ret) {
				LOG_ERR("Set PMA B[%d] calibration codes failed", i);
				return ret;
			}
			ret = pt5161l_read_block_data(msg, 0x8f7, 1, data_byte);
			if (!ret) {
				LOG_ERR("Read PMA B[%d] calibration codes failed", i);
				return ret;
			}
			cal_code = data_byte[0];
			if (cal_code == 0) {
				cal_code = 84;
			}
		} else {
			cal_code = 84;
		}
		init_args->temp_cal_code_pma_b[i] = cal_code;
	}

	// Calcualte the average PMA calibration code
	init_args->temp_cal_code_avg =
		(init_args->temp_cal_code_pma_a[0] + init_args->temp_cal_code_pma_a[1] +
		 init_args->temp_cal_code_pma_a[2] + init_args->temp_cal_code_pma_a[3] +
		 init_args->temp_cal_code_pma_b[0] + init_args->temp_cal_code_pma_b[1] +
		 init_args->temp_cal_code_pma_b[2] + init_args->temp_cal_code_pma_b[3] + 8 / 2) /
		8; // Add denominator/2 to cause integer rounding

unlock_exit:
	if (k_mutex_unlock(&pt5161l_mutex)) {
		LOG_ERR("pt5161l mutex unlock failed");
		return false;
	}

	return ret;
}

uint8_t pt5161l_read_avg_temp(I2C_MSG *i2c_msg, uint8_t temp_cal_code_avg, double *avg_temperature)
{
	CHECK_NULL_ARG_WITH_RETURN(i2c_msg, SENSOR_UNSPECIFIED_ERROR);
	uint8_t data_bytes[4] = { 0 };
	uint8_t ret = SENSOR_UNSPECIFIED_ERROR;
	int adc_code;

	if (k_mutex_lock(&pt5161l_mutex, K_MSEC(PT5161L_MUTEX_LOCK_MS))) {
		LOG_ERR("pt5161l mutex lock failed");
		return ret;
	}

	if (!pt5161l_read_block_data(i2c_msg, PT5161L_TEMP_OFFSET, 4, data_bytes)) {
		LOG_ERR("Read Avg Temperature failed");
		goto unlock_exit;
	}

	adc_code = (data_bytes[3] << 24) + (data_bytes[2] << 16) + (data_bytes[1] << 8) +
		   data_bytes[0];

	//return 0 means temperature is not ready
	if (adc_code == 0) {
		LOG_INF("Avg Temperature is not ready");
		ret = SENSOR_NOT_ACCESSIBLE;
		goto unlock_exit;
	}

	*avg_temperature = 110 - ((adc_code - (temp_cal_code_avg + 250)) * 0.32);
	ret = SENSOR_READ_SUCCESS;

unlock_exit:
	if (k_mutex_unlock(&pt5161l_mutex)) {
		LOG_ERR("pt5161l mutex unlock failed");
		return ret;
	}

	return ret;
}

uint8_t pt5161l_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	pt5161l_init_arg *init_arg = (pt5161l_init_arg *)cfg->init_args;

	if (!init_arg->is_init) {
		LOG_ERR("device isn't initialized");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	if (is_update_ongoing) {
		return SENSOR_NOT_ACCESSIBLE;
	}

	double val = 0;
	I2C_MSG msg = { 0 };
	uint8_t ret;

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;

	switch (cfg->offset) {
	case PT5161L_TEMP_OFFSET:
		ret = pt5161l_read_avg_temp(&msg, init_arg->temp_cal_code_avg, &val);
		if (ret != SENSOR_READ_SUCCESS) {
			return ret;
		}
		break;
	default:
		LOG_ERR("Invalid sensor 0x%x offset 0x%x", cfg->num, cfg->offset);
		return SENSOR_NOT_FOUND;
	}

	sensor_val *sval = (sensor_val *)reading;
	memset(sval, 0, sizeof(*sval));

	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t pt5161l_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	I2C_MSG msg;
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;

	pt5161l_init_arg *init_args = (pt5161l_init_arg *)cfg->init_args;

	if (init_args->is_init) {
		goto exit;
	}

	// Get temp caliberation codes
	if (!get_temp_calibration_codes(&msg, init_args)) {
		goto error;
	}

	init_args->is_init = true;

exit:
	cfg->read = pt5161l_read;
	return SENSOR_INIT_SUCCESS;

error:
	return SENSOR_INIT_UNSPECIFIED_ERROR;
}
