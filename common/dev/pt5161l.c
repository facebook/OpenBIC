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
#include <logging/log.h>

#include "pt5161l.h"
#include "hal_i2c.h"
#include "libutil.h"
#include "sensor.h"

LOG_MODULE_REGISTER(dev_pt5161l);

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

//Get temp caliberation codes
bool get_temp_calibration_codes(I2C_MSG *msg, pt5161l_init_arg *init_args)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	CHECK_NULL_ARG_WITH_RETURN(init_args, false);
	bool rc = false;
	uint8_t data_byte[1];
	uint8_t data_bytes5[5];
	uint8_t invalid;
	uint8_t flag;
	uint8_t offset;
	uint8_t cal_code;

	// eFuse read procedure
	// 1. Switch to refclk/8 clock for TCK
	// self.csr.misc.efuse_cntl.sms_clk_sel = 1
	rc = pt5161l_read_block_data(msg, 0x8ec, 5, data_bytes5);
	if (!rc) {
		LOG_ERR("Read refclk clock for TCK failed");
		return rc;
	}
	// Assert bit 25
	data_bytes5[3] = SETBIT(data_bytes5[3], 1);
	rc = pt5161l_write_block_data(msg, 0x8ec, 5, data_bytes5);
	if (!rc) {
		LOG_ERR("Set refclk clock for TCK failed");
		return rc;
	}
	// 2. Assert efuse_load
	// self.csr.misc.sms_efuse_cntl.sms_efuse_load = 1
	rc = pt5161l_read_block_data(msg, 0x8f6, 1, data_byte);
	if (!rc) {
		LOG_ERR("Read assert efuse_load failed");
		return rc;
	}
	// Assert bit 7
	data_byte[0] = SETBIT(data_byte[0], 7);
	rc = pt5161l_write_block_data(msg, 0x8f6, 1, data_byte);
	if (!rc) {
		LOG_ERR("Set assert efuse_load failed");
		return rc;
	}
	// 3. Assert smart_test
	// self.csr.misc.efuse_cntl.smart_test = 1
	rc = pt5161l_read_block_data(msg, 0x8ec, 5, data_bytes5);
	if (!rc) {
		LOG_ERR("Read assert smart_test failed");
		return rc;
	}
	// Assert bit 24
	data_bytes5[3] = SETBIT(data_bytes5[3], 0);
	rc = pt5161l_write_block_data(msg, 0x8ec, 5, data_bytes5);
	if (!rc) {
		LOG_ERR("Set assert smart_test failed");
		return rc;
	}
	// 4. De-assert smart_test
	// self.csr.misc.efuse_cntl.smart_test = 0
	rc = pt5161l_read_block_data(msg, 0x8ec, 5, data_bytes5);
	if (!rc) {
		LOG_ERR("Read De-assert smart_test failed");
		return rc;
	}
	// De-assert bit 24
	data_bytes5[3] = CLEARBIT(data_bytes5[3], 0);
	rc = pt5161l_write_block_data(msg, 0x8ec, 5, data_bytes5);
	if (!rc) {
		LOG_ERR("Set De-assert smart_test failed");
		return rc;
	}
	// 5. De-assert efuse_load
	// self.csr.misc.sms_efuse_cntl.sms_efuse_load = 0
	rc = pt5161l_read_block_data(msg, 0x8f6, 1, data_byte);
	if (!rc) {
		LOG_ERR("Read De-assert efuse_load failed");
		return rc;
	}
	// De-assert bit 7
	data_byte[0] = CLEARBIT(data_byte[0], 7);
	rc = pt5161l_write_block_data(msg, 0x8f6, 1, data_byte);
	if (!rc) {
		LOG_ERR("Set De-assert efuse_load failed");
		return rc;
	}

	// Read eFuse “primary page invalid” bit and adjust offset accordingly
	// Set address
	data_byte[0] = 63;
	rc = pt5161l_write_block_data(msg, 0x8f6, 1, data_byte);
	if (!rc) {
		LOG_ERR("Set eFuse primary page invalid bit failed");
		return rc;
	}
	// Read data
	rc = pt5161l_read_block_data(msg, 0x8f7, 1, data_byte);
	if (!rc) {
		LOG_ERR("Read eFuse primary page invalid bit failed");
		return rc;
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
	rc = pt5161l_write_block_data(msg, 0x8f6, 1, data_byte);
	if (!rc) {
		LOG_ERR("Set Determine calibration codes failed");
		return rc;
	}
	// Read data
	rc = pt5161l_read_block_data(msg, 0x8f7, 1, data_byte);
	if (!rc) {
		LOG_ERR("Read Determine calibration codes failed");
		return rc;
	}
	flag = data_byte[0];

	// Compute PMA A calibration codes
	int i;
	for (i = 0; i < 4; i++) {
		if (flag & 0x4) {
			data_byte[0] = 34 + (i * 4) + offset;
			rc = pt5161l_write_block_data(msg, 0x8f6, 1, data_byte);
			if (!rc) {
				LOG_ERR("Set PMA A[%d] calibration codes failed", i);
				return rc;
			}
			rc = pt5161l_read_block_data(msg, 0x8f7, 1, data_byte);
			if (!rc) {
				LOG_ERR("Read PMA A[%d] calibration codes failed", i);
				return rc;
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
			rc = pt5161l_write_block_data(msg, 0x8f6, 1, data_byte);
			if (!rc) {
				LOG_ERR("Set PMA B[%d] calibration codes failed", i);
				return rc;
			}
			rc = pt5161l_read_block_data(msg, 0x8f7, 1, data_byte);
			if (!rc) {
				LOG_ERR("Read PMA B[%d] calibration codes failed", i);
				return rc;
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
	return rc;
}

double pt5161l_read_avg_temp(I2C_MSG *i2c_msg, uint8_t temp_cal_code_avg)
{
	CHECK_NULL_ARG_WITH_RETURN(i2c_msg, 0);
	uint8_t data_bytes[4] = { 0 };
	int adc_code;
	double avg_temperature = 0;

	if (!pt5161l_read_block_data(i2c_msg, PT5161L_TEMP_OFFSET, 4, data_bytes)) {
		LOG_ERR("Read Avg Temperature failed");
		return 0;
	}

	adc_code = (data_bytes[3] << 24) + (data_bytes[2] << 16) + (data_bytes[1] << 8) +
		   data_bytes[0];

	avg_temperature = 110 - ((adc_code - (temp_cal_code_avg + 250)) * 0.32);

	return avg_temperature;
}

uint8_t pt5161l_read(uint8_t sensor_num, int *reading)
{
	if ((reading == NULL) || (sensor_num > SENSOR_NUM_MAX) ||
	    (sensor_config[sensor_config_index_map[sensor_num]].init_args == NULL)) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	pt5161l_init_arg *init_arg =
		(pt5161l_init_arg *)sensor_config[sensor_config_index_map[sensor_num]].init_args;

	if (!init_arg->is_init) {
		LOG_ERR("device isn't initialized");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	double val = 0;
	I2C_MSG msg = { 0 };

	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;

	switch (cfg->offset) {
	case PT5161L_TEMP_OFFSET:
		val = pt5161l_read_avg_temp(&msg, init_arg->temp_cal_code_avg);
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

uint8_t pt5161l_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	I2C_MSG msg;
	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;

	pt5161l_init_arg *init_args =
		(pt5161l_init_arg *)sensor_config[sensor_config_index_map[sensor_num]].init_args;

	if (init_args->is_init) {
		goto exit;
	}

	// Get temp caliberation codes
	if (!get_temp_calibration_codes(&msg, init_args)) {
		goto error;
	}

	init_args->is_init = true;

exit:
	sensor_config[sensor_config_index_map[sensor_num]].read = pt5161l_read;
	return SENSOR_INIT_SUCCESS;

error:
	return SENSOR_INIT_UNSPECIFIED_ERROR;
}
