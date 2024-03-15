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
#include "sensor.h"
#include "hal_i2c.h"
#include "libutil.h"
#include <logging/log.h>
#include "nct214.h"

LOG_MODULE_REGISTER(nct214);

/*static bool nct214_write(sensor_cfg *cfg, uint8_t offset, uint8_t val)
{
	I2C_MSG msg = { 0 };
	uint8_t retry = 5;
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;

	msg.data[0] = offset;
	msg.data[1] = val;

	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("nct214 write offset 0x%02x, val 0x%02x fail", offset, val);
		return false;
	}

	return true;
}
*/
uint8_t nct214_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	I2C_MSG msg = { 0 };
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	uint8_t offset = cfg->offset;
	uint8_t local_temperature_offset = LOCAL_TEMP_REG;
	uint8_t external_temperature_upper_byte_offset = EXTERNAL_TEMP_UPPER_BYTE_REG;
	uint8_t external_temperature_lower_byte_offset = EXTERNAL_TEMP_LOWER_BYTE_REG;
	sensor_val *sval = (sensor_val *)reading;
	msg.rx_len = 1;
	msg.tx_len = 1;
	msg.data[0] = CONFIG_REG_READ;
	if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
	}
	uint8_t temperature_range_select = (msg.data[0] >> 2) & 1;
	switch (offset) {
	case LOCAL_TEMP:

		msg.rx_len = 1;
		msg.tx_len = 1;
		msg.data[0] = local_temperature_offset;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		uint8_t val_local_temp = msg.data[0];
		if (temperature_range_select == 1){
			sval->integer = (int16_t)((float)val_local_temp-(float)64);
		}
		else{
			sval->integer = (int16_t)val_local_temp;
		}	
		sval->fraction = 0;
		return SENSOR_READ_SUCCESS;

	case EXTERNAL_TEMP:
		msg.rx_len = 1;
		msg.tx_len = 1;
		msg.data[0] = external_temperature_lower_byte_offset;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		uint8_t val_external_temp_lower_byte = msg.data[0];
		val_external_temp_lower_byte = val_external_temp_lower_byte >> 6;  // get two MSBs
		msg.data[0] = external_temperature_upper_byte_offset;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		uint8_t val_external_temp_upper_byte = msg.data[0];
		
		if (temperature_range_select == 1){
			sval->integer = (int16_t)((float)val_external_temp_upper_byte-(float)64);
		}
		else{
			sval->integer = (int16_t)val_external_temp_upper_byte;
		}	
		switch (val_external_temp_lower_byte)
		{
		case 0:
			sval->fraction = 0;
			break;
		case 1:
			sval->fraction = 0.25;
			break;
		case 2:
			sval->fraction = 0.5;
			break;
		case 3:
			sval->fraction = 0.75;
			break;
		default:
			LOG_ERR("Unknown external_temp_lower_byte");
			return SENSOR_UNSPECIFIED_ERROR;
			break;
		}
		
		return SENSOR_READ_SUCCESS;
	default:
		LOG_ERR("Unknown register offset(%d)", offset);
		return SENSOR_UNSPECIFIED_ERROR;
	}
	return SENSOR_READ_SUCCESS;
}

uint8_t nct214_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = nct214_read;
	return SENSOR_INIT_SUCCESS;
}
