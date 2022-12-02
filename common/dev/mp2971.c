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
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(mp2971);

#define MFR_RESO_SET 0xC7

float get_resolution(uint8_t sensor_num)
{
	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];

	uint8_t page = 0;
	uint16_t mfr_reso_set = 0;

	I2C_MSG msg;
	uint8_t i2c_max_retry = 5;

	//get page
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = PMBUS_PAGE;

	if (i2c_master_read(&msg, i2c_max_retry)) {
		LOG_WRN("i2c read failed.\n");
		return SENSOR_FAIL_TO_ACCESS;
	}

	page = msg.data[0];

	//get reso set
	msg.rx_len = 2;
	msg.data[0] = MFR_RESO_SET;

	if (i2c_master_read(&msg, i2c_max_retry)) {
		LOG_WRN("i2c read failed.\n");
		return SENSOR_FAIL_TO_ACCESS;
	}

	mfr_reso_set = (msg.data[1] << 8) | msg.data[0];

	uint8_t vout_reso_set;
	uint8_t iout_reso_set;
	uint8_t iin_reso_set;
	uint8_t pout_reso_set;

	float vout_reso = 0;
	float iout_reso = 0;
	float iin_reso = 0;
	float pout_reso = 0;
	float temp_reso = 1;

	//get reso from MFR_RESO_SET(C7h)
	if (page == 0) {
		vout_reso_set = (mfr_reso_set & GENMASK(7, 6)) >> 6;
		iout_reso_set = (mfr_reso_set & GENMASK(5, 4)) >> 4;
		iin_reso_set = (mfr_reso_set & GENMASK(3, 2)) >> 2;
		pout_reso_set = (mfr_reso_set & GENMASK(1, 0));

		if (vout_reso_set & BIT(1)) {
			vout_reso = 0.001;
		} else {
			LOG_WRN("vout_reso_set not supported: 0x%x\n", vout_reso_set);
		}

		if (iout_reso_set == 0) {
			iout_reso = 2;
		} else if (iout_reso_set == 1) {
			iout_reso = 1;
		} else if (iout_reso_set == 2) {
			iout_reso = 0.5;
		} else {
			LOG_WRN("iout_reso_set not supported: 0x%x\n", iout_reso_set);
		}

		if (iin_reso_set == 0) {
			iin_reso = 0.5;
		} else if (iin_reso_set == 1) {
			iin_reso = 0.25;
		} else if (iin_reso_set == 2) {
			iin_reso = 0.125;
		} else {
			LOG_WRN("iin_reso_set not supported: 0x%x\n", iin_reso_set);
		}

		if (pout_reso_set == 0) {
			pout_reso = 2;
		} else if (pout_reso_set == 1) {
			pout_reso = 1;
		} else if (pout_reso_set == 2) {
			pout_reso = 0.5;
		} else {
			LOG_WRN("pout_reso_set not supported: 0x%x\n", pout_reso_set);
		}

	} else if (page == 1) {
		vout_reso_set = (mfr_reso_set & GENMASK(4, 3)) >> 3;
		iout_reso_set = (mfr_reso_set & GENMASK(2, 2)) >> 2;
		pout_reso_set = (mfr_reso_set & GENMASK(0, 0));

		if (vout_reso_set & BIT(1)) {
			vout_reso = 0.001;
		} else {
			LOG_WRN("vout_reso_set not supported: 0x%x\n", vout_reso_set);
		}

		if (iout_reso_set == 0) {
			iout_reso = 1;
		} else if (iout_reso_set == 1) {
			iout_reso = 0.5;
		} else {
			LOG_WRN("iout_reso_set not supported: 0x%x\n", iout_reso_set);
		}

		iin_reso = 0.125;

		if (pout_reso_set == 0) {
			pout_reso = 1;
		} else if (pout_reso_set == 1) {
			pout_reso = 0.5;
		} else {
			LOG_WRN("pout_reso_set not supported: 0x%x\n", pout_reso_set);
		}
	} else {
		LOG_WRN("Page not supported: 0x%d\n", page);
	}

	uint8_t offset = cfg->offset;

	switch (offset) {
	case PMBUS_READ_VOUT:
		return vout_reso;
		break;
	case PMBUS_READ_IOUT:
		return iout_reso;
		break;
	case PMBUS_READ_IIN:
		return iin_reso;
		break;
	case PMBUS_READ_TEMPERATURE_1:
		return temp_reso;
		break;
	case PMBUS_READ_POUT:
		return pout_reso;
		break;
	default:
		LOG_WRN("offset not supported: 0x%x\n", offset);
		break;
	}
	return 0;
}

uint8_t mp2971_read(uint8_t sensor_num, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];

	uint8_t i2c_max_retry = 5;
	int val = 0;
	sensor_val *sval = (sensor_val *)reading;
	I2C_MSG msg;
	memset(sval, 0, sizeof(sensor_val));

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = cfg->offset;

	if (i2c_master_read(&msg, i2c_max_retry)) {
		/* read fail */
		return SENSOR_FAIL_TO_ACCESS;
	}

	uint8_t offset = cfg->offset;
	val = (msg.data[1] << 8) | msg.data[0];

	switch (offset) {
	case PMBUS_READ_VOUT:
		/* 1 mV/LSB, unsigned integer */
		val = val & BIT_MASK(12);
		break;
	case PMBUS_READ_IOUT:
		val = val & BIT_MASK(11);
		break;
	case PMBUS_READ_IIN:
		val = val & BIT_MASK(11);
		break;
	case PMBUS_READ_TEMPERATURE_1:
		val = val & BIT_MASK(8);
		break;
	case PMBUS_READ_POUT:
		val = val & BIT_MASK(11);
		break;
	default:
		LOG_WRN("offset not supported: 0x%x\n", offset);
		return SENSOR_FAIL_TO_ACCESS;
		break;
	}

	float resolution = get_resolution(sensor_num);
	if (resolution == 0) {
		return SENSOR_FAIL_TO_ACCESS;
	}
	sval->integer = (int16_t)(val * resolution);
	sval->fraction = (int16_t)((val - (sval->integer / resolution)) * (resolution * 1000));

	return SENSOR_READ_SUCCESS;
}

uint8_t mp2971_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = mp2971_read;
	return SENSOR_INIT_SUCCESS;
}
