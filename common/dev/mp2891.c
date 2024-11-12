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
#include "libutil.h"
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"
#include "util_pmbus.h"

LOG_MODULE_REGISTER(mp2891);

#define DAC_2P5MV_EN_BIT BIT(13)
#define MFR_VID_RES_MASK GENMASK(15, 14)
#define IOUT_SCALE_MASK GENMASK(2, 0)
#define IOUT_MASK GENMASK(10, 0)

#define MFR_SVI3_IOUT_RPT 0x65
#define MFR_VOUT_LOOP_CTRL 0xBD

#define VR_MPS_PAGE_1 0x01

/* --------- PAGE1 ---------- */
#define VR_REG_EXPECTED_USER_CRC 0xF0

enum {
	ATE_CONF_ID = 0,
	ATE_PAGE_NUM,
	ATE_REG_ADDR_HEX,
	ATE_REG_ADDR_DEC,
	ATE_REG_NAME,
	ATE_REG_DATA_HEX,
	ATE_REG_DATA_DEC,
	ATE_COL_MAX,
};

static bool mp2891_set_page(uint8_t bus, uint8_t addr, uint8_t page)
{
	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;

	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	i2c_msg.tx_len = 2;
	i2c_msg.data[0] = PMBUS_PAGE;
	i2c_msg.data[1] = page;

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to set page to 0x%02X", page);
		return false;
	}

	k_msleep(100);

	return true;
}

bool mp2891_get_fw_version(uint8_t bus, uint8_t addr, uint32_t *rev)
{
	CHECK_NULL_ARG_WITH_RETURN(rev, false);

	if (mp2891_set_page(bus, addr, VR_MPS_PAGE_1) == false) {
		LOG_ERR("Failed to set page before reading config revision");
		return false;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = VR_REG_EXPECTED_USER_CRC;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_INF("Failed to read config revision");
		return false;
	}

	*rev = (i2c_msg.data[1] << 8) | i2c_msg.data[0];

	return true;
}

float mp2891_get_resolution(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_FAIL_TO_ACCESS);

	uint8_t offset = cfg->offset;
	float reso = 0;

	I2C_MSG msg;
	uint8_t i2c_max_retry = 5;
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;

	switch (offset) {
	case PMBUS_READ_VOUT:
		msg.data[0] = MFR_VOUT_LOOP_CTRL;

		if (i2c_master_read(&msg, i2c_max_retry)) {
			LOG_WRN("I2C read failed");
			return SENSOR_FAIL_TO_ACCESS;
		}
		uint16_t mfr_vout_loop_ctrl = (msg.data[1] << 8) | msg.data[0];

		if (mfr_vout_loop_ctrl & DAC_2P5MV_EN_BIT) {
			reso = 0.0025;
		} else if ((mfr_vout_loop_ctrl & MFR_VID_RES_MASK) == BIT(14)) {
			reso = 0.005;
		} else if ((mfr_vout_loop_ctrl & MFR_VID_RES_MASK) == BIT(15)) {
			reso = 0.002;
		} else {
			LOG_WRN("vout_reso_set not supported: 0x%x", mfr_vout_loop_ctrl);
			return SENSOR_FAIL_TO_ACCESS;
		}
		return reso;
	case PMBUS_READ_IOUT:
		msg.data[0] = MFR_SVI3_IOUT_RPT;

		if (i2c_master_read(&msg, i2c_max_retry)) {
			LOG_WRN("I2C read failed");
			return SENSOR_FAIL_TO_ACCESS;
		}
		uint16_t mfr_svi3_iout_prt_data = (msg.data[1] << 8) | msg.data[0];

		switch (mfr_svi3_iout_prt_data & IOUT_SCALE_MASK) {
		case 0:
			reso = 1;
			break;
		case 1:
			reso = 0.03125;
			break;
		case 2:
			reso = 0.0625;
			break;
		case 3:
			reso = 0.125;
			break;
		case 4:
			reso = 0.25;
			break;
		case 5:
			reso = 0.5;
			break;
		case 6:
			reso = 1;
			break;
		case 7:
			reso = 2;
			break;
		default:
			LOG_WRN("iout_reso_set not supported: 0x%x", mfr_svi3_iout_prt_data);
			return SENSOR_FAIL_TO_ACCESS;
		}
		return reso;
	default:
		LOG_WRN("offset not supported: 0x%x", offset);
		return SENSOR_FAIL_TO_ACCESS;
	}
}

uint8_t mp2891_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	sensor_val *sval = (sensor_val *)reading;
	I2C_MSG msg;
	memset(sval, 0, sizeof(sensor_val));

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = cfg->offset;

	if (i2c_master_read(&msg, retry)) {
		LOG_WRN("I2C read failed");
		return SENSOR_FAIL_TO_ACCESS;
	}

	float val;
	if (cfg->offset == PMBUS_READ_VOUT) {
		uint16_t read_value = (msg.data[1] << 8) | msg.data[0];
		float resolution = mp2891_get_resolution(cfg);
		if (resolution == 0)
			return SENSOR_FAIL_TO_ACCESS;

		val = slinear11_to_float(read_value) * resolution;

	} else if (cfg->offset == PMBUS_READ_TEMPERATURE_1 || cfg->offset == PMBUS_READ_POUT) {
		uint16_t read_value = (msg.data[1] << 8) | msg.data[0];
		val = slinear11_to_float(read_value);

	} else if (cfg->offset == PMBUS_READ_IOUT) {
		uint16_t read_value = ((msg.data[1] << 8) | msg.data[0]) & IOUT_MASK;
		float resolution = mp2891_get_resolution(cfg);
		if (resolution == 0)
			return SENSOR_FAIL_TO_ACCESS;

		val = (float)read_value * resolution;

	} else {
		return SENSOR_FAIL_TO_ACCESS;
	}

	sval->integer = val;
	sval->fraction = (val - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t mp2891_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = mp2891_read;
	return SENSOR_INIT_SUCCESS;
}
