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
#include <stdlib.h>
#include <string.h>
#include <logging/log.h>
#include "mpq8746.h"
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"

LOG_MODULE_REGISTER(mpq8746);

#define VR_MPS_VEND_ID 0x4D5053 // ASCI: MPS

#define MAX_CMD_LINE 720

#define VR_MPS_PAGE_0 0x00

/* --------- PAGE0 ---------- */
#define VR_REG_STORE_USR_ALL 0x15
#define VR_REG_VOUT_MODE 0x20
#define VR_REG_MFR_ID 0x99
#define VR_REG_MFR_CFG_ID 0xC0
#define VR_REG_MFR_CFG_CODE_REV 0xC1
#define VR_REG_CRC_USR 0xF8

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

enum {
	VOUT_MODE_DIRECT,
	VOUT_MODE_VID,
	VOUT_MODE_LINEAR,
	VOUT_MODE_UNKNOWN = 0xFF,
};

uint8_t vr_vout_mode = VOUT_MODE_UNKNOWN;

struct mpq8746_data {
	uint16_t cfg_id;
	uint8_t page;
	uint8_t reg_addr;
	uint8_t reg_data[4];
	uint8_t reg_len;
};

struct mpq8746_config {
	uint8_t mode;
	uint16_t cfg_id;
	uint16_t dev_id;
	uint16_t wr_cnt;
	uint16_t product_id_exp;
	struct mpq8746_data *pdata;
};

static bool mpq8746_set_page(uint8_t bus, uint8_t addr, uint8_t page)
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

static bool mpq8746_write_data(uint8_t bus, uint8_t addr, struct mpq8746_data *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;

	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	i2c_msg.tx_len = data->reg_len + 1;
	i2c_msg.data[0] = data->reg_addr;
	memcpy(&i2c_msg.data[1], &data->reg_data[0], data->reg_len);
	//LOG_HEXDUMP_INF(&i2c_msg.data[0], i2c_msg.tx_len, "write:");

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to write data in register 0x%02X", data->reg_addr);
		return false;
	}

	return true;
}

static bool mpq8746_store(uint8_t bus, uint8_t addr)
{
	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;

	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	i2c_msg.tx_len = 1;
	i2c_msg.data[0] = VR_REG_STORE_USR_ALL;

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to store data to MTP");
		return false;
	}

	return true;
}

bool mpq8746_get_fw_version(uint8_t bus, uint8_t addr, uint16_t *rev)
{
	CHECK_NULL_ARG_WITH_RETURN(rev, false);

	if (mpq8746_set_page(bus, addr, VR_MPS_PAGE_0) == false) {
		LOG_ERR("Failed to set page before reading config code revision");
		return false;
	}

	*rev = 0;

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = VR_REG_MFR_CFG_CODE_REV;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read config code revision");
		return false;
	}

	*rev = (i2c_msg.data[1] << 8) | i2c_msg.data[0];

	return true;
}

static bool mpq8746_get_checksum(uint8_t bus, uint8_t addr, uint16_t *checksum)
{
	CHECK_NULL_ARG_WITH_RETURN(checksum, false);

	if (mpq8746_set_page(bus, addr, VR_MPS_PAGE_0) == false) {
		LOG_ERR("Failed to set page before reading user crc");
		return false;
	}

	*checksum = 0;

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = VR_REG_CRC_USR;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read user crc");
		return false;
	}

	*checksum = (i2c_msg.data[1] << 8) | i2c_msg.data[0];

	return true;
}

static bool mpq8746_pre_update(uint8_t bus, uint8_t addr, struct mpq8746_config *dev_cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(dev_cfg, false);

	if (mpq8746_set_page(bus, addr, VR_MPS_PAGE_0) == false) {
		return false;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 4;
	i2c_msg.data[0] = VR_REG_MFR_ID;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read vendor id");
		return false;
	}

	uint32_t vend_id = (i2c_msg.data[1]) | (i2c_msg.data[2] << 8) | (i2c_msg.data[3] << 16);
	if (vend_id != VR_MPS_VEND_ID) {
		LOG_ERR("Invalid vendor id 0x%x", vend_id);
		return false;
	}

	uint16_t cfg_code_rev = 0;
	if (mpq8746_get_fw_version(bus, addr, &cfg_code_rev) == false) {
		LOG_ERR("Failed to get product id");
		return false;
	}

	uint8_t dev_id = (cfg_code_rev >> 13) & BIT_MASK(3); // C1[15:13]

	if (dev_cfg->dev_id != dev_id) {
		LOG_ERR("Invalid device id 0x%x from image, should be 0x%x", dev_cfg->dev_id,
			dev_id);
		return false;
	}

	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = VR_REG_MFR_CFG_ID;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read config id");
		return false;
	}

	uint16_t cfg_id = i2c_msg.data[0] | (i2c_msg.data[1] << 8);

	LOG_INF("Update VR from cfg_id: 0x%x to cfg_id: 0x%x", cfg_id, dev_cfg->cfg_id);

	return true;
}

static bool parsing_image(uint8_t *img_buff, uint32_t img_size, struct mpq8746_config *dev_cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(img_buff, false);
	CHECK_NULL_ARG_WITH_RETURN(dev_cfg, false);

	bool ret = false;

	/* Parsing image */
	int max_line = MAX_CMD_LINE;
	dev_cfg->pdata = (struct mpq8746_data *)malloc(sizeof(struct mpq8746_data) * max_line);
	if (!dev_cfg->pdata) {
		LOG_ERR("pdata malloc failed!");
		goto exit;
	}

	struct mpq8746_data *cur_line = &dev_cfg->pdata[0];
	uint8_t cur_ele_idx = 0;
	uint32_t data_store = 0;
	uint8_t data_idx = 0;
	dev_cfg->wr_cnt = 0;

	bool cfg_id_get = false;
	bool dev_id_get = false;

	for (int i = 0; i < img_size; i++) {
		/* check valid */
		if (!img_buff[i]) {
			LOG_ERR("Get invalid buffer data at index %d", i);
			goto exit;
		}

		if ((cur_ele_idx == ATE_CONF_ID) && (i + 2 < img_size)) {
			if (!strncmp(&img_buff[i], "END", 3)) {
				break;
			}
		}

		if (((img_buff[i] != 0x09) && img_buff[i] != 0x0d)) {
			// pass non hex charactor
			int val = ascii_to_val(img_buff[i]);
			if (val == -1)
				continue;

			data_store = (data_store << 4) | val;
			data_idx++;
			continue;
		}

		switch (cur_ele_idx) {
		case ATE_CONF_ID:
			cur_line->cfg_id = data_store & 0xffff;
			break;

		case ATE_PAGE_NUM:
			cur_line->page = data_store & 0xff;
			break;

		case ATE_REG_ADDR_HEX:
			cur_line->reg_addr = data_store & 0xff;
			if (cur_line->reg_addr == VR_REG_MFR_CFG_ID)
				cfg_id_get = true;
			else if (cur_line->reg_addr == VR_REG_MFR_CFG_CODE_REV)
				dev_id_get = true;
			break;

		case ATE_REG_ADDR_DEC:
			break;

		case ATE_REG_NAME:
			break;

		case ATE_REG_DATA_HEX:
			*((uint32_t *)cur_line->reg_data) = data_store;
			cur_line->reg_len = data_idx % 2 == 0 ? data_idx / 2 : (data_idx / 2 + 1);

			if (cfg_id_get == true) {
				dev_cfg->cfg_id = data_store;
				cfg_id_get = false;
			} else if (dev_id_get == true) {
				dev_cfg->dev_id =
					(data_store & GENMASK(15, 13)) >> 13; // 0xC1[15:13]
				dev_id_get = false;
			}
			break;

		case ATE_REG_DATA_DEC:
			break;

		default:
			LOG_ERR("Got unknow element index %d", cur_ele_idx);
			goto exit;
		}

		data_idx = 0;
		data_store = 0;

		if (img_buff[i] == 0x09) {
			cur_ele_idx++;
		} else if (img_buff[i] == 0x0d) {
			LOG_DBG("vr[%d] page: %d addr:%x", dev_cfg->wr_cnt, cur_line->page,
				cur_line->reg_addr);
			LOG_HEXDUMP_DBG(cur_line->reg_data, cur_line->reg_len, "data:");

			cur_ele_idx = 0;
			dev_cfg->wr_cnt++;
			if (dev_cfg->wr_cnt > max_line) {
				LOG_ERR("Line record count is overlimit");
				goto exit;
			}
			cur_line++;
			i++; //skip 'a'
		}
	}

	LOG_INF("[FW info] cfg_id: 0x%x dev_id: 0x%x", dev_cfg->cfg_id, dev_cfg->dev_id);

	ret = true;

exit:
	if (ret == false)
		SAFE_FREE(dev_cfg->pdata);

	return ret;
}

bool mpq8746_fwupdate(uint8_t bus, uint8_t addr, uint8_t *img_buff, uint32_t img_size)
{
	CHECK_NULL_ARG_WITH_RETURN(img_buff, false);

	uint8_t ret = false;

	/* Step1. Image parsing */
	struct mpq8746_config dev_cfg = { 0 };
	if (parsing_image(img_buff, img_size, &dev_cfg) == false) {
		LOG_ERR("Failed to parsing image!");
		goto exit;
	}

	/* Step2. Before update */
	if (mpq8746_pre_update(bus, addr, &dev_cfg) == false) {
		LOG_ERR("Failed to pre-update!");
		goto exit;
	}

	/* Step3. Firmware update */
	struct mpq8746_data *cur_data;
	uint16_t line_idx = 0;

	/* program USER data */
	if (mpq8746_set_page(bus, addr, VR_MPS_PAGE_0) == false) {
		LOG_ERR("Failed to set page before fw update");
		goto exit;
	}

	for (line_idx = 0; line_idx < dev_cfg.wr_cnt; line_idx++) {
		cur_data = &dev_cfg.pdata[line_idx];

		if (mpq8746_write_data(bus, addr, cur_data) == false)
			goto exit;

		uint8_t percent = ((line_idx + 1) * 100) / dev_cfg.wr_cnt;
		if (percent % 10 == 0)
			LOG_INF("updated: %d%% (line: %d/%d page: %d)", percent, line_idx + 1,
				dev_cfg.wr_cnt, cur_data->page);
	}

	if (mpq8746_store(bus, addr) == false) {
		LOG_ERR("Failed to store USER data to MTP!");
		goto exit;
	}

	k_msleep(1000);

	/* Step4. After update */
	uint16_t checksum = 0;
	if (mpq8746_get_checksum(bus, addr, &checksum) == false) {
		LOG_ERR("Failed to get USER checksum!");
		goto exit;
	}

	LOG_INF("User checksum: 0x%x", checksum);

	ret = true;
exit:
	SAFE_FREE(dev_cfg.pdata);
	return ret;
}

uint8_t mpq8746_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t i2c_max_retry = 5;
	float val = 0;

	I2C_MSG msg = { 0 };

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = cfg->offset;

	if (i2c_master_read(&msg, i2c_max_retry)) {
		LOG_ERR("Failed to get sensor 0x%x value", cfg->num);
		return SENSOR_FAIL_TO_ACCESS;
	}

	switch (cfg->offset) {
	case PMBUS_READ_VOUT:
		val = (msg.data[1] << 8) | msg.data[0];
		if (vr_vout_mode == VOUT_MODE_LINEAR) {
			val *= 0.001953125;
		} else {
			val *= 0.0015625;
		}
		break;
	case PMBUS_READ_IOUT:
		val = (msg.data[1] << 8) | msg.data[0];
		val *= 0.0625;
		break;
	case PMBUS_READ_TEMPERATURE_1:
		val = msg.data[0];
		break;

	default:
		LOG_WRN("offset not supported: 0x%x", cfg->offset);
		return SENSOR_FAIL_TO_ACCESS;
		break;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t mpq8746_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	uint8_t i2c_max_retry = 5;

	I2C_MSG msg = { 0 };
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = VR_REG_VOUT_MODE;

	if (i2c_master_read(&msg, i2c_max_retry)) {
		LOG_ERR("Failed to get vout mode");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	vr_vout_mode = (msg.data[0] & (BIT(5) | BIT(6))) >> 5;

	if (vr_vout_mode > VOUT_MODE_LINEAR)
		vr_vout_mode = VOUT_MODE_LINEAR;

	cfg->read = mpq8746_read;
	return SENSOR_INIT_SUCCESS;
}
