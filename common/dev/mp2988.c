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
#include "mp2988.h"
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"

LOG_MODULE_REGISTER(mp2988);

#define MP2988_DEV_ID 0x98

/* --------- PAGE0 ---------- */
#define VR_REG_CODE_REV 0x26
#define VR_REG_DEV_ID 0x28
#define VR_REG_STAT_CML 0x7E
#define VR_REG_FAULT_CLR 0x03
#define VR_REG_STORE_USR_ALL 0x15
#define VR_REG_RESTORE_USR_ALL 0x16

/* --------- PAGE3 ---------- */
#define VR_REG_STORE_STAT 0x00

#define MAX_CMD_LINE 720

#define VR_MPS_PAGE_0 0x00
#define VR_MPS_PAGE_1 0x01
#define VR_MPS_PAGE_2 0x02
#define VR_MPS_PAGE_3 0x03

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
	MODE_STORE_USR_ALL,
	MODE_RESTORE_USR_ALL,
};

struct mp2988_data {
	uint16_t cfg_id;
	uint8_t page;
	uint8_t reg_addr;
	uint8_t reg_data[4];
	uint8_t reg_len;
};

struct mp2988_config {
	uint8_t mode;
	uint16_t cfg_id;
	uint16_t wr_cnt;
	uint16_t product_id_exp;
	struct mp2988_data *pdata;
};

static bool mp2988_set_page(uint8_t bus, uint8_t addr, uint8_t page)
{
	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 20;

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

static bool mp2988_write_data(uint8_t bus, uint8_t addr, struct mp2988_data *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;

	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	i2c_msg.tx_len = data->reg_len + 1;
	i2c_msg.data[0] = data->reg_addr;
	memcpy(&i2c_msg.data[1], &data->reg_data[0], data->reg_len);

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to write register 0x%02X with data:", data->reg_addr);
		LOG_HEXDUMP_ERR(&data->reg_data[0], data->reg_len, "");
		return false;
	}

	return true;
}

static bool mp2988_get_device_id(uint8_t bus, uint8_t addr, uint8_t *device_id)
{
	CHECK_NULL_ARG_WITH_RETURN(device_id, false);

	if (mp2988_set_page(bus, addr, VR_MPS_PAGE_0) == false) {
		LOG_ERR("Failed to set page before reading device id");
		return false;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;

	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = VR_REG_DEV_ID;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read device id");
		return false;
	}

	*device_id = i2c_msg.data[0];

	return true;
}

static bool mp2988_check_crc_fault(uint8_t bus, uint8_t addr, bool clear_fault)
{
	if (mp2988_set_page(bus, addr, VR_MPS_PAGE_0) == false) {
		LOG_ERR("Failed to set page before reading crc status");
		return false;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = VR_REG_STAT_CML;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read error status");
		return false;
	}

	if (i2c_msg.data[0] & BIT(4)) {
		LOG_WRN("Get error status 0x%x", i2c_msg.data[0]);
		if (clear_fault == true) {
			if (mp2988_set_page(bus, addr, VR_MPS_PAGE_3) == false) {
				LOG_ERR("Failed to set page before clear crc fault");
				return false;
			}

			i2c_msg.tx_len = 1;
			i2c_msg.data[0] = 0x03;

			if (i2c_master_write(&i2c_msg, retry)) {
				LOG_ERR("Failed to clear fault");
				return false;
			}

			return true;
		}
		return false;
	}

	return true;
}

bool mp2988_get_checksum(uint8_t bus, uint8_t addr, uint16_t *crc)
{
	CHECK_NULL_ARG_WITH_RETURN(crc, false);

	*crc = 0;

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = VR_REG_CODE_REV;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read version status");
		return false;
	}

	*crc = i2c_msg.data[0];

	return true;
}

static bool mp2988_store(uint8_t bus, uint8_t addr, uint8_t page, uint8_t mode)
{
	uint8_t cmd_code = 0;

	switch (mode) {
	case MODE_STORE_USR_ALL:
		cmd_code = VR_REG_STORE_USR_ALL;
		break;
	case MODE_RESTORE_USR_ALL:
		cmd_code = VR_REG_RESTORE_USR_ALL;
		break;

	default:
		LOG_ERR("Invalid store mode %d", mode);
		return false;
	}

	if (mp2988_set_page(bus, addr, page) == false) {
		LOG_ERR("Failed to set page before store data");
		return false;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;

	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	i2c_msg.tx_len = 1;
	i2c_msg.data[0] = cmd_code;

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to send store command 0x%02X", cmd_code);
		return false;
	}

	int wait_time = 10;
	while (wait_time) {
		if (mp2988_set_page(bus, addr, VR_MPS_PAGE_3) == false) {
			k_msleep(100);
			wait_time--;
			continue;
		}

		i2c_msg.tx_len = 1;
		i2c_msg.rx_len = 1;
		i2c_msg.data[0] = VR_REG_STORE_STAT;

		if (i2c_master_read(&i2c_msg, retry) == 0) {
			if (i2c_msg.data[0] != 0xFF)
				break;
		}

		wait_time--;
	}

	if (!wait_time) {
		LOG_ERR("Failed to store data, ret 0x%x", i2c_msg.data[0]);
		return false;
	}

	if (mp2988_check_crc_fault(bus, addr, false) == false) {
		LOG_ERR("Get error after store!");
		return false;
	}

	return true;
}

static bool mp2988_pre_update(uint8_t bus, uint8_t addr)
{
	if (mp2988_set_page(bus, addr, VR_MPS_PAGE_0) == false) {
		return false;
	}

	uint8_t dev_id = 0;
	if (mp2988_get_device_id(bus, addr, &dev_id) == false) {
		LOG_ERR("Failed to get device id");
		return false;
	}

	if (dev_id != MP2988_DEV_ID) {
		LOG_ERR("Invalid device id 0x%x", dev_id);
		return false;
	}

	if (mp2988_check_crc_fault(bus, addr, true) == false) {
		LOG_ERR("Failed to check crc fault");
		return false;
	}

	return true;
}

static bool parsing_image(uint8_t *img_buff, uint32_t img_size, struct mp2988_config *dev_cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(img_buff, false);
	CHECK_NULL_ARG_WITH_RETURN(dev_cfg, false);

	bool ret = false;

	/* Parsing image */
	int max_line = MAX_CMD_LINE;
	dev_cfg->pdata = (struct mp2988_data *)malloc(sizeof(struct mp2988_data) * max_line);
	if (!dev_cfg->pdata) {
		LOG_ERR("pdata malloc failed!");
		goto exit;
	}

	struct mp2988_data *cur_line = &dev_cfg->pdata[0];
	uint8_t cur_ele_idx = 0;
	uint32_t data_store = 0;
	uint8_t data_idx = 0;
	dev_cfg->wr_cnt = 0;
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
			break;

		case ATE_REG_ADDR_DEC:
			break;

		case ATE_REG_NAME:
			break;

		case ATE_REG_DATA_HEX:
			*((uint32_t *)cur_line->reg_data) = data_store;
			cur_line->reg_len = data_idx % 2 == 0 ? data_idx / 2 : (data_idx / 2 + 1);
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

	ret = true;

exit:
	if (ret == false)
		SAFE_FREE(dev_cfg->pdata);

	return ret;
}

bool mp2988_fwupdate(uint8_t bus, uint8_t addr, uint8_t *img_buff, uint32_t img_size)
{
	CHECK_NULL_ARG_WITH_RETURN(img_buff, false);

	uint8_t ret = false;

	/* Step1. Before update */
	if (mp2988_pre_update(bus, addr) == false) {
		LOG_ERR("Pre update failed!");
		return false;
	}

	/* Step2. Image parsing */
	struct mp2988_config dev_cfg = { 0 };
	if (parsing_image(img_buff, img_size, &dev_cfg) == false) {
		LOG_ERR("Failed to parsing image!");
		goto exit;
	}

	/* Step3. Firmware update */
	uint8_t last_page = 0xFF;
	struct mp2988_data *cur_data;
	uint16_t line_idx = 0;

	/* program USER data */
	for (line_idx = 0; line_idx < dev_cfg.wr_cnt; line_idx++) {
		cur_data = &dev_cfg.pdata[line_idx];
		if ((cur_data->page == VR_MPS_PAGE_0) || (cur_data->page == VR_MPS_PAGE_1) ||
		    (cur_data->page == VR_MPS_PAGE_2)) {
			if (cur_data->page != last_page) {
				if (line_idx != 0) {
					if (mp2988_store(bus, addr, MODE_STORE_USR_ALL,
							 last_page) == false) {
						LOG_ERR("Failed to store USER data to MTP at page %d!",
							last_page);
						goto exit;
					}
					if (mp2988_store(bus, addr, MODE_RESTORE_USR_ALL,
							 last_page) == false) {
						LOG_ERR("Failed to store USER data to MTP at page %d!",
							last_page);
						goto exit;
					}
				}
				if (mp2988_set_page(bus, addr, cur_data->page) == false) {
					LOG_ERR("Failed to set page before program data");
					goto exit;
				}
				last_page = cur_data->page;
			}

			LOG_DBG("USER page 0x%x wr 0x%x:", cur_data->page, cur_data->reg_addr);
			LOG_HEXDUMP_DBG(cur_data->reg_data, cur_data->reg_len, "");
			if (mp2988_write_data(bus, addr, cur_data) == false)
				goto exit;

			uint8_t percent = ((line_idx + 1) * 100) / dev_cfg.wr_cnt;
			if (percent % 10 == 0)
				LOG_INF("updated: %d%% (line: %d/%d page: %d)", percent,
					line_idx + 1, dev_cfg.wr_cnt, cur_data->page);
		}
	}

	if (last_page == VR_MPS_PAGE_0) {
		if (mp2988_store(bus, addr, MODE_STORE_USR_ALL, last_page) == false) {
			LOG_ERR("Failed to store USER data to MTP at page %d!", last_page);
			goto exit;
		}
		if (mp2988_store(bus, addr, MODE_RESTORE_USR_ALL, last_page) == false) {
			LOG_ERR("Failed to store USER data to MTP at page %d!", last_page);
			goto exit;
		}
	}

	uint16_t checksum = 0;
	if (mp2988_get_checksum(bus, addr, &checksum) == false)
		return false;

	LOG_INF("User checksum: 0x%x", checksum);

	ret = true;
exit:
	SAFE_FREE(dev_cfg.pdata);
	return ret;
}
