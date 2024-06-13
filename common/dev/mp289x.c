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
#include "mp289x.h"
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"

LOG_MODULE_REGISTER(mp289x);

#define USR_END_KEY "END"
#define CRC_CHECK_START_KEY "CRC_CHECK_START"
#define CRC_CHECK_STOP_KEY "CRC_CHECK_STOP"
#define CRC_USR_KEY "CRC_P0P1"
#define CRC_GP12_KEY "CRC_M1M2"
#define CRC_GP34_KEY "CRC_M3M4"
#define CRC_GP56_KEY "CRC_M5M6"

/* --------- PAGE0 ---------- */
#define VR_REG_STAT_PMBUS 0x8A
#define VR_REG_EN_WR_PROT 0x0F
#define VR_REG_MTP_CTRL 0xD0
#define VR_REG_STAT_CML 0x7E
#define VR_REG_FAULT_CLR 0x03
#define VR_REG_MTP_FAULT_CLR 0xFF
#define VR_REG_CFG_REV 0x47

//multi-cfg: 6 groups store with same data from page4
#define VR_REG_STORE_CFG_ALL 0x15
#define VR_REG_STORE_USR 0x17

#define VR_REG_CRC_USR 0xF6

/* --------- PAGE1 ---------- */
#define VR_REG_OFFLINE_CTL 0xF3

/* --------- PAGE3 ---------- */
#define VR_REG_STORE_STAT 0x35

/* --------- PAGE4 ---------- */
#define VR_REG_MFR_IMON_DIGI_GAIN 0xD0
//multi-cfg: every 2 groups store data from page4
#define VR_REG_STORE_CFG_GP12 0xF5
#define VR_REG_STORE_CFG_GP34 0xF6
#define VR_REG_STORE_CFG_GP56 0xF7

/* --------- PAGE9E ---------- */
#define VR_REG_CRC_GP12_HI 0xFE
#define VR_REG_CRC_GP12_LO 0xFF

/* --------- PAGE9F ---------- */
#define VR_REG_CRC_GP34_HI 0x7E
#define VR_REG_CRC_GP34_LO 0x7F
#define VR_REG_CRC_GP56_HI 0xFE
#define VR_REG_CRC_GP56_LO 0xFF

uint8_t total_current_set = 0xFF;

#define MAX_CMD_LINE 720

#define VR_MPS_PAGE_0 0x00
#define VR_MPS_PAGE_1 0x01
#define VR_MPS_PAGE_3 0x03
#define VR_MPS_PAGE_4 0x04

#define VR_MPS_PAGE_14 0x14
#define VR_MPS_PAGE_24 0x24
#define VR_MPS_PAGE_34 0x34
#define VR_MPS_PAGE_44 0x44
#define VR_MPS_PAGE_54 0x54
#define VR_MPS_PAGE_64 0x64

#define VR_MPS_PAGE_9E 0x9E
#define VR_MPS_PAGE_9F 0x9F

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
	MODE_USR,
	MODE_MULTI_CFG_12,
	MODE_MULTI_CFG_34,
	MODE_MULTI_CFG_56,
	MODE_MULTI_CFG_ALL,
};

struct mp289x_data {
	uint16_t cfg_id;
	uint8_t page;
	uint8_t reg_addr;
	uint8_t reg_data[4];
	uint8_t reg_len;
};

struct mp289x_config {
	uint8_t mode;
	uint16_t cfg_id;
	uint16_t wr_cnt;
	uint16_t product_id_exp;
	uint8_t cfg_rev;
	uint16_t crc_usr;
	uint16_t crc_gp12;
	uint16_t crc_gp34;
	uint16_t crc_gp56;
	struct mp289x_data *pdata;
};

static bool mp289x_set_page(uint8_t bus, uint8_t addr, uint8_t page)
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

static bool mp289x_write_data(uint8_t bus, uint8_t addr, struct mp289x_data *data)
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

static uint8_t mp289x_check_err_status(uint8_t bus, uint8_t addr)
{
	if (mp289x_set_page(bus, addr, VR_MPS_PAGE_0) == false) {
		LOG_ERR("Failed to set page before reading error status");
		return 0xFF;
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
		return 0xFF;
	}

	if (i2c_msg.data[0])
		LOG_WRN("Get error status 0x%x", i2c_msg.data[0]);

	return i2c_msg.data[0];
}

bool mp289x_rev_get(uint8_t bus, uint8_t addr, uint16_t *rev)
{
	CHECK_NULL_ARG_WITH_RETURN(rev, false);

	if (mp289x_set_page(bus, addr, VR_MPS_PAGE_0) == false) {
		LOG_ERR("Failed to set page before reading config revision");
		return false;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = VR_REG_CFG_REV;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read config revision");
		return false;
	}

	*rev = (i2c_msg.data[1] << 8) | i2c_msg.data[0];

	return true;
}

static bool mp289x_crc_get(uint8_t bus, uint8_t addr, uint8_t mode, uint16_t *crc)
{
	CHECK_NULL_ARG_WITH_RETURN(crc, false);

	uint8_t page = 0;
	uint8_t crc_hi_cmd_code = 0;
	uint8_t crc_lo_cmd_code = 0;

	*crc = 0;

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	switch (mode) {
	case MODE_USR:
		if (mp289x_set_page(bus, addr, VR_MPS_PAGE_0) == false) {
			LOG_ERR("Failed to set page before reading user crc");
			return false;
		}

		i2c_msg.tx_len = 1;
		i2c_msg.rx_len = 2;
		i2c_msg.data[0] = VR_REG_CRC_USR;

		if (i2c_master_read(&i2c_msg, retry)) {
			LOG_ERR("Failed to read user crc");
			return false;
		}

		*crc = (i2c_msg.data[1] << 8) | i2c_msg.data[0];
		return true;

	case MODE_MULTI_CFG_12:
		page = VR_MPS_PAGE_9E;
		crc_hi_cmd_code = VR_REG_CRC_GP12_HI;
		crc_lo_cmd_code = VR_REG_CRC_GP12_LO;
		break;
	case MODE_MULTI_CFG_34:
		page = VR_MPS_PAGE_9F;
		crc_hi_cmd_code = VR_REG_CRC_GP34_HI;
		crc_lo_cmd_code = VR_REG_CRC_GP34_LO;
		break;
	case MODE_MULTI_CFG_56:
		page = VR_MPS_PAGE_9F;
		crc_hi_cmd_code = VR_REG_CRC_GP56_HI;
		crc_lo_cmd_code = VR_REG_CRC_GP56_LO;
		break;
	default:
		LOG_ERR("Invalid mode %d select", mode);
		return false;
		break;
	}

	if (mp289x_set_page(bus, addr, page) == false) {
		LOG_ERR("Failed to set page before reading group crc");
		return false;
	}

	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = crc_hi_cmd_code;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read group crc high byte");
		return false;
	}

	*crc |= (i2c_msg.data[0] << 8);

	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = crc_lo_cmd_code;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read group crc low byte");
		return false;
	}

	*crc |= (i2c_msg.data[0]);

	return true;
}

static bool mp289x_store(uint8_t bus, uint8_t addr, uint8_t mode)
{
	uint8_t page = 0;
	uint8_t cmd_code = 0;

	switch (mode) {
	case MODE_USR:
		page = 0;
		cmd_code = VR_REG_STORE_USR;
		break;
	case MODE_MULTI_CFG_12:
		page = 4;
		cmd_code = VR_REG_STORE_CFG_GP12;
		break;
	case MODE_MULTI_CFG_34:
		page = 4;
		cmd_code = VR_REG_STORE_CFG_GP34;
		break;
	case MODE_MULTI_CFG_56:
		page = 4;
		cmd_code = VR_REG_STORE_CFG_GP56;
		break;
	case MODE_MULTI_CFG_ALL:
		page = 0;
		cmd_code = VR_REG_STORE_CFG_ALL;
		break;

	default:
		LOG_ERR("Invalid store mode %d", mode);
		return false;
	}

	if (mp289x_set_page(bus, addr, page) == false) {
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
		if (mp289x_set_page(bus, addr, VR_MPS_PAGE_3) == false) {
			LOG_ERR("Failed to set page before check status");
			return false;
		}

		i2c_msg.tx_len = 1;
		i2c_msg.rx_len = 1;
		i2c_msg.data[0] = VR_REG_STORE_STAT;

		if (i2c_master_read(&i2c_msg, retry) == 0) {
			if (i2c_msg.data[0] == 1)
				break;
		}

		wait_time--;
	}

	if (!wait_time) {
		LOG_ERR("Failed to store data, ret 0x%x", i2c_msg.data[0]);
		return false;
	}

	if (mp289x_check_err_status(bus, addr)) {
		LOG_ERR("Get error after program!");
		return false;
	}

	return true;
}

static bool mp289x_pre_update(uint8_t bus, uint8_t addr)
{
	if (mp289x_set_page(bus, addr, VR_MPS_PAGE_0) == false) {
		return false;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = VR_REG_STAT_PMBUS;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read register 0x%02X", VR_REG_STAT_PMBUS);
		return false;
	}

	if (((i2c_msg.data[0] | (i2c_msg.data[1] << 8)) & BIT(15)) == 0x00) {
		LOG_ERR("PMBus write is block by password!");
		return false;
	}

	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = VR_REG_EN_WR_PROT;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read register 0x%02X", VR_REG_EN_WR_PROT);
		return false;
	}

	if (i2c_msg.data[0] != 0x63) {
		LOG_INF("Try to unlock MTP...");
		i2c_msg.tx_len = 2;
		i2c_msg.data[0] = VR_REG_EN_WR_PROT;
		i2c_msg.data[1] = 0x63;

		if (i2c_master_write(&i2c_msg, retry)) {
			LOG_ERR("Failed to unlock MTP");
			return false;
		}
	}

	uint8_t state_7e = mp289x_check_err_status(bus, addr);
	if (state_7e) {
		LOG_WRN("Get unexpected 7E status 0x%x", state_7e);

		/* Try to clear 7E bit[4] */
		if ((state_7e & BIT(4)) != 0) {
			if (mp289x_set_page(bus, addr, VR_MPS_PAGE_3) == false) {
				LOG_ERR("Failed to set page before get error status from 0x%x",
					VR_REG_STORE_STAT);
				return false;
			}

			i2c_msg.tx_len = 1;
			i2c_msg.rx_len = 1;
			i2c_msg.data[0] = VR_REG_STORE_STAT;

			if (i2c_master_read(&i2c_msg, retry)) {
				LOG_ERR("Failed to read register 0x%02X", VR_REG_STORE_STAT);
				return false;
			}

			LOG_WRN("Got 0x%x status error 0x%x in previous update", VR_REG_STORE_STAT,
				i2c_msg.data[0]);

			if (mp289x_set_page(bus, addr, VR_MPS_PAGE_0) == false) {
				LOG_ERR("Failed to set page before clear mtp error");
				return false;
			}

			i2c_msg.tx_len = 1;
			i2c_msg.data[0] = VR_REG_MTP_FAULT_CLR;

			if (i2c_master_write(&i2c_msg, retry)) {
				LOG_ERR("Failed to clear 7E fault bit[4]");
				return false;
			}

			state_7e &= ~BIT(4);
		}

		/* Try to clear 7E bit[7:5][2:0] */
		if (state_7e) {
			if (mp289x_set_page(bus, addr, VR_MPS_PAGE_0) == false) {
				LOG_ERR("Failed to set page before clear fault");
				return false;
			}

			i2c_msg.tx_len = 1;
			i2c_msg.data[0] = VR_REG_FAULT_CLR;

			if (i2c_master_write(&i2c_msg, retry)) {
				LOG_ERR("Failed to clear 7E fault bit[7:5][2:0]");
				return false;
			}
		}
	}

	return true;
}

static bool mp289x_post_update(uint8_t bus, uint8_t addr, struct mp289x_config *dev_cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(dev_cfg, false);

	uint16_t checksum = 0;
	if (mp289x_crc_get(bus, addr, MODE_USR, &checksum) == false)
		return false;

	if (dev_cfg->crc_usr && checksum != dev_cfg->crc_usr) {
		LOG_ERR("User checksum mismatch! (0x%x != 0x%x)", checksum, dev_cfg->crc_usr);
		return false;
	}
	LOG_INF("User checksum: 0x%x", checksum);

	if (mp289x_crc_get(bus, addr, MODE_MULTI_CFG_12, &checksum) == false)
		return false;

	if (dev_cfg->crc_gp12 && checksum != dev_cfg->crc_gp12) {
		LOG_ERR("Group1&2 checksum mismatch! (0x%x != 0x%x)", checksum, dev_cfg->crc_gp12);
		return false;
	}
	LOG_INF("GROUP1&2 checksum: 0x%x", checksum);

	if (mp289x_crc_get(bus, addr, MODE_MULTI_CFG_34, &checksum) == false)
		return false;

	if (dev_cfg->crc_gp34 && checksum != dev_cfg->crc_gp34) {
		LOG_ERR("Group3&4 checksum mismatch! (0x%x != 0x%x)", checksum, dev_cfg->crc_gp34);
		return false;
	}
	LOG_INF("GROUP3&4 checksum: 0x%x", checksum);

	if (mp289x_crc_get(bus, addr, MODE_MULTI_CFG_56, &checksum) == false)
		return false;

	if (dev_cfg->crc_gp56 && checksum != dev_cfg->crc_gp56) {
		LOG_ERR("Group5&6 checksum mismatch! (0x%x != 0x%x)", checksum, dev_cfg->crc_gp56);
		return false;
	}
	LOG_INF("GROUP5&6 checksum: 0x%x", checksum);

	return true;
}

static bool parsing_image(uint8_t *img_buff, uint32_t img_size, struct mp289x_config *dev_cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(img_buff, false);
	CHECK_NULL_ARG_WITH_RETURN(dev_cfg, false);

	bool ret = false;

	/* Parsing image */
	int max_line = MAX_CMD_LINE;
	dev_cfg->pdata = (struct mp289x_data *)malloc(sizeof(struct mp289x_data) * max_line);
	if (!dev_cfg->pdata) {
		LOG_ERR("pdata malloc failed!");
		goto exit;
	}

	struct mp289x_data *cur_line = &dev_cfg->pdata[0];
	uint8_t cur_ele_idx = 0;
	uint32_t data_store = 0;
	uint8_t data_idx = 0;
	int val = -1;
	bool is_crc = false;
	bool is_data = true;
	uint8_t cur_crc_mode = 0xFF;
	dev_cfg->wr_cnt = 0;
	for (int i = 0; i < img_size; i++) {
		/* check valid */
		if (!img_buff[i]) {
			LOG_ERR("Get invalid buffer data at index %d", i);
			goto exit;
		}

		if (is_crc == true) {
			if (cur_ele_idx == ATE_CONF_ID) {
				if ((i + (strlen(CRC_CHECK_STOP_KEY) - 1)) < img_size) {
					if (!strncmp(&img_buff[i], CRC_CHECK_STOP_KEY,
						     strlen(CRC_CHECK_STOP_KEY))) {
						is_crc = false;
						break;
					}
				}
			}

			if (img_buff[i] == 0x09) {
				if ((cur_ele_idx + 1) == ATE_REG_NAME) {
					if (!strncmp(&img_buff[i + 1], CRC_USR_KEY,
						     strlen(CRC_USR_KEY))) {
						cur_crc_mode = MODE_USR;
						i += (strlen(CRC_USR_KEY) + 1);
						cur_ele_idx++;
					} else if (!strncmp(&img_buff[i + 1], CRC_GP12_KEY,
							    strlen(CRC_GP12_KEY))) {
						cur_crc_mode = MODE_MULTI_CFG_12;
						i += (strlen(CRC_GP12_KEY) + 1);
						cur_ele_idx++;
					} else if (!strncmp(&img_buff[i + 1], CRC_GP34_KEY,
							    strlen(CRC_GP34_KEY))) {
						cur_crc_mode = MODE_MULTI_CFG_34;
						i += (strlen(CRC_GP34_KEY) + 1);
						cur_ele_idx++;
					} else if (!strncmp(&img_buff[i + 1], CRC_GP56_KEY,
							    strlen(CRC_GP56_KEY))) {
						cur_crc_mode = MODE_MULTI_CFG_56;
						i += (strlen(CRC_GP56_KEY) + 1);
						cur_ele_idx++;
					}
				} else if (cur_ele_idx == ATE_REG_DATA_HEX) {
					switch (cur_crc_mode) {
					case MODE_USR:
						dev_cfg->crc_usr = (uint16_t)data_store;
						break;
					case MODE_MULTI_CFG_12:
						dev_cfg->crc_gp12 = (uint16_t)data_store;
						break;
					case MODE_MULTI_CFG_34:
						dev_cfg->crc_gp34 = (uint16_t)data_store;
						break;
					case MODE_MULTI_CFG_56:
						dev_cfg->crc_gp56 = (uint16_t)data_store;
						break;
					default:
						LOG_ERR("Got unknow crc mode %d", cur_crc_mode);
						goto exit;
					}
				}

				data_store = 0;
				cur_ele_idx++;
				continue;
			} else if (img_buff[i] == 0x0d) {
				i++; //skip 'a'
				cur_ele_idx = ATE_CONF_ID;
				continue;
			} else {
				val = ascii_to_val(img_buff[i]);
				if (val == -1)
					continue;

				data_store = (data_store << 4) | val;
				continue;
			}
		}

		if ((cur_ele_idx == ATE_CONF_ID)) {
			if ((i + (strlen(USR_END_KEY) - 1)) < img_size) {
				if (!strncmp(&img_buff[i], USR_END_KEY, strlen(USR_END_KEY))) {
					i += (strlen(USR_END_KEY) + 1); //pass 'END' + '\n'
					is_data = false;
					continue;
				}
			}
			if ((i + (strlen(CRC_CHECK_START_KEY) - 1)) < img_size) {
				if (!strncmp(&img_buff[i], CRC_CHECK_START_KEY,
					     strlen(CRC_CHECK_START_KEY))) {
					is_crc = true;
					i += (strlen(CRC_CHECK_START_KEY) -
					      1); //pass 'CRC_CHECK_START'
					continue;
				}
			}
		}

		if (is_data == false)
			continue;

		if (((img_buff[i] != 0x09) && img_buff[i] != 0x0d)) {
			// pass non hex charactor
			val = ascii_to_val(img_buff[i]);
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

bool mp289x_fwupdate(uint8_t bus, uint8_t addr, uint8_t *img_buff, uint32_t img_size)
{
	CHECK_NULL_ARG_WITH_RETURN(img_buff, false);

	uint8_t ret = false;

	/* Step1. Before update */
	if (mp289x_pre_update(bus, addr) == false) {
		LOG_ERR("Pre update failed!");
		return false;
	}

	/* Step2. Image parsing */
	struct mp289x_config dev_cfg = { 0 };
	if (parsing_image(img_buff, img_size, &dev_cfg) == false) {
		LOG_ERR("Failed to parsing image!");
		goto exit;
	}

	LOG_INF("* crc user: 0x%x", dev_cfg.crc_usr);
	LOG_INF("* crc group12: 0x%x", dev_cfg.crc_gp12);
	LOG_INF("* crc group34: 0x%x", dev_cfg.crc_gp34);
	LOG_INF("* crc group56: 0x%x", dev_cfg.crc_gp56);

	/* Step3. Firmware update */
	uint8_t last_page = 0xFF;
	struct mp289x_data *cur_data;
	uint16_t line_idx = 0;
	uint16_t multi_cfg_start_idx = 0;

	/* program USER data */
	for (line_idx = 0; line_idx < dev_cfg.wr_cnt; line_idx++) {
		cur_data = &dev_cfg.pdata[line_idx];
		if ((cur_data->page == VR_MPS_PAGE_0) || (cur_data->page == VR_MPS_PAGE_1)) {
			if (cur_data->page != last_page) {
				if (mp289x_set_page(bus, addr, cur_data->page) == false) {
					LOG_ERR("Failed to set page before program data");
					goto exit;
				}
				last_page = cur_data->page;
			}

			LOG_DBG("USER page 0x%x wr 0x%x:", cur_data->page, cur_data->reg_addr);
			LOG_HEXDUMP_DBG(cur_data->reg_data, cur_data->reg_len, "");
			if (mp289x_write_data(bus, addr, cur_data) == false)
				goto exit;

			uint8_t percent = ((line_idx + 1) * 100) / dev_cfg.wr_cnt;
			if (percent % 10 == 0)
				LOG_INF("updated: %d%% (line: %d/%d page: %d)", percent,
					line_idx + 1, dev_cfg.wr_cnt, cur_data->page);
		} else {
			multi_cfg_start_idx = line_idx;
			break;
		}
	}

	if (mp289x_store(bus, addr, MODE_USR) == false) {
		LOG_ERR("Failed to store USER data to MTP!");
		goto exit;
	}

	/* program Multi-config data */
	if (mp289x_set_page(bus, addr, VR_MPS_PAGE_0) == false) {
		LOG_ERR("Failed to set page before enable multi-config program");
		goto exit;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = VR_REG_MTP_CTRL;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read error status");
		return false;
	}

	uint16_t state_d0 = i2c_msg.data[0] | (i2c_msg.data[1] << 8);
	state_d0 |= (BIT(5) | BIT(12));

	i2c_msg.rx_len = 0;
	i2c_msg.tx_len = 3;
	i2c_msg.data[0] = VR_REG_MTP_CTRL;
	i2c_msg.data[1] = state_d0 & 0xFF;
	i2c_msg.data[2] = (state_d0 >> 8) & 0xFF;

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to enable multi-config program");
		return false;
	}

	uint8_t group_idx = 1;
	last_page = 0xFF;

	for (line_idx = multi_cfg_start_idx; line_idx < dev_cfg.wr_cnt; line_idx++) {
		cur_data = &dev_cfg.pdata[line_idx];
		group_idx = (cur_data->page >> 4);

		if (group_idx < 1 || group_idx > 6) {
			LOG_WRN("Invalid group index %d found from multi-config part", group_idx);
			continue;
		}

		if (group_idx % 2)
			cur_data->page = VR_MPS_PAGE_4; //group 1/3/5
		else
			cur_data->page = VR_MPS_PAGE_1; //group 2/4/6

		if (cur_data->page != last_page) {
			// store last group x and x-1 data first
			if (last_page == VR_MPS_PAGE_1) {
				if (mp289x_store(bus, addr, group_idx / 2) == false) {
					LOG_ERR("Failed to store group%d&%d MULTI-CONFIG data to MTP!",
						group_idx - 1, group_idx);
					goto exit;
				}
			}

			if (mp289x_set_page(bus, addr, cur_data->page) == false) {
				LOG_ERR("Failed to set page before program multi-config");
				goto exit;
			}

			if (cur_data->page == VR_MPS_PAGE_1) {
				i2c_msg.rx_len = 0;
				i2c_msg.tx_len = 2;
				i2c_msg.data[0] = VR_REG_OFFLINE_CTL;
				i2c_msg.data[1] = 0x8C; //borrow page1 for program group2/4/6

				if (i2c_master_write(&i2c_msg, retry)) {
					LOG_ERR("Failed to borrow page1 for multi-config program");
					return false;
				}
			}

			last_page = cur_data->page;
		}

		LOG_DBG("MULTI-CFG group %d page 0x%x wr 0x%x:", group_idx, cur_data->page,
			cur_data->reg_addr);
		LOG_HEXDUMP_DBG(cur_data->reg_data, cur_data->reg_len, "");
		if (mp289x_write_data(bus, addr, cur_data) == false)
			goto exit;

		uint8_t percent = ((line_idx + 1) * 100) / dev_cfg.wr_cnt;
		if (percent % 10 == 0)
			LOG_INF("updated: %d%% (line: %d/%d group: %d)", percent, line_idx + 1,
				dev_cfg.wr_cnt, group_idx);
	}

	//store last group x and x-1
	if (mp289x_store(bus, addr, group_idx / 2) == false) {
		LOG_ERR("Failed to store group%d and group%d data", group_idx - 1, group_idx);
		goto exit;
	}

	if (mp289x_post_update(bus, addr, &dev_cfg) == false) {
		LOG_ERR("Post update failed!");
		goto exit;
	}

	ret = true;
exit:
	SAFE_FREE(dev_cfg.pdata);
	return ret;
}

static bool mp289x_pre_read(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);

	switch (cfg->offset) {
	case PMBUS_READ_VOUT:
	case PMBUS_READ_IOUT:
	case PMBUS_READ_TEMPERATURE_1:
	case PMBUS_READ_POUT:
		if (mp289x_set_page(cfg->port, cfg->target_addr, VR_MPS_PAGE_0) == false)
			return false;
		break;

	default:
		LOG_WRN("Non-support ofset 0x%x", cfg->offset);
		break;
	}

	return true;
}

uint8_t mp289x_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	if (mp289x_pre_read(cfg) == false) {
		LOG_ERR("sensor num: 0x%x pre-read failed", cfg->num);
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
		val = ((msg.data[1] << 8) | msg.data[0]) & BIT_MASK(12);
		val *= 0.001;
		break;
	case PMBUS_READ_IOUT:
		val = ((msg.data[1] << 8) | msg.data[0]) & BIT_MASK(12);
		val = ((total_current_set == 0) ? (0.25 * val) : (0.5 * val));
		break;
	case PMBUS_READ_TEMPERATURE_1:
		val = ((msg.data[1] << 8) | msg.data[0]) & BIT_MASK(12);
		val *= 0.1;
		break;
	case PMBUS_READ_POUT:
		val = ((msg.data[1] << 8) | msg.data[0]) & BIT_MASK(11);
		val = ((total_current_set == 0) ? (0.5 * val) : (1 * val));
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

uint8_t mp289x_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	if (mp289x_set_page(cfg->port, cfg->target_addr, VR_MPS_PAGE_4) == false)
		return SENSOR_INIT_UNSPECIFIED_ERROR;

	uint8_t i2c_max_retry = 5;

	I2C_MSG msg = { 0 };
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = VR_REG_MFR_IMON_DIGI_GAIN;

	if (i2c_master_read(&msg, i2c_max_retry)) {
		LOG_ERR("Failed to get MFR_IMON_DIGI_GAIN");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	/* Get bit11 IMON_RESO */
	total_current_set = (msg.data[1] & BIT(3)) >> 3;

	cfg->read = mp289x_read;
	return SENSOR_INIT_SUCCESS;
}
