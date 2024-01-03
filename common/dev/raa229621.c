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
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"

#include <logging/log.h>

LOG_MODULE_REGISTER(raa229621);

// RAA GEN3_
#define VR_RAA_REG_REMAIN_WR 0x35
#define VR_RAA_REG_DMA_ADDR 0xC7
#define VR_RAA_REG_DMA_DATA 0xC5
#define VR_RAA_REG_PROG_STATUS 0x7E
#define VR_RAA_REG_CRC 0x94
#define VR_RAA_REG_DEVID 0xAD

#define VR_RAA_REG_HEX_MODE_CFG0 0x87
#define VR_RAA_REG_HEX_MODE_CFG1 0xBD

#define VR_RAA_GEN3_SW_REV_MIN 0x06

#define VR_RAA_DEV_ID_LEN 4
#define VR_RAA_DEV_REV_LEN 4
#define VR_RAA_CHECKSUM_LEN 4

#define VR_RAA_CFG_ID (7)
#define VR_RAA_GEN3_FILE_HEAD (5)
#define VR_RAA_GEN3_LEGACY_CRC (276 - VR_RAA_GEN3_FILE_HEAD)
#define VR_RAA_GEN3_PRODUCTION_CRC (290 - VR_RAA_GEN3_FILE_HEAD)

#define VR_WARN_REMAIN_WR 3

#define MAX_CMD_LINE 1024

struct raa_data {
	union {
		uint8_t raw[32];
		struct {
			uint8_t addr;
			uint8_t cmd;
			uint8_t data[];
		} __attribute__((packed));
	};
	uint8_t len;
};

struct raa229621_config {
	uint8_t addr;
	uint8_t mode;
	uint8_t cfg_id;
	uint16_t wr_cnt;
	uint32_t devid_exp;
	uint16_t product_id_exp;
	uint32_t rev_exp;
	uint32_t crc_exp;
	struct raa_data *pdata;
};

enum {
	RAA_GEN3_LEGACY,
	RAA_GEN3_PRODUCTION,
};

enum {
	LINE_NEW,
	LINE_PARSING,
};

static uint8_t ascii_to_byte(uint8_t *ascii_buf)
{
	CHECK_NULL_ARG_WITH_RETURN(ascii_buf, SENSOR_UNSPECIFIED_ERROR);
	return ascii_to_val(ascii_buf[0]) << 4 | ascii_to_val(ascii_buf[1]);
}

static bool adjust_of_twos_complement(uint8_t offset, int *val)
{
	if (val == NULL) {
		LOG_ERR("Input value is NULL");
		return false;
	}

	if ((offset == PMBUS_READ_IOUT) || (offset == PMBUS_READ_POUT)) {
		bool is_negative_val = ((*val & BIT(15)) == 0 ? false : true);
		if (is_negative_val) {
			*val = 0;
		}
		return true;
	}
	return false;
}

uint8_t raa229621_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	uint8_t offset = cfg->offset;

	I2C_MSG msg;

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = offset;

	if (i2c_master_read(&msg, retry)) {
		return SENSOR_FAIL_TO_ACCESS;
	}

	sensor_val *sval = (sensor_val *)reading;
	memset(sval, 0, sizeof(sensor_val));
	bool ret = false;
	int val = 0;
	val = (msg.data[1] << 8) | msg.data[0];

	switch (offset) {
	case PMBUS_READ_VOUT:
		/* 1 mV/LSB, unsigned integer */
		sval->integer = val / 1000;
		sval->fraction = val % 1000;
		break;
	case PMBUS_READ_IOUT:
		/* 0.1 A/LSB, 2's complement */
		ret = adjust_of_twos_complement(offset, &val);
		if (ret == false) {
			LOG_ERR("Adjust reading IOUT value failed - sensor number: 0x%x", cfg->num);
			return SENSOR_UNSPECIFIED_ERROR;
		}
		sval->integer = (int16_t)val / 10;
		sval->fraction = ((int16_t)val - (sval->integer * 10)) * 100;
		break;
	case PMBUS_READ_TEMPERATURE_1:
		/* 1 Degree C/LSB, 2's complement */
		sval->integer = val;
		sval->fraction = 0;
		break;
	case PMBUS_READ_POUT:
		/* 1 Watt/LSB, 2's complement */
		ret = adjust_of_twos_complement(offset, &val);
		if (ret == false) {
			LOG_ERR("Adjust reading POUT value failed - sensor number: 0x%x", cfg->num);
			return SENSOR_UNSPECIFIED_ERROR;
		}
		sval->integer = val;
		sval->fraction = 0;
		break;
	default:
		LOG_ERR("Not support offset: 0x%x", offset);
		return SENSOR_FAIL_TO_ACCESS;
		break;
	}

	return SENSOR_READ_SUCCESS;
}

static int raa_dma_rd(uint8_t bus, uint8_t addr, uint8_t *reg, uint8_t *resp)
{
	CHECK_NULL_ARG_WITH_RETURN(reg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, SENSOR_UNSPECIFIED_ERROR);
	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;

	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 3;
	i2c_msg.data[0] = VR_RAA_REG_DMA_ADDR;
	memcpy(&i2c_msg.data[1], reg, 2);

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("write register 0x%02X failed", reg[0]);
		return false;
	}

	i2c_msg.tx_len = 1;
	i2c_msg.data[0] = VR_RAA_REG_DMA_DATA;
	i2c_msg.rx_len = 4;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("read register 0x%02X failed", reg[0]);
		return false;
	}

	memcpy(resp, &i2c_msg.data[0], i2c_msg.rx_len);
	return 0;
}

int raa229621_get_hex_mode(uint8_t bus, uint8_t addr, uint8_t *mode)
{
	CHECK_NULL_ARG_WITH_RETURN(mode, SENSOR_UNSPECIFIED_ERROR);
	uint8_t tbuf[2], rbuf[4];

	tbuf[0] = VR_RAA_REG_HEX_MODE_CFG0;
	tbuf[1] = VR_RAA_REG_HEX_MODE_CFG1;
	if (raa_dma_rd(bus, addr, tbuf, rbuf) < 0) {
		LOG_ERR("Read HEX mode failed from dev: 0x%x", addr);
		return -1;
	}

	*mode = (rbuf[0] == 0) ? RAA_GEN3_LEGACY : RAA_GEN3_PRODUCTION;

	return 0;
}

int raa229621_get_remaining_wr(uint8_t bus, uint8_t addr, uint8_t *remain)
{
	CHECK_NULL_ARG_WITH_RETURN(remain, SENSOR_UNSPECIFIED_ERROR);
	uint8_t tbuf[2], rbuf[4];

	tbuf[0] = VR_RAA_REG_REMAIN_WR;
	tbuf[1] = 0x00;
	if (raa_dma_rd(bus, addr, tbuf, rbuf) < 0) {
		LOG_ERR("Read NVM counter failed from dev: 0x%x", addr);
		return -1;
	}

	*remain = rbuf[0];

	return 0;
}

static int get_raa_devid(uint8_t bus, uint8_t addr, uint32_t *devid)
{
	CHECK_NULL_ARG_WITH_RETURN(devid, SENSOR_UNSPECIFIED_ERROR);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = VR_RAA_DEV_ID_LEN + 1;
	i2c_msg.data[0] = VR_RAA_REG_DEVID;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Read device id failed from dev: 0x%x", addr);
		return false;
	}

	memcpy(devid, &i2c_msg.data[1], VR_RAA_DEV_ID_LEN);

	return 0;
}

static int get_raa_polling_status(uint8_t bus, uint8_t addr)
{
	uint8_t tbuf[2], rbuf[4];
	int retry = 3;

	do {
		tbuf[0] = VR_RAA_REG_PROG_STATUS;
		tbuf[1] = 0x00;

		if (raa_dma_rd(bus, addr, tbuf, rbuf) < 0) {
			LOG_ERR("Read polling status failed from dev: 0x%x", addr);
			return -1;
		}

		// bit1 is held to 1, it means the action is successful
		if (rbuf[0] & 0x01) {
			break;
		}

		if ((--retry) <= 0) {
			LOG_ERR("program the device failed from dev: 0x%x", addr);
			return -1;
		}
		k_msleep(1000);
	} while (retry > 0);

	return 0;
}

bool raa229621_get_crc(uint8_t bus, uint8_t addr, uint32_t *crc)
{
	CHECK_NULL_ARG_WITH_RETURN(crc, false);
	uint8_t tbuf[2], rbuf[4];

	tbuf[0] = VR_RAA_REG_CRC;
	tbuf[1] = 0x00;

	if (raa_dma_rd(bus, addr, tbuf, rbuf) < 0) {
		LOG_ERR("Read crc failed from dev: 0x%x", addr);
		return false;
	}

	memcpy(crc, rbuf, sizeof(uint32_t));

	return true;
}

static bool parsing_image(uint8_t *img_buff, uint32_t img_size, struct raa229621_config *dev_cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(img_buff, false);
	CHECK_NULL_ARG_WITH_RETURN(dev_cfg, false);

	bool ret = false;
	int i, dcnt = 0;

	/* Parsing image */
	int max_line = MAX_CMD_LINE;
	dev_cfg->pdata = (struct raa_data *)malloc(sizeof(struct raa_data) * max_line);
	if (!dev_cfg->pdata) {
		LOG_ERR("pdata malloc failed!");
		goto exit;
	}

	uint8_t newline = LINE_NEW;

	for (i = 0; i < img_size; i++) {
		/* check valid */
		if (!img_buff[i]) {
			LOG_ERR("Get invalid buffer data at index %d", i);
			goto exit;
		}

		if ((!strncmp(&img_buff[i], "49", 2)) && (newline == LINE_NEW)) {
			newline = LINE_PARSING;
			if (!strncmp(&img_buff[i + 6], "AD", 2)) {
				for (int j = 0; j < VR_RAA_DEV_ID_LEN; j++) {
					((uint8_t *)&dev_cfg->devid_exp)[j] = ascii_to_byte(
						&img_buff[i + 6 + 2 * (VR_RAA_DEV_ID_LEN - j)]);
				}
				dev_cfg->addr = ascii_to_byte(&img_buff[i + 4]);
			} else if (!strncmp(&img_buff[i + 6], "AE", 2)) {
				for (int j = 0; j < VR_RAA_DEV_REV_LEN; j++) {
					((uint8_t *)&dev_cfg->rev_exp)[j] =
						ascii_to_byte(&img_buff[i + 8 + (2 * j)]);
				}

				if ((dev_cfg->rev_exp & 0xFF) < VR_RAA_GEN3_SW_REV_MIN) {
					dev_cfg->mode = RAA_GEN3_LEGACY;
				} else {
					dev_cfg->mode = RAA_GEN3_PRODUCTION;
				}
			} else {
				continue;
			}
		} else if ((img_buff[i] == 0x0D) && (img_buff[i + 1] == 0x0A)) {
			newline = LINE_NEW;
			i++;
		} else if (((!strncmp(&img_buff[i], "00", 2))) && (newline == LINE_NEW)) {
			newline = LINE_PARSING;
			dev_cfg->pdata[dcnt].len = ascii_to_byte(&img_buff[i + 2]) - 2;
			for (int j = 0; j < dev_cfg->pdata[dcnt].len + 1; j++) {
				((uint8_t *)&dev_cfg->pdata[dcnt].raw)[j] =
					ascii_to_byte(&img_buff[i + 4 + 2 * j]);
			}

			switch (dcnt) {
			case VR_RAA_CFG_ID:
				// set Configuration ID
				dev_cfg->cfg_id = ascii_to_byte(&img_buff[i + 8]) & 0x0F;
				break;
			case VR_RAA_GEN3_LEGACY_CRC:
				if (dev_cfg->mode == RAA_GEN3_LEGACY) {
					for (int j = 0; j < VR_RAA_CHECKSUM_LEN; j++) {
						((uint8_t *)&dev_cfg->crc_exp)[j] =
							ascii_to_byte(&img_buff[i + 8 + 2 * j]);
					}
				}
				break;
			case VR_RAA_GEN3_PRODUCTION_CRC:
				if (dev_cfg->mode == RAA_GEN3_PRODUCTION) {
					for (int j = 0; j < VR_RAA_CHECKSUM_LEN; j++) {
						((uint8_t *)&dev_cfg->crc_exp)[j] =
							ascii_to_byte(&img_buff[i + 8 + 2 * j]);
					}
				}
				break;
			}
			dcnt++;
			i = i + (2 * ascii_to_byte(&img_buff[i + 2]));
		} else {
			continue;
		}
	}
	LOG_INF("Configuration CRC: %08X", dev_cfg->crc_exp);
	dev_cfg->wr_cnt = dcnt;
	ret = true;

exit:
	if (ret == false)
		SAFE_FREE(dev_cfg->pdata);

	return ret;
}

bool raa229621_fwupdate(uint8_t bus, uint8_t addr, uint8_t *img_buff, uint32_t img_size)
{
	CHECK_NULL_ARG_WITH_RETURN(img_buff, false);

	uint8_t ret = false;
	uint8_t remain = 0, mode = 0xff;
	uint32_t devid = 0;

	// check mode
	if (raa229621_get_hex_mode(bus, addr, &mode)) {
		return -1;
	}

	// check remaining writes
	if (raa229621_get_remaining_wr(bus, addr, &remain) < 0) {
		return -1;
	}

	if (!remain) {
		LOG_ERR("No remaining writes");
		return false;
	}
	if (remain <= VR_WARN_REMAIN_WR) {
		LOG_WRN("The remaining writes %d is below the threshold value %d!", remain,
			VR_WARN_REMAIN_WR);
	}

	/* Step1. Image parsing */
	struct raa229621_config dev_cfg = { 0 };
	if (parsing_image(img_buff, img_size, &dev_cfg) == false) {
		LOG_ERR("Failed to parsing image!");
		goto exit;
	}

	// check device id
	if (get_raa_devid(bus, addr, &devid) < 0) {
		goto exit;
	}

	if (devid != dev_cfg.devid_exp) {
		LOG_ERR("device id 0x%08X mismatch, expect 0x%08X", devid, dev_cfg.devid_exp);
		goto exit;
	}

	// check mode
	if (mode != dev_cfg.mode) {
		LOG_ERR("HEX mode %u mismatch, expect %u", mode, dev_cfg.mode);
		goto exit;
	}

	// write configuration data
	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	for (int i = 0; i < dev_cfg.wr_cnt; i++) {
		i2c_msg.tx_len = dev_cfg.pdata[i].len;
		memcpy(i2c_msg.data, &dev_cfg.pdata[i].cmd, dev_cfg.pdata[i].len);
		if (i2c_master_write(&i2c_msg, retry)) {
			LOG_ERR("Failed to write config data to dev: 0x%x", addr);
			goto exit;
		}

		uint8_t percent = ((i + 1) * 100) / dev_cfg.wr_cnt;
		if (percent % 10 == 0) {
			LOG_INF("updated: %d%%", percent);
		}
	}

	// check the status
	if (get_raa_polling_status(bus, addr) < 0) {
		goto exit;
	}

	ret = true;
exit:
	SAFE_FREE(dev_cfg.pdata);
	return ret;
}

uint8_t raa229621_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = raa229621_read;
	return SENSOR_INIT_SUCCESS;
}
