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
#include "raa228249.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(raa228249);

#define raa228249_READ_VOUT_RESOLUTION 0.001
#define raa228249_READ_IOUT_RESOLUTION 0.1

// RAA GEN3p5
#define VR_RAA_REG_REMAIN_WR 0x35
#define VR_RAA_REG_DMA_ADDR 0xC7
#define VR_RAA_REG_DMA_DATA 0xC5
#define VR_RAA_REG_PROG_STATUS 0x83
#define VR_RAA_REG_CRC 0xF8
#define VR_RAA_REG_DEVID 0xAD

#define VR_RAA_DEV_ID_LEN 4
#define VR_RAA_DEV_REV_LEN 4
#define VR_RAA_CHECKSUM_LEN 4

#define VR_RAA_CFG_ID (7)
#define VR_RAA_GEN3P5_FILE_HEAD (5)
#define VR_RAA_GEN3P5_CRC (336 - VR_RAA_GEN3P5_FILE_HEAD)

#define VR_WARN_REMAIN_WR 3

#define MAX_CMD_LINE 1500

#define RAA228249_VOUT_MAX_REG 0x24
#define RAA228249_VOUT_MIN_REG 0x2B

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

struct raa228249_config {
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
	LINE_NEW,
	LINE_PARSING,
};

static uint8_t ascii_to_byte(uint8_t *ascii_buf)
{
	CHECK_NULL_ARG_WITH_RETURN(ascii_buf, SENSOR_UNSPECIFIED_ERROR);
	return ascii_to_val(ascii_buf[0]) << 4 | ascii_to_val(ascii_buf[1]);
}

bool raa228249_i2c_read(uint8_t bus, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	memset(data, 0, len);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = len;
	i2c_msg.data[0] = reg;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read mp29816a, bus: %d, addr: 0x%x, reg: 0x%x", bus, addr, reg);
		return false;
	}

	memcpy(data, i2c_msg.data, len);
	return true;
}

bool raa228249_i2c_write(uint8_t bus, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = len + 1;

	i2c_msg.data[0] = reg;

	if (len > 0)
		memcpy(&i2c_msg.data[1], data, len);

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to write mp29816a, bus: %d, addr: 0x%x, reg: 0x%x", bus, addr, reg);
		return false;
	}

	return true;
}

bool raa228249_get_vout_max(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);

	uint8_t data[2] = { 0 };
	if (!raa228249_i2c_read(cfg->port, cfg->target_addr, RAA228249_VOUT_MAX_REG, data,
				sizeof(data))) {
		return false;
	}

	uint16_t val = data[0] | (data[1] << 8);
	*millivolt = val; // 1mV / LSB

	return true;
}

bool raa228249_get_vout_min(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);

	uint8_t data[2] = { 0 };
	if (!raa228249_i2c_read(cfg->port, cfg->target_addr, RAA228249_VOUT_MIN_REG, data,
				sizeof(data))) {
		return false;
	}

	uint16_t val = data[0] | (data[1] << 8);
	*millivolt = val; // 1mV / LSB

	return true;
}

bool raa228249_set_vout_max(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);

	uint8_t data[2] = { 0 };
	data[0] = *millivolt & 0xFF;
	data[1] = (*millivolt >> 8) & 0xFF;

	if (!raa228249_i2c_write(cfg->port, cfg->target_addr, RAA228249_VOUT_MAX_REG, data,
				 sizeof(data))) {
		return false;
	}

	return true;
}

bool raa228249_set_vout_min(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);

	uint8_t data[2] = { 0 };
	data[0] = *millivolt & 0xFF;
	data[1] = (*millivolt >> 8) & 0xFF;

	if (!raa228249_i2c_write(cfg->port, cfg->target_addr, RAA228249_VOUT_MIN_REG, data,
				 sizeof(data))) {
		return false;
	}

	return true;
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

int raa228249_get_remaining_wr(uint8_t bus, uint8_t addr, uint8_t *remain)
{
	CHECK_NULL_ARG_WITH_RETURN(remain, false);
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

bool raa228249_get_crc(uint8_t bus, uint8_t addr, uint32_t *crc)
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

static bool parsing_image(uint8_t *img_buff, uint32_t img_size, struct raa228249_config *dev_cfg)
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
			case VR_RAA_GEN3P5_CRC:
				for (int j = 0; j < VR_RAA_CHECKSUM_LEN; j++) {
					((uint8_t *)&dev_cfg->crc_exp)[j] =
						ascii_to_byte(&img_buff[i + 8 + 2 * j]);
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

bool raa228249_fwupdate(uint8_t bus, uint8_t addr, uint8_t *img_buff, uint32_t img_size)
{
	CHECK_NULL_ARG_WITH_RETURN(img_buff, false);

	uint8_t ret = false;
	uint8_t remain = 0;
	uint32_t devid = 0;

	// check remaining writes
	if (raa228249_get_remaining_wr(bus, addr, &remain) < 0) {
		return false;
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
	struct raa228249_config dev_cfg = { 0 };
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

bool raa228249_get_vout_command(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);

	uint8_t data[2] = { 0 };
	if (!raa228249_i2c_read(cfg->port, cfg->target_addr, PMBUS_VOUT_COMMAND, data,
				sizeof(data))) {
		return false;
	}

	uint16_t val = data[0] | (data[1] << 8);

	*millivolt = val;

	return true;
}

bool raa228249_set_vout_command(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);

	uint8_t data[2] = { 0 };
	data[0] = *millivolt & 0xFF;
	data[1] = (*millivolt >> 8) & 0xFF;

	if (!raa228249_i2c_write(cfg->port, cfg->target_addr, PMBUS_VOUT_COMMAND, data,
				 sizeof(data))) {
		return false;
	}

	return true;
}

uint8_t raa228249_read(sensor_cfg *cfg, int *reading)
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
		/* Unsigned integer */
		uint16_t read_value = (msg.data[1] << 8) | msg.data[0];
		val = read_value * raa228249_READ_VOUT_RESOLUTION;

	} else if (cfg->offset == PMBUS_READ_TEMPERATURE_1 || cfg->offset == PMBUS_READ_POUT) {
		/* 2's complement */
		int16_t read_value = (msg.data[1] << 8) | msg.data[0];
		val = read_value;

	} else if (cfg->offset == PMBUS_READ_IOUT) {
		/* 2's complement */
		int16_t read_value = (msg.data[1] << 8) | msg.data[0];
		val = read_value * raa228249_READ_IOUT_RESOLUTION;

	} else {
		return SENSOR_FAIL_TO_ACCESS;
	}

	sval->integer = (int)val;
	sval->fraction = (val - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t raa228249_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = raa228249_read;
	return SENSOR_INIT_SUCCESS;
}
