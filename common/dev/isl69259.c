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
#include <stdlib.h>
#include <logging/log.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"
#include "isl69259.h"
#include "libutil.h"

LOG_MODULE_REGISTER(isl69259);

#define VR_IMG_HDR_SYMBOL 0x49
#define VR_IMG_BODY_SYMBOL 0x00
#define VR_WARN_REMAIN_WR 3

#define VR_RAA_REG_REMAIN_WR 0x35
#define VR_RAA_REG_DMA_ADDR 0xC7
#define VR_RAA_REG_DMA_DATA 0xC5

#define VR_RAA_REG_HEX_MODE_CFG0 0x87
#define VR_RAA_REG_HEX_MODE_CFG1 0xBD

#define VR_RAA_REG_CRC 0x94
#define VR_RAA_REG_PROG_STATUS 0x7E

//RAA GEN3
#define VR_RAA_GEN3_SW_REV_MIN 0x06
#define VR_RAA_GEN3_HW_REV_MIN 0x00

//RAA GEN2
#define VR_RAA_REG_GEN2_CRC 0x3F
#define VR_RAA_GEN2_SW_REV_MIN 0x02
#define VR_RAA_GEN2_HW_REV_MIN 0x03
#define VR_RAA_REG_GEN2_REMAIN_WR 0xC2
#define VR_RAA_REG_GEN2_PROG_STATUS 0x07

enum {
	RAA_GEN2,
	RAA_GEN3_LEGACY,
	RAA_GEN3_PRODUCTION,
};

typedef struct raa_config {
	uint8_t mode;
	uint16_t remain;
	uint32_t devid;
	uint32_t rev;
	uint32_t crc;
} raa_config_t;

struct raa_data {
	uint8_t hdr;
	uint8_t len; // [addr] ~ pec
	union {
		uint8_t raw[32];
		struct {
			uint8_t addr;
			uint8_t cmd;
			uint8_t data[];
		} __attribute__((packed));
	};
	uint8_t pec;
};

static bool raa_dma_rd(uint8_t bus, uint8_t addr, uint16_t reg, uint32_t *resp)
{
	CHECK_NULL_ARG_WITH_RETURN(resp, false);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;

	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	i2c_msg.tx_len = 3;
	i2c_msg.data[0] = VR_RAA_REG_DMA_ADDR;
	i2c_msg.data[1] = reg & 0xFF;
	i2c_msg.data[2] = (reg >> 8) & 0xFF;
	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("raa dma write register 0x%x failed", reg);
		return false;
	}

	i2c_msg.tx_len = 2;
	i2c_msg.rx_len = 4;
	i2c_msg.data[0] = VR_RAA_REG_DMA_DATA;
	i2c_msg.data[1] = (addr << 8) | 0x01; // address with read bit enabled
	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("raa dma read register 0x%x failed", reg);
		return false;
	}

	memcpy(resp, i2c_msg.data, 4);

	return true;
}

static bool get_raa_polling_status(uint8_t bus, uint8_t addr, uint8_t mode)
{
	uint16_t reg_buff = 0;
	uint32_t ret_buff;
	int retry = 3;

	for (int i = 0; i < retry; i++) {
		if (mode == RAA_GEN2) {
			reg_buff = VR_RAA_REG_GEN2_PROG_STATUS | (VR_RAA_REG_GEN2_PROG_STATUS << 8);
		} else {
			reg_buff = VR_RAA_REG_PROG_STATUS;
		}

		if (raa_dma_rd(bus, addr, reg_buff, &ret_buff) == false) {
			LOG_ERR("Failed to read polling status");
			return false;
		}

		// bit1 is held to 1, it means the action is successful
		if ((ret_buff & 0xFF) & 0x01) {
			return true;
		}

		k_msleep(1000);
	}

	LOG_ERR("Failed to program the device");
	return false;
}

bool get_raa_remaining_wr(uint8_t bus, uint8_t addr, uint8_t mode, uint16_t *remain)
{
	CHECK_NULL_ARG_WITH_RETURN(remain, false);

	uint16_t reg_buff = 0;
	uint32_t ret_buff;
	if (mode == RAA_GEN2)
		reg_buff |= VR_RAA_REG_GEN2_REMAIN_WR;
	else
		reg_buff |= VR_RAA_REG_REMAIN_WR;

	if (raa_dma_rd(bus, addr, reg_buff, &ret_buff) == false) {
		LOG_ERR("Falied to read NVM counter");
		return false;
	}

	*remain = ret_buff & 0xFF;

	return true;
}

bool isl69259_get_raa_crc(uint8_t bus, uint8_t addr, uint8_t mode, uint32_t *crc)
{
	CHECK_NULL_ARG_WITH_RETURN(crc, false);

	uint16_t reg_buff = 0;
	uint32_t ret_buff;

	if (mode == RAA_GEN2)
		reg_buff |= VR_RAA_REG_GEN2_CRC;
	else
		reg_buff |= VR_RAA_REG_CRC;

	if (raa_dma_rd(bus, addr, reg_buff, &ret_buff) == false) {
		LOG_ERR("Falied to read CRC");
		return false;
	}

	memcpy(crc, &ret_buff, sizeof(uint32_t));

	return true;
}

static bool get_raa_devid(uint8_t bus, uint8_t addr, uint32_t *devid)
{
	CHECK_NULL_ARG_WITH_RETURN(devid, false);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;

	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 4 + 1;
	i2c_msg.data[0] = PMBUS_IC_DEVICE_ID;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read vr id");
		return false;
	}

	memcpy(devid, &i2c_msg.data[1], sizeof(uint32_t));

	return true;
}

static bool get_raa_dev_rev(uint8_t bus, uint8_t addr, uint32_t *rev, uint8_t mode)
{
	CHECK_NULL_ARG_WITH_RETURN(rev, false);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;

	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 4 + 1;
	i2c_msg.data[0] = PMBUS_IC_DEVICE_REV;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read IC_DEVICE_REV");
		return false;
	}

	if (mode == RAA_GEN3_LEGACY) {
		memcpy(rev, &i2c_msg.data[1], 4);
	} else {
		for (int i = 0; i < 4; i++) {
			((uint8_t *)rev)[i] = i2c_msg.data[4 - i];
		}
	}

	return true;
}

bool isl69259_get_raa_hex_mode(uint8_t bus, uint8_t addr, uint8_t *mode)
{
	CHECK_NULL_ARG_WITH_RETURN(mode, false);

	uint32_t device_id;
	if (get_raa_devid(bus, addr, &device_id) == false)
		return false;

	if (((device_id >> 8) & 0xFF) < 0x70)
		*mode = RAA_GEN2;
	else {
		uint16_t reg_buff = VR_RAA_REG_HEX_MODE_CFG1 << 8 | VR_RAA_REG_HEX_MODE_CFG0;
		uint32_t ret_buff;
		if (raa_dma_rd(bus, addr, reg_buff, &ret_buff) == false) {
			LOG_ERR("Failed to read HEX mode");
			return false;
		}
		*mode = ((ret_buff & 0xFF) == 0) ? RAA_GEN3_LEGACY : RAA_GEN3_PRODUCTION;
	}

	return true;
}

static bool check_dev_rev(uint32_t rev, uint8_t mode)
{
	uint8_t sw_rev = rev & 0xFF;
	uint8_t hw_rev = (rev >> 24) & 0xFF;

	switch (mode) {
	case RAA_GEN2:
		if (sw_rev < VR_RAA_GEN2_SW_REV_MIN || hw_rev < VR_RAA_GEN2_HW_REV_MIN) {
			LOG_ERR("GEN2 unexpected IC_DEVICE_REV %08X", rev);
			return false;
		}
		break;
	case RAA_GEN3_LEGACY:
	case RAA_GEN3_PRODUCTION:
		break;
	default:
		LOG_WRN("RAA Mode not support");
		return false;
	}

	return true;
}

static bool parsing_image(uint8_t *img_buff, uint32_t img_size, struct isl69259_config *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(img_buff, false);
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);

	uint32_t buf_idx = 0;
	for (int i = 0; i < img_size; i += 2) {
		int hi_val = ascii_to_val(img_buff[i]);
		int lo_val = ascii_to_val(img_buff[i + 1]);
		if (hi_val == -1 || lo_val == -1) {
			continue;
		}
		cfg->buff[buf_idx] = hi_val * 16 + lo_val;
		buf_idx++;
	}

	cfg->len = buf_idx;

	return true;
}

static bool check_dev_support(uint8_t bus, uint8_t addr, raa_config_t *raa_info)
{
	CHECK_NULL_ARG_WITH_RETURN(raa_info, false);

	// check mode
	if (isl69259_get_raa_hex_mode(bus, addr, &raa_info->mode) == false) {
		return false;
	}

	// check remaining writes
	if (get_raa_remaining_wr(bus, addr, raa_info->mode, &raa_info->remain) == false) {
		return false;
	}

	if (!raa_info->remain) {
		LOG_WRN("No remaining writes for component");
		return false;
	}
	if (raa_info->remain <= VR_WARN_REMAIN_WR) {
		LOG_WRN("The remaining writes is below the threshold value %d!", VR_WARN_REMAIN_WR);
	}

	// get device crc
	if (isl69259_get_raa_crc(bus, addr, raa_info->mode, &raa_info->crc) == false) {
		return false;
	}

	// get device id
	if (get_raa_devid(bus, addr, &raa_info->devid) == false) {
		return false;
	}

	// check device revision
	if (get_raa_dev_rev(bus, addr, &raa_info->rev, raa_info->mode) == false) {
		return false;
	}

	if (check_dev_rev(raa_info->rev, raa_info->mode) == false) {
		return false;
	}

	LOG_INF("RAA device(bus: %d addr: 0x%x) info:", bus, addr);
	LOG_INF("* Mode:             %s", raa_info->mode == RAA_GEN2 ? "raa gen2" :
					  raa_info->mode == RAA_GEN3_LEGACY ?
								       "raa gen3-lagacy" :
								       "raa gen3-production");
	LOG_INF("* ID:               0x%x", raa_info->devid);
	LOG_INF("* Revision:         0x%x", raa_info->rev);
	LOG_INF("* CRC:              0x%x", raa_info->crc);
	LOG_INF("* Remaining writes: %d", raa_info->remain);

	return true;
}

bool isl69259_fwupdate(uint8_t bus, uint8_t addr, uint8_t *img_buff, uint32_t img_size)
{
	CHECK_NULL_ARG_WITH_RETURN(img_buff, 1);

	uint8_t ret = false;

	/* Step1. Before update */
	static raa_config_t dev_info;
	memset(&dev_info, 0, sizeof(raa_config_t));
	if (check_dev_support(bus, addr, &dev_info) == false) {
		return false;
	}

	/* Step2. Image parsing */
	struct isl69259_config cfg = { 0 };

	cfg.buff = (uint8_t *)malloc(img_size / 2);
	if (!cfg.buff) {
		LOG_ERR("Failed to malloc cfg.buff");
		return false;
	}

	if (parsing_image(img_buff, img_size, &cfg) == false) {
		LOG_ERR("Failed to parsing image!");
		goto exit;
	}

	/* Step3. FW Update */
	uint8_t img_mode = 0;
	uint8_t mode_check_flag = 0;
	struct raa_data cmd_line;
	uint8_t *cur_data = cfg.buff;
	uint32_t img_bin_len = cfg.len;
	uint32_t remain_buf_len = img_bin_len;

	while (1) {
		if (remain_buf_len == 0)
			break;

		/* check remain length should over base length */
		if (remain_buf_len < 2) {
			LOG_ERR("Unexpected data length error!");
			break;
		}
		cmd_line.hdr = *cur_data;
		cmd_line.len = *(cur_data + 1);
		/* check remain length should over base length + data length */
		if (remain_buf_len < (2 + cmd_line.len)) {
			LOG_ERR("Data length not follow spec!");
			break;
		}
		cmd_line.addr = *(cur_data + 2);
		cmd_line.cmd = *(cur_data + 3);
		memcpy(&cmd_line.raw[2], cur_data + 4, cmd_line.len - 3);
		cmd_line.pec = *(cur_data + 1 + cmd_line.len);

		LOG_DBG("process: hdr[0x%x] len[0x%x] addr[0x%x] cmd[0x%x] pec[0x%x]", cmd_line.hdr,
			cmd_line.len, cmd_line.addr, cmd_line.cmd, cmd_line.pec);

		/* collect vr header data */
		if (cmd_line.hdr == VR_IMG_HDR_SYMBOL) {
			if (cmd_line.cmd == PMBUS_IC_DEVICE_ID) {
				if (cmd_line.data[3] != (dev_info.devid & 0xFF) &&
				    cmd_line.data[2] != ((dev_info.devid >> 8) & 0xFF) &&
				    cmd_line.data[1] != ((dev_info.devid >> 16) & 0xFF) &&
				    cmd_line.data[0] != ((dev_info.devid >> 24) & 0xFF)) {
					LOG_ERR("Invalid vr device ID received, update abort!");
					goto exit;
				}
			} else if (cmd_line.cmd == PMBUS_IC_DEVICE_REV) {
				if ((cmd_line.data[0] & 0xFF) < VR_RAA_GEN3_SW_REV_MIN)
					img_mode = RAA_GEN3_LEGACY;
				else
					img_mode = RAA_GEN3_PRODUCTION;
			} else if (cmd_line.cmd == 0x00)
				img_mode = RAA_GEN2;
		} else if (cmd_line.hdr == VR_IMG_BODY_SYMBOL) {
			/* collect vr data array */
			if (!mode_check_flag) {
				if (img_mode != dev_info.mode) {
					LOG_ERR("Invalid vr device MODE(%d) received, update abort!",
						img_mode);
					goto exit;
				}
				mode_check_flag = 1;
			}

			I2C_MSG i2c_msg = { 0 };
			i2c_msg.bus = bus;
			i2c_msg.target_addr = addr;

			i2c_msg.tx_len = cmd_line.len - 2; // avoid address and pec bytes
			i2c_msg.data[0] = cmd_line.cmd;
			memcpy(&i2c_msg.data[1], cmd_line.data, i2c_msg.tx_len - 1);

			uint8_t retry = 3;
			if (i2c_master_write(&i2c_msg, retry)) {
				LOG_ERR("Failed to write image, update abort!");
				goto exit;
			}
		} else {
			LOG_ERR("Invalid VR image symbol 0x%x received, update abort!",
				cmd_line.hdr);
			goto exit;
		}

		cur_data += (2 + cmd_line.len);
		remain_buf_len -= (2 + cmd_line.len);

		uint8_t percent = ((img_bin_len - remain_buf_len) * 100) / img_bin_len;
		if (percent % 10 == 0)
			LOG_INF("updated: %d%% (bytes: %d/%d)", percent,
				img_bin_len - remain_buf_len, img_bin_len);
	}

	if (get_raa_polling_status(bus, addr, img_mode) == false) {
		LOG_ERR("VR polling status check failed, update abort!");
		goto exit;
	}

	ret = true;
exit:
	SAFE_FREE(cfg.buff);
	return ret;
}

bool adjust_of_twos_complement(uint8_t offset, int *val)
{
	CHECK_NULL_ARG_WITH_RETURN(val, false);

	int adjust_val = *val;
	bool is_negative_val = ((adjust_val & TWO_COMPLEMENT_NEGATIVE_BIT) == 0 ? false : true);
	bool ret = true;

	switch (offset) {
	case PMBUS_READ_IOUT:
		// Convert two's complement to usigned integer
		if (is_negative_val == true) {
			adjust_val = ~(adjust_val - 1);
		}

		// Set reading value to 0 if reading value in range -0.2A ~ 0.2A
		// Because register report unit is 0.1A , set reading value to 0 if register report value is lower than 2 units
		if (adjust_val < ADJUST_IOUT_RANGE) {
			*val = 0;
		}
		break;
	case PMBUS_READ_POUT:
		// Set reading value to 0 if reading value is negative
		if (is_negative_val == true) {
			*val = 0;
		}
		break;
	default:
		LOG_ERR("Not support offset: 0x%x", offset);
		ret = false;
		break;
	}
	return ret;
}

uint8_t isl69259_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	bool ret = false;
	uint8_t retry = 5;
	int val = 0;
	sensor_val *sval = (sensor_val *)reading;
	I2C_MSG msg = { 0 };
	;
	memset(sval, 0, sizeof(sensor_val));

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = cfg->offset;

	if (i2c_master_read(&msg, retry)) {
		/* read fail */
		return SENSOR_FAIL_TO_ACCESS;
	}

	uint8_t offset = cfg->offset;
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
		sval->fraction = (int16_t)(val - (sval->integer * 10)) * 100;
		break;
	case PMBUS_READ_IIN:
		/* 0.01 A/LSB, 2's complement */
		sval->integer = (int16_t)val / 100;
		sval->fraction = (int16_t)(val - (sval->integer * 100)) * 10;
		break;
	case PMBUS_READ_TEMPERATURE_1:
		/* 1 Degree C/LSB, 2's complement */
		sval->integer = val;
		break;
	case PMBUS_READ_POUT:
		/* 1 Watt/LSB, 2's complement */
		ret = adjust_of_twos_complement(offset, &val);
		if (ret == false) {
			LOG_ERR("Adjust reading POUT value failed - sensor number: 0x%x", cfg->num);
			return SENSOR_UNSPECIFIED_ERROR;
		}

		sval->integer = val;
		break;
	default:
		LOG_ERR("Not support offset: 0x%x", offset);
		return SENSOR_FAIL_TO_ACCESS;
		break;
	}

	return SENSOR_READ_SUCCESS;
}

uint8_t isl69259_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = isl69259_read;
	return SENSOR_INIT_SUCCESS;
}
