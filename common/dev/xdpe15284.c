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
#include "libutil.h"
#include "hal_i2c.h"
#include "util_pmbus.h"
#include "pmbus.h"
#include "xdpe15284.h"

LOG_MODULE_REGISTER(xdpe15284);

#define XDPE15284_MFR_BLOCK_DATA_LEN 4
#define XDPE15284_MFR_FW_CMD_DATA_LEN 4
#define XDPE15284_REMAINING_WRITE_DATA_LEN 2
#define XDPE15284_WAIT_DATA_DELAY_MS 10
#define XDPE15284C_CONF_SIZE 1344 // Config(604) + PMBus(504) + SVID(156) + PMBusPartial(80)

#define XDPE15284_WRITE_PROTECT_DEFAULT_VAL 0xFF

/* Firmware update related define */
#define ADDRESS_FIELD "PMBus Address :"
#define CHECKSUM_FIELD "Checksum : 0x"
#define DATA_START_TAG "[Configuration Data]"
#define DATA_END_TAG "[End Configuration Data]"
#define DATA_COMMENT "//"
#define MAX_SECT_DATA_NUM 200
#define MAX_SECT_NUM 16
#define SECT_TRIM 0x02
#define SECT_CRC_OFFSET 2
#define DATA_LEN_IN_LINE 5
#define SCRATCHPAD_ADDRESS 0x2005e000
#define CML_OTHER_MEMORY_FAULT_BIT BIT(0)
#define CR_ASCII 0x0D
#define NEW_LINE_ASCII 0x0A
#define SPACE_ASCII 0x20
#define CRC32_POLY 0xEDB88320 // polynomial

enum XDPE15284_REG {
	XDPE15284_WRITE_PROTECT_CMD_REG = 0x10,
	XDPE15284_MFR_DISABLE_SECURITY_ONCE_REG = 0xCB,
	XDPE15284_IFX_MFR_AHB_ADDR_REG = 0xCE,
	XDPE15284_IFX_MFR_REG_WRITE_REG = 0xDE,
	XDPE15284_IFX_MFR_REG_READ_REG = 0xDF,
	XDPE15284_MFR_FW_CMD_DATA_REG = 0xFD,
	XDPE15284_MFR_FW_CMD_REG = 0xFE,
};

enum XDPE15284_MFR_CMD {
	XDPE15284_NOP_CMD = 0x00,
	XDPE15284_GET_REMAINING_WR_CMD = 0x10,
	XDPE15284_OTP_CONF_STO = 0x11,
	XDPE15284_OTP_FILE_INVD = 0x12,
	XDPE15284_CALCULATE_CRC_CMD = 0x2D,
};

struct config_sect {
	uint8_t type;
	uint16_t data_cnt;
	uint32_t data[MAX_SECT_DATA_NUM];
};

struct xdpe15284_config {
	uint8_t sect_cnt;
	uint16_t total_cnt;
	uint32_t exp_crc;
	struct config_sect section[MAX_SECT_NUM];
};

const uint32_t REG_LOCK_PASSWORD = 0x7F48680C;
static uint8_t write_protect_default_val = XDPE15284_WRITE_PROTECT_DEFAULT_VAL;

static bool xdpe15284_set_write_protect_reg(uint8_t bus, uint8_t addr, uint8_t optional)
{
	uint8_t set_val = 0;

	switch (optional) {
	case XDPE15284_ENABLE_WRITE_PROTECT:
		set_val = write_protect_default_val;
		break;
	case XDPE15284_DISABLE_WRITE_PROTECT:
		set_val = XDPE15284_DISABLE_WRITE_PROTECT_VAL;
		break;
	default:
		LOG_ERR("Invalid optional: 0x%x to set write protect reg", optional);
		return false;
	}

	uint8_t retry = 3;
	I2C_MSG i2c_msg = { 0 };

	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 2;
	i2c_msg.data[0] = XDPE15284_WRITE_PROTECT_CMD_REG;
	i2c_msg.data[1] = set_val;

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Set write protect register fail, bus: 0x%x, addr: 0x%x, set_val: 0x%x",
			bus, addr, set_val);
		return false;
	}

	k_msleep(XDPE15284_WAIT_DATA_DELAY_MS);

	memset(&i2c_msg, 0, sizeof(I2C_MSG));
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = XDPE15284_WRITE_PROTECT_CMD_REG;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Read write protect register fail, bus: 0x%x, addr: 0x%x", bus, addr);
		return false;
	}

	if (i2c_msg.data[0] != set_val) {
		LOG_ERR("Set write protect register fail, bus: 0x%x, addr: 0x%x, ret_val: 0x%x, set_val: 0x%x",
			bus, addr, i2c_msg.data[0], set_val);
		return false;
	}

	return true;
}

static bool init_write_protect_default_val(uint8_t bus, uint8_t addr)
{
	uint8_t retry = 3;
	I2C_MSG i2c_msg = { 0 };

	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = XDPE15284_WRITE_PROTECT_CMD_REG;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Read write protect register fail, bus: 0x%x, addr: 0x%x", bus, addr);
		return false;
	}

	write_protect_default_val = i2c_msg.data[0];
	return true;
}

bool xdpe15284_set_write_protect(uint8_t bus, uint8_t addr, uint8_t option)
{
	if (write_protect_default_val != XDPE15284_DISABLE_WRITE_PROTECT_VAL) {
		if (write_protect_default_val == XDPE15284_WRITE_PROTECT_DEFAULT_VAL) {
			if (init_write_protect_default_val(bus, addr) != true) {
				return false;
			}
		}
		return xdpe15284_set_write_protect_reg(bus, addr, option);
	}

	return true;
}

void xdpe15284_set_write_protect_default_val(uint8_t val)
{
	// Set write protection default value according to different projects
	write_protect_default_val = val;
}

bool xdpe15284_mfr_fw_operation(uint8_t bus, uint8_t addr, uint8_t cmd, uint8_t tx_len,
				uint8_t *tx_buf, uint8_t rx_len, uint8_t *rx_buf)
{
	if (tx_len != 0) {
		CHECK_NULL_ARG_WITH_RETURN(tx_buf, false);

		if (tx_len != 4) {
			LOG_ERR("Invalid tx_len: 0x%x, cmd: 0x%x", tx_len, cmd);
			return false;
		}
	}
	if (rx_len != 0) {
		CHECK_NULL_ARG_WITH_RETURN(rx_buf, false);
	}

	int ret = 0;
	uint8_t index = 0;
	uint8_t retry = 3;
	I2C_MSG i2c_msg = { 0 };

	for (index = 0; index < retry; ++index) {
		/* Reset the MFR data register to default value */
		memset(&i2c_msg, 0, sizeof(I2C_MSG));
		i2c_msg.bus = bus;
		i2c_msg.target_addr = addr;
		i2c_msg.tx_len = 2;
		i2c_msg.data[0] = XDPE15284_MFR_FW_CMD_REG;
		i2c_msg.data[1] = XDPE15284_NOP_CMD;

		ret = i2c_master_write(&i2c_msg, 1);
		if (ret != 0) {
			LOG_ERR("Reset data register fail, cmd: 0x%x", cmd);
			continue;
		}

		if (tx_buf != NULL) {
			memset(&i2c_msg, 0, sizeof(I2C_MSG));
			i2c_msg.bus = bus;
			i2c_msg.target_addr = addr;
			i2c_msg.tx_len = 6;
			i2c_msg.data[0] = XDPE15284_MFR_FW_CMD_DATA_REG;
			i2c_msg.data[1] = XDPE15284_MFR_BLOCK_DATA_LEN; // Block write 4 bytes
			memcpy(&i2c_msg.data[2], tx_buf, XDPE15284_MFR_BLOCK_DATA_LEN);

			ret = i2c_master_write(&i2c_msg, 1);
			if (ret != 0) {
				LOG_ERR("Write data to MFR data register fail, cmd: 0x%x", cmd);
				continue;
			}

			k_msleep(XDPE15284_WAIT_DATA_DELAY_MS);
		}

		/* Send the command to get request data */
		memset(&i2c_msg, 0, sizeof(I2C_MSG));
		i2c_msg.bus = bus;
		i2c_msg.target_addr = addr;
		i2c_msg.tx_len = 2;
		i2c_msg.data[0] = XDPE15284_MFR_FW_CMD_REG;
		i2c_msg.data[1] = cmd;

		ret = i2c_master_write(&i2c_msg, 1);
		if (ret != 0) {
			LOG_ERR("Write command to register fail, cmd: 0x%x", cmd);
			continue;
		}

		k_msleep(XDPE15284_WAIT_DATA_DELAY_MS);

		if (rx_buf != NULL) {
			/* Read data in MFR firmware command data register */
			memset(&i2c_msg, 0, sizeof(I2C_MSG));
			i2c_msg.bus = bus;
			i2c_msg.target_addr = addr;
			i2c_msg.tx_len = 1;
			i2c_msg.rx_len = rx_len + 1;
			i2c_msg.data[0] = XDPE15284_MFR_FW_CMD_DATA_REG;

			ret = i2c_master_read(&i2c_msg, 1);
			if (ret != 0) {
				LOG_ERR("Read data from register fail, cmd: 0x%x", cmd);
				continue;
			}

			memcpy(rx_buf, &i2c_msg.data[1], rx_len);
		}
		break;
	}

	if (ret != 0) {
		LOG_ERR("Retry reach max, bus: 0x%x, addr: 0x%x, cmd: 0x%x", bus, addr, cmd);
		return false;
	}
	return true;
}

bool xdpe15284_get_checksum(uint8_t bus, uint8_t addr, uint8_t *checksum)
{
	CHECK_NULL_ARG_WITH_RETURN(checksum, false);

	bool ret = xdpe15284_set_write_protect(bus, addr, XDPE15284_DISABLE_WRITE_PROTECT);
	if (ret != true) {
		LOG_ERR("Disable write protect fail before getting checksum");
		return ret;
	}

	ret = xdpe15284_mfr_fw_operation(bus, addr, XDPE15284_CALCULATE_CRC_CMD, 0, NULL,
					 XDPE15284_MFR_FW_CMD_DATA_LEN, checksum);
	if (ret != true) {
		LOG_ERR("Get checksum fail, bus: 0x%x, addr: 0x%x", bus, addr);
		return ret;
	}

	ret = xdpe15284_set_write_protect(bus, addr, XDPE15284_ENABLE_WRITE_PROTECT);
	if (ret != true) {
		LOG_ERR("Enable write protect fail after getting checksum");
	}

	reverse_array(checksum, XDPE15284_MFR_FW_CMD_DATA_LEN);
	return ret;
}

bool xdpe15284_get_remaining_wr(uint8_t bus, uint8_t addr, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	uint8_t buf[XDPE15284_REMAINING_WRITE_DATA_LEN] = { 0 };
	bool ret = xdpe15284_set_write_protect(bus, addr, XDPE15284_DISABLE_WRITE_PROTECT);
	if (ret != true) {
		LOG_ERR("Disable write protect fail before getting remaining write");
		return ret;
	}

	ret = xdpe15284_mfr_fw_operation(bus, addr, XDPE15284_GET_REMAINING_WR_CMD, 0, NULL,
					 XDPE15284_REMAINING_WRITE_DATA_LEN, buf);
	if (ret != true) {
		LOG_ERR("Get remaining write count fail, bus: 0x%x, addr: 0x%x", bus, addr);
		return ret;
	}

	ret = xdpe15284_set_write_protect(bus, addr, XDPE15284_ENABLE_WRITE_PROTECT);
	if (ret != true) {
		LOG_ERR("Enable write protect fail after getting remaining write");
	}

	*data = ((buf[1] << 8 | buf[0]) / XDPE15284C_CONF_SIZE) & 0xFF;
	return ret;
}

bool xdpe15284_get_status_byte(uint8_t bus, uint8_t addr, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	int ret = -1;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = bus;
	msg.target_addr = addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = PMBUS_STATUS_BYTE;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Get status byte fail, bus: 0x%x, addr: 0x%x", bus, addr);
		return false;
	}

	*data = msg.data[0];
	return true;
}

uint8_t xdpe15284_read(sensor_cfg *cfg, int *reading)
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

	if (i2c_master_read(&msg, retry))
		return SENSOR_FAIL_TO_ACCESS;

	uint8_t offset = cfg->offset;
	if (offset == PMBUS_READ_VOUT) {
		/* ULINEAR16, get exponent from VOUT_MODE */
		float exponent;
		if (!get_exponent_from_vout_mode(cfg, &exponent))
			return SENSOR_FAIL_TO_ACCESS;

		float actual_value = ((msg.data[1] << 8) | msg.data[0]) * exponent;
		sval->integer = actual_value;
		sval->fraction = (actual_value - sval->integer) * 1000;
	} else if (offset == PMBUS_READ_IOUT || offset == PMBUS_READ_TEMPERATURE_1 ||
		   offset == PMBUS_READ_POUT) {
		/* SLINEAR11 */
		uint16_t read_value = (msg.data[1] << 8) | msg.data[0];
		float actual_value = slinear11_to_float(read_value);
		if (offset == PMBUS_READ_IOUT && actual_value < 0) {
			/* In the case POUT is 0, IOUT may read small negative value, replace this case with 0 */
			sval->integer = 0;
			sval->fraction = 0;
		} else {
			sval->integer = actual_value;
			sval->fraction = (actual_value - sval->integer) * 1000;
		}
	} else {
		return SENSOR_FAIL_TO_ACCESS;
	}

	return SENSOR_READ_SUCCESS;
}

bool xdpe15284_lock_reg(uint8_t bus, uint8_t addr)
{
	I2C_MSG i2c_msg = { 0 };
	int retry = 3;

	/* Reset the MFR data register to default value */
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 6;
	i2c_msg.data[0] = XDPE15284_MFR_DISABLE_SECURITY_ONCE_REG;
	i2c_msg.data[1] = 0x04;
	i2c_msg.data[2] = 0x00;
	i2c_msg.data[3] = 0x00;
	i2c_msg.data[4] = 0x00;

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Lock register fail");
		return false;
	}
	return true;
}

bool xdpe15284_unlock_reg(uint8_t bus, uint8_t addr)
{
	I2C_MSG i2c_msg = { 0 };
	int retry = 3;

	/* Reset the MFR data register to default value */
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 6;
	i2c_msg.data[0] = XDPE15284_MFR_DISABLE_SECURITY_ONCE_REG;
	i2c_msg.data[1] = 0x04;
	memcpy(&(i2c_msg.data[2]), (uint8_t *)&REG_LOCK_PASSWORD, 4);

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Unlock register fail");
		return false;
	}
	k_usleep(300);
	return true;
}

uint8_t xdpe15284_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = xdpe15284_read;
	return SENSOR_INIT_SUCCESS;
}

static uint32_t cal_crc32(uint32_t const *data, int len)
{
	uint32_t crc = 0xFFFFFFFF;
	int i = 0, j = 0;

	if (data == NULL) {
		return 0;
	}

	for (i = 0; i < len; i++) {
		crc ^= data[i];
		for (j = 0; j < 32; j++) {
			if (crc & 0x1) {
				crc = (crc >> 1) ^ CRC32_POLY; // lsb-first
			} else {
				crc >>= 1;
			}
		}
	}

	return ~crc;
}

static int check_xdpe15284_image_crc(struct xdpe15284_config *config)
{
	CHECK_NULL_ARG_WITH_RETURN(config, -1);

	uint8_t index = 0;
	uint32_t sum = 0;
	uint32_t crc = 0;

	for (index = 0; index < config->sect_cnt; ++index) {
		struct config_sect *sect = &config->section[index];
		CHECK_NULL_ARG_WITH_RETURN(sect, -1);

		// Check CRC of section header
		crc = cal_crc32(&sect->data[0], 2);
		if (crc != sect->data[SECT_CRC_OFFSET]) {
			LOG_ERR("Section: 0x%x header CRC mismatch, expect: 0x%x, actual: 0x%x",
				index, sect->data[SECT_CRC_OFFSET], crc);
			return -1;
		}

		sum += crc;
		// Check CRC of section data
		crc = cal_crc32(
			&sect->data[3],
			sect->data_cnt -
				4); // Deduct header (2-byte), header CRC (1-byte) and data CRC (1-byte)
		if (crc != sect->data[sect->data_cnt - 1]) {
			LOG_ERR("Section: 0x%x data CRC mismatch, expect: 0x%x, actual: 0x%x",
				index, sect->data[sect->data_cnt - 1], crc);
			return -1;
		}

		sum += crc;
	}

	if (sum != config->exp_crc) {
		LOG_ERR("Image CRC mismatch, expect: 0x%x, actual: 0x%x", config->exp_crc, sum);
		return -1;
	}
	return 0;
}

static int program_xdpe15284(uint8_t bus, uint8_t addr, struct xdpe15284_config *config)
{
	CHECK_NULL_ARG_WITH_RETURN(config, -1);

	uint8_t tbuf[8] = { 0 };
	uint8_t i = 0, j = 0;
	uint32_t address = SCRATCHPAD_ADDRESS;
	int size = 0, ret = -1, retry = 3;
	I2C_MSG msg = { 0 };

	if (xdpe15284_set_write_protect(bus, addr, XDPE15284_DISABLE_WRITE_PROTECT) != true) {
		LOG_ERR("Disable write protect fail before programing image, bus: 0x%x, addr: 0x%x",
			bus, addr);
		return ret;
	}

	for (i = 0; i < config->sect_cnt; ++i) {
		struct config_sect *sect = &config->section[i];
		if (sect == NULL) {
			ret = -1;
			goto exit;
		}

		memset(tbuf, 0, sizeof(tbuf));
		memset(&msg, 0, sizeof(I2C_MSG));
		if ((i == 0) || (sect->type != config->section[i - 1].type)) {
			// Clear bit of PMBUS_STATUS_CML
			memset(tbuf, 0, sizeof(tbuf));
			tbuf[0] = PMBUS_STATUS_CML;
			tbuf[1] = 0x00;
			msg = construct_i2c_message(bus, addr, 2, tbuf, 0);

			ret = i2c_master_write(&msg, retry);
			if (ret != 0) {
				LOG_ERR("Section: 0x%x write CML status fail, type: 0x%x, ret: %d",
					i, sect->type, ret);
				goto exit;
			}

			// Invalidate existing data
			memset(tbuf, 0, sizeof(tbuf));
			tbuf[0] = sect->type; // Section type
			tbuf[1] = 0x00; // XV0
			tbuf[2] = 0x00;
			tbuf[3] = 0x00;
			if (xdpe15284_mfr_fw_operation(bus, addr, XDPE15284_OTP_FILE_INVD, 4, tbuf,
						       0, NULL) != true) {
				LOG_ERR("Section: 0x%x invalidate data fail, type: 0x%x", i,
					sect->type);
				ret = -1;
				goto exit;
			}

			k_msleep(XDPE15284_WAIT_DATA_DELAY_MS);

			// Set scratchpad address
			memset(tbuf, 0, sizeof(tbuf));
			tbuf[0] = XDPE15284_IFX_MFR_AHB_ADDR_REG;
			tbuf[1] = 4;
			memcpy(&tbuf[2], &address, sizeof(uint32_t));
			msg = construct_i2c_message(bus, addr, 6, tbuf, 0);

			ret = i2c_master_write(&msg, retry);
			if (ret != 0) {
				LOG_ERR("Section: 0x%x set scratchpad addr fail, type: 0x%x", i,
					sect->type);
				goto exit;
			}

			k_msleep(XDPE15284_WAIT_DATA_DELAY_MS);
			size = 0;
		}

		// Program data into scratch
		for (j = 0; j < sect->data_cnt; ++j) {
			memset(tbuf, 0, sizeof(tbuf));
			tbuf[0] = XDPE15284_IFX_MFR_REG_WRITE_REG;
			tbuf[1] = 4;
			memcpy(&tbuf[2], &sect->data[j], 4);
			msg = construct_i2c_message(bus, addr, 6, tbuf, 0);

			ret = i2c_master_write(&msg, retry);
			if (ret != 0) {
				LOG_ERR("Section: 0x%x write section data fail, type: 0x%x, data: 0x%x",
					i, sect->type, sect->data[j]);
				goto exit;
			}
			k_msleep(XDPE15284_WAIT_DATA_DELAY_MS);
		}

		size += sect->data_cnt * 4;

		if ((i + 1 >= config->sect_cnt) || (sect->type != config->section[i + 1].type)) {
			// Upload scratchpad to OTP
			memset(tbuf, 0, sizeof(tbuf));
			memcpy(tbuf, &size, 2);
			if (xdpe15284_mfr_fw_operation(bus, addr, XDPE15284_OTP_CONF_STO, 4, tbuf,
						       0, NULL) != true) {
				LOG_ERR("Section: 0x%x upload data to OTP fail, type: 0x%x", i,
					sect->type);
				ret = -1;
				goto exit;
			}

			// Wait for programming soak (2ms/byte, at least 200ms)
			k_msleep(size * 2 + 200);

			// Check CML status
			tbuf[0] = PMBUS_STATUS_CML;
			msg = construct_i2c_message(bus, addr, 1, tbuf, 1);

			ret = i2c_master_read(&msg, retry);
			if (ret != 0) {
				LOG_ERR("Section: 0x%x read CML status fail, type: 0x%x", i,
					sect->type);
				goto exit;
			}

			if (msg.data[0] & CML_OTHER_MEMORY_FAULT_BIT) {
				LOG_ERR("Section: 0x%x occur CML other memory fault: 0x%x, type: 0x%x",
					i, msg.data[0], sect->type);
				ret = -1;
				goto exit;
			}
		}
	}
	ret = 0;

exit:
	if (xdpe15284_set_write_protect(bus, addr, XDPE15284_ENABLE_WRITE_PROTECT) != true) {
		LOG_ERR("Enable write protect fail after programing image, bus: 0x%x, addr: 0x%x",
			bus, addr);
	}
	return ret;
}

int xdpe15284_line_split(char **str, uint8_t *image_buf, uint32_t start_index, uint32_t end_index,
			 int max_size)
{
	CHECK_NULL_ARG_WITH_RETURN(str, -1);
	CHECK_NULL_ARG_WITH_RETURN(image_buf, -1);

	int ret = 0;
	int index = 0;
	int size = 0;

	for (index = start_index; index < end_index; ++index) {
		ret = find_byte_data_in_buf(image_buf, SPACE_ASCII, index, end_index);
		if (ret < 0) {
			ret = find_byte_data_in_buf(image_buf, CR_ASCII, index,
						    end_index); // last dword
			if (ret > 0) {
				*str++ = &image_buf[index];
				image_buf[ret] = '\0';
				++size;
			}
			break;
		}

		*str++ = &image_buf[index];
		image_buf[ret] = '\0';
		index = ret;
		if ((++size) > max_size) {
			break;
		}
	}

	return size;
}

int xdpe15284_parse_file(struct xdpe15284_config *config, uint8_t *image_buf, uint32_t img_size)
{
	CHECK_NULL_ARG_WITH_RETURN(config, -1);
	CHECK_NULL_ARG_WITH_RETURN(image_buf, -1);

	const size_t len_start = strlen(DATA_START_TAG);
	const size_t len_end = strlen(DATA_END_TAG);
	const size_t len_comment = strlen(DATA_COMMENT);
	const size_t len_checksum = strlen(CHECKSUM_FIELD);
	int data_cnt = 0, sect_idx = -1, ret = 0;
	uint8_t index = 0, sect_type = 0;
	uint16_t offset = 0;
	uint32_t dword = 0, buf_index = 0;
	bool is_data = false;

	for (buf_index = 0; buf_index < img_size; ++buf_index) {
		if (is_data) {
			if (buf_index + len_end < img_size) {
				if (strncmp(&image_buf[buf_index], DATA_END_TAG, len_end) == 0) {
					break;
				}
			}

			ret = find_byte_data_in_buf(image_buf, NEW_LINE_ASCII, buf_index, img_size);
			if (ret < 0) {
				LOG_ERR("Find new line byte fail at data field, stop index: 0x%x",
					buf_index);
				return -1;
			}

			if (strncmp(&image_buf[buf_index], DATA_COMMENT, len_comment) == 0) {
				buf_index = ret;
				continue;
			}

			uint32_t end_index = ret;
			char *token_list[DATA_LEN_IN_LINE] = { 0 };
			int token_size = xdpe15284_line_split(token_list, image_buf, buf_index,
							      end_index, DATA_LEN_IN_LINE);
			if (token_size < 1) {
				// Unexpected data line
				continue;
			}

			offset = (uint16_t)strtol(token_list[0], NULL, 16);
			if (sect_type == SECT_TRIM && offset != 0x0) { // skip Trim section
				continue;
			}

			for (index = 1; index < token_size; ++index) {
				dword = (uint32_t)strtoul(token_list[index], NULL, 16);
				if (offset == 0 && index == 1) {
					sect_type = (uint8_t)dword; // section type
					if (sect_type == SECT_TRIM) {
						break;
					}
					if ((++sect_idx) >= MAX_SECT_NUM) {
						LOG_ERR("Exceed max section number, index: 0x%x, sect_type: 0x%x",
							index, sect_type);
						return -1;
					}
					config->section[sect_idx].type = sect_type;
					config->sect_cnt = sect_idx + 1;
					data_cnt = 0;
				}
				if (data_cnt >= MAX_SECT_DATA_NUM) {
					LOG_ERR("Exceed max data count, index: 0x%x, sect_type: 0x%x",
						index, sect_type);
					return -1;
				}
				config->section[sect_idx].data[data_cnt++] = dword;
				config->section[sect_idx].data_cnt = data_cnt;
				config->total_cnt++;
			}
			buf_index = end_index;
		} else {
			if (buf_index + len_checksum < img_size) {
				if (strncmp(&image_buf[buf_index], CHECKSUM_FIELD, len_checksum) ==
				    0) {
					config->exp_crc = (uint32_t)strtoul(
						&image_buf[buf_index + len_checksum], NULL, 16);
					buf_index += len_checksum;
					continue;
				}
			}
			if (buf_index + len_start < img_size) {
				if (strncmp(&image_buf[buf_index], DATA_START_TAG, len_start) ==
				    0) {
					buf_index += len_start;
					is_data = true;
					continue;
				}
			}
		}
	}
	if (config->exp_crc == 0) {
		LOG_ERR("Can find expected CRC");
		return -1;
	}
	return 0;
}

bool xdpe15284_fwupdate(uint8_t bus, uint8_t addr, uint8_t *img_buff, uint32_t img_size)
{
	CHECK_NULL_ARG_WITH_RETURN(img_buff, false);

	int ret = 0;
	uint8_t remain_wr = 0;

	struct xdpe15284_config *config =
		(struct xdpe15284_config *)malloc(sizeof(struct xdpe15284_config));
	if (config == NULL) {
		LOG_ERR("Allocate config fail");
		return false;
	}

	memset(config, 0, sizeof(struct xdpe15284_config));
	if (xdpe15284_get_remaining_wr(bus, addr, &remain_wr) != true) {
		goto error_exit;
	}

	if (remain_wr == 0) {
		LOG_ERR("Insufficient remaining writes");
		goto error_exit;
	}

	ret = xdpe15284_parse_file(config, img_buff, img_size);
	if (ret != 0) {
		LOG_ERR("Parse file fail");
		goto error_exit;
	}

	ret = check_xdpe15284_image_crc(config);
	if (ret != 0) {
		LOG_ERR("Check image crc fail");
		goto error_exit;
	}

	LOG_INF("XDPE15284 device bus: %d, addr: 0x%x", bus, addr);
	LOG_INF("* Image CRC:        0x%x", config->exp_crc);
	LOG_INF("* Remaining writes: 0x%x", remain_wr);

	ret = program_xdpe15284(bus, addr, config);
	if (ret != 0) {
		LOG_ERR("Program image fail");
		goto error_exit;
	}

	SAFE_FREE(config);
	return true;

error_exit:
	SAFE_FREE(config);
	return false;
}
