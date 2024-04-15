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
#include <stdlib.h>

#include "libutil.h"
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"
#include "util_pmbus.h"
#include "tps53689.h"

#define TI_REG_NVM_CHECKSUM 0xF4
#define TI_REG_USER_NVM_INDEX 0xF5
#define TI_REG_USER_NVM_EXECUTE 0xF6

LOG_MODULE_REGISTER(tps53689);

enum LINE_INDEX {
	FIRST_LINE = 1,
	DEV_ID_LINE = 2,
	LAST_LINE = 11,
};

enum LINE_FORMAT {
	HEADER_LEN = 8,
	DATA_LEN = 32,
};

enum DEV_ID_LINE_OFFSET {
	DEV_REV1_OFFSET = 6,
	DEV_REV2_OFFSET = 7,
	ADDR_OFFSET = 8,
	CRC1_OFFSET = 9,
	CRC2_OFFSET = 10,
};

uint8_t tps536c5_dev_id[] = { 0x54, 0x49, 0x53, 0x6C, 0x50, 0x00 };

/*
 * VR image format:
 *
 * :020000040000FA			// Line 1, 20000004 is header
 * :200000005449536C5000010076A1AE969610FFFFBFFFFFF516F69A039A0300000006CD0465
 * :200020009933000C0000000046289780223763622524246060E060202060AA2525AA252550
 * ```
 * :200100000000000000000000000000000000000000000000000000000000000000000000DF
 * :00000001FF				// Line 11
*/

// This feature is for parsing hex file
bool tps536xx_parse_image(uint8_t *img_buff, uint32_t img_size, struct tps_config *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(img_buff, false);
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);

	int line = FIRST_LINE, buff_index = 0, cfg_data_index = 0, line_data_bytes = 0;

	while (buff_index < img_size) {
		if (img_buff[buff_index] == '\n') {
			LOG_DBG("Line %d. img_buff[%d] = '\\n'. Change to the next line.", line,
				buff_index);
			line++;
			buff_index++;
			continue;
		}

		if (line == FIRST_LINE) {
			LOG_DBG("Line %d (First Line). img_buff[%d] = %x.", line, buff_index,
				img_buff[buff_index]);
			// The first line is unused. Ignore it.
			buff_index++;
			continue;
		}

		if (line == LAST_LINE) {
			LOG_DBG("Line %d (Last Line). img_buff[%d] = %x.", line, buff_index,
				img_buff[buff_index]);
			// The last line is unused. Ignore it.
			break;
		}

		if (img_buff[buff_index] == ':') {
			LOG_DBG("Line %d. img_buff[%d] = ':'.", line, buff_index);
			// The header is unused. Igonre it.
			buff_index += HEADER_LEN + 1;
			continue;
		}

		if (line_data_bytes == DATA_LEN) {
			LOG_DBG("Line %d. Line Checksum: %c%c", line, img_buff[buff_index],
				img_buff[buff_index + 1]);
			// Ignore the last byte and '\r'
			line_data_bytes = 0;
			buff_index += 3;
			continue;
		}

		// There are DATA_LEN bytes of updating data in each line
		for (; line_data_bytes < DATA_LEN;
		     line_data_bytes++, cfg_data_index++, buff_index += 2) {
			char str[2];
			str[0] = img_buff[buff_index];
			str[1] = img_buff[buff_index + 1];
			uint8_t byte = (uint8_t)strtoul(str, NULL, 16);

			// According to the TPS536XX specification, BYTE 0 ~ 8 in Line 2 must be written as 0xFF when programming.
			if (line == DEV_ID_LINE) {
				if (line_data_bytes < DEV_REV1_OFFSET) { //Device ID
					cfg->devid[line_data_bytes] = byte;
					byte = 0xFF;
				} else if (line_data_bytes == DEV_REV1_OFFSET ||
					   line_data_bytes == DEV_REV2_OFFSET) {
					byte = 0xFF;
				} else if (line_data_bytes == ADDR_OFFSET) {
					cfg->addr = byte;
					byte = 0xFF;
				} else if (line_data_bytes == CRC1_OFFSET ||
					   line_data_bytes == CRC2_OFFSET) {
					cfg->crc[line_data_bytes - CRC1_OFFSET] = byte;
				}
			}

			cfg->data[cfg_data_index] = byte;
		}
	}
	return true;
}

bool tps536xx_get_crc(uint8_t bus, uint8_t addr, uint32_t *crc)
{
	CHECK_NULL_ARG_WITH_RETURN(crc, false);

	I2C_MSG i2c_msg = { 0 };

	uint8_t retry = 5;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = TI_REG_NVM_CHECKSUM;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_DBG("read register 0x%02X failed", i2c_msg.data[0]);
		return false;
	}

	*crc = (i2c_msg.data[3] << 24) | (i2c_msg.data[2] << 16) | (i2c_msg.data[1] << 8) |
	       i2c_msg.data[0];

	return true;
}

bool tps536xx_fwupdate(uint8_t bus, uint8_t addr, uint8_t *img_buff, uint32_t img_size)
{
	CHECK_NULL_ARG_WITH_RETURN(img_buff, false);

	// Parse image
	struct tps_config config = { 0 };
	if (tps536xx_parse_image(img_buff, img_size, &config) == false) {
		LOG_ERR("Failed to parse image!");
		return false;
	}

	for (int index = 0; index < sizeof(tps536c5_dev_id); index++) {
		if (config.devid[index] != tps536c5_dev_id[index]) {
			LOG_ERR("Failed to update firmware, device ID is not matched!");
			return false;
		}
	}

	if (config.addr != addr) {
		LOG_ERR("Failed to update firmware, address is not matched!");
		return false;
	}

	uint32_t dev_crc = 0;
	if (tps536xx_get_crc(bus, addr, &dev_crc) == true) {
		uint32_t img_crc = (config.crc[1] << 8) | config.crc[0];
		if (dev_crc == img_crc) {
			LOG_ERR("Failed to update firmware becasue CRC is matched!");
			return false;
		}
	}

	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = bus;
	msg.target_addr = addr;
	msg.tx_len = 2;
	msg.rx_len = 0;

	// set USER_NVM_INDEX 00h
	msg.data[0] = TI_REG_USER_NVM_INDEX;
	msg.data[1] = 0x00;

	if (i2c_master_write(&msg, retry) != 0) {
		LOG_ERR("Failed to set USER_NVM_INDEX 00h");
		return false;
	}

	// program the image
	msg.data[0] = TI_REG_USER_NVM_EXECUTE;
	msg.data[1] = TPS536XX_UPDATE_INFO_BYTES;
	msg.tx_len = TPS536XX_UPDATE_INFO_BYTES +
		     2; // TI_REG_USER_NVM_EXECUTE + write block len + 32 bytes write data

	for (int index = 0; index < 9; index++) {
		memcpy(&msg.data[2], &config.data[index * TPS536XX_UPDATE_INFO_BYTES],
		       TPS536XX_UPDATE_INFO_BYTES);

		if (i2c_master_write(&msg, retry) != 0) {
			LOG_ERR("Failed to program the image. Invalid index: %d", index);
			return false;
		}

		memset(&msg.data[2], 0, TPS536XX_UPDATE_INFO_BYTES);
	}

	k_msleep(100); // Wait 100 ms for non-volatile memory programming to complete successfully.

	return true;
}

uint8_t tps53689_read(sensor_cfg *cfg, int *reading)
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
	} else if (offset == PMBUS_READ_IOUT || offset == PMBUS_READ_IIN ||
		   offset == PMBUS_READ_TEMPERATURE_1 || offset == PMBUS_READ_POUT) {
		/* SLINEAR11 */
		uint16_t read_value = (msg.data[1] << 8) | msg.data[0];
		float actual_value = slinear11_to_float(read_value);
		sval->integer = actual_value;
		sval->fraction = (actual_value - sval->integer) * 1000;
	} else {
		return SENSOR_FAIL_TO_ACCESS;
	}

	return SENSOR_READ_SUCCESS;
}

uint8_t tps53689_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = tps53689_read;
	return SENSOR_INIT_SUCCESS;
}
