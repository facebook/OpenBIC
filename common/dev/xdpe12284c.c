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
#include "util_pmbus.h"
#include "libutil.h"
#include "xdpe12284c.h"
#include "pldm_firmware_update.h"

LOG_MODULE_REGISTER(xdpe12284c);

// XDPE12284C
#define VR_XDPE_PAGE_20 0x20
#define VR_XDPE_PAGE_32 0x32
#define VR_XDPE_PAGE_50 0x50
#define VR_XDPE_PAGE_60 0x60
#define VR_XDPE_PAGE_62 0x62

#define VR_XDPE_REG_REMAIN_WR 0x82 // page 0x50
#define VR_XDPE_REG_CRC_L 0x42 // page 0x62
#define VR_XDPE_REG_CRC_H 0x43 // page 0x62
#define VR_XDPE_REG_NEXT_MEM 0x65 // page 0x62

#define VR_XDPE_TOTAL_RW_SIZE 1080

#define VR_XDPE_REG_LOCK 0x1A

#define VR_WARN_REMAIN_WR 3

#define MAX_CMD_LINE 1080

enum { VR12 = 1,
       VR13,
       IMVP9,
};

enum { VID_IDENTIFIER = 1,
};

struct xdpe_config {
	uint8_t addr;
	uint16_t memptr;
	uint32_t crc_exp;
	uint8_t *data;
};

static bool set_page(uint8_t bus, uint8_t addr, uint8_t page)
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

	return true;
}

bool xdpe12284c_get_checksum(uint8_t bus, uint8_t target_addr, uint8_t *checksum)
{
	CHECK_NULL_ARG_WITH_RETURN(checksum, false);

	if (set_page(bus, target_addr, VR_XDPE_PAGE_62) == false)
		return false;

	I2C_MSG i2c_msg;
	uint8_t retry = 3;

	i2c_msg.bus = bus;
	i2c_msg.target_addr = target_addr;

	//Read lower word for the 32bit checksum value
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = VR_XDPE_REG_CRC_H;
	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read register 0x%02X", VR_XDPE_REG_CRC_H);
		return false;
	}

	checksum[0] = i2c_msg.data[1];
	checksum[1] = i2c_msg.data[0];

	if (set_page(bus, target_addr, VR_XDPE_PAGE_62) == false)
		return false;

	//Read higher word for the 32bit checksum value
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = VR_XDPE_REG_CRC_L;
	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read register 0x%02X", VR_XDPE_REG_CRC_L);
		return false;
	}

	checksum[2] = i2c_msg.data[1];
	checksum[3] = i2c_msg.data[0];

	return true;
}

bool xdpe12284c_get_remaining_write(uint8_t bus, uint8_t target_addr, uint16_t *remain_write)
{
	CHECK_NULL_ARG_WITH_RETURN(remain_write, false);

	if (set_page(bus, target_addr, VR_XDPE_PAGE_50) == false)
		return false;

	I2C_MSG i2c_msg;
	uint8_t retry = 3;

	i2c_msg.bus = bus;
	i2c_msg.target_addr = target_addr;

	//Read the remaining writes from register address 0x82
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = VR_XDPE_REG_REMAIN_WR;
	if (i2c_master_read(&i2c_msg, retry)) {
		printf("<error> XDPE12284C get remaining write while i2c reading\n");
		return false;
	}

	//the data residing in bit11~bit6 is the number of the remaining writes.
	*remain_write = (((i2c_msg.data[1] << 8) | i2c_msg.data[0]) & 0xFC0) >> 6;
	return true;
}

/*  Reference: Infineon spec section 8.24: VID table
 *  PMBUS spec section 8.2: VOUT mode
 */
static float vid_to_float(int val, uint8_t vout_mode)
{
	uint8_t mode = 0;

	//VID 0 is always 0 V
	if (val == 0) {
		return 0;
	}

	mode = (vout_mode >> 5);

	if (mode != VID_IDENTIFIER) {
		printf("%s Infineon VR reading with invalid VID IDENTIFIER: %x", __func__, mode);
		return -1;
	}

	switch (vout_mode & 0x1f) {
	case VR12:
		if (val > 0) {
			return ((val - 1) * 5 + 250);
		}
	case VR13:
		if (val > 0) {
			return ((val - 1) * 10 + 500);
		}
	case IMVP9:
		if (val > 0) {
			return ((val - 1) * 10 + 200);
		}
	default:
		printf("%s Infineon VR reading with invalid vout mode: %x", __func__, vout_mode);
		return -1;
	}

	return 0;
}

static bool find_addr_and_crc(uint8_t *buff, struct xdpe_config *dev_cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(buff, false);
	CHECK_NULL_ARG_WITH_RETURN(dev_cfg, false);

	if (strncmp(buff, "XDPE12284C", 10))
		return false;

	uint8_t collect_level = 0;

	int idx = 0, val = 0;
	while (buff[idx] != 0x0d) {
		if (!strncmp(&buff[idx], " - 0x", 5)) {
			collect_level++;
			idx += 5;
		}

		// collect address
		if (collect_level == 1) {
			dev_cfg->addr = ascii_to_val(buff[idx]) * 16 + ascii_to_val(buff[idx + 1]);
			LOG_DBG("addr get = %x", dev_cfg->addr);
			idx += 2;
		} else if (collect_level == 2) {
			dev_cfg->crc_exp = 0;
			while (buff[idx] != 0x0d) {
				val = ascii_to_val(buff[idx]);
				if (val == -1) {
					LOG_ERR("Image got format error in line %d", __LINE__);
					return false;
				}
				dev_cfg->crc_exp = (dev_cfg->crc_exp << 4) | val;
				if (idx == fw_update_cfg.image_size) {
					LOG_ERR("Image got format error in line %d", __LINE__);
					return false;
				}
				idx++;
			}
			LOG_DBG("crc get = %x", dev_cfg->crc_exp);
			return true;
		} else {
			idx++;
		}
	}

	return true;
}

static bool parsing_image(uint8_t *hex_buff, struct xdpe_config *dev_cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(hex_buff, false);
	CHECK_NULL_ARG_WITH_RETURN(dev_cfg, false);

	bool ret = false;
	int max_data_size = MAX_CMD_LINE;
	dev_cfg->data = (uint8_t *)malloc(max_data_size);
	if (!dev_cfg->data) {
		LOG_ERR("Failed to malloc data!");
		goto exit;
	}

	dev_cfg->crc_exp = 0;

	bool rec_flag = false;
	bool new_line = true;
	bool got_end_flag = false;
	uint16_t ofst = 0;
	uint16_t value = 0;
	int data_idx = 0, val;
	for (int i = 0; i < fw_update_cfg.image_size; i++) {
		/* collect data */
		if (rec_flag == true) {
			// grep offset and exit keyword
			if (new_line == true) {
				val = ascii_to_val(hex_buff[i]);
				if (val == -1) {
					LOG_ERR("Image got format error in line %d", __LINE__);
					goto exit;
				}
				if (val != 2) {
					if (i + 3 < fw_update_cfg.image_size) {
						if (!strncmp(&hex_buff[i], "[End", 4)) {
							got_end_flag = true;
							break;
						}
					}
					/* Assume there's no other key to access */
					got_end_flag = true;
					break;
				} else {
					if (i + 3 < fw_update_cfg.image_size) {
						ofst = 0;
						for (int j = i; j < (i + 4); j++) {
							val = ascii_to_val(hex_buff[j]);
							if (val == -1) {
								LOG_ERR("Image got format error in line %d",
									__LINE__);
								goto exit;
							}
							ofst = (ofst << 4) | val;
						}
						i += 3; //pass next 4 bytes offset
					}
				}
				new_line = false;
			} else {
				if (hex_buff[i] == ' ') {
					if (i + 4 >= fw_update_cfg.image_size) {
						LOG_ERR("Image got format error in line %d",
							__LINE__);
						goto exit;
					}

					// skip collect empty data '----'
					if (hex_buff[i + 1] == '-') {
						ofst++;
						i += 4; //pass '----'
						continue;
					}

					value = 0;
					for (int j = i + 1; j < (i + 5); j++) {
						val = ascii_to_val(hex_buff[j]);
						if (val == -1) {
							LOG_ERR("Image got format error in line %d",
								__LINE__);
							goto exit;
						}
						value = (value << 4) | val;
					}

					if (data_idx + 3 >= MAX_CMD_LINE) {
						LOG_ERR("Data collect over limit size %d",
							max_data_size);
						goto exit;
					}
					LOG_DBG("collect new data ofst: 0x%x val: 0x%x", ofst,
						value);
					memcpy(&dev_cfg->data[data_idx], &ofst, 2);
					memcpy(&dev_cfg->data[data_idx + 2], &value, 2);
					data_idx += 4;
					ofst++;
					i += 4; //pass 4 bytes data
				} else if (hex_buff[i] == 0x0d) {
					// '\n'
					i++;
					new_line = true;
				} else {
					LOG_ERR("Image got format error in line %d", __LINE__);
					goto exit;
				}
			}
		}

		/* parsing address and crc */
		if (i + 9 < fw_update_cfg.image_size) {
			if (!strncmp(&hex_buff[i], "XDPE12284C", 10)) {
				if (find_addr_and_crc(&hex_buff[i], dev_cfg) == false) {
					goto exit;
				}
			}
		}

		/* parsing main data */
		if (i + 13 < fw_update_cfg.image_size) {
			if (!strncmp(&hex_buff[i], "[Config Data]", 13)) {
				while (hex_buff[i] != 0x0a) {
					i++; //pass the rest of byte in current line
					if (i == fw_update_cfg.image_size) {
						LOG_ERR("Image got format error8");
						goto exit;
					}
				}
				rec_flag = true;
				continue;
			}
		}
	}

	ret = got_end_flag;

exit:
	if (ret == false)
		SAFE_FREE(dev_cfg->data);

	return ret;
}

bool xdpe12284c_fwupdate(uint8_t bus, uint8_t addr, uint8_t *hex_buff)
{
	CHECK_NULL_ARG_WITH_RETURN(hex_buff, false);

	uint8_t ret = false;

	uint8_t dev_i2c_bus = bus;
	uint8_t dev_i2c_addr = addr;

	uint8_t crc[4] = { 0 };
	uint16_t remain = 0;

	/* Step1. Before update */
	if (xdpe12284c_get_checksum(dev_i2c_bus, dev_i2c_addr, crc) == false) {
		return false;
	}

	if (xdpe12284c_get_remaining_write(dev_i2c_bus, dev_i2c_addr, &remain) == false) {
		return false;
	}

	if (!remain) {
		LOG_ERR("No remaining writes");
		return false;
	}
	if (remain <= VR_WARN_REMAIN_WR) {
		LOG_WRN("The remaining writes %d is below the threshold value %d!\n", remain,
			VR_WARN_REMAIN_WR);
	}

	/* Step2. Image parsing */
	struct xdpe_config dev_cfg = { 0 };
	if (parsing_image(hex_buff, &dev_cfg) == false) {
		LOG_ERR("Failed to parsing image!");
		goto exit;
	}

	LOG_INF("XDPE12284c device(bus: %d addr: 0x%x) info:", dev_i2c_bus, dev_i2c_addr);
	LOG_INF("* crc:              0x%02x%02x%02x%02x", crc[0], crc[1], crc[2], crc[3]);
	LOG_INF("* image crc:        0x%x", dev_cfg.crc_exp);
	LOG_INF("* image addr:       0x%x", dev_cfg.addr >> 1);
	LOG_INF("* remaining writes: %d", remain);

	/* Step3. FW Update */
	bool rc = false;
	uint8_t page = 0;
	float dsize = 0, next_prog = 0;

	I2C_MSG i2c_msg;
	uint8_t retry = 3;
	i2c_msg.bus = dev_i2c_bus;
	i2c_msg.target_addr = dev_i2c_addr;

	// read next memory location
	if (set_page(dev_i2c_bus, dev_i2c_addr, VR_XDPE_PAGE_62) == false) {
		goto exit;
	}

	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = VR_XDPE_REG_NEXT_MEM;
	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read register 0x%02X", VR_XDPE_REG_NEXT_MEM);
		goto exit;
	}

	dev_cfg.memptr = ((i2c_msg.data[1] << 8) | i2c_msg.data[0]) & 0x3FF;
	LOG_INF("Memory pointer: 0x%X", dev_cfg.memptr);

	// write configuration data
	uint8_t *data = dev_cfg.data;
	dsize = (float)VR_XDPE_TOTAL_RW_SIZE / 100;
	next_prog = dsize;
	for (int i = 0; i < VR_XDPE_TOTAL_RW_SIZE; i += 4) {
		if (page != data[i + 1]) {
			page = data[i + 1];
			if ((rc = set_page(dev_i2c_bus, dev_i2c_addr, page)) == false) {
				goto exit;
			}
		}

		i2c_msg.tx_len = 3;
		i2c_msg.data[0] = data[i]; //offset
		i2c_msg.data[1] = data[i + 2];
		i2c_msg.data[2] = data[i + 3];
		if (i2c_master_write(&i2c_msg, retry)) {
			LOG_ERR("wr failed: page=%02X offset=%02X data=%02X%02X", page, data[i],
				data[i + 3], data[i + 2]);
			goto exit;
		}

		// read back to compare
		i2c_msg.tx_len = 1;
		i2c_msg.rx_len = 2;
		i2c_msg.data[0] = data[i]; //offset
		if (i2c_master_read(&i2c_msg, retry)) {
			LOG_ERR("rd failed: page=%02X offset=%02X", page, data[i]);
			goto exit;
		}

		if (memcmp(&data[i + 2], i2c_msg.data, 2)) {
			LOG_ERR("data %02X%02X mismatch, expect %02X%02X", data[i + 3], data[i + 2],
				i2c_msg.data[1], i2c_msg.data[0]);
			goto exit;
		}

		if ((i + 4) >= (int)next_prog) {
			LOG_INF("updated: %d%%", (int)((next_prog + dsize / 2) / dsize));
			next_prog += dsize;
		}
	}

	// save configuration to EMTP
	if (set_page(dev_i2c_bus, dev_i2c_addr, VR_XDPE_PAGE_32) == false) {
		goto exit;
	}

	i2c_msg.tx_len = 3;
	i2c_msg.data[0] = VR_XDPE_REG_LOCK;
	i2c_msg.data[1] = 0xA1;
	i2c_msg.data[2] = 0x08;
	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to unlock register 0x%02X", VR_XDPE_REG_LOCK);
		goto exit;
	}

	i2c_msg.tx_len = 1;
	i2c_msg.data[0] = 0x1D; //clear fault
	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to write register 0x%02X", 0x1D);
		goto exit;
	}

	i2c_msg.tx_len = 1;
	i2c_msg.data[0] = 0x26; //upload from the registers to EMTP
	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to write register 0x%02X", 0x26);
		goto exit;
	}

	k_msleep(500);

	if (set_page(dev_i2c_bus, dev_i2c_addr, VR_XDPE_PAGE_60) == false) {
		goto exit;
	}

	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = 0x01;
	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read register 0x%02X", 0x01);
		goto exit;
	}
	if ((i2c_msg.data[0] & 0x01)) {
		LOG_ERR("Unexpected status, reg[%02X]=%02X", 0x01, i2c_msg.data[0]);
		goto exit;
	}

	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = 0x02;
	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read register 0x%02X", 0x02);
		goto exit;
	}
	if ((i2c_msg.data[0] & 0x0A)) {
		LOG_ERR("Unexpected status, reg[%02X]=%02X", 0x02, i2c_msg.data[0]);
		goto exit;
	}

	if (set_page(dev_i2c_bus, dev_i2c_addr, VR_XDPE_PAGE_32) == false) {
		goto exit;
	}

	i2c_msg.tx_len = 3;
	i2c_msg.data[0] = VR_XDPE_REG_LOCK;
	i2c_msg.data[1] = 0x00;
	i2c_msg.data[2] = 0x00;
	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to lock register 0x%02X", VR_XDPE_REG_LOCK);
		goto exit;
	}

	ret = true;
exit:
	SAFE_FREE(dev_cfg.data);
	return ret;
}

uint8_t xdpe12284c_read(uint8_t sensor_num, int *reading)
{
	if (reading == NULL || (sensor_num > SENSOR_NUM_MAX)) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	int val = 0;
	sensor_val *sval = (sensor_val *)reading;
	I2C_MSG msg;
	memset(sval, 0, sizeof(sensor_val));
	float actual_value = 0;
	uint8_t offset = sensor_config[sensor_config_index_map[sensor_num]].offset;

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = offset;

	if (i2c_master_read(&msg, retry)) {
		return SENSOR_FAIL_TO_ACCESS;
	}

	val = (msg.data[1] << 8) | msg.data[0];

	switch (offset) {
	case PMBUS_READ_IOUT:
	case PMBUS_READ_POUT:
	case PMBUS_READ_TEMPERATURE_1:
		actual_value = slinear11_to_float(val);
		sval->integer = actual_value;
		sval->fraction = (actual_value - sval->integer) * 1000;
		break;
	case PMBUS_READ_VOUT:
		msg.tx_len = 1;
		msg.rx_len = 1;
		msg.data[0] = PMBUS_VOUT_MODE;

		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}

		actual_value = vid_to_float(val, msg.data[0]);
		actual_value /= 1000; // mV to V
		sval->integer = actual_value;
		sval->fraction = (actual_value - sval->integer) * 1000;
		break;
	default:
		return SENSOR_FAIL_TO_ACCESS;
	}

	return SENSOR_READ_SUCCESS;
}

uint8_t xdpe12284c_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = xdpe12284c_read;
	return SENSOR_INIT_SUCCESS;
}
