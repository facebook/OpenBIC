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
#include "tda38741.h"
#include "hal_i2c.h"
#include "pmbus.h"
#include "libutil.h"

LOG_MODULE_REGISTER(tda38741);

#define TDA38640_CNFG_BYTE_NUM 4
#define TDA38640_MAX_SECT_NUM 48
#define TDA38640_SECT_COLUMN_NUM 64

#define CHECKSUM_FIELD "[Image 00] : "
#define IMAGE_COUNT "Image Count : "
#define CNFG_TAG "[CNFG]"
#define DATA_END_TAG "[End]"
#define DATA_COMMENT "//"

#define DATA_LEN_IN_LINE 17
#define CNFG_REMAINING_WRITES_MAX 5
#define USER_REMAINING_WRITES_MAX 48
#define VR_PROGRAM_DELAY 200 //200 ms
#define VR_PROGRAM_RECHECK 3

enum {
	CRC_HIGH_REG = 0xAE,
	CRC_LOW_REG = 0xB0,
	CNFG_REG = 0xB2,
	USER_1_REG = 0xB4,
	USER_2_REG = 0xB6,
	USER_3_REG = 0xB8,
	UNLOCK_REGS_REG = 0xD4,
	PROG_CMD_REG = 0xD6,
	PAGE_REG = 0xff,
};

enum {
	CNFG_WR = 0x12,
	USER_RD = 0x41,
	USER_WR = 0x42,
};

enum {
	VR_PAGE_0,
};

struct tda38640_config_sect {
	uint16_t offset[TDA38640_SECT_COLUMN_NUM];
	uint8_t offset_count;
	uint8_t data[TDA38640_SECT_COLUMN_NUM * 16];
};

struct tda38640_config {
	uint32_t checksum;
	uint8_t cnfg_data[TDA38640_CNFG_BYTE_NUM];
	uint8_t sect_count;
	struct tda38640_config_sect section[TDA38640_MAX_SECT_NUM];
};

int pal_bitcount(unsigned int val)
{
	int bitcount = 0;

	while (val) {
		bitcount++;
		val &= (val - 1);
	}
	return bitcount;
}

static bool tda38741_set_page(uint8_t bus, uint8_t addr, uint8_t page)
{
	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;

	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	i2c_msg.tx_len = 2;
	i2c_msg.data[0] = PAGE_REG;
	i2c_msg.data[1] = page;

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to set page to 0x%02X", page);
		return false;
	}

	k_msleep(100);

	return true;
}

static bool tda38741_unlock_reg(uint8_t bus, uint8_t addr, bool unlock)
{
	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	i2c_msg.tx_len = 2;
	i2c_msg.data[0] = UNLOCK_REGS_REG;
	i2c_msg.data[1] = (unlock == true) ? 0x3 : 0x7;

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to do lock control %d", unlock);
		return false;
	}

	return true;
}

static bool tda38741_prog_cmd(uint8_t bus, uint8_t addr, uint8_t action, uint8_t offset,
			      uint8_t *rbuf)
{
	if ((rbuf == NULL) && (action == USER_RD)) {
		LOG_ERR("Invalid parameter pointer is NULL");
		return false;
	}

	I2C_MSG i2c_msg = { 0 };
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	if (action != USER_RD) {
		i2c_msg.tx_len = 2;
		i2c_msg.data[0] = PROG_CMD_REG;
		i2c_msg.data[1] = action;
		if (i2c_master_write(&i2c_msg, 3)) {
			LOG_ERR("Failed to write reg 0x%x", PROG_CMD_REG);
			return false;
		}

		i2c_msg.tx_len = 2;
		i2c_msg.data[0] = PROG_CMD_REG + 1;
		i2c_msg.data[1] = offset;
		if (i2c_master_write(&i2c_msg, 3)) {
			LOG_ERR("Failed to write reg 0x%x", PROG_CMD_REG + 1);
			return false;
		}
	} else {
		i2c_msg.tx_len = 1;
		i2c_msg.rx_len = 1;
		i2c_msg.data[0] = PROG_CMD_REG + 1;
		i2c_msg.data[1] = offset;
		if (i2c_master_write(&i2c_msg, 3)) {
			LOG_ERR("Failed to write reg 0x%x", PROG_CMD_REG + 1);
			return false;
		}

		*rbuf = i2c_msg.data[0];
	}

	return true;
}

static bool read_tda38741_remaining_wr_reg(uint8_t bus, uint8_t addr, uint8_t reg_offset,
					   size_t read_byte, uint8_t *writes_bit_cnt)
{
	CHECK_NULL_ARG_WITH_RETURN(writes_bit_cnt, false);

	uint8_t cnfg_writes_bit_temp;
	uint16_t user_writes_bit_temp;

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = read_byte;
	i2c_msg.data[0] = reg_offset;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to get remaining writes to VR addr:%x", addr);
		return false;
	}

	if (read_byte == sizeof(uint8_t)) {
		// cnfg section only check 5 images mapping to bit[7:3]
		i2c_msg.data[0] &= 0xf8;
		memcpy(&cnfg_writes_bit_temp, i2c_msg.data, sizeof(uint8_t));
		*writes_bit_cnt += pal_bitcount((unsigned int)cnfg_writes_bit_temp);
	} else {
		// user section only check 48 images mapping to bit[7:0] in 3 offsets
		memcpy(&user_writes_bit_temp, i2c_msg.data, sizeof(uint16_t));
		*writes_bit_cnt += pal_bitcount((unsigned int)user_writes_bit_temp);
	}

	return true;
}

bool tda38741_get_remaining_wr(uint8_t bus, uint8_t addr, uint8_t *user_remain,
			       uint8_t *cnfg_remain)
{
	CHECK_NULL_ARG_WITH_RETURN(user_remain, false);
	CHECK_NULL_ARG_WITH_RETURN(cnfg_remain, false);

	uint8_t user_writes_bit = 0;
	uint8_t cnfg_writes_bit = 0;

	if (tda38741_set_page(bus, addr, VR_PAGE_0) == false) {
		LOG_ERR("Failed to set VR page to 0x%x", VR_PAGE_0);
		return false;
	}

	if (read_tda38741_remaining_wr_reg(bus, addr, CNFG_REG, 1, &cnfg_writes_bit) == false) {
		LOG_ERR("Failed to read CNFG remaining writes");
		return false;
	}

	if (read_tda38741_remaining_wr_reg(bus, addr, USER_1_REG, 2, &user_writes_bit) == false) {
		LOG_ERR("Failed to read USER1 remaining writes");
		return false;
	}

	if (read_tda38741_remaining_wr_reg(bus, addr, USER_2_REG, 2, &user_writes_bit) == false) {
		LOG_ERR("Failed to read USER2 remaining writes");
		return false;
	}

	if (read_tda38741_remaining_wr_reg(bus, addr, USER_3_REG, 2, &user_writes_bit) == false) {
		LOG_ERR("Failed to read USER3 remaining writes");
		return false;
	}

	*cnfg_remain = CNFG_REMAINING_WRITES_MAX - cnfg_writes_bit;
	*user_remain = USER_REMAINING_WRITES_MAX - user_writes_bit;

	return true;
}

bool tda38741_get_checksum(uint8_t bus, uint8_t addr, uint32_t *crc)
{
	CHECK_NULL_ARG_WITH_RETURN(crc, false);

	*crc = 0;

	if (tda38741_set_page(bus, addr, VR_PAGE_0) == false) {
		LOG_ERR("Failed to set VR page to 0x%x", VR_PAGE_0);
		return false;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = CRC_HIGH_REG;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to get version high reg");
		return false;
	}

	*crc |= i2c_msg.data[0] << 16;
	*crc |= i2c_msg.data[1] << 24;

	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = CRC_LOW_REG;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to get version low reg");
		return false;
	}

	*crc |= i2c_msg.data[0];
	*crc |= i2c_msg.data[1] << 8;

	return true;
}

static bool parsing_image(uint8_t *img_buff, uint32_t img_size, struct tda38640_config *dev_cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(img_buff, false);
	CHECK_NULL_ARG_WITH_RETURN(dev_cfg, false);

	int i = 0;
	int sect_cnt = 0;
	int cnfg_cnt = 0;
	int offset_cnt = 0;
	int data_cnt = 0;
	bool is_data = false;

	for (i = 0; i < img_size; i++) {
		// pass '\n'
		if (((img_buff[i] == 0x0d) || img_buff[i] == 0x0a)) {
			continue;
		}

		// skip comment '//'
		if (img_buff[i] == 0x2F) {
			i += 2;
			// pass comment
			while (img_buff[i] != 0x0a) {
				i++;
			}
			continue;
		} else if (img_buff[i] == 0x09) {
			// skip tab
			continue;
		} else if (img_buff[i] == '[') {
			if (i + strlen(DATA_END_TAG) <= img_size) {
				if (!strncmp(&img_buff[i], DATA_END_TAG, strlen(DATA_END_TAG))) {
					LOG_INF("End of data");
					break;
				}
			}
			if (i + strlen(CNFG_TAG) <= img_size) {
				if (!strncmp(&img_buff[i], CNFG_TAG, strlen(CNFG_TAG))) {
					i += strlen(CNFG_TAG); //pass "[CNFG]"
					i += 2; //go to next line
					i += 5; //pass 4 words and space
					while (img_buff[i] != 0x0d) {
						if (img_buff[i] == 0x20) {
							i++;
							continue;
						}
						dev_cfg->cnfg_data[cnfg_cnt++] =
							ascii_to_val(img_buff[i]) * 16 +
							ascii_to_val(img_buff[i + 1]);
						i += 2; //pass the rest of byte in current line
					}
					continue;
				}
			}
			if (i + strlen(CHECKSUM_FIELD) <= img_size) {
				if (!strncmp(&img_buff[i], CHECKSUM_FIELD,
					     strlen(CHECKSUM_FIELD))) {
					i += strlen(CHECKSUM_FIELD);
					i += 2; //pass '0x'
					dev_cfg->checksum = 0;
					dev_cfg->checksum |= (ascii_to_val(img_buff[i]) * 16 +
							      ascii_to_val(img_buff[i + 1]))
							     << 24;
					i += 2;
					dev_cfg->checksum |= (ascii_to_val(img_buff[i]) * 16 +
							      ascii_to_val(img_buff[i + 1]))
							     << 16;
					i += 2;
					dev_cfg->checksum |= (ascii_to_val(img_buff[i]) * 16 +
							      ascii_to_val(img_buff[i + 1]))
							     << 8;
					i += 2;
					dev_cfg->checksum |= (ascii_to_val(img_buff[i]) * 16 +
							      ascii_to_val(img_buff[i + 1]));
					i++;

					is_data = true;
					continue;
				}
			}
		} else {
			if (is_data == true) {
				uint16_t offset = 0;
				offset |= (ascii_to_val(img_buff[i]) * 16 +
					   ascii_to_val(img_buff[i + 1]))
					  << 8;
				i += 2;
				offset |= (ascii_to_val(img_buff[i]) * 16 +
					   ascii_to_val(img_buff[i + 1]));
				i += 2;

				dev_cfg->section[sect_cnt].offset[offset_cnt++] = offset;

				while (img_buff[i] != 0x0d) {
					if (img_buff[i] == 0x20) {
						i++;
						continue;
					}
					if (data_cnt >= TDA38640_SECT_COLUMN_NUM * 16) {
						LOG_ERR("Exceed max data count");
						return false;
					}
					dev_cfg->section[sect_cnt].data[data_cnt++] =
						ascii_to_val(img_buff[i]) * 16 +
						ascii_to_val(img_buff[i + 1]);
					i += 2;
				}

				if (offset == 0x0070) {
					is_data = false;
					dev_cfg->section[sect_cnt].offset_count = offset_cnt;
					sect_cnt++;
				}
			}

			if (i + strlen(IMAGE_COUNT) <= img_size) {
				if (!strncmp(&img_buff[i], IMAGE_COUNT, strlen(IMAGE_COUNT))) {
					i += strlen(IMAGE_COUNT);
					dev_cfg->sect_count = ascii_to_val(img_buff[i]);

					if ((dev_cfg->sect_count != 1) ||
					    (img_buff[i + 1] != 0x0d)) {
						LOG_ERR("Invalid image count field, should only be 1");
						return false;
					}
					continue;
				}
			}
		}
	}

	return true;
}

bool tda38741_fwupdate(uint8_t bus, uint8_t addr, uint8_t *img_buff, uint32_t img_size)
{
	CHECK_NULL_ARG_WITH_RETURN(img_buff, false);

	bool ret = false;
	uint8_t user_remain, cnfg_remain;
	uint8_t rbuf_prog_progress;

	/* Parsing image */
	struct tda38640_config *config = NULL;
	config = (struct tda38640_config *)calloc(1, sizeof(struct tda38640_config));
	if (config == NULL) {
		LOG_ERR("No space for creating config!");
		return false;
	}

	if (parsing_image(img_buff, img_size, config) == false) {
		LOG_ERR("Failed to parsing image!");
		goto exit;
	}

	/* Image parsing result */
	LOG_INF("* crc32 : 0x%x", config->checksum);
	LOG_HEXDUMP_INF(config->cnfg_data, TDA38640_CNFG_BYTE_NUM, "* cnfg data: ");
	LOG_INF("* sec count: 0x%x", config->sect_count);
	LOG_INF("* offset count: %d", config->section[0].offset_count);
	for (int i = 0; i < config->section[0].offset_count; i++)
		LOG_INF("  offset[%d]: 0x%x", i, config->section[0].offset[i]);
	LOG_HEXDUMP_INF(config->section[0].data, config->section[0].offset_count * 16, "* data: ");

	uint32_t sum = 0;
	if (tda38741_get_checksum(bus, addr, &sum) == false) {
		LOG_ERR("Failed to get tda38640 checksum");
		goto exit;
	}

	LOG_INF("* Device checksum: 0x%x", sum);

	/* program user section into VR register through i2c
	step 1: check remaining writes
	step 2: write 0x3 to register 0xd4 to unlock device registers
	step 3: write bytes by following .mic file's order
	step 4: write program command to register 0xd6
	step 5: check register 0xd7[7] for programming progress
	*/
	for (int i = 0; i < config->sect_count; i++) {
		if (tda38741_get_remaining_wr(bus, addr, &user_remain, &cnfg_remain) == false) {
			LOG_ERR("Failed to get remaining writes");
			goto exit;
		}

		LOG_INF("* Remaining Writes for USER: %d CFG: %d", user_remain, cnfg_remain);
		if (!user_remain) {
			LOG_WRN("No remaining writes for USER while writing section 0x%x", i);
			goto exit;
		}

		if (tda38741_unlock_reg(bus, addr, true) == false)
			goto exit;

		uint8_t page_tmp = 0;
		for (int j = 0; j < config->section[i].offset_count; j++) {
			uint8_t offset_tmp[2];
			memcpy(&offset_tmp, &config->section[i].offset[j], sizeof(uint16_t));

			if (page_tmp != offset_tmp[1]) {
				if (tda38741_unlock_reg(bus, addr, false) == false)
					goto exit;

				if (tda38741_set_page(bus, addr, offset_tmp[1]) == false)
					goto exit;

				page_tmp = offset_tmp[1];

				if (tda38741_unlock_reg(bus, addr, true) == false)
					goto exit;
			}

			//write from byte 0 to byte 15 per column
			for (int k = 0; k < 16; k++) {
				int data_iter = (j * 16) + k;

				I2C_MSG i2c_msg = { 0 };
				i2c_msg.bus = bus;
				i2c_msg.target_addr = addr;

				i2c_msg.tx_len = 2;
				i2c_msg.data[0] = (offset_tmp[0] + k);
				i2c_msg.data[1] = config->section[i].data[data_iter];

				if (i2c_master_write(&i2c_msg, 3)) {
					LOG_ERR("Failed to write program data to VR in section 0x%x offset 0x%x byte 0x%x",
						i, j, k);
					goto exit;
				}
			}
		}

		if (tda38741_prog_cmd(bus, addr, USER_WR, (USER_REMAINING_WRITES_MAX - user_remain),
				      NULL) == false) {
			LOG_ERR("Failed to do write program cmd to VR in section 0x%x", i);
			goto exit;
		}

		k_msleep(VR_PROGRAM_DELAY);

		int max_retry_time = 200;
		int retry = 0;
		for (retry = 0; retry < max_retry_time; retry++) {
			if (tda38741_prog_cmd(bus, addr, USER_RD, 0, &rbuf_prog_progress) == true) {
				break;
			}
		}

		if ((retry == max_retry_time) || ((rbuf_prog_progress & 0x80) != 0x80)) {
			LOG_ERR("Failed to check program progress done in section 0x%x", i);
			goto exit;
		}
	}

	k_msleep(500);

	if (tda38741_get_checksum(bus, addr, &sum) == false) {
		LOG_ERR("Failed to get tda38640 checksum");
		goto exit;
	}

	if (sum != config->checksum) {
		LOG_ERR("Checksum verify failed!");
		goto exit;
	}

	ret = true;
exit:
	free(config);

	if (tda38741_unlock_reg(bus, addr, false) == false) {
		LOG_ERR("Failed to lock VR registers");
		ret = false;
	}

	return ret;
}
