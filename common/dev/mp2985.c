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
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"
#include "mp2985.h"
#include "util_pmbus.h"

LOG_MODULE_REGISTER(mp2985);

#define ENABLE_LOW_POWER_MODE_BIT BIT(0)
#define ENABLE_PROGRAM_MODE_BIT BIT(2)
#define PASSWORD_MATCH_BIT BIT(3)
#define MFR_TEST_MODE_EN_BIT BIT(7)

#define MP2985_VENDOR_ID 0x4D5053
#define MP2985_MODULE_ID 0x0185

#define CR_ASCII 0x0D
#define TAB_ASCII 0x09
#define NEW_LINE_ASCII 0x0A
#define MP2985_DATA_END_TAG "END"
#define MP2985_PRODUCT_ID_TAG "Product ID:	"
#define MP2985_4_DIGITAL_TAG "4-digital Code:	"
#define MP2985_CRC_CHECK_START_TAG "CRC_CHECK_START"
#define MP2985_PRODUCT_ID "MP2985H"

enum MP2985_REG {
	MP2985_MFR_DEBUG = 0x04,
	MP2985_PAGE_PLUS_WRITE = 0x05,
	MP2985_STORE_USER_CODE = 0x15,
	MP2985_MFR_VR_CONFIG2 = 0x35,
	MP2985_PAGE3_DISABLE_STORE_FAULT = 0x51,
	MP2985_MFR_ID = 0x99,
	MP2985_MFR_MODULE = 0x9A,
	MP2985_MFR_REVISION = 0x9B,
	MP2985_GET_USER_DATA = 0xB8,
	MP2985_USER_PWD_CHECK_CMD = 0xC7,
	MP2985_MFR_USER_PWD = 0xC9,
};

enum MP2985_CONFIG_FILE_COLUMN {
	ATE_CONF_ID,
	ATE_PAGE_NUM,
	ATE_REG_ADDR_HEX,
	ATE_REG_ADDR_DEC,
	ATE_REG_NAME,
	ATE_REG_DATA_HEX,
	ATE_REG_DATA_DEC,
	ATE_WRITE_TYPE,
	ATE_COLUMN_MAX,
};

enum ENTER_PAGE3_CONTROL_OPTION {
	DISABLE_ENTER_PAGE3 = false,
	ENABLE_ENTER_PAGE = true,
};

struct mp2985_data {
	uint16_t cfg_id;
	uint8_t page;
	uint8_t reg_addr;
	uint8_t reg_data[4];
	uint8_t reg_len;
};

struct mp2985_config {
	uint16_t cfg_id;
	uint16_t wr_cnt;
	uint16_t product_id_exp;
	uint16_t crc_code[2];
	struct mp2985_data pdata[1024];
};

static int mp2985_unlock_password(uint8_t bus, uint8_t addr)
{
	int ret = 0;
	uint8_t retry = 3;
	uint8_t tbuf[3] = { 0 };
	I2C_MSG msg = { 0 };

	ret = pmbus_set_page(bus, addr, PMBUS_PAGE_0);
	if (ret != 0) {
		LOG_ERR("Fail to set page to 0 for checking CML status on unlock password func, bus: 0x%x, addr: 0x%x, ret: %d",
			bus, addr, ret);
		return ret;
	}

	tbuf[0] = PMBUS_STATUS_CML;
	msg = construct_i2c_message(bus, addr, 1, tbuf, 1);
	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Fail to read CML status on unlock password func, bus: 0x%x, addr: 0x%x, ret: %d",
			bus, addr, ret);
		return ret;
	}

	if ((msg.data[0] & PASSWORD_MATCH_BIT) == 0) {
		ret = pmbus_set_page(bus, addr, PMBUS_PAGE_1);
		if (ret != 0) {
			LOG_ERR("Fail to set page to 1 for reading password, bus: 0x%x, addr: 0x%x, ret: %d",
				bus, addr, ret);
			return ret;
		}

		tbuf[0] = MP2985_MFR_USER_PWD;
		msg = construct_i2c_message(bus, addr, 1, tbuf, 2); // Read password
		ret = i2c_master_read(&msg, retry);
		if (ret != 0) {
			LOG_ERR("Fail to read password, bus: 0x%x, addr: 0x%x, ret: %d", bus, addr,
				ret);
			return ret;
		}

		ret = pmbus_set_page(bus, addr, PMBUS_PAGE_0);
		if (ret != 0) {
			LOG_ERR("Fail to set page to 0 for unlocking password, bus: 0x%x, addr: 0x%x, ret: %d",
				bus, addr, ret);
			return ret;
		}

		tbuf[0] = MP2985_USER_PWD_CHECK_CMD;
		tbuf[1] = msg.data[0];
		tbuf[2] = msg.data[1];
		msg = construct_i2c_message(bus, addr, 3, tbuf, 0); // Unlock password
		ret = i2c_master_write(&msg, retry);
		if (ret != 0) {
			LOG_ERR("Fail to unlock password, bus: 0x%x, addr: 0x%x, ret: %d", bus,
				addr, ret);
			return ret;
		}

		tbuf[0] = PMBUS_STATUS_CML;
		msg = construct_i2c_message(bus, addr, 1, tbuf, 1);
		ret = i2c_master_read(&msg, retry);
		if (ret != 0) {
			LOG_ERR("Fail to recheck CML status, bus: 0x%x, addr: 0x%x, ret: %d", bus,
				addr, ret);
			return ret;
		}

		if ((msg.data[0] & PASSWORD_MATCH_BIT) == 0) {
			LOG_ERR("Fail to unlock password after checking CML status, bus: 0x%x, addr: 0x%x",
				bus, addr);
			return -1;
		}
	}
	return 0;
}

static int mp2985_unlock_write_protection(uint8_t bus, uint8_t addr)
{
	int ret = 0;
	uint8_t retry = 3;
	I2C_MSG msg = { 0 };

	ret = pmbus_set_page(bus, addr, PMBUS_PAGE_0);
	if (ret != 0) {
		LOG_ERR("Fail to set page to 0 for checking mfr config, bus: 0x%x, addr: 0x%x, ret: %d",
			bus, addr, ret);
		return ret;
	}

	msg.bus = bus;
	msg.target_addr = addr;
	msg.tx_len = 2;
	msg.data[0] = PMBUS_WRITE_PROTECT;
	msg.data[1] = 0x00; // Disable memory write protection

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Fail to disable write protection, bus: 0x%x, addr: 0x%x, ret: %d", bus,
			addr, ret);
	}
	return ret;
}

static int mp2985_control_entering_page3(uint8_t bus, uint8_t addr, bool optional)
{
	int ret = 0;
	uint8_t retry = 3;
	uint8_t tbuf[3] = { 0 };
	I2C_MSG msg = { 0 };

	ret = pmbus_set_page(bus, addr, PMBUS_PAGE_1);
	if (ret != 0) {
		LOG_ERR("Fail to set page to 1 for entering page3, bus: 0x%x, addr: 0x%x, ret: %d",
			bus, addr, ret);
		return ret;
	}

	tbuf[0] = MP2985_MFR_DEBUG;
	msg = construct_i2c_message(bus, addr, 1, tbuf, 2);
	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Fail to read mfr debug register, bus: 0x%x, addr: 0x%x, ret: %d", bus,
			addr, ret);
		return ret;
	}

	if ((optional == DISABLE_ENTER_PAGE3 && ((msg.data[1] & MFR_TEST_MODE_EN_BIT) != 0)) ||
	    ((optional == ENABLE_ENTER_PAGE) && ((msg.data[1] & MFR_TEST_MODE_EN_BIT) == 0))) {
		if (optional == DISABLE_ENTER_PAGE3) {
			tbuf[2] = (msg.data[1] | MFR_TEST_MODE_EN_BIT);
		} else {
			tbuf[2] = (msg.data[1] & (~MFR_TEST_MODE_EN_BIT));
		}

		tbuf[0] = MP2985_MFR_DEBUG;
		tbuf[1] = msg.data[0];
		msg = construct_i2c_message(bus, addr, 3, tbuf, 0);
		ret = i2c_master_write(&msg, retry);
		if (ret != 0) {
			LOG_ERR("Fail to write mfr debug register, bus: 0x%x, addr: 0x%x, ret: %d",
				bus, addr, ret);
			return ret;
		}
	}
	return ret;
}

static int mp2985_disable_store_fault_triggering(uint8_t bus, uint8_t addr)
{
	int ret = 0;
	uint8_t retry = 3;
	I2C_MSG msg = { 0 };

	ret = pmbus_set_page(bus, addr, PMBUS_PAGE_3);
	if (ret != 0) {
		LOG_ERR("Fail to set page to 3 for disable store fault, bus: 0x%x, addr: 0x%x, ret: %d",
			bus, addr, ret);
		return ret;
	}

	msg.bus = bus;
	msg.target_addr = addr;
	msg.tx_len = 3;
	msg.data[0] = MP2985_PAGE3_DISABLE_STORE_FAULT;
	msg.data[1] = 0x00;
	msg.data[2] = 0x10;

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Fail to write page3 disable store fault register, bus: 0x%x, addr: 0x%x, ret: %d",
			bus, addr, ret);
	}
	return ret;
}

static int mp2985_write_data(uint8_t bus, uint8_t addr, uint8_t reg_addr, struct mp2985_data *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, -1);

	int ret = 0;
	uint8_t retry = 3;
	I2C_MSG msg = { 0 };

	ret = pmbus_set_page(bus, addr, data->page);
	if (ret != 0) {
		LOG_ERR("Fail to set page to %d for write data, bus: 0x%x, addr: 0x%x, reg_addr: 0x%x",
			data->page, bus, addr, reg_addr);
		return ret;
	}

	msg.bus = bus;
	msg.target_addr = addr;
	msg.tx_len = data->reg_len + 1;
	msg.data[0] = reg_addr;
	memcpy(&msg.data[1], &data->reg_data[0], data->reg_len);

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Fail to write data to register 0x%x, bus: 0x%x, addr: 0x%x", reg_addr, bus,
			addr);
	}
	return ret;
}

static int mp2985_store_user_code(uint8_t bus, uint8_t addr)
{
	int ret = 0;
	uint8_t retry = 3;
	I2C_MSG msg = { 0 };

	ret = pmbus_set_page(bus, addr, PMBUS_PAGE_0);
	if (ret != 0) {
		LOG_ERR("Fail to set page to 0 for stored user code, bus: 0x%x, addr: 0x%x", bus,
			addr);
		return ret;
	}

	msg.bus = bus;
	msg.target_addr = addr;
	msg.tx_len = 1;
	msg.data[0] = MP2985_STORE_USER_CODE;

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Fail to write the store user code register, bus: 0x%x, addr: 0x%x", bus,
			addr);
	}
	return ret;
}

static int mp2985_check_mfr_config(uint8_t bus, uint8_t addr, uint16_t module_id,
				   uint16_t revision_id)
{
	int ret = 0;
	uint8_t retry = 3;
	uint8_t offset = 0;
	uint32_t compare_val = 0;
	I2C_MSG msg = { 0 };

	ret = pmbus_set_page(bus, addr, PMBUS_PAGE_0);
	if (ret != 0) {
		LOG_ERR("Fail to set page to 0 for checking mfr config, bus: 0x%x, addr: 0x%x, ret: %d",
			bus, addr, ret);
		return ret;
	}

	offset = MP2985_MFR_ID;
	msg = construct_i2c_message(bus, addr, 1, &offset, 4);
	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Fail to read mfr id, bus: 0x%x, addr: 0x%x, ret: %d", bus, addr, ret);
		return ret;
	}

	compare_val = MP2985_VENDOR_ID;
	if (memcmp(&compare_val, &msg.data[1], 3) != 0) {
		LOG_ERR("Vendor id is not match, read id: 0x%x",
			((msg.data[3] << 16) | (msg.data[2] << 8) | msg.data[1]));
		return -1;
	}

	offset = MP2985_MFR_MODULE;
	msg = construct_i2c_message(bus, addr, 1, &offset, 3);
	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Fail to read mfr module id, bus: 0x%x, addr: 0x%x, ret: %d", bus, addr,
			ret);
		return ret;
	}

	if (memcmp(&module_id, &msg.data[1], 2) != 0) {
		LOG_ERR("Module id is not match, read id: 0x%x",
			((msg.data[2] << 8) | msg.data[1]));
		return -1;
	}

	offset = MP2985_MFR_REVISION;
	msg = construct_i2c_message(bus, addr, 1, &offset, 3);
	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Fail to read mfr revision id, bus: 0x%x, addr: 0x%x, ret: %d", bus, addr,
			ret);
		return ret;
	}

	if (memcmp(&revision_id, &msg.data[1], 2) != 0) {
		LOG_ERR("Revision id is not match, read id: 0x%x, revision id: 0x%x",
			((msg.data[2] << 8) | msg.data[1]), revision_id);
		return -1;
	}

	return ret;
}

int mp2985_line_split(char **str, uint8_t *image_buf, uint32_t start_index, uint32_t end_index,
		      int max_size)
{
	CHECK_NULL_ARG_WITH_RETURN(str, -1);
	CHECK_NULL_ARG_WITH_RETURN(image_buf, -1);

	int ret = 0;
	int index = 0;
	int size = 0;

	for (index = start_index; index < end_index; ++index) {
		ret = find_byte_data_in_buf(image_buf, TAB_ASCII, index, end_index);
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

int mp2985_parse_file(struct mp2985_config *config, uint8_t *image_buf, uint32_t image_size)
{
	CHECK_NULL_ARG_WITH_RETURN(config, -1);
	CHECK_NULL_ARG_WITH_RETURN(image_buf, -1);

	int ret = 0;
	int data_count = 0;
	int column_count = 0;
	uint8_t column = 0;
	uint8_t reg_len = 0;
	uint32_t buf_index = 0;
	uint32_t end_index = 0;
	char buf[10] = { 0 };
	char *column_list[ATE_COLUMN_MAX] = { 0 };
	const size_t len_end = strlen(MP2985_DATA_END_TAG);
	const size_t len_product = strlen(MP2985_PRODUCT_ID_TAG);
	const size_t len_dig = strlen(MP2985_4_DIGITAL_TAG);
	const size_t len_crc = strlen(MP2985_CRC_CHECK_START_TAG);
	const size_t len_product_id = strlen(MP2985_PRODUCT_ID);

	for (buf_index = 0; buf_index < image_size; ++buf_index) {
		if (buf_index + len_end < image_size) {
			if (strncmp(&image_buf[buf_index], MP2985_DATA_END_TAG, len_end) == 0) {
				end_index = buf_index + len_end;
				break;
			}
		}

		ret = find_byte_data_in_buf(image_buf, NEW_LINE_ASCII, buf_index, image_size);
		if (ret < 0) {
			LOG_ERR("Find new line byte fail at data field, stop index: 0x%x",
				buf_index);
			return -1;
		}

		end_index = ret;
		column_count = mp2985_line_split(column_list, image_buf, buf_index, end_index,
						 ATE_COLUMN_MAX);
		if (column_count != ATE_COLUMN_MAX) {
			LOG_ERR("Unexpected column: %d, stop index: 0x%x", column_count, buf_index);
			return -1;
		}

		for (column = 0; column < ATE_COLUMN_MAX; ++column) {
			if (data_count >= ARRAY_SIZE(config->pdata)) {
				LOG_ERR("Data count exceed pdata size");
				return -1;
			}

			switch (column) {
			case ATE_CONF_ID:
				config->pdata[data_count].cfg_id =
					(uint16_t)strtol(column_list[column], NULL, 16);
				break;
			case ATE_PAGE_NUM:
				config->pdata[data_count].page =
					(uint8_t)strtol(column_list[column], NULL, 16);
				break;
			case ATE_REG_ADDR_HEX:
				config->pdata[data_count].reg_addr =
					(uint8_t)strtol(column_list[column], NULL, 16);
				break;
			case ATE_REG_DATA_HEX:
				reg_len = (strlen(column_list[column]) / 2);
				for (uint8_t index = 0; index < reg_len; ++index) {
					memcpy(buf, &(column_list[column][index * 2]), 2);

					if ((reg_len - 1 - index) >=
					    ARRAY_SIZE(config->pdata[data_count].reg_data)) {
						LOG_ERR("Register length exceed register data size, register len: 0x%x",
							reg_len);
						return -1;
					}
					config->pdata[data_count].reg_data[reg_len - 1 - index] =
						(uint8_t)strtol(buf, NULL, 16);
				}
				config->pdata[data_count].reg_len = reg_len;
				break;
			case ATE_WRITE_TYPE:
				if (!strncmp(column_list[column], "B", 1)) {
					memmove(&config->pdata[data_count].reg_data[1],
						&config->pdata[data_count].reg_data[0],
						config->pdata[data_count].reg_len);
					config->pdata[data_count].reg_data[0] =
						config->pdata[data_count].reg_len;
					config->pdata[data_count].reg_len += 1;
				}
				break;
			}
		}

		data_count += 1;
		buf_index = end_index;
	}

	config->wr_cnt = data_count;

	for (buf_index = end_index; buf_index < image_size; ++buf_index) {
		ret = find_byte_data_in_buf(image_buf, NEW_LINE_ASCII, buf_index, image_size);
		if (ret < 0) {
			LOG_ERR("Find new line byte fail at additional field, stop index: 0x%x",
				buf_index);
			break;
		}

		end_index = ret;

		if ((buf_index + len_product + len_product_id) < image_size) {
			if (strncmp(&image_buf[buf_index], MP2985_PRODUCT_ID_TAG, len_product) ==
			    0) {
				if (strncmp(&image_buf[buf_index + len_product], MP2985_PRODUCT_ID,
					    len_product_id) == 0) {
					config->product_id_exp = MP2985_MODULE_ID;
					buf_index = end_index;
					continue;
				}
			}
		}

		if (buf_index + len_dig < image_size) {
			if (strncmp(&image_buf[buf_index], MP2985_4_DIGITAL_TAG, len_dig) == 0) {
				column_count = mp2985_line_split(column_list, image_buf, buf_index,
								 end_index, ATE_COLUMN_MAX);
				if (column_count < 2) {
					LOG_ERR("Invalid 4-digital line ,stop index: 0x%x",
						buf_index);
					return -1;
				}
				config->cfg_id = (uint16_t)strtol(column_list[1], NULL, 16);
				buf_index = end_index;
				continue;
			}
		}

		if (buf_index + len_crc < image_size) {
			if (strncmp(&image_buf[buf_index], MP2985_CRC_CHECK_START_TAG, len_crc) ==
			    0) {
				buf_index = (end_index + 1); // Next line

				uint8_t crc_count_index = 0;
				const uint8_t crc_count_max = 2; // CRC code is in the next two line

				for (crc_count_index = 0; crc_count_index < crc_count_max;
				     ++crc_count_index) {
					ret = find_byte_data_in_buf(image_buf, NEW_LINE_ASCII,
								    buf_index, image_size);
					if (ret < 0) {
						LOG_ERR("Find new line byte fail at CRC field, stop index: 0x%x, crc count index: 0x%x",
							buf_index, crc_count_index);
						return -1;
					}

					end_index = ret;
					column_count =
						mp2985_line_split(column_list, image_buf, buf_index,
								  end_index, ATE_COLUMN_MAX);
					if (column_count < (ATE_COLUMN_MAX - 1)) {
						LOG_ERR("Invalid CRC line, stop index: 0x%x",
							buf_index);
						return -1;
					}

					config->crc_code[crc_count_index] =
						strtol(column_list[5], NULL, 16);
					buf_index = (end_index + 1);
				}
			}
		}
	}

	return 0;
}

int program_mp2985(uint8_t bus, uint8_t addr, struct mp2985_config *config)
{
	CHECK_NULL_ARG_WITH_RETURN(config, -1);

	int ret = 0;
	int index = 0;
	int page3_start = 0;
	struct mp2985_data *data = NULL;

	// Program page 0~2
	for (index = 0; index < config->wr_cnt; ++index) {
		data = &config->pdata[index];
		if (data->page == PMBUS_PAGE_3) {
			page3_start = index;
			break;
		}

		ret = mp2985_write_data(bus, addr, data->reg_addr, data);
		if (ret != 0) {
			LOG_ERR("Write reg: 0x%x data fail, index: 0x%x", data->reg_addr, index);
			return ret;
		}
	}

	ret = mp2985_store_user_code(bus, addr);
	if (ret != 0) {
		LOG_ERR("Store user code fail");
		return ret;
	}

	// Wait store_user_code command finish
	k_msleep(1000);

	ret = mp2985_control_entering_page3(bus, addr, ENABLE_ENTER_PAGE);
	if (ret != 0) {
		LOG_ERR("Fail to enable entering page3 on program func");
		return ret;
	}

	for (index = page3_start; index < config->wr_cnt; ++index) {
		data = &config->pdata[index];
		if (data->page != PMBUS_PAGE_3) {
			break;
		}

		ret = mp2985_write_data(bus, addr, MP2985_PAGE_PLUS_WRITE, data);
		if (ret != 0) {
			LOG_ERR("Write page plus wirte data fail, index: 0x%x", index);
			return ret;
		}

		// Each command wait 20ms according to vender provided guide
		k_msleep(20);
	}

	ret = mp2985_control_entering_page3(bus, addr, DISABLE_ENTER_PAGE3);
	if (ret != 0) {
		LOG_ERR("Fail to disable entering page3 on program func");
		return ret;
	}

	// Wait 100 ms after lock MTP page according to vender provided guide
	k_msleep(100);

	ret = pmbus_set_page(bus, addr, PMBUS_PAGE_0);
	if (ret != 0) {
		LOG_ERR("Fail to set page to 0 after disable entering page3, bus: 0x%x, addr: 0x%x",
			bus, addr);
	}

	return 0;
}

bool mp2985_fwupdate(uint8_t bus, uint8_t addr, uint8_t *img_buff, uint32_t img_size)
{
	CHECK_NULL_ARG_WITH_RETURN(img_buff, false);

	int ret = 0;
	struct mp2985_config *config = (struct mp2985_config *)malloc(sizeof(struct mp2985_config));
	if (config == NULL) {
		LOG_ERR("Allocate config fail");
		return false;
	}

	memset(config, 0, sizeof(struct mp2985_config));

	ret = mp2985_parse_file(config, img_buff, img_size);
	if (ret != 0) {
		LOG_ERR("Parse file fail");
		goto error_exit;
	}

	ret = mp2985_check_mfr_config(bus, addr, config->product_id_exp, config->cfg_id);
	if (ret != 0) {
		LOG_ERR("Check MFR config fail, bus: 0x%x, addr: 0x%x, module id: 0x%x, cfg_id: 0x%x",
			bus, addr, config->product_id_exp, config->cfg_id);
		goto error_exit;
	}

	LOG_INF("MP2985 device bus: %d, addr: 0x%x", bus, addr);
	LOG_INF("* Image CRC: 0x%x%x, CFG id: 0x%x", config->crc_code[0], config->crc_code[1],
		config->cfg_id);

	ret = mp2985_unlock_password(bus, addr);
	if (ret != 0) {
		LOG_ERR("Fail to unlock password for firmware update");
		goto error_exit;
	}

	ret = mp2985_unlock_write_protection(bus, addr);
	if (ret != 0) {
		LOG_ERR("Fail to unlock write protection for firmware update");
		goto error_exit;
	}

	ret = mp2985_control_entering_page3(bus, addr, ENABLE_ENTER_PAGE);
	if (ret != 0) {
		LOG_ERR("Fail to enable entering page3 for firmware update");
		goto error_exit;
	}

	ret = mp2985_disable_store_fault_triggering(bus, addr);
	if (ret != 0) {
		LOG_ERR("Fail to disable store fault for firmware update");
		goto error_exit;
	}

	ret = program_mp2985(bus, addr, config);
	if (ret != 0) {
		LOG_ERR("Fail to program image");
		goto error_exit;
	}

	SAFE_FREE(config);
	return true;

error_exit:
	SAFE_FREE(config);
	return false;
}

int mp2985_set_power_regular_mode(uint8_t bus, uint8_t addr)
{
	int ret = 0;
	uint8_t retry = 3;
	uint8_t reg_val = 0;
	uint8_t tbuf[3] = { 0 };
	I2C_MSG msg = { 0 };

	ret = pmbus_set_page(bus, addr, PMBUS_PAGE_1);
	if (ret != 0) {
		LOG_ERR("Fail to set page to 1 for setting power regular mode, bus: 0x%x, addr: 0x%x, ret: %d",
			bus, addr, ret);
		return ret;
	}

	tbuf[0] = MP2985_MFR_VR_CONFIG2;
	msg = construct_i2c_message(bus, addr, 1, tbuf, 2);

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Fail to read vr config2 for setting power regular mode, bus: 0x%x, addr: 0x%x, ret: %d",
			bus, addr, ret);
		return ret;
	}

	reg_val = msg.data[0] & (~ENABLE_LOW_POWER_MODE_BIT); // Clear low-power mode bit
	reg_val = reg_val & (~ENABLE_PROGRAM_MODE_BIT); // Clear program mode bit

	tbuf[0] = MP2985_MFR_VR_CONFIG2;
	tbuf[1] = reg_val;
	tbuf[2] = msg.data[1];
	msg = construct_i2c_message(bus, addr, 3, tbuf, 0);

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Fail to set power regular mode, bus: 0x%x, addr: 0x%x, ret: %d", bus, addr,
			ret);
	}

	return ret;
}

bool mp2985_get_checksum(uint8_t bus, uint8_t addr, uint8_t *checksum)
{
	CHECK_NULL_ARG_WITH_RETURN(checksum, false);

	uint8_t retry = 3;
	I2C_MSG i2c_msg = { 0 };

	if (pmbus_set_page(bus, addr, PMBUS_PAGE_0)) {
		LOG_ERR("Fail to set page to 0, bus: 0x%x, addr: 0x%x", bus, addr);
		return false;
	}

	memset(&i2c_msg, 0, sizeof(I2C_MSG));
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 5;
	i2c_msg.data[0] = MP2985_GET_USER_DATA;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Fail to read checksum, bus: 0x%x, addr: 0x%x", bus, addr);
		return false;
	}

	checksum[0] = i2c_msg.data[4];
	checksum[1] = i2c_msg.data[3];
	checksum[2] = i2c_msg.data[2];
	checksum[3] = i2c_msg.data[1];

	return true;
}

uint8_t mp2985_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	mp2985_init_arg *init_arg = (mp2985_init_arg *)cfg->init_args;
	if (!init_arg->is_init) {
		LOG_ERR("device isn't initialized");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t i2c_max_retry = 5;
	float val = 0;
	sensor_val *sval = (sensor_val *)reading;
	I2C_MSG msg;
	memset(sval, 0, sizeof(sensor_val));

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = cfg->offset;

	if (i2c_master_read(&msg, i2c_max_retry)) {
		/* read fail */
		return SENSOR_FAIL_TO_ACCESS;
	}

	float pow2_n_iout = 0.0f;
	int y_iout = 0;
	switch (cfg->offset) {
	case PMBUS_READ_TEMPERATURE_1:
		/* TEMP[15:11] are reserved.
		 * TEMP[10:0] are two’s complement integer.
		 * BIC reports 0 degeee if the temperature is negative.
		 */
		if (GETBIT(msg.data[1], 2)) {
			val = 0;
		} else {
			val = ((msg.data[1] & 0x3) << 8) | msg.data[0];
		}
		break;
	case PMBUS_READ_VOUT:
		/* The VOUT mode can be set into offset 20h.(VOUT_MODE)
		 * VOUT mode is set as direct format and the resolution is 1mV/LSB in initialization stage.
		 * VOUT[15:12] are reserved.
		 * VOUT[11:0] are output voltage.
		 */
		val = ((msg.data[1] & 0x7) << 8) | msg.data[0];
		val *= 0.001;
		break;
	case PMBUS_READ_IOUT:
		/* The IOUT calculation format is linear-11.
		 * Refer to PMBus spec 7.3, the linear-11 formula is
		 * X = Y * (2 ^N)
		 * Where, as described above:
		 * X: real world value
		 * Y: 11-bit(IOUT[10:0]), two’s complement integer
		 * N: 5-bit(IOUT[15:11]), two’s complement integer
		 */
		if (GETBIT(msg.data[1], 7)) {
			pow2_n_iout = 1 / (float)(1 << ((~(msg.data[1] >> 3) + 1) & 0x1F));
		} else {
			pow2_n_iout = 1 / (float)(1 << (msg.data[1] >> 3));
		}

		if (GETBIT(msg.data[1], 2)) {
			y_iout = ~(((msg.data[1] & 0x7) << 8) | msg.data[0]) + 1;
		} else {
			y_iout = ((msg.data[1] & 0x7) << 8) | msg.data[0];
		}

		val = y_iout * pow2_n_iout;
		break;
	case PMBUS_READ_POUT:
		/* Output power = POUT[10:0] * 0.25W/LSB */
		val = ((msg.data[1] & 0x7) << 8) | msg.data[0];
		val *= 0.25;
		break;
	default:
		LOG_ERR("unsupported offset 0x%x reading", cfg->offset);
		break;
	}

	sval->integer = (int32_t)val;
	sval->fraction = (int32_t)(val * 1000) % 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t mp2985_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	mp2985_init_arg *init_args = (mp2985_init_arg *)cfg->init_args;
	if (init_args->is_init)
		goto skip_init;

	uint8_t retry = 5;
	I2C_MSG msg;
	memset(&msg, 0, sizeof(msg));
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;

	/* Set Page0 */
	msg.tx_len = 2;
	msg.data[0] = PMBUS_PAGE;
	msg.data[1] = 0x0;
	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("failed to set page0");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	/* Set VOUT mode as direct format, 1mV/LSB
	 * Offset 20h VOUT_MODE[7:0]:
	 * 8'h40: direct format, 1mV/LSB
	 * 8'h17: linear format, 1/512 V/LSB
	 * 8'h21: VID format
	 */
	msg.tx_len = 2;
	msg.data[0] = PMBUS_VOUT_MODE;
	msg.data[1] = 0x40;
	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("failed to vout mode");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	init_args->is_init = true;

skip_init:
	cfg->read = mp2985_read;
	return SENSOR_INIT_SUCCESS;
}
