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

/*
  NAME: I2C TARGET INIT
  FILE: plat_i2c_target.c
  DESCRIPTION: Provide i2c target EN/CFG table "I2C_TARGET_EN_TABLE[]/I2C_TARGET_CFG_TABLE[]" for init target config.
  AUTHOR: MouchenHung
  DATE/VERSION: 2021.11.26 - v1.1
  Note: 
    (1) "plat_i2c_target.h" is included by "hal_i2c_target.h"
*/

#include <zephyr.h>
#include <stdio.h>
#include <stdlib.h>
#include <logging/log.h>
#include "plat_i2c_target.h"
#include "plat_mctp.h"
#include "plat_class.h"
#include <drivers/flash.h>

LOG_MODULE_REGISTER(plat_i2c_target);

static bool command_reply_data_handle(void *arg);
static void command_set_slot_handle(void *arg);
void set_slot_handle(struct k_work *work);
void plat_set_slot_init(int slot_id);
K_WORK_DELAYABLE_DEFINE(set_slot_work, set_slot_handle);

/* I2C target init-enable table */
const bool I2C_TARGET_ENABLE_TABLE[MAX_TARGET_NUM] = {
	TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE,
	TARGET_DISABLE, TARGET_ENABLE,	TARGET_DISABLE, TARGET_DISABLE,
	TARGET_DISABLE, TARGET_DISABLE, TARGET_ENABLE,	TARGET_DISABLE,
};

/* I2C target init-config table */
const struct _i2c_target_config I2C_TARGET_CONFIG_TABLE[MAX_TARGET_NUM] = {
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0x40, 0xA },
	{ 0xFF, 0xA },
	{ 0x42, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0x42, 0xA, command_reply_data_handle, command_set_slot_handle },
	{ 0xFF, 0xA },
};

static bool command_reply_data_handle(void *arg)
{
	struct i2c_target_data *data = (struct i2c_target_data *)arg;

	if (data->wr_buffer_idx < 1) {
		LOG_ERR("No register offset received before read");
		data->target_rd_msg.msg_length = 1;
		data->target_rd_msg.msg[0] = 0xFF;
		return false;
	}

	uint8_t reg_offset = data->target_wr_msg.msg[0];

	switch (reg_offset) {
	case GET_MMC_INFO_REG: {
		uint8_t slot = get_slot_id();
		if (slot == 0xFF) {
			data->target_rd_msg.msg_length = 1;
			data->target_rd_msg.msg[0] = 0xFF;
			return false;
		}

		uint8_t eid = plat_get_eid();

		data->target_rd_msg.msg_length = 2;
		data->target_rd_msg.msg[0] = slot;
		data->target_rd_msg.msg[1] = eid;

		LOG_DBG("Reply SLOT=%d, EID=%d", slot, eid);
		break;
	}

	default:
		LOG_WRN("Unsupported read register: 0x%02x", reg_offset);
		data->target_rd_msg.msg_length = 1;
		data->target_rd_msg.msg[0] = 0xFF;
		break;
	}

	return true;
}

static void command_set_slot_handle(void *arg)
{
	struct i2c_target_data *data = (struct i2c_target_data *)arg;
	data->skip_msg_wr = true;

	if (data->wr_buffer_idx < 1) {
		LOG_ERR("Received data length: 0x%02x", data->wr_buffer_idx);
		data->target_rd_msg.msg_length = 1;
		data->target_rd_msg.msg[0] = 0xFF;
		return;
	}

	if (data->wr_buffer_idx == 1) {
		uint8_t reg_offset = data->target_wr_msg.msg[0];
		const mmc_info_t *cfg = &mmc_info_table[0];
		LOG_INF("Received reg_offset: 0x%02x", reg_offset);

		switch (reg_offset) {
		case SLOT_0_I2C_SET_SLOT_REG:
			cfg = &mmc_info_table[0];
			plat_set_slot_init(cfg->slot);
			break;
		case SLOT_1_I2C_SET_SLOT_REG:
			cfg = &mmc_info_table[1];
			plat_set_slot_init(cfg->slot);
			break;
		case SLOT_2_I2C_SET_SLOT_REG:
			cfg = &mmc_info_table[2];
			plat_set_slot_init(cfg->slot);
			break;
		case SLOT_3_I2C_SET_SLOT_REG:
			cfg = &mmc_info_table[3];
			plat_set_slot_init(cfg->slot);
			break;
		default:
			LOG_ERR("Unknown reg offset: 0x%02x", reg_offset);
			data->target_rd_msg.msg_length = 1;
			data->target_rd_msg.msg[0] = 0xFF;
			break;
		}
	} else {
		LOG_ERR("Unexpected data length: %d", data->wr_buffer_idx);
		data->target_rd_msg.msg_length = 1;
		data->target_rd_msg.msg[0] = 0xFF;
	}

	return;
}

void set_slot_handle(struct k_work *work)
{
	mmc_work_info_t *info = CONTAINER_OF(work, mmc_work_info_t, set_slot_work);
	uint8_t slot = info->slot;
	uint8_t eid = mmc_info_table[slot].eid;
	uint32_t op_addr = FLASH_SLOT_ADDRESS;
	uint32_t erase_sz = FLASH_SECTOR;
	uint8_t write_buf = slot;
	uint8_t read_back_buf = 0xFF;
	uint32_t ret = 0;

	if (slot >= MAX_SLOT) {
		LOG_ERR("Invalid slot_id: %d", slot);
		free(info);
		return;
	}

	const struct device *flash_dev = device_get_binding("spi_spim0_cs0");
	if (!flash_dev) {
		LOG_ERR("Failed to get flash device.");
		free(info);
		return;
	}

	ret = flash_read(flash_dev, op_addr, &read_back_buf, 1);
	if (ret != 0) {
		LOG_ERR("Failed to read %u.\n", op_addr);
		free(info);
		return;
	}

	LOG_DBG("SLOT read from flash: %d", read_back_buf);

	if (read_back_buf != write_buf) {
		ret = flash_erase(flash_dev, op_addr, erase_sz);
		if (ret != 0) {
			LOG_ERR("Failed to erase %u.\n", op_addr);
			free(info);
			return;
		}

		ret = flash_write(flash_dev, op_addr, &write_buf, 1);
		if (ret != 0) {
			LOG_ERR("Failed to write %u.\n", op_addr);
			free(info);
			return;
		}

		ret = flash_read(flash_dev, op_addr, &read_back_buf, 1);
		if (ret != 0) {
			LOG_ERR("Failed to read %u.\n", op_addr);
			free(info);
			return;
		}

		if (memcmp(&write_buf, &read_back_buf, 1) != 0) {
			LOG_ERR("Flash verification failed at 0x%x.", op_addr);
			LOG_ERR("Expected: %d, Got: %d", write_buf, read_back_buf);
		}
	} else {
		LOG_DBG("Flash content already matches. No write needed.");
	}

	LOG_INF("Setting EID %d for slot %d", eid, slot);
	plat_set_eid(eid);
	LOG_DBG("EID after set: %d", plat_get_eid());

	free(info);
	return;
}

void plat_set_slot_init(int slot_id)
{
	mmc_work_info_t *info = malloc(sizeof(mmc_work_info_t));
	if (!info) {
		LOG_ERR("Failed to allocate memory for mmc_info");
		return;
	}

	info->slot = slot_id;
	k_work_init(&info->set_slot_work, set_slot_handle);
	k_work_submit(&info->set_slot_work);
}
