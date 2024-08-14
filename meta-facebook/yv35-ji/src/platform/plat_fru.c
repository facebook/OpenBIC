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

#include "fru.h"
#include "plat_fru.h"
#include "plat_class.h"
#include <string.h>
#include <logging/log.h>
#include "libutil.h"
#include "eeprom.h"
#include "util_sys.h"

LOG_MODULE_REGISTER(plat_fru);

const EEPROM_CFG plat_fru_config[] = {
	{
		NV_ATMEL_24C128,
		MB_FRU_ID,
		MB_FRU_PORT,
		MB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
};

// BIOS version is stored in MB EEPROM, but the location of EEPROM is different from fru information
const EEPROM_CFG plat_bios_version_area_config = {
	NV_ATMEL_24C128,
	MB_FRU_ID,
	MB_FRU_PORT,
	MB_FRU_ADDR,
	FRU_DEV_ACCESS_BYTE,
	BIOS_FW_VERSION_START,
	BIOS_FW_VERSION_MAX_SIZE,
};

const EEPROM_CFG plat_vr_remain_cnt_area_config[] = {
	{
		NV_ATMEL_24C128,
		MB_FRU_ID,
		MB_FRU_PORT,
		MB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		VR_MPS_CPUDVDD_RM_CNT_START,
		VR_RM_CNT_MAX_SIZE,
	},
	{
		NV_ATMEL_24C128,
		MB_FRU_ID,
		MB_FRU_PORT,
		MB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		VR_MPS_CPUVDD_RM_CNT_START,
		VR_RM_CNT_MAX_SIZE,
	},
	{
		NV_ATMEL_24C128,
		MB_FRU_ID,
		MB_FRU_PORT,
		MB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		VR_MPS_SOCVDD_RM_CNT_START,
		VR_RM_CNT_MAX_SIZE,
	},
	{
		NV_ATMEL_24C128,
		MB_FRU_ID,
		MB_FRU_PORT,
		MB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		VR_MPS_FBVDDP2_RM_CNT_START,
		VR_RM_CNT_MAX_SIZE,
	},
	{
		NV_ATMEL_24C128,
		MB_FRU_ID,
		MB_FRU_PORT,
		MB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		VR_MPS_1V2_RM_CNT_START,
		VR_RM_CNT_MAX_SIZE,
	},
};

const EEPROM_CFG plat_rtc_clr_record_area_config = {
	NV_ATMEL_24C128,     MB_FRU_ID,		MB_FRU_PORT,	      MB_FRU_ADDR,
	FRU_DEV_ACCESS_BYTE, RTC_CLR_REC_START, RTC_CLR_REC_MAX_SIZE,
};

void pal_load_fru_config(void)
{
	memcpy(&fru_config, &plat_fru_config, sizeof(plat_fru_config));
}

int set_bios_version(EEPROM_ENTRY *entry, uint8_t block_index)
{
	CHECK_NULL_ARG_WITH_RETURN(entry, -1);

	if (block_index >= BIOS_FW_VERSION_BLOCK_NUM)
		return -1;

	bool ret = false;
	entry->config = plat_bios_version_area_config;
	if (block_index == 1)
		entry->config.start_offset += BIOS_FW_VERSION_SECOND_BLOCK_OFFSET;
	entry->data_len = BIOS_FW_VERSION_BLOCK_MAX_SIZE;

	ret = eeprom_write(entry);
	if (ret == false) {
		LOG_ERR("eeprom_write fail");
		return -1;
	}

	return 0;
}

int get_bios_version(EEPROM_ENTRY *entry, uint8_t block_index)
{
	CHECK_NULL_ARG_WITH_RETURN(entry, -1);

	if (block_index >= BIOS_FW_VERSION_BLOCK_NUM)
		return -1;

	bool ret = false;
	entry->config = plat_bios_version_area_config;
	if (block_index == 1)
		entry->config.start_offset += BIOS_FW_VERSION_SECOND_BLOCK_OFFSET;
	entry->data_len = BIOS_FW_VERSION_BLOCK_MAX_SIZE;

	ret = eeprom_read(entry);
	if (ret == false) {
		LOG_ERR("eeprom_read fail");
		return -1;
	}

	return 0;
}

bool access_vr_remain_cnt(EEPROM_ENTRY *entry, uint8_t comp_id, bool update_flag)
{
	CHECK_NULL_ARG_WITH_RETURN(entry, false);

	int idx = 0;

	switch (comp_id) {
	case JI_COMPNT_CPUDVDD:
	case JI_COMPNT_CPUVDD:
	case JI_COMPNT_SOCVDD:
		idx = comp_id - JI_COMPNT_CPUDVDD;
		break;

	case JI_COMPNT_FBVDDP2:
	case JI_COMPNT_1V2:
		idx = comp_id - JI_COMPNT_FBVDDP2 + 3;
		break;

	default:
		LOG_ERR("Unsupported component id");
		return false;
	}

	entry->config = plat_vr_remain_cnt_area_config[idx];
	entry->data_len = 2;

	/* check whether VR remain count has been write before */
	if (eeprom_read(entry) == false) {
		LOG_ERR("eeprom_read fail");
		return false;
	}

	uint16_t cur_rm_cnt = (entry->data[0] << 8) | entry->data[1];

	if (cur_rm_cnt == 0xFFFF) {
		LOG_WRN("VR comp_id %d remain count area first access, set %d to default.", comp_id,
			VR_MPS_MAX_RM_CNT);
		cur_rm_cnt = VR_MPS_MAX_RM_CNT;
		entry->data[0] = (cur_rm_cnt >> 8) & 0xFF;
		entry->data[1] = cur_rm_cnt & 0xFF;

		if (eeprom_write(entry) == false) {
			LOG_ERR("Failed to write default remain write count");
			return false;
		}
	}

	if (update_flag == false)
		return true;

	if (cur_rm_cnt == 0) {
		LOG_WRN("VR comp_id %d remain count area is 0, no need to update.", comp_id);
		return true;
	}

	cur_rm_cnt--;
	entry->data[0] = (cur_rm_cnt >> 8) & 0xFF;
	entry->data[1] = cur_rm_cnt & 0xFF;

	if (eeprom_write(entry) == false) {
		LOG_ERR("eeprom_write fail");
		return false;
	}

	LOG_INF("VR comp_id %d remain count has beem updated to %d", comp_id, cur_rm_cnt);

	return true;
}

bool access_rtc_clr_flag(uint8_t val)
{
	if (get_board_revision() >= SYS_BOARD_PVT2)
		return true;

	EEPROM_ENTRY entry = { 0 };
	entry.config = plat_rtc_clr_record_area_config;
	entry.data_len = 1;

	entry.data[0] = val;
	if (eeprom_write(&entry) == false) {
		LOG_ERR("eeprom_write fail");
		return false;
	}

	return true;
}

#define REG_RTC_STATUS 0x0B
void handle_rtc_clr_flag()
{
	if (get_board_revision() >= SYS_BOARD_PVT2)
		return;

	if (is_ac_lost() == false) {
		LOG_INF("Not AC on, skip RTC_CLR flag check");
		return;
	}

	EEPROM_ENTRY entry = { 0 };
	entry.config = plat_rtc_clr_record_area_config;
	entry.data_len = 1;

	/* check whether rtc_clr has been cleared by BIC */
	if (eeprom_read(&entry) == false) {
		LOG_ERR("eeprom_read fail");
		return;
	}

	/* While DC on, clear RTC_CLR status, if RTC_CLR triggered last time */
	if (entry.data[0] != 0x00) {
		uint8_t retry = 3;
		I2C_MSG msg = { 0 };

		msg.bus = I2C_BUS10;
		msg.target_addr = RTC_I2C_ADDR >> 1;
		msg.tx_len = 1;
		msg.rx_len = 1;
		msg.data[0] = REG_RTC_STATUS;

		if (i2c_master_read(&msg, retry)) {
			LOG_ERR("Failed to read rtc status");
			return;
		}

		/* Clear fake rtc_clr trigger */
		if (msg.data[0] & BIT(4)) {
			LOG_WRN("Clear RTC fake status");
			msg.tx_len = 2;
			msg.data[1] = msg.data[0] & ~BIT(4);
			msg.data[0] = REG_RTC_STATUS;
			if (i2c_master_write(&msg, retry)) {
				LOG_ERR("Failed to clear rtc status");
				return;
			}
		}
	} else {
		LOG_INF("RTC_CLR flag has been triggered by user");
	}
}
