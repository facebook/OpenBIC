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

#include <string.h>
#include <logging/log.h>
#include "fru.h"
#include "plat_fru.h"
#include "libutil.h"

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
	{
		NV_ATMEL_24C128,
		DPV2_FRU_ID,
		DPV2_FRU_PORT,
		DPV2_FRU_ADDR,
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

void pal_load_fru_config(void)
{
	memcpy(fru_config, plat_fru_config, sizeof(plat_fru_config));
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
