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
#include "fru.h"
#include "plat_fru.h"
#include <logging/log.h>
#include "eeprom.h"
#include "libutil.h"

LOG_MODULE_REGISTER(plat_fru);

#define RF_FRU_PORT 0x02
#define RF_FRU_ADDR (0xA8 >> 1)

const EEPROM_CFG plat_fru_config[] = {
	{
		NV_ATMEL_24C128,
		RF_FRU_ID,
		RF_FRU_PORT,
		RF_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
};

const EEPROM_CFG plat_cxl_version_area_config = {
	NV_ATMEL_24C128,
	RF_FRU_ID,
	RF_FRU_PORT,
	RF_FRU_ADDR,
	FRU_DEV_ACCESS_BYTE,
	CXL_FW_VERSION_START,
	CXL_FW_VERSION_MAX_SIZE,
};

bool set_cxl_version(EEPROM_ENTRY *entry)
{
	CHECK_NULL_ARG_WITH_RETURN(entry, false);

	bool ret = false;
	entry->config = plat_cxl_version_area_config;
	entry->data_len = CXL_FW_VERSION_MAX_SIZE;

	ret = eeprom_write(entry);
	if (ret == false) {
		printf("eeprom_write fail");
		return ret;
	}

	return ret;
}

bool get_cxl_version(EEPROM_ENTRY *entry)
{
	CHECK_NULL_ARG_WITH_RETURN(entry, false);

	bool ret = false;
	entry->config = plat_cxl_version_area_config;
	entry->data_len = CXL_FW_VERSION_MAX_SIZE;

	ret = eeprom_read(entry);
	if (ret == false) {
		printf("eeprom_read fail");
		return ret;
	}

	return ret;
}

void pal_load_fru_config(void)
{
	memcpy(&fru_config, &plat_fru_config, sizeof(plat_fru_config));
}
