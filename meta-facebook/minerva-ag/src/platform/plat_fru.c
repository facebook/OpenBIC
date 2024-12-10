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
#include "plat_i2c.h"
#include <libutil.h>
#include <stdlib.h>
#include <logging/log.h>
#include "fru.h"
#include "plat_fru.h"

LOG_MODULE_REGISTER(plat_fru);

#define AEGIS_FRU_START 0x0000
#define AEGIS_FRU_SIZE 0x0400 // size 1KB

const EEPROM_CFG plat_fru_config[] = {
	{
		ROHM_BR24G512,
		LOG_EEPROM_ID,
		I2C_BUS12,
		LOG_EEPROM_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
};

void pal_load_fru_config(void)
{
	memcpy(&fru_config, &plat_fru_config, sizeof(plat_fru_config));
}

bool plat_eeprom_write(uint32_t offset, uint8_t *data, uint16_t data_len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	LOG_DBG("plat_eeprom_write, offset: 0x%x, data_len: %d", offset, data_len);

	EEPROM_ENTRY entry;

	entry.offset = offset;
	entry.data_len = data_len;

	uint8_t fru_index = 0;
	if (!find_FRU_ID(LOG_EEPROM_ID, &fru_index)) {
		LOG_ERR("find_FRU_ID fail when write eeprom 0x%x ", offset);
		return false;
	}

	memcpy(entry.data, data, data_len);
	memcpy(&entry.config, &fru_config[fru_index], sizeof(fru_config[fru_index]));

	if (!eeprom_write(&entry)) {
		LOG_ERR("write eeprom 0x%x fail", offset);
		return false;
	}

	return true;
}

bool plat_eeprom_read(uint32_t offset, uint8_t *data, uint16_t data_len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);
	EEPROM_ENTRY entry;

	entry.offset = offset;
	entry.data_len = data_len;

	uint8_t fru_index = 0;
	if (!find_FRU_ID(LOG_EEPROM_ID, &fru_index)) {
		LOG_ERR("find_FRU_ID fail when read eeprom 0x%x ", offset);
		return false;
	}

	memcpy(&entry.config, &fru_config[fru_index], sizeof(fru_config[fru_index]));
	memset(entry.data, 0xFF, sizeof(entry.data));

	if (!eeprom_read(&entry)) {
		LOG_ERR("read eeprom 0x%x fail", offset);
		return false;
	}

	memcpy(data, entry.data, data_len);

	return true;
}