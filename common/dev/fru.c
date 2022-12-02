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
#include <logging/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

LOG_MODULE_REGISTER(dev_fru);

EEPROM_CFG fru_config[FRU_CFG_NUM];

static uint8_t find_FRU_ID(uint8_t FRUID)
{
	uint8_t ret;

	for (ret = 0; ret < MAX_FRU_ID; ret++) {
		if (FRUID == fru_config[ret].dev_id) {
			break;
		}
	}

	if (ret == MAX_FRU_ID) { // FRU ID not found
		return MAX_FRU_ID;
	}

	return ret;
}

uint8_t get_FRU_access(uint8_t FRUID)
{
	uint8_t ID_No;

	ID_No = find_FRU_ID(FRUID);

	if (ID_No == MAX_FRU_ID) { // FRU ID not found
		return 0xFF;
	}

	return fru_config[ID_No].access;
}

uint16_t find_FRU_size(uint8_t FRUID)
{
	uint8_t ID_No;

	ID_No = find_FRU_ID(FRUID);

	if (ID_No == MAX_FRU_ID) { // FRU ID not found
		return 0xFFFF;
	}

	return fru_config[ID_No].max_size;
}

uint8_t FRU_read(EEPROM_ENTRY *entry)
{
	if (entry == NULL) {
		return FRU_FAIL_TO_ACCESS;
	}

	if (entry->config.dev_id >= MAX_FRU_ID) { // check if FRU is defined
		LOG_ERR("fru read device ID %x doesn't exist\n", entry->config.dev_id);
		return FRU_INVALID_ID;
	}

	if ((entry->offset + entry->data_len) >=
	    (FRU_START + FRU_SIZE)) { // Check data write out of range
		LOG_ERR("fru read out of range, type: %x, ID: %x\n", entry->config.dev_type,
			entry->config.dev_id);
		return FRU_OUT_OF_RANGE;
	}

	memcpy(&entry->config, &fru_config[entry->config.dev_id],
	       sizeof(fru_config[entry->config.dev_id]));

	if (!eeprom_read(entry)) {
		return FRU_FAIL_TO_ACCESS;
	}

	return FRU_READ_SUCCESS;
}

uint8_t FRU_write(EEPROM_ENTRY *entry)
{
	if (entry == NULL) {
		return FRU_FAIL_TO_ACCESS;
	}

	if (entry->config.dev_id >= MAX_FRU_ID) { // check if FRU is defined
		LOG_ERR("fru write device ID %x doesn't exist\n", entry->config.dev_id);
		return FRU_INVALID_ID;
	}

	if ((entry->offset + entry->data_len) >=
	    (FRU_START + FRU_SIZE)) { // Check data write out of range
		LOG_ERR("fru write out of range, type: %x, ID: %x\n", entry->config.dev_type,
			entry->config.dev_id);
		return FRU_OUT_OF_RANGE;
	}

	memcpy(&entry->config, &fru_config[entry->config.dev_id],
	       sizeof(fru_config[entry->config.dev_id]));

	if (!eeprom_write(entry)) {
		return FRU_FAIL_TO_ACCESS;
	}

	return FRU_WRITE_SUCCESS;
}

__weak void pal_load_fru_config(void)
{
	return;
}

void FRU_init(void)
{
	pal_load_fru_config();
}

__weak bool write_psb_inform(EEPROM_ENTRY *entry)
{
	return false;
}
