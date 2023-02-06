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
#include "libutil.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(dev_fru);

EEPROM_CFG fru_config[FRU_CFG_NUM];

static bool find_FRU_ID(uint8_t FRUID, uint8_t *fru_id)
{
	CHECK_NULL_ARG_WITH_RETURN(fru_id, false);

	uint8_t index = 0;
	for (index = 0; index < FRU_CFG_NUM; index++) {
		if (FRUID == fru_config[index].dev_id) {
			*fru_id = index;
			break;
		}
	}

	if (index == FRU_CFG_NUM) { // FRU ID not found
		return false;
	}

	return true;
}

uint8_t get_FRU_access(uint8_t FRUID)
{
	bool ret = false;
	uint8_t ID_No = 0;

	ret = find_FRU_ID(FRUID, &ID_No);

	if (ret == false) { // FRU ID not found
		return FRU_ID_NOT_FOUND;
	}

	return fru_config[ID_No].access;
}

uint16_t find_FRU_size(uint8_t FRUID)
{
	bool ret = false;
	uint8_t ID_No = 0;

	ret = find_FRU_ID(FRUID, &ID_No);

	if (ret == false) { // FRU ID not found
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
		LOG_ERR("FRU read device ID %x doesn't exist", entry->config.dev_id);
		return FRU_INVALID_ID;
	}

	if ((entry->offset + entry->data_len) >=
	    (FRU_START + FRU_SIZE)) { // Check data write out of range
		LOG_ERR("FRU read out of range, type: %x, ID: %x", entry->config.dev_type,
			entry->config.dev_id);
		return FRU_OUT_OF_RANGE;
	}

	uint8_t fru_index = 0;
	bool ret = find_FRU_ID(entry->config.dev_id, &fru_index);
	if (ret == false) {
		LOG_ERR("find FRU read config fail via FRU id: 0x%x", entry->config.dev_id);
		return FRU_INVALID_ID;
	}

	memcpy(&entry->config, &fru_config[fru_index], sizeof(fru_config[fru_index]));

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
		LOG_ERR("FRU write device ID %x doesn't exist", entry->config.dev_id);
		return FRU_INVALID_ID;
	}

	if ((entry->offset + entry->data_len) >=
	    (FRU_START + FRU_SIZE)) { // Check data write out of range
		LOG_ERR("FRU write out of range, type: %x, ID: %x", entry->config.dev_type,
			entry->config.dev_id);
		return FRU_OUT_OF_RANGE;
	}

	uint8_t fru_index = 0;
	bool ret = find_FRU_ID(entry->config.dev_id, &fru_index);
	if (ret == false) {
		LOG_ERR("find fru write config fail via fru id: 0x%x", entry->config.dev_id);
		return FRU_INVALID_ID;
	}

	memcpy(&entry->config, &fru_config[fru_index], sizeof(fru_config[fru_index]));

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
