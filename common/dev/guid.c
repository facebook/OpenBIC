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
#include <zephyr.h>
#include "guid.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(dev_guid);

// size of 1 to satisfy compiler warning
__weak const EEPROM_CFG guid_config[1] = {};

uint8_t GUID_read(EEPROM_ENTRY *entry)
{
	if (entry == NULL) {
		LOG_ERR("entry pointer is NULL");
		return GUID_FAIL_TO_ACCESS;
	}

	if (entry->config.dev_id >= ARRAY_SIZE(guid_config)) {
		LOG_ERR("GUID read device ID %x not exist", entry->config.dev_id);
		return GUID_INVALID_ID;
	}

	if ((entry->offset + entry->data_len) >=
	    (GUID_START + GUID_SIZE)) { // Check data write out of range
		LOG_ERR("GUID read out of range, type: %x, ID: %x", entry->config.dev_type,
			entry->config.dev_id);
		return GUID_OUT_OF_RANGE;
	}

	memcpy(&entry->config, &guid_config[entry->config.dev_id], sizeof(EEPROM_CFG));

	if (!eeprom_read(entry)) {
		LOG_ERR("Failed to read eeprom");
		return GUID_FAIL_TO_ACCESS;
	}

	return GUID_READ_SUCCESS;
}

uint8_t GUID_write(EEPROM_ENTRY *entry)
{
	if (entry == NULL) {
		LOG_ERR("entry pointer is NULL");
		return GUID_FAIL_TO_ACCESS;
	}

	if (entry->config.dev_id >= ARRAY_SIZE(guid_config)) {
		LOG_ERR("GUID write device ID %x not exist", entry->config.dev_id);
		return GUID_INVALID_ID;
	}

	if ((entry->offset + entry->data_len) >=
	    (GUID_START + GUID_SIZE)) { // Check data write out of range
		LOG_ERR("GUID write out of range, type: %x, ID: %x", entry->config.dev_type,
			entry->config.dev_id);
		return GUID_OUT_OF_RANGE;
	}

	memcpy(&entry->config, &guid_config[entry->config.dev_id], sizeof(EEPROM_CFG));

	if (!eeprom_write(entry)) {
		LOG_ERR("Failed to write eeprom");
		return GUID_FAIL_TO_ACCESS;
	}

	return GUID_WRITE_SUCCESS;
}

__weak uint8_t get_system_guid(uint16_t *data_len, uint8_t *data)
{
	return GUID_FAIL_TO_ACCESS;
}

__weak uint8_t set_system_guid(const uint16_t *data_len, uint8_t *data)
{
	return GUID_FAIL_TO_ACCESS;
}
