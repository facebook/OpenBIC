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
#include "guid.h"
#include "plat_guid.h"
#include "fru.h"
#include <string.h>

#define MB_GUID_PORT 0x0A
#define MB_GUID_ADDR 0x50

const EEPROM_CFG guid_config[] = {
	{
		NV_ATMEL_24C128,
		MB_SYS_GUID_ID,
		MB_GUID_PORT,
		MB_GUID_ADDR,
		GUID_ACCESS_BYTE,
		GUID_START,
		GUID_SIZE,
	},
};

uint8_t get_system_guid(uint16_t *data_len, uint8_t *data)
{
	EEPROM_ENTRY guid_entry;

	guid_entry.offset = 0;
	guid_entry.data_len = 16;
	guid_entry.config.dev_id = MB_SYS_GUID_ID;

	uint8_t status = GUID_read(&guid_entry);
	if (status == GUID_READ_SUCCESS) {
		*data_len = guid_entry.data_len;
		memcpy(data, &guid_entry.data, guid_entry.data_len);
	}

	return status;
}

uint8_t set_system_guid(const uint16_t *data_len, uint8_t *data)
{
	EEPROM_ENTRY guid_entry;

	guid_entry.offset = 0;
	guid_entry.data_len = *data_len;
	guid_entry.config.dev_id = MB_SYS_GUID_ID;
	memcpy(&guid_entry.data[0], data, guid_entry.data_len);
	return GUID_write(&guid_entry);
}
