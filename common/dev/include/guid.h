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

#ifndef GUID_H
#define GUID_H

#include "eeprom.h"

enum {
	GUID_WRITE_SUCCESS,
	GUID_READ_SUCCESS,
	GUID_INVALID_ID,
	GUID_OUT_OF_RANGE,
	GUID_FAIL_TO_ACCESS,
};

enum {
	GUID_ACCESS_BYTE,
	GUID_ACCESS_WORD,
};

uint8_t GUID_read(EEPROM_ENTRY *entry);
uint8_t GUID_write(EEPROM_ENTRY *entry);

uint8_t get_system_guid(uint16_t *data_len, uint8_t *data);
uint8_t set_system_guid(uint16_t *data_len, uint8_t *data);

#endif
