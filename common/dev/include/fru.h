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

#ifndef FRU_H
#define FRU_H

#include "eeprom.h"
#define FRU_CFG_NUM 5

enum {
	NV_ATMEL_24C02,
	NV_ATMEL_24C64,
	NV_ATMEL_24C128,
};

enum {
	FRU_WRITE_SUCCESS,
	FRU_READ_SUCCESS,
	FRU_INVALID_ID,
	FRU_OUT_OF_RANGE,
	FRU_FAIL_TO_ACCESS,
};

enum {
	FRU_DEV_ACCESS_BYTE,
	FRU_DEV_ACCESS_WORD,
};

extern EEPROM_CFG fru_config[];

uint8_t get_FRU_access(uint8_t FRUID);
uint16_t find_FRU_size(uint8_t FRUID);
uint8_t FRU_read(EEPROM_ENTRY *entry);
uint8_t FRU_write(EEPROM_ENTRY *entry);
void pal_load_fru_config(void);
void FRU_init(void);
bool write_psb_inform(EEPROM_ENTRY *entry);

#endif
