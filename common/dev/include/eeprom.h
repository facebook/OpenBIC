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

#ifndef EEPROM_H
#define EEPROM_H

#include <stdbool.h>
#include <stdint.h>
#include "plat_def.h"

#ifndef EEPROM_WRITE_SIZE
#define EEPROM_WRITE_SIZE 0x20
#endif

// define offset, size and order for EEPROM write/read
#define FRU_START 0x0000 // start at 0x000
#define FRU_SIZE 0x0400 // size 1KB

#define SEL_START 0x0400
#define SEL_SIZE 0x0400

#define GUID_START 0x0800
#define GUID_SIZE 0x0100

#define BIC_CONFIG_START 0x0900
#define BIC_CONFIG_SIZE 0x0100
// next start should be 0x0A00

typedef struct _EEPROM_CFG_ {
	uint8_t dev_type;
	uint8_t dev_id;
	uint8_t port;
	uint8_t target_addr;
	uint8_t access;
	uint16_t start_offset;
	uint16_t max_size;
	bool mux_present;
	uint8_t mux_addr;
	uint8_t mux_channel;
	struct k_mutex *bus_mutex;
} EEPROM_CFG;

typedef struct _EEPROM_ENTRY_ {
	EEPROM_CFG config;
	uint16_t offset;
	uint16_t data_len;
	uint8_t data[EEPROM_WRITE_SIZE];
} EEPROM_ENTRY;

bool eeprom_write(EEPROM_ENTRY *entry);
bool eeprom_read(EEPROM_ENTRY *entry);

#endif
