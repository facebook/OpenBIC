/*
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef HAL_EEPROM_H
#define HAL_EEPROM_H


#define EEPROM_WRITE_SIZE 0x20

// define offset, size and order for EEPROM write/read
#define FRU_START 0x0000  // start at 0x000
#define FRU_SIZE  0x0400  // size 1KB

#define SEL_START 0x0400
#define SEL_SIZE  0x0800
// next start should be 0x0C00

typedef struct _EEPROM_CFG_{
  uint8_t dev_type;
  uint8_t dev_id;
  uint8_t port;
  uint8_t slave_addr;
  uint8_t access;
  uint16_t start_offset;
  uint16_t max_size;
} EEPROM_CFG;

typedef struct _EEPROM_ENTRY_{
  EEPROM_CFG config;
  uint16_t offset;
  uint16_t data_len;
  uint8_t data[EEPROM_WRITE_SIZE];
} EEPROM_ENTRY;

uint8_t eeprom_write(EEPROM_ENTRY *entry);
uint8_t eeprom_read(EEPROM_ENTRY *entry);

#endif 
