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

#ifndef M88RT51632_H
#define M88RT51632_H

#include "hal_i2c.h"

#define I2C_READ_CCODE_START 0x82
#define I2C_READ_CCODE_END 0x81
#define I2C_READ_BYTCNT 0x2
#define I2C_WRITE_CCODE_START 0x87
#define I2C_WRITE_BYTCNT 0x6

#define RETIMER_VERSION 0x0104c
#define RETIMER_ADDRPORT 0xfff0
#define RETIMER_DATAPORT 0xfff4

#define M88RT51632_TEMP_OFFSET 0x00
#define M88RT51632_EEPROM_BASE_OFFSET 0x800000

#define MAX_SENSORS 2

#define IMAGE_PACKAGE_SIZE 0x40

#define M88RT51632_MUTEX_LOCK_MS 1000

#define MAX_RETRY 3

bool m88rt51632_get_vendor_id(I2C_MSG *msg);
bool m88rt51632_get_fw_version(I2C_MSG *msg, uint32_t *version);
uint8_t m88rt51632_fw_update(I2C_MSG *msg, uint32_t offset, uint16_t msg_len, uint8_t *msg_buf,
			     uint8_t flag);

#endif
