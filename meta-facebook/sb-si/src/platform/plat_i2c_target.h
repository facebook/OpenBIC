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

#ifndef PLAT_I2C_SLAVE_H
#define PLAT_I2C_SLAVE_H

#include <drivers/i2c.h>
#include "hal_i2c_target.h"

#define MAX_SLOT 4

#define FLASH_EID_ADDRESS 0x0FF000
#define FLASH_SECTOR 0x1000

static const uint8_t eid_table[MAX_SLOT] = {
	10, // SLOT 0
	20, // SLOT 1
	30, // SLOT 2
	40 // SLOT 3
};

struct mmc_info {
	struct k_work set_eid_work;
	int slot;
};

#define SLOT_0_I2C_SET_EID_REG 0x40
#define SLOT_1_I2C_SET_EID_REG 0x41
#define SLOT_2_I2C_SET_EID_REG 0x42
#define SLOT_3_I2C_SET_EID_REG 0x43

#define TARGET_ENABLE 1
#define TARGET_DISABLE 0

#endif
