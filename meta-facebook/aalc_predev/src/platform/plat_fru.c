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

#include <string.h>
#include "fru.h"
#include "plat_fru.h"
#include "plat_i2c.h"

#define AALC_FRU_PORT I2C_BUS10
#define AALC_FRU_ADDR (0xA6 >> 1)

#define AALC_FRU_START 0x3800 // start at 0x3800
#define AALC_FRU_SIZE 0x0400 // size 1KB

const EEPROM_CFG plat_fru_config[] = {
	{
		NV_ATMEL_24C128,
		AALC_FRU_ID,
		AALC_FRU_PORT,
		AALC_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
	},
};

void pal_load_fru_config(void)
{
	memcpy(&fru_config, &plat_fru_config, sizeof(plat_fru_config));
}
