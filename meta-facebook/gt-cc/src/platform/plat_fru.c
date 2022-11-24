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
#include <string.h>

extern struct k_mutex i2c_bus6_mutex;

const EEPROM_CFG plat_fru_config[] = {
	{
		NV_ATMEL_24C64,
		SWB_FRU_ID,
		SWB_FRU_PORT,
		SWB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
		/* Because there has mux before eeprom, so mux_present is true to access mux first */
		true,
		SWB_FRU_MUX_ADDR,
		SWB_FRU_MUX_CHAN,
		&i2c_bus6_mutex,
	},
	{
		NV_ATMEL_24C64,
		FIO_FRU_ID,
		FIO_FRU_PORT,
		FIO_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	{
		NV_ATMEL_24C64,
		HSC_MODULE_FRU_ID,
		HSC_MODULE_FRU_PORT,
		HSC_MODULE_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
		/* Because there has mux before eeprom, so mux_present is true to access mux first */
		true,
		HSC_MODULE_FRU_MUX_ADDR,
		HSC_MODULE_FRU_MUX_CHAN,
		&i2c_bus6_mutex,
	},
};

void pal_load_fru_config(void)
{
	memcpy(&fru_config, &plat_fru_config, sizeof(plat_fru_config));
}
