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
#include <sys/util.h>
#include <string.h>
#include <stdio.h>

#include "fru.h"
#include "plat_fru.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_fru);

const EEPROM_CFG plat_fru_config[] = {
	{
		NV_ATMEL_24C128,
		MB_FRU_ID,
		MB_FRU_PORT,
		MB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
};

void pal_load_fru_config(void)
{
	if (ARRAY_SIZE(plat_fru_config) > FRU_CFG_NUM) {
		LOG_ERR("Failed to load fru configuration: index is over than max value");
		goto out;
	}

	memcpy(&fru_config, &plat_fru_config, sizeof(plat_fru_config));
out:
	return;
}
