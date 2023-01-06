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
#include <logging/log.h>

#include "fru.h"
#include "plat_fru.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_fru);

const EEPROM_CFG plat_OPA_fru_config[] = {
	{
		NV_ATMEL_24C128,
		OP_FRU_ID,
		OP_FRU_PORT,
		OPA_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
};

const EEPROM_CFG plat_OPB_fru_config[] = {
	{
		NV_ATMEL_24C128,
		OP_FRU_ID,
		OP_FRU_PORT,
		OPB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
};

void fru_config_oversize_handle(const EEPROM_CFG *plat_fru_config, uint8_t card_type)
{
	LOG_WRN("Failed to load OP%s's fru configuration. Index: %d is over than max value: %d",
		((card_type == 0) ? "A" : "B"), sizeof(plat_fru_config), FRU_CFG_NUM);
	memcpy(&fru_config, &plat_fru_config, FRU_CFG_NUM);
}

void pal_load_fru_config(void)
{
	uint8_t card_type = get_card_type();

	if (card_type == CARD_TYPE_OPA) {
		if (ARRAY_SIZE(plat_OPA_fru_config) > FRU_CFG_NUM) {
			fru_config_oversize_handle(plat_OPA_fru_config, CARD_TYPE_OPA);
			goto out;
		}
		memcpy(&fru_config, &plat_OPA_fru_config, sizeof(plat_OPA_fru_config));
	}

	else if (card_type == CARD_TYPE_OPB) {
		if (ARRAY_SIZE(plat_OPB_fru_config) > FRU_CFG_NUM) {
			fru_config_oversize_handle(plat_OPB_fru_config, CARD_TYPE_OPB);
			goto out;
		}
		memcpy(&fru_config, &plat_OPB_fru_config, sizeof(plat_OPB_fru_config));
	} else {
		LOG_ERR("Unsupported card type, Card type: 0x%x", card_type);
	}
out:
	return;
}
