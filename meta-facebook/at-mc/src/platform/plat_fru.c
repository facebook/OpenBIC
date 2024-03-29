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

#include "plat_fru.h"

#include <stdio.h>
#include <string.h>
#include "fru.h"
#include "i2c-mux-pca954x.h"
#include "plat_class.h"
#include "plat_ipmi.h"

const EEPROM_CFG plat_fru_config[] = {
	{
		PUYA_P24C128F,
		MC_FRU_ID,
		MC_FRU_PORT,
		MC_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	{
		ST_M24128_BW,
		CXL_FRU_ID1,
		CXL_FRU_PORT,
		CXL_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
		true,
		CXL_FRU_MUX1_ADDR,
		CXL_FRU_MUX1_CHANNEL,
	},
	{
		ST_M24128_BW,
		CXL_FRU_ID2,
		CXL_FRU_PORT,
		CXL_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
		true,
		CXL_FRU_MUX1_ADDR,
		CXL_FRU_MUX1_CHANNEL,
	},
	{
		ST_M24128_BW,
		CXL_FRU_ID3,
		CXL_FRU_PORT,
		CXL_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
		true,
		CXL_FRU_MUX1_ADDR,
		CXL_FRU_MUX1_CHANNEL,
	},
	{
		ST_M24128_BW,
		CXL_FRU_ID4,
		CXL_FRU_PORT,
		CXL_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
		true,
		CXL_FRU_MUX1_ADDR,
		CXL_FRU_MUX1_CHANNEL,
	},
	{
		ST_M24128_BW,
		CXL_FRU_ID5,
		CXL_FRU_PORT,
		CXL_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
		true,
		CXL_FRU_MUX1_ADDR,
		CXL_FRU_MUX1_CHANNEL,
	},
	{
		ST_M24128_BW,
		CXL_FRU_ID6,
		CXL_FRU_PORT,
		CXL_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
		true,
		CXL_FRU_MUX1_ADDR,
		CXL_FRU_MUX1_CHANNEL,
	},
	{
		ST_M24128_BW,
		CXL_FRU_ID7,
		CXL_FRU_PORT,
		CXL_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
		true,
		CXL_FRU_MUX1_ADDR,
		CXL_FRU_MUX1_CHANNEL,
	},
	{
		ST_M24128_BW,
		CXL_FRU_ID8,
		CXL_FRU_PORT,
		CXL_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
		true,
		CXL_FRU_MUX1_ADDR,
		CXL_FRU_MUX1_CHANNEL,
	},
	{
		PUYA_P24C128F,
		SYS_DEBUG_ID,
		MC_FRU_PORT,
		MC_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		BIC_CONFIG_START,
		BIC_CONFIG_SIZE,
	},
};

void pal_load_fru_config(void)
{
	memcpy(&fru_config, &plat_fru_config, sizeof(plat_fru_config));
}

uint8_t pal_cxl_map_mux0_channel(uint8_t cxl_fru_id)
{
	uint8_t channel = 0;

	uint8_t pcie_card_id = cxl_fru_id - PCIE_CARD_ID_OFFSET;

	switch (pcie_card_id) {
	case CARD_1_INDEX:
		channel = PCA9548A_CHANNEL_0;
		break;
	case CARD_2_INDEX:
		channel = PCA9548A_CHANNEL_1;
		break;
	case CARD_3_INDEX:
		channel = PCA9548A_CHANNEL_2;
		break;
	case CARD_4_INDEX:
		channel = PCA9548A_CHANNEL_3;
		break;
	case CARD_9_INDEX:
		channel = PCA9548A_CHANNEL_4;
		break;
	case CARD_10_INDEX:
		channel = PCA9548A_CHANNEL_5;
		break;
	case CARD_11_INDEX:
		channel = PCA9548A_CHANNEL_6;
		break;
	case CARD_12_INDEX:
		channel = PCA9548A_CHANNEL_7;
		break;
	default:
		channel = PCA9548A_CHANNEL_DISABLE;
		break;
	}

	return channel;
}
