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
#include <logging/log.h>
#include "fru.h"
#include "libutil.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_fru);

const EEPROM_CFG plat_fru_config[] = {
	// ACB BIC fru
	{
		PUYA_P24C128F,
		CB_FRU_ID,
		CB_FRU_PORT,
		CB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACB FIO fru
	{
		ST_M24C64_W,
		FIO_FRU_ID,
		FIO_FRU_PORT,
		FIO_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_1 fru
	{
		PUYA_P24C128F,
		ACCL_1_FRU_ID,
		ACCL_1_6_FRU_PORT,
		ACCL_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_2 fru
	{
		PUYA_P24C128F,
		ACCL_2_FRU_ID,
		ACCL_1_6_FRU_PORT,
		ACCL_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_3 fru
	{
		PUYA_P24C128F,
		ACCL_3_FRU_ID,
		ACCL_1_6_FRU_PORT,
		ACCL_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_4 fru
	{
		PUYA_P24C128F,
		ACCL_4_FRU_ID,
		ACCL_1_6_FRU_PORT,
		ACCL_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_5 fru
	{
		PUYA_P24C128F,
		ACCL_5_FRU_ID,
		ACCL_1_6_FRU_PORT,
		ACCL_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_6 fru
	{
		PUYA_P24C128F,
		ACCL_6_FRU_ID,
		ACCL_1_6_FRU_PORT,
		ACCL_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_7 fru
	{
		PUYA_P24C128F,
		ACCL_7_FRU_ID,
		ACCL_7_12_FRU_PORT,
		ACCL_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_8 fru
	{
		PUYA_P24C128F,
		ACCL_8_FRU_ID,
		ACCL_7_12_FRU_PORT,
		ACCL_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_9 fru
	{
		PUYA_P24C128F,
		ACCL_9_FRU_ID,
		ACCL_7_12_FRU_PORT,
		ACCL_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_10 fru
	{
		PUYA_P24C128F,
		ACCL_10_FRU_ID,
		ACCL_7_12_FRU_PORT,
		ACCL_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_11 fru
	{
		PUYA_P24C128F,
		ACCL_11_FRU_ID,
		ACCL_7_12_FRU_PORT,
		ACCL_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_12 fru
	{
		PUYA_P24C128F,
		ACCL_12_FRU_ID,
		ACCL_7_12_FRU_PORT,
		ACCL_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_1 freya ch1 fru
	{
		ST_M24C64_W,
		ACCL_1_CH1_FREYA_FRU_ID,
		ACCL_1_6_FRU_PORT,
		ACCL_FREYA_CH1_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_1 freya ch2 fru
	{
		ST_M24C64_W,
		ACCL_1_CH2_FREYA_FRU_ID,
		ACCL_1_6_FRU_PORT,
		ACCL_FREYA_CH2_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_2 freya ch1 fru
	{
		ST_M24C64_W,
		ACCL_2_CH1_FREYA_FRU_ID,
		ACCL_1_6_FRU_PORT,
		ACCL_FREYA_CH1_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_2 freya ch2 fru
	{
		ST_M24C64_W,
		ACCL_2_CH2_FREYA_FRU_ID,
		ACCL_1_6_FRU_PORT,
		ACCL_FREYA_CH2_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_3 freya ch1 fru
	{
		ST_M24C64_W,
		ACCL_3_CH1_FREYA_FRU_ID,
		ACCL_1_6_FRU_PORT,
		ACCL_FREYA_CH1_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_3 freya ch2 fru
	{
		ST_M24C64_W,
		ACCL_3_CH2_FREYA_FRU_ID,
		ACCL_1_6_FRU_PORT,
		ACCL_FREYA_CH2_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_4 freya ch1 fru
	{
		ST_M24C64_W,
		ACCL_4_CH1_FREYA_FRU_ID,
		ACCL_1_6_FRU_PORT,
		ACCL_FREYA_CH1_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_4 freya ch2 fru
	{
		ST_M24C64_W,
		ACCL_4_CH2_FREYA_FRU_ID,
		ACCL_1_6_FRU_PORT,
		ACCL_FREYA_CH2_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_5 freya ch1 fru
	{
		ST_M24C64_W,
		ACCL_5_CH1_FREYA_FRU_ID,
		ACCL_1_6_FRU_PORT,
		ACCL_FREYA_CH1_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_5 freya ch2 fru
	{
		ST_M24C64_W,
		ACCL_5_CH2_FREYA_FRU_ID,
		ACCL_1_6_FRU_PORT,
		ACCL_FREYA_CH2_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_6 freya ch1 fru
	{
		ST_M24C64_W,
		ACCL_6_CH1_FREYA_FRU_ID,
		ACCL_1_6_FRU_PORT,
		ACCL_FREYA_CH1_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_6 freya ch2 fru
	{
		ST_M24C64_W,
		ACCL_6_CH2_FREYA_FRU_ID,
		ACCL_1_6_FRU_PORT,
		ACCL_FREYA_CH2_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_7 freya ch1 fru
	{
		ST_M24C64_W,
		ACCL_7_CH1_FREYA_FRU_ID,
		ACCL_7_12_FRU_PORT,
		ACCL_FREYA_CH1_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_7 freya ch2 fru
	{
		ST_M24C64_W,
		ACCL_7_CH2_FREYA_FRU_ID,
		ACCL_7_12_FRU_PORT,
		ACCL_FREYA_CH2_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_8 freya ch1 fru
	{
		ST_M24C64_W,
		ACCL_8_CH1_FREYA_FRU_ID,
		ACCL_7_12_FRU_PORT,
		ACCL_FREYA_CH1_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_8 freya ch2 fru
	{
		ST_M24C64_W,
		ACCL_8_CH2_FREYA_FRU_ID,
		ACCL_7_12_FRU_PORT,
		ACCL_FREYA_CH2_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_9 freya ch1 fru
	{
		ST_M24C64_W,
		ACCL_9_CH1_FREYA_FRU_ID,
		ACCL_7_12_FRU_PORT,
		ACCL_FREYA_CH1_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_9 freya ch2 fru
	{
		ST_M24C64_W,
		ACCL_9_CH2_FREYA_FRU_ID,
		ACCL_7_12_FRU_PORT,
		ACCL_FREYA_CH2_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_10 freya ch1 fru
	{
		ST_M24C64_W,
		ACCL_10_CH1_FREYA_FRU_ID,
		ACCL_7_12_FRU_PORT,
		ACCL_FREYA_CH1_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_10 freya ch2 fru
	{
		ST_M24C64_W,
		ACCL_10_CH2_FREYA_FRU_ID,
		ACCL_7_12_FRU_PORT,
		ACCL_FREYA_CH2_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_11 freya ch1 fru
	{
		ST_M24C64_W,
		ACCL_11_CH1_FREYA_FRU_ID,
		ACCL_7_12_FRU_PORT,
		ACCL_FREYA_CH1_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_11 freya ch2 fru
	{
		ST_M24C64_W,
		ACCL_11_CH2_FREYA_FRU_ID,
		ACCL_7_12_FRU_PORT,
		ACCL_FREYA_CH2_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_12 freya ch1 fru
	{
		ST_M24C64_W,
		ACCL_12_CH1_FREYA_FRU_ID,
		ACCL_7_12_FRU_PORT,
		ACCL_FREYA_CH1_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	// ACCL_12 freya ch2 fru
	{
		ST_M24C64_W,
		ACCL_12_CH2_FREYA_FRU_ID,
		ACCL_7_12_FRU_PORT,
		ACCL_FREYA_CH2_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
};

void pal_load_fru_config(void)
{
	memcpy(&fru_config, &plat_fru_config, sizeof(plat_fru_config));
}

bool pal_accl_fru_id_map_accl_id_dev_id(uint8_t accl_fru_id, uint8_t *accl_id, uint8_t *dev_id)
{
	CHECK_NULL_ARG_WITH_RETURN(accl_id, false);
	CHECK_NULL_ARG_WITH_RETURN(dev_id, false);

	switch (accl_fru_id) {
	case ACCL_1_FRU_ID:
	case ACCL_2_FRU_ID:
	case ACCL_3_FRU_ID:
	case ACCL_4_FRU_ID:
	case ACCL_5_FRU_ID:
	case ACCL_6_FRU_ID:
	case ACCL_7_FRU_ID:
	case ACCL_8_FRU_ID:
	case ACCL_9_FRU_ID:
	case ACCL_10_FRU_ID:
	case ACCL_11_FRU_ID:
	case ACCL_12_FRU_ID:
		*accl_id = accl_fru_id - ACCL_1_FRU_ID;
		*dev_id = PCIE_DEVICE_ID1;
		break;
	case ACCL_1_CH1_FREYA_FRU_ID:
	case ACCL_2_CH1_FREYA_FRU_ID:
	case ACCL_3_CH1_FREYA_FRU_ID:
	case ACCL_4_CH1_FREYA_FRU_ID:
	case ACCL_5_CH1_FREYA_FRU_ID:
	case ACCL_6_CH1_FREYA_FRU_ID:
	case ACCL_7_CH1_FREYA_FRU_ID:
	case ACCL_8_CH1_FREYA_FRU_ID:
	case ACCL_9_CH1_FREYA_FRU_ID:
	case ACCL_10_CH1_FREYA_FRU_ID:
	case ACCL_11_CH1_FREYA_FRU_ID:
	case ACCL_12_CH1_FREYA_FRU_ID:
		*accl_id = (accl_fru_id - ACCL_1_CH1_FREYA_FRU_ID) / 2;
		*dev_id = PCIE_DEVICE_ID2;
		break;
	case ACCL_1_CH2_FREYA_FRU_ID:
	case ACCL_2_CH2_FREYA_FRU_ID:
	case ACCL_3_CH2_FREYA_FRU_ID:
	case ACCL_4_CH2_FREYA_FRU_ID:
	case ACCL_5_CH2_FREYA_FRU_ID:
	case ACCL_6_CH2_FREYA_FRU_ID:
	case ACCL_7_CH2_FREYA_FRU_ID:
	case ACCL_8_CH2_FREYA_FRU_ID:
	case ACCL_9_CH2_FREYA_FRU_ID:
	case ACCL_10_CH2_FREYA_FRU_ID:
	case ACCL_11_CH2_FREYA_FRU_ID:
	case ACCL_12_CH2_FREYA_FRU_ID:
		*accl_id = (accl_fru_id - ACCL_1_CH2_FREYA_FRU_ID) / 2;
		*dev_id = PCIE_DEVICE_ID3;
		break;
	default:
		return false;
	}

	return true;
}

bool card_id_dev_id_map_fru_id(uint8_t card_id, uint8_t dev_id, uint8_t *fru_id)
{
	CHECK_NULL_ARG_WITH_RETURN(fru_id, false);

	switch (dev_id) {
	case PCIE_DEVICE_ID1:
		*fru_id = ACCL_1_FRU_ID + card_id;
		break;
	case PCIE_DEVICE_ID2:
		*fru_id = ACCL_1_CH1_FREYA_FRU_ID + (card_id * 2);
		break;
	case PCIE_DEVICE_ID3:
		*fru_id = ACCL_1_CH2_FREYA_FRU_ID + (card_id * 2);
		break;
	default:
		return false;
	}

	return true;
}
