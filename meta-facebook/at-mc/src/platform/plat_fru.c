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
		CXL_FRU_MUX0_ADDR,
		CXL_FRU_MUX0_CHANNEL0,
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
		CXL_FRU_MUX0_ADDR,
		CXL_FRU_MUX0_CHANNEL1,
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
		CXL_FRU_MUX0_ADDR,
		CXL_FRU_MUX0_CHANNEL2,
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
		CXL_FRU_MUX0_ADDR,
		CXL_FRU_MUX0_CHANNEL3,
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
		CXL_FRU_MUX0_ADDR,
		CXL_FRU_MUX0_CHANNEL4,
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
		CXL_FRU_MUX0_ADDR,
		CXL_FRU_MUX0_CHANNEL5,
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
		CXL_FRU_MUX0_ADDR,
		CXL_FRU_MUX0_CHANNEL6,
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
		CXL_FRU_MUX0_ADDR,
		CXL_FRU_MUX0_CHANNEL7,
	},
};

void pal_load_fru_config(void)
{
	memcpy(&fru_config, &plat_fru_config, sizeof(plat_fru_config));
}
