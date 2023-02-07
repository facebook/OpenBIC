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

#ifndef PLAT_FRU_H
#define PLAT_FRU_H

#include "plat_i2c.h"
#include "i2c-mux-pca984x.h"

#define MC_FRU_PORT I2C_BUS1
#define MC_FRU_ADDR (0xA0 >> 1)
#define CXL_FRU_PORT I2C_BUS2
#define CXL_FRU_ADDR (0xA8 >> 1)
#define CXL_FRU_MUX0_ADDR (0xE0 >> 1)
#define CXL_FRU_MUX1_ADDR (0xE2 >> 1)
#define CXL_FRU_MUX1_CHANNEL 2

enum FRU_ID {
	MC_FRU_ID = 0x11,
	CXL_FRU_ID1 = 0x1E,
	CXL_FRU_ID2,
	CXL_FRU_ID3,
	CXL_FRU_ID4,
	CXL_FRU_ID5 = 0x26,
	CXL_FRU_ID6,
	CXL_FRU_ID7,
	CXL_FRU_ID8,
	// OTHER_FRU_ID,
	MAX_FRU_ID,
};

enum {
	CXL_FRU_MUX0_CHANNEL0,
	CXL_FRU_MUX0_CHANNEL1,
	CXL_FRU_MUX0_CHANNEL2,
	CXL_FRU_MUX0_CHANNEL3,
	CXL_FRU_MUX0_CHANNEL4,
	CXL_FRU_MUX0_CHANNEL5,
	CXL_FRU_MUX0_CHANNEL6,
	CXL_FRU_MUX0_CHANNEL7,
};

/* Skip fru id 0~16, 18~29 */
#define FRU_CFG_NUM 9
#define BIC_FRU_DEV_ID MC_FRU_ID

int pal_cxl_map_mux0_channel(uint8_t cxl_id);

#endif
