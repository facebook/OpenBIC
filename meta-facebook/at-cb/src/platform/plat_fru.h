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
#include "i2c-mux-pca954x.h"

enum FRU_ID {
	FIO_FRU_ID = 0x0D,
	CB_FRU_ID = 0x10,
	ACCL_1_FRU_ID = 0x12,
	ACCL_2_FRU_ID,
	ACCL_3_FRU_ID,
	ACCL_4_FRU_ID,
	ACCL_5_FRU_ID,
	ACCL_6_FRU_ID,
	ACCL_7_FRU_ID,
	ACCL_8_FRU_ID,
	ACCL_9_FRU_ID,
	ACCL_10_FRU_ID,
	ACCL_11_FRU_ID,
	ACCL_12_FRU_ID,
	ACCL_1_CH1_FREYA_FRU_ID = 0x1E,
	ACCL_1_CH2_FREYA_FRU_ID,
	ACCL_2_CH1_FREYA_FRU_ID,
	ACCL_2_CH2_FREYA_FRU_ID,
	ACCL_3_CH1_FREYA_FRU_ID,
	ACCL_3_CH2_FREYA_FRU_ID,
	ACCL_4_CH1_FREYA_FRU_ID,
	ACCL_4_CH2_FREYA_FRU_ID,
	ACCL_5_CH1_FREYA_FRU_ID,
	ACCL_5_CH2_FREYA_FRU_ID,
	ACCL_6_CH1_FREYA_FRU_ID,
	ACCL_6_CH2_FREYA_FRU_ID,
	ACCL_7_CH1_FREYA_FRU_ID,
	ACCL_7_CH2_FREYA_FRU_ID,
	ACCL_8_CH1_FREYA_FRU_ID,
	ACCL_8_CH2_FREYA_FRU_ID,
	ACCL_9_CH1_FREYA_FRU_ID,
	ACCL_9_CH2_FREYA_FRU_ID,
	ACCL_10_CH1_FREYA_FRU_ID,
	ACCL_10_CH2_FREYA_FRU_ID,
	ACCL_11_CH1_FREYA_FRU_ID,
	ACCL_11_CH2_FREYA_FRU_ID,
	ACCL_12_CH1_FREYA_FRU_ID,
	ACCL_12_CH2_FREYA_FRU_ID,
	// OTHER_FRU_ID,
	MAX_FRU_ID,
};

enum ACCL_FRU_OPTION {
	ACCL_FRU_READ,
	ACCL_FRU_WRITE,
};

/* Skip fru id 0~12, 14~15, 17 */
#define FRU_CFG_NUM 38
#define BIC_FRU_DEV_ID CB_FRU_ID

#define CB_FRU_PORT I2C_BUS2
#define CB_FRU_ADDR (0xAC >> 1)
#define FIO_FRU_PORT I2C_BUS10
#define FIO_FRU_ADDR (0xA2 >> 1)
#define ACCL_1_6_FRU_PORT I2C_BUS7
#define ACCL_7_12_FRU_PORT I2C_BUS8
#define ACCL_FRU_ADDR (0xA6 >> 1)
#define ACCL_1_6_FRU_MUX_ADDR (0xE0 >> 1)
#define ACCL_7_12_FRU_MUX_ADDR (0xE8 >> 1)
#define ACCL_1_7_FRU_MUX_CHAN 0
#define ACCL_2_8_FRU_MUX_CHAN 1
#define ACCL_3_9_FRU_MUX_CHAN 2
#define ACCL_4_10_FRU_MUX_CHAN 3
#define ACCL_5_11_FRU_MUX_CHAN 4
#define ACCL_6_12_FRU_MUX_CHAN 5
#define ACCL_FREYA_CH1_FRU_ADDR (0xA4 >> 1)
#define ACCL_FREYA_CH2_FRU_ADDR (0xA2 >> 1)

bool pal_accl_fru_id_map_accl_id_dev_id(uint8_t accl_fru_id, uint8_t *accl_id, uint8_t *dev_id);
bool card_id_dev_id_map_fru_id(uint8_t card_id, uint8_t dev_id, uint8_t *fru_id);

#endif
