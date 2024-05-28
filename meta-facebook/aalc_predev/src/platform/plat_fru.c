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
#include "plat_i2c.h"
#include "i2c-mux-pca954x.h"
#include "modbus_server.h"
#include <libutil.h>
#include <stdlib.h>
#include <logging/log.h>
#include "fru.h"
#include "plat_fru.h"

LOG_MODULE_REGISTER(plat_fru);

#define AALC_FRU_START 0x0000
#define AALC_FRU_SIZE 0x0400 // size 1KB

#define FRU_ADDR_MIN FRU_FB_PART_ADDR
#define FRU_ADDR_MAX FRU_MFR_SERIEL_ADDR

const EEPROM_CFG plat_fru_config[] = {
	{
		ST_M24512_RDW,
		MB_FRU_ID,
		I2C_BUS10,
		MB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
	},
	{
		NV_ATMEL_24C02,
		BB_FRU_ID,
		I2C_BUS4,
		BB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
	},
	{
		NV_ATMEL_24C02,
		BPB_FRU_ID,
		I2C_BUS4,
		BPB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
	},
	{
		NV_ATMEL_24C02,
		SB_FRU_ID,
		I2C_BUS9,
		SB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		SB_MUX_ADDR,
		PCA9546A_CHANNEL_1,
	},
	{
		NV_ATMEL_24C02,
		PDB_FRU_ID,
		I2C_BUS9,
		PDB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		SB_MUX_ADDR,
		PCA9546A_CHANNEL_2,
	},
	{
		NV_ATMEL_24C02,
		PB_1_FRU_ID,
		I2C_BUS8,
		PB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		PB_MUX_ADDR,
		PCA9546A_CHANNEL_0,
	},
	{
		NV_ATMEL_24C02,
		PB_2_FRU_ID,
		I2C_BUS8,
		PB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		PB_MUX_ADDR,
		PCA9546A_CHANNEL_1,
	},
	{
		NV_ATMEL_24C02,
		PB_3_FRU_ID,
		I2C_BUS8,
		PB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		PB_MUX_ADDR,
		PCA9546A_CHANNEL_2,
	},
	{
		NV_ATMEL_24C02,
		FB_1_FRU_ID,
		I2C_BUS1,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		FB_1_4_MUX_ADDR,
		PCA9546A_CHANNEL_0,
	},
	{
		NV_ATMEL_24C02,
		FB_2_FRU_ID,
		I2C_BUS1,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		FB_1_4_MUX_ADDR,
		PCA9546A_CHANNEL_1,
	},
	{
		NV_ATMEL_24C02,
		FB_3_FRU_ID,
		I2C_BUS1,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		FB_1_4_MUX_ADDR,
		PCA9546A_CHANNEL_2,
	},
	{
		NV_ATMEL_24C02,
		FB_4_FRU_ID,
		I2C_BUS1,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		FB_1_4_MUX_ADDR,
		PCA9546A_CHANNEL_3,
	},
	{
		NV_ATMEL_24C02,
		FB_5_FRU_ID,
		I2C_BUS2,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		FB_5_8_MUX_ADDR,
		PCA9546A_CHANNEL_0,
	},
	{
		NV_ATMEL_24C02,
		FB_6_FRU_ID,
		I2C_BUS2,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		FB_5_8_MUX_ADDR,
		PCA9546A_CHANNEL_1,
	},
	{
		NV_ATMEL_24C02,
		FB_7_FRU_ID,
		I2C_BUS2,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		FB_5_8_MUX_ADDR,
		PCA9546A_CHANNEL_2,
	},
	{
		NV_ATMEL_24C02,
		FB_8_FRU_ID,
		I2C_BUS2,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		FB_5_8_MUX_ADDR,
		PCA9546A_CHANNEL_3,
	},
	{
		NV_ATMEL_24C02,
		FB_9_FRU_ID,
		I2C_BUS6,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		FB_9_12_MUX_ADDR,
		PCA9546A_CHANNEL_0,
	},
	{
		NV_ATMEL_24C02,
		FB_10_FRU_ID,
		I2C_BUS6,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		FB_9_12_MUX_ADDR,
		PCA9546A_CHANNEL_1,
	},
	{
		NV_ATMEL_24C02,
		FB_11_FRU_ID,
		I2C_BUS6,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		FB_9_12_MUX_ADDR,
		PCA9546A_CHANNEL_2,
	},
	{
		NV_ATMEL_24C02,
		FB_12_FRU_ID,
		I2C_BUS6,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		FB_9_12_MUX_ADDR,
		PCA9546A_CHANNEL_3,
	},
	{
		NV_ATMEL_24C02,
		FB_13_FRU_ID,
		I2C_BUS7,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		FB_13_14_MUX_ADDR,
		PCA9546A_CHANNEL_0,
	},
	{
		NV_ATMEL_24C02,
		FB_14_FRU_ID,
		I2C_BUS7,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		FB_13_14_MUX_ADDR,
		PCA9546A_CHANNEL_1,
	},
};

void pal_load_fru_config(void)
{
	memcpy(&fru_config, &plat_fru_config, sizeof(plat_fru_config));
}

// plat data save in EEPROM
bool plat_eeprom_write(uint32_t offset, uint8_t *data, uint16_t data_len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);
	EEPROM_ENTRY entry;

	entry.offset = offset;
	entry.data_len = data_len;

	memcpy(entry.data, data, data_len);
	memcpy(&entry.config, &fru_config[MB_FRU_ID], sizeof(fru_config[MB_FRU_ID]));

	if (!eeprom_write(&entry)) {
		LOG_ERR("write eeprom 0x%x fail", offset);
		return false;
	}

	return true;
}

bool plat_eeprom_read(uint32_t offset, uint8_t *data, uint16_t data_len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);
	EEPROM_ENTRY entry;

	entry.offset = offset;
	entry.data_len = data_len;

	memcpy(&entry.config, &fru_config[MB_FRU_ID], sizeof(fru_config[MB_FRU_ID]));
	memset(entry.data, 0xFF, sizeof(entry.data));

	if (!eeprom_read(&entry)) {
		LOG_ERR("read eeprom 0x%x fail", offset);
		return false;
	}

	memcpy(data, entry.data, data_len);

	return true;
}