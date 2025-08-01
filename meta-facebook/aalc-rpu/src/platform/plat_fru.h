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

#include <stdint.h>

enum FRU_ID {
	MB_FRU_ID = 0x00,
	BB_FRU_ID,
	BPB_FRU_ID,
	PDB_FRU_ID,
	SB_FRU_ID,
	PB_1_FRU_ID,
	PB_2_FRU_ID,
	PB_3_FRU_ID,
	FB_1_FRU_ID,
	FB_2_FRU_ID,
	FB_3_FRU_ID,
	FB_4_FRU_ID,
	FB_5_FRU_ID,
	FB_6_FRU_ID,
	FB_7_FRU_ID,
	FB_8_FRU_ID,
	FB_9_FRU_ID,
	FB_10_FRU_ID,
	FB_11_FRU_ID,
	FB_12_FRU_ID,
	FB_13_FRU_ID,
	FB_14_FRU_ID,
	FIO_FRU_ID,
	MAX_FRU_ID,
};

#define FRU_CFG_NUM MAX_FRU_ID

#define MB_FRU_ADDR (0xA6 >> 1)
#define FIO_FRU_ADDR (0xA4 >> 1)
#define BB_FRU_ADDR (0xA4 >> 1)
#define BPB_FRU_ADDR (0xA6 >> 1)
#define SB_FRU_ADDR (0xA6 >> 1)
#define PDB_FRU_ADDR (0xAA >> 1)
#define PB_FRU_ADDR (0xA6 >> 1)
#define FB_FRU_ADDR (0xA6 >> 1)

#define I2C_1_MUX_ADDR (0xE0 >> 1)
#define I2C_2_MUX_ADDR (0xE2 >> 1)
#define I2C_6_MUX_ADDR (0xE4 >> 1)
#define I2C_7_MUX_ADDR (0xE6 >> 1)
#define PB_MUX_ADDR (0xE8 >> 1)
#define SB_MUX_ADDR (0xE8 >> 1)

#define MUX_CHANNEL_0 0
#define MUX_CHANNEL_1 1
#define MUX_CHANNEL_2 2
#define MUX_CHANNEL_3 3

// plat eeprom save in rpu eeprom
#define MANAGEMENT_BOARD_FRU_EEPROM_OFFSET 0x0000
#define MANAGEMENT_BOARD_FRU_EEPROM_START_OFFSET 0x0008
#define MANAGEMENT_BOARD_FRU_EEPROM_BOARD_AREA_SIZE 0x0009
#define PLAT_EEPROM_OFFSET 0x2000 // 8kb
#define EEPROM_HMI_VERSION_OFFSET PLAT_EEPROM_OFFSET
#define EEPROM_HMI_VERSION_SIZE 8 // 8 bytes
#define EEPROM_RPU_ADDR_OFFSET (EEPROM_HMI_VERSION_OFFSET + EEPROM_HMI_VERSION_SIZE)
#define EEPROM_RPU_ADDR_VERSION_SIZE 1 // 1 bytes

#define EEPROM_UPTIME_OFFSET (EEPROM_RPU_ADDR_OFFSET + EEPROM_RPU_ADDR_VERSION_SIZE)
#define EEPROM_UPTIME_SIZE 4 // 4 bytes

#define EEPROM_PUMP1_UPTIME_OFFSET (EEPROM_UPTIME_OFFSET + EEPROM_UPTIME_SIZE)
#define EEPROM_PUMP1_UPTIME_SIZE 4 // 4 bytes

#define EEPROM_PUMP2_UPTIME_OFFSET (EEPROM_PUMP1_UPTIME_OFFSET + EEPROM_PUMP1_UPTIME_SIZE)
#define EEPROM_PUMP2_UPTIME_SIZE 4 // 4 bytes

#define EEPROM_PUMP3_UPTIME_OFFSET (EEPROM_PUMP2_UPTIME_OFFSET + EEPROM_PUMP2_UPTIME_SIZE)
#define EEPROM_PUMP3_UPTIME_SIZE 4 // 4 bytes

bool plat_eeprom_write(uint32_t offset, uint8_t *data, uint16_t data_len);
bool plat_eeprom_read(uint32_t offset, uint8_t *data, uint16_t data_len);

#define CHASSIS_CUSTOM_DATA_MAX 24
#define BOARD_CUSTOM_DATA_MAX 10
#define PRODUCT_CUSTOM_DATA_MAX 10
typedef struct {
	uint8_t chassis_type;
	char chassis_part_number[32];
	char chassis_serial_number[32];
	char chassis_custom_data[CHASSIS_CUSTOM_DATA_MAX][32];
} ChassisInfo;

typedef struct {
	uint8_t language;
	char board_mfg_date[32];
	char board_mfg[32];
	char board_product[32];
	char board_serial[32];
	char board_part_number[32];
	char board_fru_id[32];
	char board_custom_data[BOARD_CUSTOM_DATA_MAX][32];
} BoardInfo;

typedef struct {
	uint8_t language;
	char product_manufacturer[32];
	char product_name[32];
	char product_part_number[32];
	char product_version[32];
	char product_serial[32];
	char product_asset_tag[32];
	char product_fru_id[32];
	char product_custom_data[PRODUCT_CUSTOM_DATA_MAX][32];
} ProductInfo;

typedef struct {
	ChassisInfo chassis;
	BoardInfo board;
	ProductInfo product;
} FRU_INFO;

void print_fru_info(uint8_t board_fru_id);
FRU_INFO *get_single_fru_info(uint8_t board_fru_id);
#endif
