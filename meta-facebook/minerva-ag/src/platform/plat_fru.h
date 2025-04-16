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

#define FRU_CFG_NUM MAX_FRU_ID
#define LOG_EEPROM_ADDR (0xA0 >> 1)
#define CPLD_EEPROM_ADDR (0xA0 >> 1)

#define CHASSIS_CUSTOM_DATA_MAX 24
#define BOARD_CUSTOM_DATA_MAX 8
#define PRODUCT_CUSTOM_DATA_MAX 8

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

enum FRU_ID {
	LOG_EEPROM_ID = 0x00,
	CPLD_EEPROM_ID,
	MAX_FRU_ID,
};

bool init_fru_info(void);
void print_fru_info(void);
bool plat_eeprom_write(uint32_t offset, uint8_t *data, uint16_t data_len);
bool plat_eeprom_read(uint32_t offset, uint8_t *data, uint16_t data_len);
FRU_INFO *get_fru_info(void);

#endif
