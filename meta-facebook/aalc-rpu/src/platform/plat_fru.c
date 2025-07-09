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
#include <time.h>

LOG_MODULE_REGISTER(plat_fru);

#define AALC_FRU_START 0x0000
#define AALC_FRU_SIZE 0x0400 // size 1KB

#define FRU_ADDR_MIN FRU_FB_PART_ADDR
#define FRU_ADDR_MAX FRU_MFR_SERIEL_ADDR

extern struct k_mutex i2c_1_PCA9546a_mutex;
extern struct k_mutex i2c_2_PCA9546a_mutex;
extern struct k_mutex i2c_6_PCA9546a_mutex;
extern struct k_mutex i2c_7_PCA9546a_mutex;
extern struct k_mutex i2c_8_PCA9546a_mutex;
extern struct k_mutex i2c_9_PCA9546a_mutex;

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
		PDB_FRU_ID,
		I2C_BUS9,
		PDB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		SB_MUX_ADDR,
		MUX_CHANNEL_2,
		&i2c_9_PCA9546a_mutex,
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
		MUX_CHANNEL_1,
		&i2c_9_PCA9546a_mutex,
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
		MUX_CHANNEL_0,
		&i2c_8_PCA9546a_mutex,
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
		MUX_CHANNEL_1,
		&i2c_8_PCA9546a_mutex,
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
		MUX_CHANNEL_2,
		&i2c_8_PCA9546a_mutex,
	},
	{
		NV_ATMEL_24C02,
		FB_1_FRU_ID,
		I2C_BUS6,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		I2C_6_MUX_ADDR,
		MUX_CHANNEL_0,
		&i2c_6_PCA9546a_mutex,
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
		I2C_1_MUX_ADDR,
		MUX_CHANNEL_0,
		&i2c_1_PCA9546a_mutex,
	},
	{
		NV_ATMEL_24C02,
		FB_3_FRU_ID,
		I2C_BUS6,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		I2C_6_MUX_ADDR,
		MUX_CHANNEL_1,
		&i2c_6_PCA9546a_mutex,
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
		I2C_1_MUX_ADDR,
		MUX_CHANNEL_1,
		&i2c_1_PCA9546a_mutex,
	},
	{
		NV_ATMEL_24C02,
		FB_5_FRU_ID,
		I2C_BUS6,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		I2C_6_MUX_ADDR,
		MUX_CHANNEL_2,
		&i2c_6_PCA9546a_mutex,
	},
	{
		NV_ATMEL_24C02,
		FB_6_FRU_ID,
		I2C_BUS1,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		I2C_1_MUX_ADDR,
		MUX_CHANNEL_2,
		&i2c_1_PCA9546a_mutex,
	},
	{
		NV_ATMEL_24C02,
		FB_7_FRU_ID,
		I2C_BUS6,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		I2C_6_MUX_ADDR,
		MUX_CHANNEL_3,
		&i2c_6_PCA9546a_mutex,
	},
	{
		NV_ATMEL_24C02,
		FB_8_FRU_ID,
		I2C_BUS1,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		I2C_1_MUX_ADDR,
		MUX_CHANNEL_3,
		&i2c_1_PCA9546a_mutex,
	},
	{
		NV_ATMEL_24C02,
		FB_9_FRU_ID,
		I2C_BUS7,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		I2C_7_MUX_ADDR,
		MUX_CHANNEL_0,
		&i2c_7_PCA9546a_mutex,
	},
	{
		NV_ATMEL_24C02,
		FB_10_FRU_ID,
		I2C_BUS2,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		I2C_2_MUX_ADDR,
		MUX_CHANNEL_0,
		&i2c_2_PCA9546a_mutex,
	},
	{
		NV_ATMEL_24C02,
		FB_11_FRU_ID,
		I2C_BUS7,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		I2C_7_MUX_ADDR,
		MUX_CHANNEL_1,
		&i2c_7_PCA9546a_mutex,
	},
	{
		NV_ATMEL_24C02,
		FB_12_FRU_ID,
		I2C_BUS2,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		I2C_2_MUX_ADDR,
		MUX_CHANNEL_1,
		&i2c_2_PCA9546a_mutex,
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
		I2C_7_MUX_ADDR,
		MUX_CHANNEL_2,
		&i2c_7_PCA9546a_mutex,
	},
	{
		NV_ATMEL_24C02,
		FB_14_FRU_ID,
		I2C_BUS2,
		FB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
		true,
		I2C_2_MUX_ADDR,
		MUX_CHANNEL_2,
		&i2c_2_PCA9546a_mutex,
	},
	{
		ST_M24512_RDW,
		FIO_FRU_ID,
		I2C_BUS10,
		FIO_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AALC_FRU_START,
		AALC_FRU_SIZE,
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

	uint8_t fru_index = 0;
	if (!find_FRU_ID(MB_FRU_ID, &fru_index)) {
		LOG_ERR("find_FRU_ID fail when write eeprom 0x%x ", offset);
		return false;
	}

	memcpy(entry.data, data, data_len);
	memcpy(&entry.config, &fru_config[fru_index], sizeof(fru_config[fru_index]));

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

	uint8_t fru_index = 0;
	if (!find_FRU_ID(MB_FRU_ID, &fru_index)) {
		LOG_ERR("find_FRU_ID fail when read eeprom 0x%x ", offset);
		return false;
	}

	memcpy(&entry.config, &fru_config[fru_index], sizeof(fru_config[fru_index]));
	memset(entry.data, 0xFF, sizeof(entry.data));

	if (!eeprom_read(&entry)) {
		LOG_ERR("read eeprom 0x%x fail", offset);
		return false;
	}

	memcpy(data, entry.data, data_len);

	return true;
}

static void decode_field(const uint8_t *src, int len, char *dest, int dest_size)
{
	CHECK_NULL_ARG(src);
	CHECK_NULL_ARG(dest);

	int copy_len = (len < dest_size - 1) ? len : dest_size - 1;
	memcpy(dest, src, copy_len);
	dest[copy_len] = '\0';

	LOG_HEXDUMP_DBG(src, len, "decode_field result:");
}

static void parse_board_mfg_date(const uint8_t *src, char *dest, int dest_size)
{
	CHECK_NULL_ARG(src);
	CHECK_NULL_ARG(dest);

	// According to the IPMI specification, these 3 bytes represent the number of minutes
	// since 1996-01-01 00:00:00 GMT.
	uint32_t minutes = src[0] | (src[1] << 8) | (src[2] << 16);
	time_t epoch = 820454400; // Unix time for 1996-01-01 00:00:00 GMT
	time_t t = epoch + minutes * 60;
	const struct tm *time_info = gmtime(&t);

	// Manually format date string as "YYYY-MM-DD HH:MM:SS" using snprintf
	snprintf(dest, dest_size, "%04d-%02d-%02d %02d:%02d:%02d", time_info->tm_year + 1900,
		 time_info->tm_mon + 1, time_info->tm_mday, time_info->tm_hour, time_info->tm_min,
		 time_info->tm_sec);
}

#define FRU_DATA_SIZE 0x0200

bool plat_fru_read(uint32_t offset, uint8_t *data, uint16_t data_len, uint8_t board_fru_id)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);
	EEPROM_ENTRY entry;

	entry.offset = offset;
	entry.data_len = data_len;

	uint8_t fru_index = 0;
	if (!find_FRU_ID(board_fru_id, &fru_index)) {
		LOG_ERR("find_FRU_ID fail when read eeprom 0x%x ", offset);
		return false;
	}

	memcpy(&entry.config, &fru_config[fru_index], sizeof(fru_config[fru_index]));
	memset(entry.data, 0xFF, sizeof(entry.data));

	if (!eeprom_read(&entry)) {
		LOG_ERR("read eeprom 0x%x fail", offset);
		return false;
	}

	memcpy(data, entry.data, data_len);

	return true;
}
bool plat_get_fru_data(uint8_t *data, uint8_t board_fru_id)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	const uint32_t total_size = FRU_DATA_SIZE; // 0x0200
	const uint32_t chunk_size = 0x10; // 0x10 bytes per read
	uint32_t offset = 0;

	while (offset < total_size) {
		if (!plat_fru_read(AALC_FRU_START + offset, data + offset, chunk_size,
					board_fru_id)) {
			LOG_ERR("Failed to read FRU chunk at offset 0x%x", offset);
			return false;
		}
		offset += chunk_size;
	}

	return true;
}
FRU_INFO *plat_fru_info = NULL;

FRU_INFO *create_fru_info(void)
{
	FRU_INFO *info = malloc(sizeof(FRU_INFO));
	if (!info) {
		printk("Failed to allocate memory for FRU_INFO");
		return NULL;
	}
	memset(info, 0, sizeof(FRU_INFO));
	return info;
}

bool get_fru_info(uint8_t board_fru_id)
{
	plat_fru_info = create_fru_info();
	if (!plat_fru_info) {
		printk("Failed to create_fru_info");
		return false;
	}

	uint8_t fru_data[FRU_DATA_SIZE];

	/* Read FRU data from EEPROM */
	if (!plat_get_fru_data(fru_data, board_fru_id)) {
		printk("Failed to read FRU data\n");
		return false;
	}

	/* Parse common header (first 8 bytes) */
	uint8_t common_header[8];
	memcpy(common_header, fru_data, 8);
	uint16_t chassis_offset = common_header[2] * 8;
	uint16_t board_offset = common_header[3] * 8;
	uint16_t product_offset = common_header[4] * 8;

	/* --------------------- Parse Chassis Area --------------------- */
	if (chassis_offset + 3 < FRU_DATA_SIZE) {
		uint16_t area_len = fru_data[chassis_offset + 1] * 8;
		plat_fru_info->chassis.chassis_type = fru_data[chassis_offset + 2];

		int offset = chassis_offset + 3;
		/* Field 1: Chassis Part Number */
		if (offset < chassis_offset + area_len - 1) {
			uint8_t type_length = fru_data[offset++];
			if (type_length != 0xC1) {
				int len = type_length & 0x3F;
				decode_field(&fru_data[offset], len,
					     plat_fru_info->chassis.chassis_part_number,
					     sizeof(plat_fru_info->chassis.chassis_part_number));
				offset += len;
			}
		}
		/* Field 2: Chassis Serial Number */
		if (offset < chassis_offset + area_len - 1) {
			uint8_t type_length = fru_data[offset++];
			if (type_length != 0xC1) {
				int len = type_length & 0x3F;
				decode_field(&fru_data[offset], len,
					     plat_fru_info->chassis.chassis_serial_number,
					     sizeof(plat_fru_info->chassis.chassis_serial_number));
				offset += len;
			}
		}
		/* Fields 3 ~ 26: Chassis Custom Data 1 ~ 24 */
		for (int i = 0; i < CHASSIS_CUSTOM_DATA_MAX && offset < chassis_offset + area_len;
		     i++) {
			uint8_t type_length = fru_data[offset++];
			if (type_length == 0xC1) {
				printk("Chassis Custom Data %d is empty \n", i + 1);
				break;
			}
			int len = type_length & 0x3F;
			decode_field(&fru_data[offset], len,
				     plat_fru_info->chassis.chassis_custom_data[i],
				     sizeof(plat_fru_info->chassis.chassis_custom_data[i]));
			offset += len;
		}
	} else {
		printk("Invalid chassis offset\n");
		return false;
	}

	/* --------------------- Parse Board Area --------------------- */
	if (board_offset + 6 < FRU_DATA_SIZE) {
		uint16_t area_len = fru_data[board_offset + 1] * 8;
		plat_fru_info->board.language = fru_data[board_offset + 2];
		parse_board_mfg_date(&fru_data[board_offset + 3],
				     plat_fru_info->board.board_mfg_date,
				     sizeof(plat_fru_info->board.board_mfg_date));

		int offset = board_offset + 6;
		char buffer[32];
		/* Field 1: Board Mfg */
		if (offset < board_offset + area_len - 1) {
			uint8_t type_length = fru_data[offset++];
			if (type_length != 0xC1) {
				int len = type_length & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(plat_fru_info->board.board_mfg, buffer,
					sizeof(plat_fru_info->board.board_mfg));
				offset += len;
			}
		}
		/* Field 2: Board Product */
		if (offset < board_offset + area_len - 1) {
			uint8_t type_length = fru_data[offset++];
			if (type_length != 0xC1) {
				int len = type_length & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(plat_fru_info->board.board_product, buffer,
					sizeof(plat_fru_info->board.board_product));
				offset += len;
			}
		}
		/* Field 3: Board Serial */
		if (offset < board_offset + area_len - 1) {
			uint8_t type_length = fru_data[offset++];
			if (type_length != 0xC1) {
				int len = type_length & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(plat_fru_info->board.board_serial, buffer,
					sizeof(plat_fru_info->board.board_serial));
				offset += len;
			}
		}
		/* Field 4: Board Part Number */
		if (offset < board_offset + area_len - 1) {
			uint8_t type_length = fru_data[offset++];
			if (type_length != 0xC1) {
				int len = type_length & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(plat_fru_info->board.board_part_number, buffer,
					sizeof(plat_fru_info->board.board_part_number));
				offset += len;
			}
		}
		/* Field 5: Board FRU ID */
		if (offset < board_offset + area_len - 1) {
			uint8_t type_length = fru_data[offset++];
			if (type_length != 0xC1) {
				int len = type_length & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(plat_fru_info->board.board_fru_id, buffer,
					sizeof(plat_fru_info->board.board_fru_id));
				offset += len;
			}
		}
		/* Fields 6 ~ 9: Board Custom Data 1 ~ 10 */
		for (int i = 0; i < BOARD_CUSTOM_DATA_MAX && offset < board_offset + area_len - 1;
		     i++) {
			uint8_t type_length = fru_data[offset++];
			if (type_length == 0xC1)
				break;
			int len = type_length & 0x3F;
			decode_field(&fru_data[offset], len,
				     plat_fru_info->board.board_custom_data[i],
				     sizeof(plat_fru_info->board.board_custom_data[i]));
			offset += len;
		}
	} else {
		printk("Invalid board offset\n");
		return false;
	}

	/* --------------------- Parse Product Area --------------------- */
	if (product_offset + 3 < FRU_DATA_SIZE) {
		uint16_t area_len = fru_data[product_offset + 1] * 8;
		plat_fru_info->product.language = fru_data[product_offset + 2];
		int offset = product_offset + 3;
		char buffer[32];
		/* Field 1: Product Manufacturer */
		if (offset < product_offset + area_len - 1) {
			uint8_t type_length = fru_data[offset++];
			if (type_length != 0xC1) {
				int len = type_length & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(plat_fru_info->product.product_manufacturer, buffer,
					sizeof(plat_fru_info->product.product_manufacturer));
				offset += len;
			}
		}
		/* Field 2: Product Name */
		if (offset < product_offset + area_len - 1) {
			uint8_t type_length = fru_data[offset++];
			if (type_length != 0xC1) {
				int len = type_length & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(plat_fru_info->product.product_name, buffer,
					sizeof(plat_fru_info->product.product_name));
				offset += len;
			}
		}
		/* Field 3: Product Part Number */
		if (offset < product_offset + area_len - 1) {
			uint8_t type_length = fru_data[offset++];
			if (type_length != 0xC1) {
				int len = type_length & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(plat_fru_info->product.product_part_number, buffer,
					sizeof(plat_fru_info->product.product_part_number));
				offset += len;
			}
		}
		/* Field 4: Product Version */
		if (offset < product_offset + area_len - 1) {
			uint8_t type_length = fru_data[offset++];
			if (type_length != 0xC1) {
				int len = type_length & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(plat_fru_info->product.product_version, buffer,
					sizeof(plat_fru_info->product.product_version));
				offset += len;
			}
		}
		/* Field 5: Product Serial */
		if (offset < product_offset + area_len - 1) {
			uint8_t type_length = fru_data[offset++];
			if (type_length != 0xC1) {
				int len = type_length & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(plat_fru_info->product.product_serial, buffer,
					sizeof(plat_fru_info->product.product_serial));
				offset += len;
			}
		}
		/* Field 6: Product Asset Tag */
		if (offset < product_offset + area_len - 1) {
			uint8_t type_length = fru_data[offset++];
			if (type_length != 0xC1) {
				int len = type_length & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(plat_fru_info->product.product_asset_tag, buffer,
					sizeof(plat_fru_info->product.product_asset_tag));
				offset += len;
			}
		}
		/* Field 7: Product FRU ID */
		if (offset < product_offset + area_len - 1) {
			uint8_t type_length = fru_data[offset++];
			if (type_length != 0xC1) {
				int len = type_length & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(plat_fru_info->product.product_fru_id, buffer,
					sizeof(plat_fru_info->product.product_fru_id));
				offset += len;
			}
		}
		/* Fields 8 ~ 15: Product Custom Data 1 ~ 10 */
		for (int i = 0;
		     i < PRODUCT_CUSTOM_DATA_MAX && offset < product_offset + area_len - 1; i++) {
			uint8_t type_length = fru_data[offset++];
			if (type_length == 0xC1)
				break;
			int len = type_length & 0x3F;
			decode_field(&fru_data[offset], len,
				     plat_fru_info->product.product_custom_data[i],
				     sizeof(plat_fru_info->product.product_custom_data[i]));
			offset += len;
		}
	} else {
		printk("Invalid product offset\n");
		return false;
	}

	/* FRU information has been successfully parsed and stored in plat_fru_info */
	return true;
}

void print_fru_info(uint8_t board_fru_id)
{
	if (!get_fru_info(board_fru_id)) {
		printk("get fru info failed \n");
		return;
	};

	if (!plat_fru_info) {
		printk("FRU info not initialized\n ");
		return;
	}

	printk("show fru data\n");

	/* Print Chassis Info */
	printk("Chassis Info: \n");
	printk("  Chassis Type: %d\n", plat_fru_info->chassis.chassis_type);
	printk("  Chassis Part Number: %s\n", plat_fru_info->chassis.chassis_part_number);
	printk("  Chassis Serial Number: %s\n", plat_fru_info->chassis.chassis_serial_number);
	for (int i = 0; i < CHASSIS_CUSTOM_DATA_MAX; i++) {
		if (strlen(plat_fru_info->chassis.chassis_custom_data[i]) > 0) {
			printk("  Chassis Custom Data %d: %s\n", i + 1,
			       plat_fru_info->chassis.chassis_custom_data[i]);
		}
	}
	printk("\n");

	/* Print Board Info */
	printk("Board Info: \n");
	printk("  Language: %d\n", plat_fru_info->board.language);
	printk("  Board Mfg Date: %s\n", plat_fru_info->board.board_mfg_date);
	printk("  Board Mfg: %s\n", plat_fru_info->board.board_mfg);
	printk("  Board Product: %s\n", plat_fru_info->board.board_product);
	printk("  Board Serial: %s\n", plat_fru_info->board.board_serial);
	printk("  Board Part Number: %s\n", plat_fru_info->board.board_part_number);
	printk("  Board FRU ID: %s\n", plat_fru_info->board.board_fru_id);
	for (int i = 0; i < BOARD_CUSTOM_DATA_MAX; i++) {
		if (strlen(plat_fru_info->board.board_custom_data[i]) > 0) {
			printk("  Board Custom Data %d: %s\n", i + 1,
			       plat_fru_info->board.board_custom_data[i]);
		}
	}
	printk("\n");

	/* Print Product Info */
	printk("Product Info: \n");
	printk("  Language: %d\n", plat_fru_info->product.language);
	printk("  Product Manufacturer: %s\n", plat_fru_info->product.product_manufacturer);
	printk("  Product Name: %s\n", plat_fru_info->product.product_name);
	printk("  Product Part Number: %s\n", plat_fru_info->product.product_part_number);
	printk("  Product Version: %s\n", plat_fru_info->product.product_version);
	printk("  Product Serial: %s\n", plat_fru_info->product.product_serial);
	printk("  Product Asset Tag: %s\n", plat_fru_info->product.product_asset_tag);
	printk("  Product FRU ID: %s\n", plat_fru_info->product.product_fru_id);
	for (int i = 0; i < PRODUCT_CUSTOM_DATA_MAX; i++) {
		if (strlen(plat_fru_info->product.product_custom_data[i]) > 0) {
			printk("  Product Custom Data %d: %s\n", i + 1,
			       plat_fru_info->product.product_custom_data[i]);
		}
	}
	printk("\n");
	free(plat_fru_info);
}

FRU_INFO *get_single_fru_info(uint8_t board_fru_id)
{
	if (!get_fru_info(board_fru_id)) {
		printk("get fru info failed \n");
		return 0;
	};

	if (!plat_fru_info) {
		printk("FRU info not initialized\n ");
		return 0;
	}
	return plat_fru_info;
}