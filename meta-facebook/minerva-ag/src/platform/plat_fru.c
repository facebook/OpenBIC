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
#include <libutil.h>
#include <stdlib.h>
#include <logging/log.h>
#include "fru.h"
#include "plat_fru.h"
#include <time.h>

LOG_MODULE_REGISTER(plat_fru);

#define AEGIS_FRU_START 0x0000
#define AEGIS_FRU_SIZE 0x0400 // size 1KB

#define AEGIS_CPLD_FRU_START 0x0000
#define AEGIS_CPLD_FRU_SIZE 0x0400

const EEPROM_CFG plat_fru_config[] = {
	{
		ROHM_BR24G512,
		LOG_EEPROM_ID,
		I2C_BUS12,
		LOG_EEPROM_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	{
		ROHM_BR24G512, // using CPLD UFM
		CPLD_EEPROM_ID,
		I2C_BUS5,
		CPLD_EEPROM_ADDR,
		FRU_DEV_ACCESS_BYTE,
		AEGIS_CPLD_FRU_START,
		AEGIS_CPLD_FRU_SIZE,
	},
};

void pal_load_fru_config(void)
{
	memcpy(&fru_config, &plat_fru_config, sizeof(plat_fru_config));
}

bool plat_eeprom_write(uint32_t offset, uint8_t *data, uint16_t data_len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	LOG_DBG("plat_eeprom_write, offset: 0x%x, data_len: %d", offset, data_len);

	EEPROM_ENTRY entry;

	entry.offset = offset;
	entry.data_len = data_len;

	uint8_t fru_index = 0;
	if (!find_FRU_ID(LOG_EEPROM_ID, &fru_index)) {
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
	if (!find_FRU_ID(LOG_EEPROM_ID, &fru_index)) {
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

bool plat_cpld_fru_read(uint32_t offset, uint8_t *data, uint16_t data_len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);
	EEPROM_ENTRY entry;

	entry.offset = offset;
	entry.data_len = data_len;

	uint8_t fru_index = 0;
	if (!find_FRU_ID(CPLD_EEPROM_ID, &fru_index)) {
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

bool plat_get_cpld_fru_data(uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	const uint32_t total_size = AEGIS_CPLD_FRU_SIZE; // 0x0400
	const uint32_t chunk_size = 0x80; // 0x80 bytes per read
	uint32_t offset = 0;

	while (offset < total_size) {
		if (!plat_cpld_fru_read(AEGIS_CPLD_FRU_START + offset, data + offset, chunk_size)) {
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
		LOG_ERR("Failed to allocate memory for FRU_INFO");
		return NULL;
	}
	memset(info, 0, sizeof(FRU_INFO));
	return info;
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

bool init_fru_info(void)
{
	plat_fru_info = create_fru_info();
	if (!plat_fru_info) {
		return false;
	}

	uint8_t fru_data[AEGIS_CPLD_FRU_SIZE];

	/* Read FRU data from CPLD EEPROM */
	if (!plat_get_cpld_fru_data(fru_data)) {
		LOG_ERR("Failed to read CPLD FRU data");
		return false;
	}

	/* Parse common header (first 8 bytes) */
	uint8_t common_header[8];
	memcpy(common_header, fru_data, 8);
	uint16_t chassis_offset = common_header[2] * 8;
	uint16_t board_offset = common_header[3] * 8;
	uint16_t product_offset = common_header[4] * 8;

	/* --------------------- Parse Chassis Area --------------------- */
	if (chassis_offset + 3 < AEGIS_CPLD_FRU_SIZE) {
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
				LOG_INF("Chassis Custom Data %d is empty", i + 1);
				break;
			}
			int len = type_length & 0x3F;
			decode_field(&fru_data[offset], len,
				     plat_fru_info->chassis.chassis_custom_data[i],
				     sizeof(plat_fru_info->chassis.chassis_custom_data[i]));
			offset += len;
		}
	} else {
		LOG_ERR("Invalid chassis offset");
		return false;
	}

	/* --------------------- Parse Board Area --------------------- */
	if (board_offset + 6 < AEGIS_CPLD_FRU_SIZE) {
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
		/* Fields 6 ~ 9: Board Custom Data 1 ~ 4 */
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
		LOG_ERR("Invalid board offset");
		return false;
	}

	/* --------------------- Parse Product Area --------------------- */
	if (product_offset + 3 < AEGIS_CPLD_FRU_SIZE) {
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
		/* Fields 8 ~ 15: Product Custom Data 1 ~ 8 */
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
		LOG_ERR("Invalid product offset");
		return false;
	}

	/* FRU information has been successfully parsed and stored in plat_fru_info */
	return true;
}

void print_fru_info(void)
{
	if (!plat_fru_info) {
		LOG_ERR("FRU info not initialized");
		return;
	}

	printf("\n");

	/* Print Chassis Info */
	printf("Chassis Info: \n");
	printf("  Chassis Type: %d\n", plat_fru_info->chassis.chassis_type);
	printf("  Chassis Part Number: %s\n", plat_fru_info->chassis.chassis_part_number);
	printf("  Chassis Serial Number: %s\n", plat_fru_info->chassis.chassis_serial_number);
	for (int i = 0; i < CHASSIS_CUSTOM_DATA_MAX; i++) {
		if (strlen(plat_fru_info->chassis.chassis_custom_data[i]) > 0) {
			printf("  Chassis Custom Data %d: %s\n", i + 1,
			       plat_fru_info->chassis.chassis_custom_data[i]);
		}
	}
	printf("\n");

	/* Print Board Info */
	printf("Board Info: \n");
	printf("  Language: %d\n", plat_fru_info->board.language);
	printf("  Board Mfg Date: %s\n", plat_fru_info->board.board_mfg_date);
	printf("  Board Mfg: %s\n", plat_fru_info->board.board_mfg);
	printf("  Board Product: %s\n", plat_fru_info->board.board_product);
	printf("  Board Serial: %s\n", plat_fru_info->board.board_serial);
	printf("  Board Part Number: %s\n", plat_fru_info->board.board_part_number);
	printf("  Board FRU ID: %s\n", plat_fru_info->board.board_fru_id);
	for (int i = 0; i < BOARD_CUSTOM_DATA_MAX; i++) {
		if (strlen(plat_fru_info->board.board_custom_data[i]) > 0) {
			printf("  Board Custom Data %d: %s\n", i + 1,
			       plat_fru_info->board.board_custom_data[i]);
		}
	}
	printf("\n");

	/* Print Product Info */
	printf("Product Info: \n");
	printf("  Language: %d\n", plat_fru_info->product.language);
	printf("  Product Manufacturer: %s\n", plat_fru_info->product.product_manufacturer);
	printf("  Product Name: %s\n", plat_fru_info->product.product_name);
	printf("  Product Part Number: %s\n", plat_fru_info->product.product_part_number);
	printf("  Product Version: %s\n", plat_fru_info->product.product_version);
	printf("  Product Serial: %s\n", plat_fru_info->product.product_serial);
	printf("  Product Asset Tag: %s\n", plat_fru_info->product.product_asset_tag);
	printf("  Product FRU ID: %s\n", plat_fru_info->product.product_fru_id);
	for (int i = 0; i < PRODUCT_CUSTOM_DATA_MAX; i++) {
		if (strlen(plat_fru_info->product.product_custom_data[i]) > 0) {
			printf("  Product Custom Data %d: %s\n", i + 1,
			       plat_fru_info->product.product_custom_data[i]);
		}
	}
	printf("\n");
}

FRU_INFO *get_fru_info(void)
{
	return plat_fru_info;
}
