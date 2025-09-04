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
#include <stdio.h>
#include <logging/log.h>
#include "libutil.h"
#include "fru.h"
#include "plat_fru.h"
#include "hal_gpio.h"
#include "plat_sensor_table.h"
#include "plat_pldm_monitor.h"

LOG_MODULE_REGISTER(plat_fru);

extern struct k_mutex i2c_bus6_mutex;

const EEPROM_CFG plat_fru_config[] = {
	{
		NV_ATMEL_24C64,
		SWB_FRU_ID,
		SWB_FRU_PORT,
		SWB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
		/* Because there has mux before eeprom, so mux_present is true to access mux first */
		true,
		SWB_FRU_MUX_ADDR,
		SWB_FRU_MUX_CHAN,
		&i2c_bus6_mutex,
	},
	{
		NV_ATMEL_24C64,
		FIO_FRU_ID,
		FIO_FRU_PORT,
		FIO_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	{
		NV_ATMEL_24C64,
		HSC_MODULE_FRU_ID,
		HSC_MODULE_FRU_PORT,
		HSC_MODULE_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
		/* Because there has mux before eeprom, so mux_present is true to access mux first */
		true,
		HSC_MODULE_FRU_MUX_ADDR,
		HSC_MODULE_FRU_MUX_CHAN,
		&i2c_bus6_mutex,
	},
	{
		NV_ATMEL_24C64,
		NIC0_FRU_ID,
		NIC0_FRU_PORT,
		NIC_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	{
		NV_ATMEL_24C64,
		NIC1_FRU_ID,
		NIC1_FRU_PORT,
		NIC_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	{
		NV_ATMEL_24C64,
		NIC2_FRU_ID,
		NIC2_FRU_PORT,
		NIC_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	{
		NV_ATMEL_24C64,
		NIC3_FRU_ID,
		NIC3_FRU_PORT,
		NIC_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	{
		NV_ATMEL_24C64,
		NIC4_FRU_ID,
		NIC4_FRU_PORT,
		NIC_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	{
		NV_ATMEL_24C64,
		NIC5_FRU_ID,
		NIC5_FRU_PORT,
		NIC_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	{
		NV_ATMEL_24C64,
		NIC6_FRU_ID,
		NIC6_FRU_PORT,
		NIC_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	{
		NV_ATMEL_24C64,
		NIC7_FRU_ID,
		NIC7_FRU_PORT,
		NIC_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
};

static bool find_nic_product_name(uint8_t dev_id, uint8_t *part_number_p)
{
	CHECK_NULL_ARG_WITH_RETURN(part_number_p, false);

	EEPROM_ENTRY fru_entry;
	fru_entry.config.dev_id = dev_id;
	fru_entry.offset = 0x0000;
	fru_entry.data_len = 8;

	if (FRU_read(&fru_entry) != FRU_READ_SUCCESS)
		return false;

	uint16_t board_area_offset;

	fru_entry.offset = board_area_offset = (fru_entry.data[3] * 8);
	fru_entry.data_len = 2;

	if (FRU_read(&fru_entry) != FRU_READ_SUCCESS)
		return false;

	uint16_t board_area_len = fru_entry.data[1] * 8;
	uint8_t board_data[board_area_len];

	uint16_t total_read_byte = 0;
	uint8_t bytes_read = 0;
	uint8_t buf_ofs = 0;

	for (; total_read_byte < board_area_len; total_read_byte += bytes_read) {
		bytes_read = ((total_read_byte + bytes_read) > board_area_len) ?
				     (board_area_len - total_read_byte) :
				     EEPROM_WRITE_SIZE;

		fru_entry.offset = board_area_offset + buf_ofs;
		fru_entry.data_len = bytes_read;

		if (FRU_read(&fru_entry) != FRU_READ_SUCCESS)
			return false;

		memcpy(&board_data[buf_ofs], fru_entry.data, bytes_read);
		buf_ofs += bytes_read;
	}

	buf_ofs = 6; // board manufactor type/length offset

	// Get board part number type/length offset
	for (uint8_t i = 0; i < 3; i++)
		buf_ofs += (board_data[buf_ofs] & 0x1F) + 1;

	// Get board part number offset
	buf_ofs += 1;

	memcpy(part_number_p, &board_data[buf_ofs], 18);
	LOG_HEXDUMP_INF(part_number_p, 18, "NIC product name");
	return true;
}

uint8_t check_nic_type_by_fru()
{
	uint8_t config = 0x00; // 0: default is CX7, 1: unknown or FRU read failed
	bool is_ib_nic = false;

	for (uint8_t i = 0; i < NIC_MAX_NUMBER; i++) {
		if (gpio_get(nic_prsnt_pin[i]))
			continue;

		uint8_t part_number_data[18];
		bool ret = find_nic_product_name(NIC0_FRU_ID + i, part_number_data);

		/* MCX75343AMC-NEAC andCX75343AMC-NEAC_FB are both CX7 IB NIC */
		/* CX71343DAC-WEAF_FB is CX7 NIC */
		if ((ret && !strncmp(part_number_data, "MCX75343AMC-NEAC", 16)) ||
		    (ret && !strncmp(part_number_data, "CX75343AMC-NEAC_FB", 18))) {
			LOG_INF("NIC%d is CX7 IB NIC", i);
			is_ib_nic = true;
			continue;
		} else if (ret && !strncmp(part_number_data, "CX71343DAC-WEAF_FB", 18)) {
			LOG_INF("NIC%d is CX7 NIC", i);
			continue;
		} else {
			LOG_WRN("NIC%d is UNKNOWN NIC", i);
			config |= 1 << i;
		}
	}

	if (is_ib_nic) {
		return NIC_CONFIG_IB_CX7;
	}

	return (config ? NIC_CONFIG_UNKNOWN : NIC_CONFIG_CX7);
}

void pal_load_fru_config(void)
{
	memcpy(&fru_config, &plat_fru_config, sizeof(plat_fru_config));
}

static void decode_field(uint8_t *data, int len, char *buffer, int buffer_size)
{
	CHECK_NULL_ARG(data);

	if (len <= 0 || len >= buffer_size) {
		buffer[0] = '\0';
		return;
	}

	memcpy(buffer, data, len);
	buffer[len] = '\0';
}

static bool get_nic_product_manufacturer(uint8_t fru_id, char *manufacturer,
					 uint8_t manufacturer_size)
{
	CHECK_NULL_ARG_WITH_RETURN(manufacturer, false);

	if (manufacturer_size == 0) {
		LOG_ERR("Invalid manufacturer size: %d", manufacturer_size);
		return false;
	}

	EEPROM_ENTRY fru_entry;
	memset(&fru_entry, 0, sizeof(fru_entry));

	fru_entry.config = fru_config[fru_id];
	fru_entry.offset = 0;
	fru_entry.data_len = 8;

	if (FRU_read(&fru_entry) != FRU_READ_SUCCESS) {
		LOG_ERR("Failed to read FRU header for FRU ID %d", fru_id);
		return false;
	}

	// Check FRU common header format identifier
	if (fru_entry.data[0] != 0x01) {
		LOG_ERR("Invalid FRU format for FRU ID %d", fru_id);
		return false;
	}

	// Get product area offset (5th byte * 8)
	uint16_t product_area_offset = fru_entry.data[4] * 8;
	if (product_area_offset == 0) {
		LOG_ERR("No product area found for FRU ID %d", fru_id);
		return false;
	}

	// Read product area header
	fru_entry.offset = product_area_offset;
	fru_entry.data_len = 3;

	if (FRU_read(&fru_entry) != FRU_READ_SUCCESS) {
		LOG_ERR("Failed to read product area header for FRU ID %d", fru_id);
		return false;
	}

	// Check product area format version
	if (fru_entry.data[0] != 0x01) {
		LOG_ERR("Invalid product area format for FRU ID %d", fru_id);
		return false;
	}

	uint16_t area_len = fru_entry.data[1] * 8;
	if (area_len < 4) {
		LOG_ERR("Invalid product area length for FRU ID %d", fru_id);
		return false;
	}

	// Read the first field (product manufacturer) type/length
	fru_entry.offset = product_area_offset + 3;
	fru_entry.data_len = 1;

	if (FRU_read(&fru_entry) != FRU_READ_SUCCESS) {
		LOG_ERR("Failed to read product manufacturer field for FRU ID %d", fru_id);
		return false;
	}

	uint8_t type_length = fru_entry.data[0];
	if (type_length == 0xC1) {
		// End of fields marker
		LOG_WRN("No product manufacturer field found for FRU ID %d", fru_id);
		return false;
	}

	int field_len = type_length & 0x3F;
	if (field_len <= 0 || field_len >= manufacturer_size) {
		LOG_ERR("Invalid product manufacturer field length for FRU ID %d", fru_id);
		return false;
	}

	// Read the product manufacturer field data
	fru_entry.offset = product_area_offset + 4;
	fru_entry.data_len = field_len;

	if (FRU_read(&fru_entry) != FRU_READ_SUCCESS) {
		LOG_ERR("Failed to read product manufacturer data for FRU ID %d", fru_id);
		return false;
	}

	decode_field(fru_entry.data, field_len, manufacturer, manufacturer_size);

	LOG_DBG("NIC FRU ID %d product manufacturer: %s", fru_id, manufacturer);
	return true;
}

bool get_first_nic_manufacturer(char *manufacturer, uint8_t manufacturer_size)
{
	CHECK_NULL_ARG_WITH_RETURN(manufacturer, false);

	if (manufacturer_size == 0) {
		LOG_ERR("Invalid manufacturer size: %d", manufacturer_size);
		return false;
	}

	bool found_manufacturer = false;
	char temp_manufacturer[32];

	for (uint8_t i = 0; i < NIC_MAX_NUMBER; i++) {
		// Check if NIC is present (presence pin is active low)
		if (gpio_get(nic_prsnt_pin[i])) {
			LOG_INF("NIC%d: Not present", i);
			continue;
		}

		if (get_nic_product_manufacturer(NIC0_FRU_ID + i, temp_manufacturer,
						 sizeof(temp_manufacturer))) {
			LOG_INF("NIC%d: Product Manufacturer = %s", i, temp_manufacturer);

			// If this is the first manufacturer found, save it
			if (!found_manufacturer) {
				strncpy(manufacturer, temp_manufacturer, manufacturer_size - 1);
				manufacturer[manufacturer_size - 1] = '\0';
				found_manufacturer = true;
			}
		} else {
			LOG_INF("NIC%d: Failed to read product manufacturer", i);
		}
	}

	return found_manufacturer;
}
