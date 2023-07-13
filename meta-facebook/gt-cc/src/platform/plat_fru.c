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

	memcpy(part_number_p, &board_data[buf_ofs], 4);
	return true;
}

uint8_t check_nic_type_by_fru()
{
	uint8_t config = 0x00; // 0: default is CX7, 1: unknown or FRU read failed

	for (uint8_t i = 0; i < NIC_MAX_NUMBER; i++) {
		if (gpio_get(nic_prsnt_pin[i]))
			continue;

		uint8_t part_number_data[4];
		bool ret = find_nic_product_name(NIC0_FRU_ID + i, part_number_data);

		if (ret && !strncmp(part_number_data, "MCX7", 4)) {
			LOG_INF("NIC%d is CX7 IB NIC", i);
			return NIC_CONFIG_IB_CX7;
		} else if (ret && !strncmp(part_number_data, "CX7", 3)) {
			LOG_INF("NIC%d is CX7 NIC", i);
			continue;
		} else {
			LOG_WRN("NIC%d is UNKNOWN NIC", i);
			config |= 1 << i;
		}
	}

	return (config ? NIC_CONFIG_UNKNOWN : NIC_CONFIG_CX7);
}

void pal_load_fru_config(void)
{
	memcpy(&fru_config, &plat_fru_config, sizeof(plat_fru_config));
}
