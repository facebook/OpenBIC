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
#include "fru.h"
#include "plat_fru.h"
#include "hal_gpio.h"
#include "plat_sensor_table.h"
#include "plat_pldm_monitor.h"

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

uint8_t check_nic_type_by_fru()
{
	uint8_t config = 0x00; // 0: default is CX7, 1: unknown or FRU read failed

	for (uint8_t i = 0; i <= NIC_MAX_NUMBER; i++) {
		if (gpio_get(nic_prsnt_pin[i]))
			continue;

		EEPROM_ENTRY fru_entry;
		fru_entry.config.dev_id = NIC0_FRU_ID + i;
		fru_entry.offset = 0x48;
		fru_entry.data_len = 4;

		uint8_t ret = FRU_read(&fru_entry);

		if ((ret == FRU_READ_SUCCESS) && !strncmp(&fru_entry.data[0], "MCX7", 4)) {
			printf("NIC%d is CX7 IB NIC\n", i);
			return NIC_CONFIG_IB_CX7;
		} else if ((ret == FRU_READ_SUCCESS) && !strncmp(&fru_entry.data[0], "CX7", 3)) {
			printf("NIC%d is CX7 NIC\n", i);
			continue;
		} else {
			printf("NIC%d is UNKNOWN NIC\n", i);
			config |= 1 << i;
		}
	}

	return config ? NIC_CONFIG_UNKNOWN : NIC_CONFIG_CX7;
}

void pal_load_fru_config(void)
{
	memcpy(&fru_config, &plat_fru_config, sizeof(plat_fru_config));
}
