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
#include <logging/log.h>
#include <sys/crc.h>
#include "fru.h"
#include "hal_gpio.h"
#include "power_status.h"
#include "plat_fru.h"
#include "plat_gpio.h"
#include "libutil.h"

LOG_MODULE_REGISTER(plat_fru);

#define BSD_IMG_SIZE_OFST 40
#define MAX_BSD_IMG_SIZE 1024

const EEPROM_CFG plat_fru_config[] = {
	{
		NV_ATMEL_24C128,
		MB_FRU_ID,
		MB_FRU_PORT,
		MB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	{
		NV_ATMEL_24C128,
		DPV2_FRU_ID,
		DPV2_FRU_PORT,
		DPV2_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
};

// BIOS version is stored in MB EEPROM, but the location of EEPROM is different from fru information
const EEPROM_CFG plat_bios_version_area_config = {
	NV_ATMEL_24C128,
	MB_FRU_ID,
	MB_FRU_PORT,
	MB_FRU_ADDR,
	FRU_DEV_ACCESS_BYTE,
	BIOS_FW_VERSION_START,
	BIOS_FW_VERSION_MAX_SIZE,
};

/* CPU eeprom - BSD image */
const EEPROM_CFG plat_bsd_area_config = {
	NV_ATMEL_24C128,     MB_FRU_ID,	      MB_CPU_FRU_PORT,	MB_CPU_FRU_ADDR,
	FRU_DEV_ACCESS_BYTE, BSD_IMAGE_START, MAX_BSD_IMG_SIZE,
};

/* MB eeprom - BSD version(crc32) */
const EEPROM_CFG plat_bsd_version_area_config = {
	NV_ATMEL_24C128,     MB_FRU_ID,		MB_FRU_PORT,	      MB_FRU_ADDR,
	FRU_DEV_ACCESS_BYTE, BSD_VERSION_START, BSD_VERSION_MAX_SIZE,
};

void pal_load_fru_config(void)
{
	memcpy(fru_config, plat_fru_config, sizeof(plat_fru_config));
}

int set_bios_version(EEPROM_ENTRY *entry, uint8_t block_index)
{
	CHECK_NULL_ARG_WITH_RETURN(entry, -1);

	if (block_index >= BIOS_FW_VERSION_BLOCK_NUM)
		return -1;

	bool ret = false;
	entry->config = plat_bios_version_area_config;
	if (block_index == 1)
		entry->config.start_offset += BIOS_FW_VERSION_SECOND_BLOCK_OFFSET;
	entry->data_len = BIOS_FW_VERSION_BLOCK_MAX_SIZE;

	ret = eeprom_write(entry);
	if (ret == false) {
		LOG_ERR("eeprom_write fail");
		return -1;
	}

	return 0;
}

int get_bios_version(EEPROM_ENTRY *entry, uint8_t block_index)
{
	CHECK_NULL_ARG_WITH_RETURN(entry, -1);

	if (block_index >= BIOS_FW_VERSION_BLOCK_NUM)
		return -1;

	bool ret = false;
	entry->config = plat_bios_version_area_config;
	if (block_index == 1)
		entry->config.start_offset += BIOS_FW_VERSION_SECOND_BLOCK_OFFSET;
	entry->data_len = BIOS_FW_VERSION_BLOCK_MAX_SIZE;

	ret = eeprom_read(entry);
	if (ret == false) {
		LOG_ERR("eeprom_read fail");
		return -1;
	}

	return 0;
}

bool write_bsd_version()
{
	bool ret = false;

	/* Access CPU eeprom only if CPU off */
	if (get_DC_status() == true)
		goto exit;

	gpio_set(BMC_GPIOT6_SPI0_PROGRAM_SEL, GPIO_HIGH);
	k_msleep(100);

	uint16_t bsd_img_size = 0;
	uint8_t bsd_img_buff[MAX_BSD_IMG_SIZE] = { 0 };

	uint8_t retry = 5;
	I2C_MSG msg = { 0 };
	msg.bus = MB_CPU_FRU_PORT;
	msg.target_addr = MB_CPU_FRU_ADDR;
	msg.tx_len = 2;
	msg.rx_len = 2;
	msg.data[0] = (BSD_IMG_SIZE_OFST >> 8) & 0xFF;
	msg.data[1] = BSD_IMG_SIZE_OFST & 0xFF;

	if (i2c_master_read(&msg, retry)) {
		LOG_WRN("I2C read failed.");
		goto exit;
	}

	bsd_img_size = msg.data[0] | (msg.data[1] << 8);
	if (!bsd_img_size || (bsd_img_size > MAX_BSD_IMG_SIZE)) {
		LOG_ERR("Get invalid bsd image size.");
		goto exit;
	}

	EEPROM_ENTRY entry = { 0 };
	entry.config = plat_bsd_area_config;
	entry.config.start_offset = 0x0000;
	entry.data_len = EEPROM_WRITE_SIZE;

	while (entry.config.start_offset + EEPROM_WRITE_SIZE <= MAX_BSD_IMG_SIZE) {
		ret = eeprom_read(&entry);
		if (ret == false) {
			LOG_ERR("eeprom_read fail");
			goto exit;
		}

		if (entry.config.start_offset + EEPROM_WRITE_SIZE > bsd_img_size) {
			memcpy(&bsd_img_buff[entry.config.start_offset], entry.data,
			       bsd_img_size - entry.config.start_offset);
			break;
		} else {
			memcpy(&bsd_img_buff[entry.config.start_offset], entry.data,
			       EEPROM_WRITE_SIZE);
			entry.config.start_offset += EEPROM_WRITE_SIZE;
			entry.data_len = EEPROM_WRITE_SIZE;
		}
	}

	LOG_HEXDUMP_DBG(bsd_img_buff, bsd_img_size, "BSD image:");

	uint32_t digest = crc32_ieee(bsd_img_buff, bsd_img_size);
	LOG_INF("BSD crc32: 0x%x", digest);

	entry.config = plat_bsd_version_area_config;
	entry.data_len = sizeof(digest);
	memcpy(entry.data, &digest, entry.data_len);

	ret = eeprom_write(&entry);
	if (ret == false) {
		LOG_ERR("eeprom_write fail");
		goto exit;
	}

	ret = true;
exit:
	gpio_set(BMC_GPIOT6_SPI0_PROGRAM_SEL, GPIO_LOW);

	return ret;
}

uint32_t read_bsd_version()
{
	EEPROM_ENTRY entry = { 0 };
	entry.config = plat_bsd_version_area_config;
	entry.data_len = sizeof(uint32_t);

	if (eeprom_read(&entry) == false) {
		LOG_ERR("eeprom_read fail");
		return 0;
	}

	uint32_t bsd_ver = 0;
	for (int i = 0; i < entry.data_len; i++)
		bsd_ver |= (entry.data[i] << (8 * i));

	return bsd_ver;
}
