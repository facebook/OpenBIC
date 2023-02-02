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

#ifndef UTIL_SPI_H
#define UTIL_SPI_H

#include <zephyr.h>
#include <stdbool.h>
#include <stdint.h>
#include <drivers/spi_nor.h>

#define NUM_SPI_DEV 3
#define SECTOR_SZ_64K 0x10000
#define SECTOR_SZ_32K 0x08000
#define SECTOR_SZ_16K 0x04000
#define SECTOR_SZ_4K 0x01000
#define SECTOR_SZ_1K 0x00400

#define SHA256_DIGEST_SIZE 32

#define SECTOR_END_FLAG BIT(7)
#define NO_RESET_FLAG BIT(0)

enum DEVICE_POSITIONS {
	DEVSPI_FMC_CS0,
	DEVSPI_FMC_CS1,
	DEVSPI_SPI1_CS0,
	DEVSPI_SPI1_CS1,
	DEVSPI_SPI2_CS0,
	DEVSPI_SPI2_CS1,
};

uint8_t fw_update(uint32_t offset, uint16_t msg_len, uint8_t *msg_buf, uint8_t flag,
		  uint8_t flash_position);
int read_fw_image(uint32_t offset, uint8_t msg_len, uint8_t *msg_buf, uint8_t flash_position);
uint8_t fw_update_cxl(uint32_t offset, uint16_t msg_len, uint8_t *msg_buf, bool sector_end);

uint8_t get_fw_sha256(uint8_t *msg_buf, uint32_t offset, uint32_t length, uint8_t flash_position);

int pal_get_bios_flash_position();
int pal_get_prot_flash_position();
bool pal_switch_bios_spi_mux(int gpio_status);
int pal_get_cxl_flash_position();
int do_update(const struct device *flash_device, off_t offset, uint8_t *buf, size_t len);

enum FIRMWARE_UPDATE_RETURN_CODE {
	FWUPDATE_SUCCESS,
	FWUPDATE_OUT_OF_HEAP,
	FWUPDATE_OVER_LENGTH,
	FWUPDATE_REPEATED_UPDATED,
	FWUPDATE_UPDATE_FAIL,
	FWUPDATE_ERROR_OFFSET,
	FWUPDATE_NOT_SUPPORT,
};

#if DT_NODE_HAS_STATUS(DT_PATH(soc, spi_7e620000), okay)
#define SPI_fmc
#endif

#if DT_NODE_HAS_STATUS(DT_PATH(soc, spi_7e630000), okay)
#define SPI_spi1
#endif

#if DT_NODE_HAS_STATUS(DT_PATH(soc, spi_7e640000), okay)
#define SPI_spi2
#endif

#endif
