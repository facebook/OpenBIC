#ifndef UTIL_SPI_H
#define UTIL_SPI_H

#include <zephyr.h>
#include <stdbool.h>
#include <stdint.h>

#define NUM_SPI_DEV 3
#define sector_sz_64k 0x10000
#define sector_sz_32k 0x08000
#define sector_sz_16k 0x04000
#define sector_sz_4k 0x01000
#define sector_sz_1k 0x00400

#define FW_UPDATE_DEBUG 0

enum {
	devspi_fmc_cs0,
	devspi_fmc_cs1,
	devspi_spi1_cs0,
	devspi_spi1_cs1,
	devspi_spi2_cs0,
	devspi_spi2_cs1,
};

uint8_t fw_update(uint32_t offset, uint16_t msg_len, uint8_t *msg_buf, bool update_en,
		  uint8_t spi_bus);

int pal_get_bios_flash_pos();

enum {
	fwupdate_success,
	fwupdate_out_of_heap,
	fwupdate_over_length,
	fwupdate_repeated_updated,
	fwupdate_update_fail,
	fwupdate_error_offset,
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
