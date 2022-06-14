#ifndef UTIL_SPI_H
#define UTIL_SPI_H

#include <zephyr.h>
#include <stdbool.h>
#include <stdint.h>

#define NUM_SPI_DEV 3
#define SECTOR_SZ_64K 0x10000
#define SECTOR_SZ_32K 0x08000
#define SECTOR_SZ_16K 0x04000
#define SECTOR_SZ_4K 0x01000
#define SECTOR_SZ_1K 0x00400

#define FW_UPDATE_DEBUG 0

enum DEVICE_POSITIONS {
	DEVSPI_FMC_CS0,
	DEVSPI_FMC_CS1,
	DEVSPI_SPI1_CS0,
	DEVSPI_SPI1_CS1,
	DEVSPI_SPI2_CS0,
	DEVSPI_SPI2_CS1,
};

uint8_t fw_update(uint32_t offset, uint16_t msg_len, uint8_t *msg_buf, bool sector_end,
		  uint8_t flash_position);
uint8_t fw_update_cxl(uint8_t flash_position);

int pal_get_bios_flash_position();
bool pal_switch_bios_spi_mux(int gpio_status);
int pal_get_cxl_flash_position();
bool pal_switch_cxl_spi_mux();

enum FIRMWARE_UPDATE_RETURN_CODE {
	FWUPDATE_SUCCESS,
	FWUPDATE_OUT_OF_HEAP,
	FWUPDATE_OVER_LENGTH,
	FWUPDATE_REPEATED_UPDATED,
	FWUPDATE_UPDATE_FAIL,
	FWUPDATE_ERROR_OFFSET,
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
