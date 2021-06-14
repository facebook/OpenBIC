
#ifndef __FLASH_API__
#define __FLASH_API__

#include "flash.h"
#include "objects.h"

uint32_t aspeed_flash_probe(spi_t *spi, uint32_t cs);
uint32_t aspeed_spi_init(spi_t *spi);
void aspeed_spi_start_user(flash_t *flash);
void aspeed_spi_stop_user(flash_t *flash);
void aspeed_spi_send_cmd_addr(flash_t *flash, uint8_t cmd, uint32_t addr);
void aspeed_spi_fill_safs_cmd(spi_t *spi, flash_t flash);
void aspeed_spi_timing_calibration(flash_t *flash);

void aspeed_spi_flash_read_reg(flash_t *flash, uint8_t opcode, uint8_t *buf,
			uint32_t len);
void aspeed_spi_flash_write_reg(flash_t *flash, uint8_t opcode, uint8_t *buf,
			uint32_t len);
void aspeed_spi_flash_read(flash_t *flash, uint32_t from,
			uint32_t len, uint8_t *read_buf);
void aspeed_spi_flash_write_internal(flash_t *flash, uint32_t to,
			uint32_t len, uint8_t *write_buf);
void aspeed_spi_flash_write(flash_t *flash, uint32_t to,
			uint32_t len, uint8_t *write_buf);
void aspeed_spi_flash_erase(flash_t *flash, uint32_t offset);

uint32_t aspeed_spi_driver_caps(void);
uint32_t aspeed_spi_decode_range_reinit(flash_t *flash);
uint32_t aspeed_spi_flash_info_deploy(flash_t *flash);
void aspeed_spi_set_4byte_mode(flash_t *flash);
uint32_t aspeed_read_from_flash(spi_t spi, uint32_t cs,
			uint8_t *read_buf, uint32_t flash_offset,
			uint32_t read_sz);
uint32_t aspeed_update_flash(spi_t spi, uint32_t cs,
			uint8_t *update_buf, uint32_t flash_offset, uint32_t len,
			bool show_progress);

uint32_t ast2600_segment_addr_start(uint32_t reg_val);
uint32_t ast2600_segment_addr_end(uint32_t reg_val);
uint32_t ast2600_segment_addr_val(uint32_t start, uint32_t end);
uint32_t ast1030_fmc_segment_addr_start(uint32_t reg_val);
uint32_t ast1030_fmc_segment_addr_end(uint32_t reg_val);
uint32_t ast1030_fmc_segment_addr_val(uint32_t start, uint32_t end);
uint32_t ast1030_spi_segment_addr_start(uint32_t reg_val);
uint32_t ast1030_spi_segment_addr_end(uint32_t reg_val);
uint32_t ast1030_spi_segment_addr_val(uint32_t start, uint32_t end);

#endif

