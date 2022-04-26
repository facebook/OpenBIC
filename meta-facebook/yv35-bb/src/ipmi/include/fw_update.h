#ifndef FW_UPDATE_H
#define FW_UPDATE_H

#define fmc_bus 0
#define spi0_bus 0
#define spi1_bus 1
#define spi2_bus 2

#define fmc_cs 0
#define spi0_cs 0
#define spi1_cs 1
#define spi2_cs 2

uint8_t fw_update(uint32_t offset, uint16_t msg_len, uint8_t *msg_buf, bool update_en, uint8_t bus,
		  uint8_t cs);

enum {
	fwupdate_success,
	fwupdate_out_of_heap,
	fwupdate_over_length,
	fwupdate_repeated_updated,
	fwupdate_update_fail,
};

#endif
