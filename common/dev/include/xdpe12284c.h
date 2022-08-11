#ifndef XDPE12284C_H
#define XDPE12284C_H

bool xdpe12284c_get_checksum(uint8_t bus, uint8_t target_addr, uint8_t *checksum);
bool xdpe12284c_get_remaining_write(uint8_t bus, uint8_t target_addr, uint8_t *remain_write);

enum INFINEON_PAGE {
	INFINEON_STATUS_PAGE = 0x60,
};

#endif
