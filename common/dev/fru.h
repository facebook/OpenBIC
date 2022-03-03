#ifndef FRU_H
#define FRU_H

#include "eeprom.h"
#define FRU_CFG_NUM 5

enum {
	NV_ATMEL_24C02,
	NV_ATMEL_24C64,
	NV_ATMEL_24C128,
};

enum {
	FRU_WRITE_SUCCESS,
	FRU_READ_SUCCESS,
	FRU_INVALID_ID,
	FRU_OUT_OF_RANGE,
	FRU_FAIL_TO_ACCESS,
};

enum {
	FRU_DEV_ACCESS_BYTE,
	FRU_DEV_ACCESS_WORD,
};

extern EEPROM_CFG fru_config[];

uint8_t get_FRU_access(uint8_t FRUID);
uint16_t find_FRU_size(uint8_t FRUID);
uint8_t FRU_read(EEPROM_ENTRY *entry);
uint8_t FRU_write(EEPROM_ENTRY *entry);
void FRU_init(void);

#endif
