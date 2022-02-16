#ifndef GUID_H
#define GUID_H

#include "hal_eeprom.h"

enum {
	GUID_WRITE_SUCCESS,
	GUID_READ_SUCCESS,
	GUID_INVALID_ID,
	GUID_OUT_OF_RANGE,
	GUID_FAIL_TO_ACCESS,
};

enum {
	GUID_ACCESS_BYTE,
	GUID_ACCESS_WORD,
};

uint8_t GUID_read(EEPROM_ENTRY *entry);
uint8_t GUID_write(EEPROM_ENTRY *entry);

#endif
