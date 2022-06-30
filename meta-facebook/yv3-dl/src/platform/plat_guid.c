#include <stdio.h>
#include "guid.h"
#include "plat_guid.h"
#include "fru.h"
#include <string.h>

#define MB_GUID_BUS 0x05
#define MB_GUID_ADDR 0x54

const EEPROM_CFG guid_config[] = {
	{
		NV_ATMEL_24C128,
		MB_SYS_GUID_ID,
		MB_GUID_BUS,
		MB_GUID_ADDR,
		GUID_ACCESS_BYTE,
		GUID_START,
		GUID_SIZE,
	},
};

uint8_t get_system_guid(uint16_t *data_len, uint8_t *data)
{
	EEPROM_ENTRY guid_entry;

	guid_entry.offset = 0;
	guid_entry.data_len = 16;
	guid_entry.config.dev_id = MB_SYS_GUID_ID;

	uint8_t status = GUID_read(&guid_entry);
	if (status == GUID_READ_SUCCESS) {
		*data_len = guid_entry.data_len;
		memcpy(data, &guid_entry.data, guid_entry.data_len);
	}

	return status;
}

uint8_t set_system_guid(uint16_t *data_len, uint8_t *data)
{
	EEPROM_ENTRY guid_entry;

	guid_entry.offset = 0;
	guid_entry.data_len = *data_len;
	guid_entry.config.dev_id = MB_SYS_GUID_ID;
	memcpy(&guid_entry.data[0], data, guid_entry.data_len);
	return GUID_write(&guid_entry);
}
