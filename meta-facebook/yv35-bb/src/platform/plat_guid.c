#include <stdio.h>
#include "guid.h"
#include "plat_guid.h"
#include "fru.h"

#define BB_GUID_PORT 0x01
#define BB_GUID_ADDR (0xA2 >> 1)

const EEPROM_CFG guid_config[] = {
	{
		NV_ATMEL_24C128,
		BB_GUID_ID,
		BB_GUID_PORT,
		BB_GUID_ADDR,
		GUID_ACCESS_BYTE,
		GUID_START,
		GUID_SIZE,
	},
};
