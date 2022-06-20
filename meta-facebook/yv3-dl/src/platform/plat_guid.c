#include <stdio.h>
#include "guid.h"
#include "plat_guid.h"
#include "fru.h"

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
