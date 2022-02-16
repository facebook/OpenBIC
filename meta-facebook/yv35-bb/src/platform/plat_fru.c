#include <stdio.h>
#include "fru.h"
#include "plat_fru.h"
#include "pal.h"
#include "fru.h"

#define BB_FRU_PORT 0x01
#define BB_FRU_ADDR (0xA2 >> 1)

const EEPROM_CFG plat_fru_config[] = {
	{
		NV_ATMEL_24C128,
		BB_FRU_ID,
		BB_FRU_PORT,
		BB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
};

void pal_load_fru_config(void)
{
	memcpy(&fru_config, &plat_fru_config, sizeof(plat_fru_config));
}
