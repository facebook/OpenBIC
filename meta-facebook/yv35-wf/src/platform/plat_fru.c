#include <stdio.h>
#include <string.h>
#include "plat_fru.h"
#include "fru.h"

const EEPROM_CFG plat_fru_config[] = {
	{
		NV_ATMEL_24C128,
		WF_FRU_ID,
		WF_FRU_PORT,
		WF_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
};

void pal_load_fru_config(void)
{
	memcpy(&fru_config, &plat_fru_config, sizeof(plat_fru_config));
}
