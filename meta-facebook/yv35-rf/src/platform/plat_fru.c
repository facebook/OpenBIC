#include <string.h>
#include "fru.h"
#include "plat_fru.h"

#define RF_FRU_PORT 0x02
#define RF_FRU_ADDR (0xA8 >> 1)

const EEPROM_CFG plat_fru_config[] = {
	{
		NV_ATMEL_24C128,
		RF_FRU_ID,
		RF_FRU_PORT,
		RF_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
};

void pal_load_fru_config(void)
{
	memcpy(&fru_config, &plat_fru_config, sizeof(plat_fru_config));
}
