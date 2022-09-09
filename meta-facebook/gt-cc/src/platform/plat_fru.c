#include "fru.h"
#include "plat_fru.h"
#include <string.h>

const EEPROM_CFG plat_fru_config[] = {
	{
		NV_ATMEL_24C64,
		SWB_FRU_ID,
		SWB_FRU_PORT,
		SWB_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
		/* Because there has mux before eeprom, so mux_present is true to access mux first */
		true,
		SWB_FRU_MUX_ADDR,
		SWB_FRU_MUX_CHAN,
	},
	{
		NV_ATMEL_24C64,
		FIO_FRU_ID,
		FIO_FRU_PORT,
		FIO_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	{
		NV_ATMEL_24C64,
		HSC_MODULE_FRU_ID,
		HSC_MODULE_FRU_PORT,
		HSC_MODULE_FRU_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
		/* Because there has mux before eeprom, so mux_present is true to access mux first */
		true,
		HSC_MODULE_FRU_MUX_ADDR,
		HSC_MODULE_FRU_MUX_CHAN,
	},
};

void pal_load_fru_config(void)
{
	memcpy(&fru_config, &plat_fru_config, sizeof(plat_fru_config));
}
