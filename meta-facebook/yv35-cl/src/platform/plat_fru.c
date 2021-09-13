#include <stdio.h>
#include "fru.h"
#include "plat_fru.h"
#include "pal.h"
#include "fru.h"

#define MB_FRU_PORT 0x01
#define MB_FRU_ADDR 0x54

const EEPROM_CFG plat_fru_config[] = {
  {
    NV_ATMEL_24C128,
    MB_FRU_ID,
    MB_FRU_PORT,
    MB_FRU_ADDR,
    FRU_DEV_ACCESS_BYTE,
    FRU_START,
    FRU_SIZE,
  },
};


void pal_load_fru_config(void) {
  memcpy(&fru_config, &plat_fru_config, sizeof(plat_fru_config));
}
